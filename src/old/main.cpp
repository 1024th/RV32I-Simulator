#include <cstdlib>
#include <iostream>
#include <string>

using u32 = u_int32_t;
using i32 = int32_t;
using u8 = u_int8_t;

class Memory {
  static constexpr const u32 kMemSize = 500000;
  u8 mem_[kMemSize];
  u32 pos = 0;

 public:
  Memory() = default;
  void InitializeFrom(std::istream &input_stream) {
    std::string input;
    char *nxt;
    while (input_stream >> input) {
      if (input[0] == '@') {
        pos = strtoul(input.c_str() + 1, &nxt, 16);
      } else {
        mem_[pos++] = strtoul(input.c_str(), &nxt, 16);
        // printf("%d %02X\n", pos - 1, mem_[pos - 1]);
      }
    }
  }
  u8 &operator[](u32 i) { return mem_[i]; }
};

Memory memory;

class RISC_V {
 public:
  Memory memory;
  u32 reg[32] = {}, pc = 0;
  RISC_V() = default;
  u32 GetOpcode(u32 inst) { return inst & 0b1111111; }
  u32 GetFunct3(u32 inst) { return inst >> 12 & 0b111; }
  u32 GetRs1(u32 inst) { return inst >> 15 & 0b11111; }
  u32 GetRs2(u32 inst) { return inst >> 20 & 0b11111; }
  u32 GetShamt(u32 inst) { return inst >> 20 & 0b111111; }
  u32 GetRd(u32 inst) { return inst >> 7 & 0b11111; }
  enum InstFormat {
    R,  // 寄存器-寄存器操作
    I,  // 短立即数和访存 load 操作
    S,  // 访存 store 操作
    B,  // 条件跳转操作
    U,  // 用于长立即数
    J,  // 无条件跳转
  };
  // clang-format off
  enum InstType {
    LUI,    // U, load upper immediate
    AUIPC,  // U, add upper immediate to pc
    JAL,    // J, jump and link
    JALR,   // I, jump and link register
    BEQ,    // B, branch equal
    BNE,    // B, branch not equal
    BLT,    // B, branch less than
    BGE,    // B, branch greater than or equal
    BLTU,   // B, branch less than unsigned
    BGEU,   // B, branch greater than or equal unsigned
    LB,     // I, load byte
    LH,     // I, load halfword
    LW,     // I, load word
    LBU,    // I, load byte unsigned
    LHU,    // I, load halfword unsigned
    SB,     // S, store byte
    SH,     // S, store halfword
    SW,     // S, store word
    ADDI,   // I, add immediate
    SLTI,   // I, set if less than immediate
    SLTIU,  // I, set if less than immediate unsigned
    XORI,   // I, exclusive or immediate
    ORI,    // I, or immediate
    ANDI,   // I, and immediate
    SLLI,   // I, shift left logical immediate
    SRLI,   // I, shift right logical immediate
    SRAI,   // I, shift right arithmetic immediate
    ADD,    // R, add
    SUB,    // R, subtract
    SLL,    // R, shift left logical
    SLT,    // R, set less than
    SLTU,   // R, set less than unsigned
    XOR,    // R, exclusive or
    SRL,    // R, shift right logical
    SRA,    // R, shift right arithmetic
    OR,     // R, or
    AND     // R, and
  };
  InstFormat get_format[40] = {
    U, U, J, I,  // lui, auipc, jal, jalr
    B, B, B, B, B, B,  // beq, bne, blt, bge, bltu, bgeu
    I, I, I, I, I,  // lb, lh, lw, lbu, lhu
    S, S, S,  // sb, sh, sw
    I, I, I, I, I, I, I, I, I,  // addi, slti, sltiu, xori, ori, andi, slli, srli, srai
    R, R, R, R, R, R, R, R, R, R  // add, sub, sll, slt, sltu, xor, srl, sra, or, and
  };
  const char* inst_type_name[40] = {
    "LUI", "AUIPC", "JAL", "JALR",
    "BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU",
    "LB", "LH", "LW", "LBU", "LHU", "SB", "SH", "SW",
    "ADDI", "SLTI", "SLTIU", "XORI", "ORI", "ANDI", "SLLI", "SRLI", "SRAI",
    "ADD", "SUB", "SLL", "SLT", "SLTU", "XOR", "SRL", "SRA", "OR", "AND"
  };
  const char* inst_format_name[10] = {
    "R", "I", "S", "B", "U", "J"
  };
  // high_bit is the original bit number of x
  u32 SignExtend(u32 x, u8 high_bit) {
    if (x >> (high_bit - 1) & 1) x |= 0xFFFFFFFFu << high_bit;
    return x;
  }
  u32 GetImm(u32 inst, InstFormat format) {
    u32 imm = 0;
    switch (format) {
      case R: break;
      case I: imm = SignExtend(inst >> 20, 12); break;
      case S: imm = SignExtend(
          inst >> 7  & 0b000000011111u |  //
          inst >> 20 & 0b111111100000u, 12);
        break;
      case B:
        imm = SignExtend(  // 76543210
          inst >> 19 & 0b1000000000000 |  //
          inst << 4  & 0b0100000000000 |  //
          inst >> 20 & 0b0011111100000 |  //
          inst >> 7  & 0b0000000011110, 13);
        break;
      case U: imm = inst & 0b11111111111111111111000000000000u; break;
      case J:
        imm = SignExtend(  // 5432109876543210
          inst >> 11 & 0b100000000000000000000 |  //
          inst       & 0b011111111000000000000 |  //
          inst >> 9  & 0b000000000100000000000 |  //
          inst >> 20 & 0b000000000011111111110, 21);
        break;
    }
    return imm;
  }
  void (RISC_V::*Do[40])() = {
    &RISC_V::DoLUI,
    &RISC_V::DoAUIPC,
    &RISC_V::DoJAL,
    &RISC_V::DoJALR,
    &RISC_V::DoBEQ,
    &RISC_V::DoBNE,
    &RISC_V::DoBLT,
    &RISC_V::DoBGE,
    &RISC_V::DoBLTU,
    &RISC_V::DoBGEU,
    &RISC_V::DoLB,
    &RISC_V::DoLH,
    &RISC_V::DoLW,
    &RISC_V::DoLBU,
    &RISC_V::DoLHU,
    &RISC_V::DoSB,
    &RISC_V::DoSH,
    &RISC_V::DoSW,
    &RISC_V::DoADDI,
    &RISC_V::DoSLTI,
    &RISC_V::DoSLTIU,
    &RISC_V::DoXORI,
    &RISC_V::DoORI,
    &RISC_V::DoANDI,
    &RISC_V::DoSLLI,
    &RISC_V::DoSRLI,
    &RISC_V::DoSRAI,
    &RISC_V::DoADD,
    &RISC_V::DoSUB,
    &RISC_V::DoSLL,
    &RISC_V::DoSLT,
    &RISC_V::DoSLTU,
    &RISC_V::DoXOR,
    &RISC_V::DoSRL,
    &RISC_V::DoSRA,
    &RISC_V::DoOR,
    &RISC_V::DoAND
  };
  // clang-format on
  u32 opcode, rs1, rs2, rd, shamt, imm;
  void DecodeInstruction(u32 inst) {
    opcode = GetOpcode(inst);
    rs2 = GetRs2(inst), rs1 = GetRs1(inst), rd = GetRd(inst), shamt = GetShamt(inst);
    InstType inst_type;
    switch (opcode) {
      case 0b0110111: inst_type = LUI; break;
      case 0b0010111: inst_type = AUIPC; break;
      case 0b1101111: inst_type = JAL; break;
      case 0b1100111: inst_type = JALR; break;
      case 0b1100011: {
        u32 funct3 = GetFunct3(inst);
        switch (funct3) {
          case 0b000: inst_type = BEQ; break;
          case 0b001: inst_type = BNE; break;
          case 0b100: inst_type = BLT; break;
          case 0b101: inst_type = BGE; break;
          case 0b110: inst_type = BLTU; break;
          case 0b111: inst_type = BGEU; break;
        }
        break;
      }
      case 0b0000011: {
        u32 funct3 = GetFunct3(inst);
        switch (funct3) {
          case 0b000: inst_type = LB; break;
          case 0b001: inst_type = LH; break;
          case 0b010: inst_type = LW; break;
          case 0b100: inst_type = LBU; break;
          case 0b101: inst_type = LHU; break;
        }
        break;
      }
      case 0b0100011: {
        u32 funct3 = GetFunct3(inst);
        switch (funct3) {
          case 0b000: inst_type = SB; break;
          case 0b001: inst_type = SH; break;
          case 0b010: inst_type = SW; break;
        }
        break;
      }
      case 0b0010011: {
        u32 funct3 = GetFunct3(inst);
        switch (funct3) {
          case 0b000: inst_type = ADDI; break;
          case 0b010: inst_type = SLTI; break;
          case 0b011: inst_type = SLTIU; break;
          case 0b100: inst_type = XORI; break;
          case 0b110: inst_type = ORI; break;
          case 0b111: inst_type = ANDI; break;
          case 0b001: inst_type = SLLI; break;
          case 0b101: inst_type = (inst >> 30 & 1) ? SRAI : SRLI; break;
        }
        break;
      }
      case 0b0110011: {
        u32 funct3 = GetFunct3(inst);
        switch (funct3) {
          case 0b000: inst_type = (inst >> 30 & 1) ? SUB : ADD; break;
          case 0b001: inst_type = SLL; break;
          case 0b010: inst_type = SLT; break;
          case 0b011: inst_type = SLTU; break;
          case 0b100: inst_type = XOR; break;
          case 0b101: inst_type = (inst >> 30 & 1) ? SRA : SRL; break;
          case 0b110: inst_type = OR; break;
          case 0b111: inst_type = AND; break;
        }
        break;
      }
    }
    InstFormat inst_format = get_format[inst_type];
    imm = GetImm(inst, inst_format);
    printf("inst %08X [%s]%s, rs1: %d, rs2: %d, imm: %d, rd:%d\n", inst, inst_format_name[inst_format],
        inst_type_name[inst_type], rs1, rs2, imm, rd);
    (this->*(Do[inst_type]))();
  }
  void DoLUI() {
    reg[rd] = imm;
    pc += 4;
  }
  void DoAUIPC() {
    pc += imm;
    reg[rd] = pc;
  }
  void DoJAL() {
    reg[rd] = pc + 4;
    pc += imm;
  }
  void DoJALR() {
    reg[rd] = pc + 4;
    pc = (reg[rs1] + imm) & ~(u32)1;
  }
  void DoBEQ() {
    if (reg[rs1] == reg[rs2])
      pc += imm;
    else
      pc += 4;
  }
  void DoBNE() {
    if (reg[rs1] != reg[rs2])
      pc += imm;
    else
      pc += 4;
  }
  void DoBLT() {
    if ((i32)reg[rs1] < (i32)reg[rs2])
      pc += imm;
    else
      pc += 4;
  }
  void DoBGE() {
    if ((i32)reg[rs1] >= (i32)reg[rs2])
      pc += imm;
    else
      pc += 4;
  }
  void DoBLTU() {
    if (reg[rs1] < reg[rs2])
      pc += imm;
    else
      pc += 4;
  }
  void DoBGEU() {
    if (reg[rs1] >= reg[rs2])
      pc += imm;
    else
      pc += 4;
  }
  void DoLB() {
    reg[rd] = SignExtend(memory[reg[rs1] + imm], 8);
    pc += 4;
  }
  void DoLH() {
    reg[rd] = SignExtend((u32)memory[reg[rs1] + imm] | (u32)memory[reg[rs1] + imm + 1] << 8, 16);
    pc += 4;
  }
  void DoLW() {
    reg[rd] = (u32)memory[reg[rs1] + imm] | (u32)memory[reg[rs1] + imm + 1] << 8 |
              (u32)memory[reg[rs1] + imm + 2] << 16 | (u32)memory[reg[rs1] + imm + 3] << 24;
    pc += 4;
  }
  void DoLBU() {
    reg[rd] = memory[reg[rs1] + imm];
    pc += 4;
  }
  void DoLHU() {
    reg[rd] = (u32)memory[reg[rs1] + imm] | (u32)memory[reg[rs1] + imm + 1] << 8;
    pc += 4;
  }
  void DoSB() {
    memory[reg[rs1] + imm] = (u8)reg[rs2];
    pc += 4;
  }
  void DoSH() {
    memory[reg[rs1] + imm] = (u8)reg[rs2];
    memory[reg[rs1] + imm + 1] = (u8)(reg[rs2] >> 8);
    pc += 4;
  }
  void DoSW() {
    memory[reg[rs1] + imm] = (u8)reg[rs2];
    memory[reg[rs1] + imm + 1] = (u8)(reg[rs2] >> 8);
    memory[reg[rs1] + imm + 2] = (u8)(reg[rs2] >> 16);
    memory[reg[rs1] + imm + 3] = (u8)(reg[rs2] >> 24);
    pc += 4;
  }
  void DoADDI() {
    reg[rd] = reg[rs1] + imm;
    pc += 4;
  }
  void DoSLTI() {
    reg[rd] = (i32)reg[rs1] < (i32)imm;
    pc += 4;
  }
  void DoSLTIU() {
    reg[rd] = reg[rs1] < imm;
    pc += 4;
  }
  void DoXORI() {
    reg[rd] = reg[rs1] ^ imm;
    pc += 4;
  }
  void DoORI() {
    reg[rd] = reg[rs1] | imm;
    pc += 4;
  }
  void DoANDI() {
    reg[rd] = reg[rs1] & imm;
    pc += 4;
  }
  void DoSLLI() {
    reg[rd] = reg[rs1] << shamt;
    pc += 4;
  }
  void DoSRLI() {
    reg[rd] = reg[rs1] >> shamt;
    pc += 4;
  }
  void DoSRAI() {
    reg[rd] = SignExtend(reg[rs1] >> shamt, 32 - shamt);
    pc += 4;
  }
  void DoADD() {
    reg[rd] = reg[rs1] + reg[rs2];
    pc += 4;
  }
  void DoSUB() {
    reg[rd] = reg[rs1] - reg[rs2];
    pc += 4;
  }
  void DoSLL() {
    reg[rd] = reg[rs1] << (reg[rs2] & 31u);
    pc += 4;
  }
  void DoSLT() {
    reg[rd] = (i32)reg[rs1] < (i32)reg[rs2];
    pc += 4;
  }
  void DoSLTU() {
    reg[rd] = reg[rs1] < reg[rs2];
    pc += 4;
  }
  void DoXOR() {
    reg[rd] = reg[rs1] ^ reg[rs2];
    pc += 4;
  }
  void DoSRL() {
    reg[rd] = reg[rs1] >> (reg[rs2] & 31u);
    pc += 4;
  }
  void DoSRA() {
    reg[rd] = SignExtend(reg[rs1] >> (reg[rs2] & 31u), 32 - (reg[rs2] & 31u));
    pc += 4;
  }
  void DoOR() {
    reg[rd] = reg[rs1] | reg[rs2];
    pc += 4;
  }
  void DoAND() {
    reg[rd] = reg[rs1] & reg[rs2];
    pc += 4;
  }
  void Run() {
    while (1) {
#ifdef LTC
      // printf("----\npc(hex): %02X, mem: %02X %02X %02X %02X\n", pc, memory[pc], memory[pc + 1], memory[pc + 2],
      //     memory[pc + 3]);
      // for (int i = 0; i < 32; ++i) printf("%d ", reg[i]);
      puts("---");
      for (int i = 0; i < 32; ++i) {
        if (i && i % 8 == 0) puts("");
        printf("%6d ", reg[i]);
      }
      puts("\nmemory 131040-131079:");
      for (int i = 131040; i < 131080; ++i) {
        printf("%02X ", memory[i]);
        if (i % 10 == 9) puts("");
      }
      // puts("");
      printf("pc: %d\n", pc);
#endif  // LTC
      u32 inst = (u32)memory[pc] | (u32)memory[pc + 1] << 8 | (u32)memory[pc + 2] << 16 | (u32)memory[pc + 3] << 24;
      // printf("inst: %08X\n", inst);
      if (inst == 0x0ff00513u) {
        printf("%d", reg[10] & 255u);
        break;
      }
      DecodeInstruction(inst);
      reg[0] = 0;  // prevent x0 from being modified
    }
  }
} simulator;

int main() {
  // freopen("../sample/sample.data", "r", stdin);
  simulator.memory.InitializeFrom(std::cin);
  simulator.Run();
  return 0;
}