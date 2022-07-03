#include <cstdio>
#include <iostream>
#include <optional>
#include <tuple>

using u32 = u_int32_t;
using u16 = u_int16_t;
using i32 = int32_t;
using u8 = u_int8_t;

constexpr const u32 kMemorySize = 500000;
constexpr const u32 kReservationStationSize = 32, kStoreLoadBufferSize = 32, kReorderBufferSize = 32,
                    kInstQueueSize = 32, kRegisterSize = 32;
constexpr const u32 kBranchPredictorSize = 512;
constexpr const u8 kBranchPredictorHistoryLen = 3;

template <u32 kSize = kMemorySize>
class Memory {
  u8 mem_[kSize];
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
#ifdef LTC
        // printf("%d %02X\n", pos - 1, mem_[pos - 1]);
#endif  // LTC
      }
    }
  }
  u8 &operator[](u32 i) { return mem_[i]; }
  u8 GetByte(u32 i) { return mem_[i]; }
  u16 GetHalfWord(u32 i) { return (u16)mem_[i] | (u16)mem_[i + 1] << 8; }
  u32 GetWord(u32 i) { return (u32)mem_[i] | (u32)mem_[i + 1] << 8 | (u32)mem_[i + 2] << 16 | (u32)mem_[i + 3] << 24; }
  void WriteByte(u32 i, u8 val) { mem_[i] = val; }
  void WriteHalfWord(u32 i, u16 val) { mem_[i] = (u8)val, mem_[i + 1] = (u8)(val >> 8); }
  void WriteWord(u32 i, u32 val) {
    mem_[i] = (u8)val;
    mem_[i + 1] = (u8)(val >> 8);
    mem_[i + 2] = (u8)(val >> 16);
    mem_[i + 3] = (u8)(val >> 24);
  }
};

template <typename T, u32 kSize>
class LoopQueue {
  T val[kSize];

 public:
  u32 head = 1, tail = 1;  // starts at 1 because 0 represents ready in ROB id
  T &operator[](u32 pos) { return val[pos % kSize]; }
  const T &operator[](u32 pos) const { return val[pos % kSize]; }

  T &front() { return val[head % kSize]; }
  void push(const T &v) { val[tail++ % kSize] = v; }
  void pop() { ++head; }
  u32 size() { return tail - head; }
  void clear() { head = tail = 1; }
  bool empty() { return head == tail; }
  bool full() { return size() >= kSize; }
  std::optional<u32> Allocate() {
    if (full()) return std::nullopt;
    return tail;
  }
};

struct Instruction {
  // clang-format off
  enum InstFormat {
    R,  // 寄存器-寄存器操作
    I,  // 短立即数和访存 load 操作
    S,  // 访存 store 操作
    B,  // 条件跳转操作
    U,  // 用于长立即数
    J,  // 无条件跳转
  };
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
  // high_bit is the original bit number of x
  static u32 SignExtend(u32 x, u8 high_bit) {
    if (x >> (high_bit - 1) & 1) x |= 0xFFFFFFFFu << high_bit;
    return x;
  }

  u32 inst;
  u32 opcode, rs1 = 0, rs2 = 0, shamt = 0, rd = 0, imm;
  
  InstType inst_type = AND;
  // InstFormat inst_format;
  Instruction() = default;
  Instruction(u32 inst) : inst(inst) {
    Decode();
  }

  u32 GetOpcode() { return inst & 0b1111111; }
  u32 GetFunct3() { return inst >> 12 & 0b111; }
  u32 GetRs1() { return inst >> 15 & 0b11111; }
  u32 GetRs2() { return inst >> 20 & 0b11111; }
  u32 GetShamt() { return inst >> 20 & 0b111111; }
  u32 GetRd() { return inst >> 7 & 0b11111; }
  // static InstFormat GetFormat(InstType t) {
  //   static InstFormat get_format[40] = {
  //     U, U, J, I,  // lui, auipc, jal, jalr
  //     B, B, B, B, B, B,  // beq, bne, blt, bge, bltu, bgeu
  //     I, I, I, I, I,  // lb, lh, lw, lbu, lhu
  //     S, S, S,  // sb, sh, sw
  //     I, I, I, I, I, I, I, I, I,  // addi, slti, sltiu, xori, ori, andi, slli, srli, srai
  //     R, R, R, R, R, R, R, R, R, R  // add, sub, sll, slt, sltu, xor, srl, sra, or, and
  //   };
  //   return get_format[t];
  // }
  #ifdef LTC
  static constexpr const char* inst_type_name[40] = {
    "LUI", "AUIPC", "JAL", "JALR",
    "BEQ", "BNE", "BLT", "BGE", "BLTU", "BGEU",
    "LB", "LH", "LW", "LBU", "LHU", "SB", "SH", "SW",
    "ADDI", "SLTI", "SLTIU", "XORI", "ORI", "ANDI", "SLLI", "SRLI", "SRAI",
    "ADD", "SUB", "SLL", "SLT", "SLTU", "XOR", "SRL", "SRA", "OR", "AND"
  };
  static constexpr const char* inst_format_name[10] = {
    "R", "I", "S", "B", "U", "J"
  };
  void Print(bool newline = true) const {
    printf("inst %08X %s rs1:%d rs2:%d rd:%d imm:%d", inst, inst_type_name[inst_type], rs1, rs2, rd, imm);
    if (newline) puts("");
  }
  #endif //LTC
  
  u32 GetImm(u32 inst, InstFormat format) {
    u32 imm = 0;
    switch (format) {
      case R: break;
      case I: imm = SignExtend(inst >> 20, 12); break;
      case S: imm = SignExtend(
          (inst >> 7  & 0b000000011111u) |  //
          (inst >> 20 & 0b111111100000u), 12);
        break;
      case B:
        imm = SignExtend(  // 76543210
          (inst >> 19 & 0b1000000000000) |  //
          (inst << 4  & 0b0100000000000) |  //
          (inst >> 20 & 0b0011111100000) |  //
          (inst >> 7  & 0b0000000011110), 13);
        break;
      case U: imm = inst & 0b11111111111111111111000000000000u; break;
      case J:
        imm = SignExtend(  // 5432109876543210
          (inst >> 11 & 0b100000000000000000000) |  //
          (inst       & 0b011111111000000000000) |  //
          (inst >> 9  & 0b000000000100000000000) |  //
          (inst >> 20 & 0b000000000011111111110), 21);
        break;
    }
    return imm;
  }
  void Decode() {
    opcode = GetOpcode();
    InstFormat inst_format = R;
    switch (opcode) {
      case 0b0110111: inst_type = LUI, inst_format = U; break;
      case 0b0010111: inst_type = AUIPC, inst_format = U; break;
      case 0b1101111: inst_type = JAL, inst_format = J; break;
      case 0b1100111: inst_type = JALR, inst_format = I; break;
      case 0b1100011: {
        inst_format = B;
        u32 funct3 = GetFunct3();
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
        inst_format = I;
        u32 funct3 = GetFunct3();
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
        inst_format = S;
        u32 funct3 = GetFunct3();
        switch (funct3) {
          case 0b000: inst_type = SB; break;
          case 0b001: inst_type = SH; break;
          case 0b010: inst_type = SW; break;
        }
        break;
      }
      case 0b0010011: {
        inst_format = I;
        u32 funct3 = GetFunct3();
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
        inst_format = R;
        u32 funct3 = GetFunct3();
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
    if (inst_format != U && inst_format != J) {
      if (inst_format != I) rs2 = GetRs2();
      rs1 = GetRs1();
    }
    if (inst_format != S && inst_format != B) rd = GetRd();
    if (inst_type == SLLI || inst_type == SRLI || inst_type == SRAI) shamt = GetShamt();
    imm = GetImm(inst, inst_format);
    // printf("inst type: [%s]%s, rs1: %d, rs2: %d, imm: %d, rd:%d\n", inst_format_name[inst_format],
    //     inst_type_name[inst_type], rs1, rs2, imm, rd);
  }
  bool IsStore() const { return opcode == 0b0100011; }
  bool IsLoad() const { return opcode == 0b0000011; }
  bool IsStoreLoad() const { return IsStore() || IsLoad(); }
  bool IsBranch() const { return opcode == 0b1100011; }
  bool IsJump() const { return inst_type == JAL || inst_type == JALR; }
  bool NeedWriteReg() const {
    return !IsStore() && !IsBranch() && rd != 0;
  }
  bool IsJumpBranch() const { return IsBranch() || IsJump(); }
  // clang-format on
};

struct Register {
  u32 val, rob;
};

template <u32 kSize = kRegisterSize>
struct RegisterFile {
  Register reg[kSize];
  Register &operator[](u32 i) { return reg[i]; }
};

template <class T>
struct Wire {
  bool stall = true;
  T val;
  operator bool() const { return !stall; }
  T *operator->() { return &val; }
  void Set(const T &v) { val = v, stall = false; }
  void Reset() { stall = true; }
};

class Simulator {
  Memory<> mem;
  RegisterFile<> reg_prev, reg_next;
  u32 pc = 0;

  struct InstQueueItem {
    Instruction inst;
    u32 pc;
    bool predict_jump = false;
  };
  struct ReservationStationItem {
    bool ready = true;
    Instruction inst;
    u32 rs1_val, rs1_rob, rs2_val, rs2_rob, rd_rob;
    u32 imm, pc;
  };
  template <u32 kSize = kReservationStationSize>
  class ReservationStation {
   public:
    ReservationStationItem node_[kSize];
    ReservationStationItem &operator[](u32 i) { return node_[i]; }
    // bool Full() const {
    //   for (u32 i = 0; i < kSize; ++i) {
    //     if (node_[i].ready) return false;
    //   }
    //   return true;
    // }
    std::optional<u32> Allocate() {
      for (u32 i = 0; i < kSize; ++i) {
        if (node_[i].ready) return i;
      }
      return std::nullopt;
    }
  };
  struct ex_result_t {
    u32 value, pc;
    bool jump = false;
  };
  struct ReorderBufferItem {
    bool ready = false;
    Instruction inst;
    u32 reg_id, rs_id, slb_id;
    bool predict_jump = false;
    ex_result_t result;
  };
  struct StoreLoadBufferItem {
    // bool ready;
    Instruction inst;
    u32 rs1_val, rs1_rob, rs2_val, rs2_rob, rd_rob, time;
    u32 imm;
    bool committed, busy = false;
  };

  LoopQueue<InstQueueItem, kInstQueueSize> inst_queue_prev, inst_queue_next;
  ReservationStation<> rs_prev, rs_next;
  LoopQueue<StoreLoadBufferItem, kStoreLoadBufferSize> slb_prev, slb_next;
  LoopQueue<ReorderBufferItem, kReorderBufferSize> rob_prev, rob_next;

  template <u32 kSize = kBranchPredictorSize, u8 kHisLen = kBranchPredictorHistoryLen>
  class BranchPredictor {
    u8 predict_table[kSize][1 << kHisLen] = {};
    u32 branch_history[kSize] = {};
    u32 total_cnt = 0, success_cnt = 0;
    static constexpr const u32 his_mask = (1u << kHisLen) - 1;
    inline u32 Hash(u32 pc) { return pc % kSize; }

   public:
    // true: jump; false: not jump
    bool Predict(u32 pc) {
      auto hash = Hash(pc);
      return predict_table[hash][branch_history[hash]] > 1;
    }
    void RecordBranch(u32 pc, bool jump) {
      auto hash = Hash(pc), his = branch_history[hash];
      if (jump) {
        if (predict_table[hash][his] < 3) ++predict_table[hash][his];
      } else {
        if (predict_table[hash][his] > 0) --predict_table[hash][his];
      }
      branch_history[hash] = ((his << 1) & his_mask) | jump;
    }
    void CountResult(bool success) {
      ++total_cnt;
      if (success) ++success_cnt;
    }
    void PrintCount() {
      fprintf(stderr, "total branch number:%d, successfully predicted:%d, rate:%f", total_cnt, success_cnt,
          (double)success_cnt / (double)total_cnt);
    }
  };
  BranchPredictor<> predictor;

  struct issue_to_slb_t {
    Instruction inst;
    u32 rs1, rs2, rd_rob, imm, time;
  };
  Wire<issue_to_slb_t> issue_to_slb;
  struct issue_to_rs_t {
    Instruction inst;
    u32 rs1, rs2, rd_rob, imm, pc, rs_id;
  };
  Wire<issue_to_rs_t> issue_to_rs;
  Wire<ReorderBufferItem> issue_to_rob;
  struct issue_to_reg_t {
    u32 reg_id, rob;
  };
  Wire<issue_to_reg_t> issue_to_reg;

  struct ex_to_rs_t {
    u32 rob_id, val;
  };
  Wire<ex_to_rs_t> ex_to_rs, slb_to_rs_prev, slb_to_rs_next;
  using ex_to_slb_t = ex_to_rs_t;
  Wire<ex_to_slb_t> ex_to_slb, slb_to_slb_prev, slb_to_slb_next;
  struct ex_to_rob_t {
    u32 rob_id;
    ex_result_t result;
  };
  Wire<ex_to_rob_t> ex_to_rob;
  struct slb_to_rob_t {
    u32 rob_id;
    bool is_load;
    ex_result_t result;
  };
  Wire<slb_to_rob_t> slb_to_rob_prev, slb_to_rob_next;

  struct rs_to_ex_t {
    Instruction inst;
    u32 v1, v2, imm, rob_id, pc;
  };
  Wire<rs_to_ex_t> rs_to_ex;

  struct rob_to_commit_t {
    Instruction inst;
    u32 reg_id, rob_id, slb_id;
    ex_result_t result;
  };
  Wire<rob_to_commit_t> rob_to_commit;

  struct commit_to_reg_t {
    u32 reg_id, rob_id, value;
  };
  Wire<commit_to_reg_t> commit_to_reg;
  struct commit_to_slb_t {
    u32 slb_id;
  };
  Wire<commit_to_slb_t> commit_to_slb;

  struct Signal {
    bool clear_all = 0;
  } signal;

  ex_result_t ExecuteBranch(u32 inst_pc, u32 imm, bool jump) {
    predictor.RecordBranch(inst_pc, jump);
    return ex_result_t{0, jump ? inst_pc + imm : inst_pc + 4, jump};
  }

  ex_result_t ExecuteInst(Instruction inst, u32 v1, u32 v2, u32 imm, u32 pc) {
    switch (inst.inst_type) {
      case Instruction::LUI: return ex_result_t{imm, 0, 0};
      case Instruction::AUIPC: return ex_result_t{pc + imm, pc + imm, 1};

      case Instruction::JAL: return ex_result_t{pc + 4, pc + imm, 1};
      case Instruction::JALR: return ex_result_t{pc + 4, (v1 + imm) & ~(u32)1, 1};

      case Instruction::BEQ: return ExecuteBranch(pc, imm, v1 == v2);
      case Instruction::BNE: return ExecuteBranch(pc, imm, v1 != v2);
      case Instruction::BLT: return ExecuteBranch(pc, imm, (i32)v1 < (i32)v2);
      case Instruction::BGE: return ExecuteBranch(pc, imm, (i32)v1 >= (i32)v2);
      case Instruction::BLTU: return ExecuteBranch(pc, imm, v1 < v2);
      case Instruction::BGEU: return ExecuteBranch(pc, imm, v1 >= v2);

      case Instruction::LB: return ex_result_t{Instruction::SignExtend(mem.GetByte(v1 + imm), 8), 0, 0};
      case Instruction::LH: return ex_result_t{Instruction::SignExtend(mem.GetHalfWord(v1 + imm), 16), 0, 0};
      case Instruction::LW: return ex_result_t{mem.GetWord(v1 + imm), 0, 0};
      case Instruction::LBU: return ex_result_t{mem.GetByte(v1), 0, 0};
      case Instruction::LHU: return ex_result_t{mem.GetHalfWord(v1), 0, 0};

      case Instruction::SB: mem.WriteByte(v1 + imm, v2); return ex_result_t{0, 0, 0};
      case Instruction::SH: mem.WriteHalfWord(v1 + imm, v2); return ex_result_t{0, 0, 0};
      case Instruction::SW: mem.WriteWord(v1 + imm, v2); return ex_result_t{0, 0, 0};

      case Instruction::ADDI: return ex_result_t{v1 + imm, 0, 0};
      case Instruction::SLTI: return ex_result_t{(i32)v1 < (i32)imm, 0, 0};
      case Instruction::SLTIU: return ex_result_t{v1 < imm, 0, 0};
      case Instruction::XORI: return ex_result_t{v1 ^ imm, 0, 0};
      case Instruction::ORI: return ex_result_t{v1 | imm, 0, 0};
      case Instruction::ANDI: return ex_result_t{v1 & imm, 0, 0};
      case Instruction::SLLI: return ex_result_t{v1 << inst.shamt, 0, 0};
      case Instruction::SRLI: return ex_result_t{v1 >> inst.shamt, 0, 0};
      case Instruction::SRAI: return ex_result_t{u32((i32)v1 >> (i32)inst.shamt), 0, 0};

      case Instruction::ADD: return ex_result_t{v1 + v2, 0, 0};
      case Instruction::SUB: return ex_result_t{v1 - v2, 0, 0};
      case Instruction::SLL: return ex_result_t{v1 << (v2 & 31u), 0, 0};
      case Instruction::SLT: return ex_result_t{(i32)v1 < (i32)v2, 0, 0};
      case Instruction::SLTU: return ex_result_t{v1 < v2, 0, 0};
      case Instruction::XOR: return ex_result_t{v1 ^ v2, 0, 0};
      case Instruction::SRL: return ex_result_t{v1 >> (v2 & 31u), 0, 0};
      case Instruction::SRA: return ex_result_t{u32((i32)v1 >> (i32)(v2 & 31u)), 0, 0};
      case Instruction::OR: return ex_result_t{v1 | v2, 0, 0};
      case Instruction::AND: return ex_result_t{v1 & v2, 0, 0};
      default: throw -1; return ex_result_t{0, 0, 0};
    }
  }

 public:
  void InitializeMemory(std::istream &input_stream) { mem.InitializeFrom(input_stream); }
  void Run() {
    for (u32 cycle = 0;; ++cycle) {
      /*在这里使用了两阶段的循环部分：
        1. 实现时序电路部分，即在每个周期初同步更新的信息。
        2. 实现逻辑电路部分，即在每个周期中如 ex、issue 的部分
        已在下面给出代码
      */

#ifdef LTC
      if (cycle > 1000) {
        puts("Infinite loop!");
        break;
      }
      printf("---\nCycle %d, reg_prev:", cycle);
      for (u32 i = 0; i < kRegisterSize; ++i) {
        if (i % 8 == 0) puts("");
        printf("%6d,%d ", reg_prev[i].val, reg_prev[i].rob);
      }
      puts("\nmemory 131068-131071:");
      for (u32 i = 131068; i < 131072; ++i) {
        printf("%02X", mem.GetByte(i));
      }
      puts("");
#endif  // LTC

      RunReorderBuffer();
#ifdef LTC
      if (rob_to_commit) {
        const auto &inst = rob_to_commit->inst;
        const auto &res = rob_to_commit->result;
        printf("rob to commit rob:%d res val:%d reg#%d jump?%d to_pc:%d ", rob_to_commit->rob_id, res.value,
            rob_to_commit->reg_id, res.jump, res.pc);
        inst.Print();
      }
#endif  // LTC
      if (rob_to_commit && rob_to_commit->inst.inst == 0x0ff00513) {
        printf("%u\n", reg_prev[10].val & 255u);
        predictor.PrintCount();
        break;
      }

      RunStoreLoadBuffer();
      RunReservationStation();
      RunRegfile();
      FetchInst();

      Update();

      Execute();
      Issue();
      Commit();
    }
  }

  void FetchInst() {
    /*
    在这一部分你需要完成的工作：
    1. 实现一个先进先出的指令队列
    2. 读取指令并存放到指令队列中
    3. 准备好下一条 issue 的指令
    tips: 考虑边界问题（满/空...）
    */
    if (signal.clear_all) {
      inst_queue_next.clear();
      return;
    }
    if (!inst_queue_next.full()) {
      Instruction inst{mem.GetWord(pc)};
      auto inst_pc = pc;
      bool predict_jump = false;
#ifdef LTC
      printf("fetch ");
      inst.Print(false);
      printf(" pc:%d\n", pc);
#endif  // LTC
      if (inst.inst_type == Instruction::JAL) {
        pc += inst.imm;
      } else if (inst.IsBranch()) {
        predict_jump = predictor.Predict(inst_pc);
        if (predict_jump)
          pc += inst.imm;
        else
          pc += 4;
      } else {
        pc += 4;
      }
      inst_queue_next.push(InstQueueItem{inst, inst_pc, predict_jump});
    }
  }

  void Issue() {
    /*
    在这一部分你需要完成的工作：
    1. 从 FetchInst()中得到 issue 的指令
    2. 对于 issue 的所有类型的指令向 ROB 申请一个位置（或者也可以通过 ROB 预留位置），并修改 regfile 中相应的值
    2. 对于 非 Load/Store 的指令，将指令进行分解后发到 Reservation Station
      tip: 1. 这里需要考虑怎么得到 rs1、rs2 的值，并考虑如当前 rs1、rs2 未被计算出的情况，参考书上内容进行处理
           2. 在本次作业中，我们认为相应寄存器的值已在 ROB 中存储但尚未 commit
              的情况是可以直接获得的，即你需要实现这个功能 而对于 rs1、rs2 不 ready 的情况，只需要 stall
              即可，有兴趣的同学可以考虑下怎么样直接从 EX 完的结果更快的得到计算结果
    3. 对于 Load/Store 指令，将指令分解后发到 SLBuffer（需注意 SLBUFFER 也该是个先进先出的队列实现）
    tips: 考虑边界问题（是否还有足够的空间存放下一条指令）
    */
    issue_to_reg.Reset(), issue_to_rob.Reset(), issue_to_slb.Reset(), issue_to_rs.Reset();

    if (inst_queue_prev.empty()) return;
    auto tmp = rob_prev.Allocate();
    if (!tmp.has_value()) return;
    auto rob_id = tmp.value();

    const auto &it = inst_queue_prev.front();
    const auto &inst = it.inst;
    u32 rs_id, slb_id;

    if (inst.IsStoreLoad()) {
      tmp = slb_prev.Allocate();
      if (!tmp.has_value()) return;
      rs_id = 0, slb_id = tmp.value();
      issue_to_slb.Set(issue_to_slb_t{inst, inst.rs1, inst.rs2, rob_id, inst.imm, 3});
    } else {
      tmp = rs_prev.Allocate();
      if (!tmp.has_value()) return;
      rs_id = tmp.value(), slb_id = 0;
      issue_to_rs.Set(issue_to_rs_t{inst, inst.rs1, inst.rs2, rob_id, inst.imm, it.pc, rs_id});
    }
    if (inst.NeedWriteReg()) {
      issue_to_reg.Set(issue_to_reg_t{.reg_id = inst.rd, .rob = rob_id});
    }
    issue_to_rob.Set(ReorderBufferItem{false, inst, inst.rd, rs_id, slb_id, it.predict_jump});

#ifdef LTC
    printf("issue rob_id:%d rs:%d slb:%d ", rob_id, rs_id, slb_id);
    inst.Print();
#endif  // LTC
    inst_queue_next.pop();
  }
  void UpdateValueInRS(u32 rob_id, u32 val) {
    for (u32 i = 0; i < kReservationStationSize; ++i) {
      if (rs_prev[i].ready) continue;
      if (rs_prev[i].rs1_rob == rob_id) rs_next[i].rs1_rob = 0, rs_next[i].rs1_val = val;
      if (rs_prev[i].rs2_rob == rob_id) rs_next[i].rs2_rob = 0, rs_next[i].rs2_val = val;
    }
  }
  // Returns <value, rob_id> pair
  std::pair<u32, u32> GetRegStatusInRS(u32 reg_id) {
    if (reg_id == 0) return {0, 0};
    auto rob_id = reg_prev[reg_id].rob;
    if (rob_id == 0) {
      return {reg_prev[reg_id].val, 0};
    } else if (ex_to_rs && ex_to_rs->rob_id == rob_id) {
      return {ex_to_rs->val, 0};
    } else if (slb_to_rs_prev && slb_to_rs_prev->rob_id == rob_id) {
      return {slb_to_rs_prev->val, 0};
    } else if (rob_prev[rob_id].ready) {
      return {rob_prev[rob_id].result.value, 0};
    } else {
      return {0, rob_id};
    }
  }
  void RunReservationStation() {
    /*
    在这一部分你需要完成的工作：
    1. 设计一个 Reservation Station，其中要存储的东西可以参考 CAAQA 或其余资料，至少需要有用到的寄存器信息等
    2. 如存在，从 issue 阶段收到要存储的信息，存进 Reservation Station（如可以计算也可以直接进入计算）
    3. 从 Reservation Station 或者 issue 进来的指令中选择一条可计算的发送给 EX 进行计算
    4. 根据上个周期 EX 阶段或者 SLBUFFER 的计算得到的结果遍历 Reservation Station，更新相应的值
    */
    rs_to_ex.Reset();
    if (signal.clear_all) {
      for (u32 i = 0; i < kReservationStationSize; ++i) rs_next[i].ready = true;
      return;
    }
    if (issue_to_rs) {
      const auto &it = issue_to_rs.val;
      auto &rs_it = rs_next[it.rs_id];
      rs_it = ReservationStationItem{.ready = false, .inst = it.inst, .rd_rob = it.rd_rob, .imm = it.imm, .pc = it.pc};
      std::tie(rs_it.rs1_val, rs_it.rs1_rob) = GetRegStatusInRS(it.rs1);
      std::tie(rs_it.rs2_val, rs_it.rs2_rob) = GetRegStatusInRS(it.rs2);
#ifdef LTC
      printf("rs from issue ");
      it.inst.Print();
      printf("              pc:%d  rd_rob:%d  rs1:%d,%d  rs2:%d,%d rs_id:%d\n", rs_it.pc, rs_it.rd_rob, rs_it.rs1_val,
          rs_it.rs1_rob, rs_it.rs2_val, rs_it.rs2_rob, it.rs_id);
#endif  // LTC
    }
    if (ex_to_rs) {
      UpdateValueInRS(ex_to_rs->rob_id, ex_to_rs->val);
    }
    if (slb_to_rs_prev) {
      UpdateValueInRS(slb_to_rs_prev->rob_id, slb_to_rs_prev->val);
    }

    for (u32 i = 0; i < kReservationStationSize; ++i) {
      const auto &it = rs_prev[i];
      if (it.ready) continue;
      if (it.rs1_rob == 0 && it.rs2_rob == 0) {
        rs_to_ex.Set(rs_to_ex_t{it.inst, it.rs1_val, it.rs2_val, it.imm, it.rd_rob, it.pc});
        rs_next[i].ready = true;  // !modify to next
#ifdef LTC
        printf("rs send to ex rs_id:%d ", i);
        it.inst.Print();
#endif  // LTC
        break;
      }
    }
  }

  void RunRegfile() {
    /*
    每个寄存器会记录 Q 和 V，含义参考 ppt。
    这一部分会进行写寄存器，内容包括：根据 issue 和 commit 的通知修改对应寄存器的 Q 和 V。
    tip: 请注意 issue 和 commit 同一个寄存器时的情况: 先 commit 后 issue
    */
    if (commit_to_reg && commit_to_reg->reg_id) {
      auto &reg = reg_next[commit_to_reg->reg_id];
      reg.val = commit_to_reg->value;
      if (reg.rob == commit_to_reg->rob_id) reg.rob = 0;
    }
    if (signal.clear_all) {  // !after commit, before issue
      for (u32 i = 0; i < kRegisterSize; ++i) reg_next[i].rob = 0;
      return;
    }
    if (issue_to_reg) {
      reg_next[issue_to_reg->reg_id].rob = issue_to_reg->rob;
    }
  }

  void Execute() {
    /*
    在这一部分你需要完成的工作：
    根据 Reservation Station 发出的信息进行相应的计算
    tips: 考虑如何处理跳转指令并存储相关信息
          Store/Load 的指令并不在这一部分进行处理
    */
    ex_to_rs.Reset(), ex_to_slb.Reset(), ex_to_rob.Reset();
    if (rs_to_ex) {
      auto result = ExecuteInst(rs_to_ex->inst, rs_to_ex->v1, rs_to_ex->v2, rs_to_ex->imm, rs_to_ex->pc);
#ifdef LTC
      printf("execute rob:%d pc:%d ", rs_to_ex->rob_id, rs_to_ex->pc);
      rs_to_ex->inst.Print();
      printf("        v1:%d v2:%d imm:%d pc:%d\n", rs_to_ex->v1, rs_to_ex->v2, rs_to_ex->imm, rs_to_ex->pc);
      printf("        result val:%d jump?:%d pc:%d\n", result.value, result.jump, result.pc);
#endif  // LTC
      ex_to_rob.Set(ex_to_rob_t{.rob_id = rs_to_ex->rob_id, .result = result});
      ex_to_rs.Set(ex_to_rs_t{.rob_id = rs_to_ex->rob_id, .val = result.value});
      ex_to_slb.Set(ex_to_slb_t{.rob_id = rs_to_ex->rob_id, .val = result.value});
    }
  }

  void UpdateValueInSLB(u32 rob_id, u32 val) {
    for (u32 i = slb_prev.head; i != slb_prev.tail; ++i) {
      if (slb_prev[i].rs1_rob == rob_id) {
#ifdef LTC
        printf("UpdateValueInSLB rod_id:%d val:%d\n", rob_id, val);
#endif  // LTC
        slb_next[i].rs1_rob = 0, slb_next[i].rs1_val = val;
      }
      if (slb_prev[i].rs2_rob == rob_id) {
#ifdef LTC
        printf("UpdateValueInSLB rod_id:%d val:%d\n", rob_id, val);
#endif  // LTC
        slb_next[i].rs2_rob = 0, slb_next[i].rs2_val = val;
      }
    }
  }
  void RunStoreLoadBuffer() {
    /*
    在这一部分中，由于 SLBUFFER 的设计较为多样，在这里给出两种助教的设计方案：
    1. 1）通过循环队列，设计一个先进先出的 SLBUFFER，同时存储 head1、head2、tail 三个变量。
       其中，head1 是真正的队首，记录第一条未执行的内存操作的指令；
       tail 是队尾，记录当前最后一条未执行的内存操作的指令。
       而 head2 负责确定处在 head1 位置的指令是否可以进行内存操作，其具体实现为在 ROB 中增加一个 head_ensure
       的变量，每个周期 head_ensure 做取模意义下的加法，直到等于 tail 或遇到第一条跳转指令，这个时候我们就可以保证位于
       head_ensure 及之前位置的指令，因中间没有跳转指令，一定会执行。
       因而，只要当 head_ensure 当前位置的指令是 Store、Load 指令，我们就可以向 slbuffer 发信息，增加 head2。
       简单概括即对 head2 之前的 Store/Load 指令，我们根据判断出 ROB 中该条指令之前没有 jump 指令尚未执行，
       从而确定该条指令会被执行。
       2）同时 SLBUFFER 还需根据上个周期 EX 和 SLBUFFER 的计算结果遍历 SLBUFFER 进行数据的更新。
       3）此外，在我们的设计中，将 SLBUFFER 进行内存访问时计算需要访问的地址和对应的读取/存储内存的操作在 SLBUFFER 中
       一并实现，也可以考虑分成两个模块，该部分的实现只需判断队首的指令是否能执行并根据指令相应执行即可。

    2. 1）SLB 每个周期会查看队头，若队头指令还未 ready，则阻塞。
       2）当队头 ready 且是 load 指令时，SLB 会直接执行 load 指令，包括计算地址和读内存，
       然后把结果通知 ROB，同时将队头弹出。ROB commit 到这条指令时通知 Regfile 写寄存器。
       3）当队头 ready 且是 store 指令时，SLB 会等待 ROB 的 commit，commit 之后 SLB 会执行这
       条 store 指令，包括计算地址和写内存，写完后将队头弹出。
    */
    if (issue_to_slb) {
      const auto &it = issue_to_slb.val;
      auto slb_it = StoreLoadBufferItem{.inst = it.inst, .rd_rob = it.rd_rob, .imm = it.imm, .committed = false};
      std::tie(slb_it.rs1_val, slb_it.rs1_rob) = GetRegStatusInRS(it.rs1);
      std::tie(slb_it.rs2_val, slb_it.rs2_rob) = GetRegStatusInRS(it.rs2);
      slb_next.push(slb_it);
#ifdef LTC
      printf("slb from issue ");
      it.inst.Print();
      printf("               rd_rob:%d  rs1:%d,%d  rs2:%d,%d\n", slb_it.rd_rob, slb_it.rs1_val, slb_it.rs1_rob,
          slb_it.rs2_val, slb_it.rs2_rob);
#endif  // LTC
    }
    if (ex_to_slb) {
      UpdateValueInSLB(ex_to_slb->rob_id, ex_to_slb->val);
    }
    if (slb_to_slb_prev) {
      UpdateValueInSLB(slb_to_slb_prev->rob_id, slb_to_slb_prev->val);
    }
    if (commit_to_slb) {
      slb_next[commit_to_slb->slb_id].committed = true;
    }

    // execute load/store
    slb_to_rob_next.Reset(), slb_to_rs_next.Reset(), slb_to_slb_next.Reset();
    if (!slb_prev.empty()) {
      const auto &it = slb_prev.front();
      if (it.rs1_rob == 0 && it.rs2_rob == 0) {
        if (it.inst.IsStore() && !it.committed) {  // uncommitted store
          if (!it.busy) {
            slb_to_rob_next.Set(slb_to_rob_t{.rob_id = it.rd_rob, .is_load = false});
            slb_next.front().busy = true;
          }
        } else if (it.time > 1) {
          --slb_next.front().time;
        } else {
          auto result = ExecuteInst(it.inst, it.rs1_val, it.rs2_val, it.imm, 0);
#ifdef LTC
          printf("SLB, execute! ");
          it.inst.Print();
          printf("              rs1_v:%d rs2_v:%d imm:%d\n", it.rs1_val, it.rs2_val, it.imm);
#endif  // LTC
          slb_next.pop();
          if (it.inst.IsLoad()) {
            slb_to_rob_next.Set(slb_to_rob_t{it.rd_rob, true, result});
            slb_to_rs_next.Set(ex_to_rs_t{it.rd_rob, result.value});
            slb_to_slb_next.Set(ex_to_slb_t{it.rd_rob, result.value});
          }
        }
      }
    }
    if (signal.clear_all) {  // !after execute load/store to ensure unfinished committed store is not affected
      slb_to_rob_next.Reset(), slb_to_rs_next.Reset(), slb_to_slb_next.Reset();
      const auto &it = slb_prev.front();
      if (it.inst.IsStore() && it.committed && it.time > 1) {  // unfinished committed store
        slb_next.tail = slb_next.head + 1;
      } else {
        slb_next.clear();
      }
    }
  }

  void RunReorderBuffer() {
    /*
    在这一部分你需要完成的工作：
    1. 实现一个先进先出的 ROB，存储所有指令
    1. 根据 issue 阶段发射的指令信息分配空间进行存储。
    2. 根据 EX 阶段和 SLBUFFER 的计算得到的结果，遍历 ROB，更新 ROB 中的值
    3. 对于队首的指令，如果已经完成计算及更新，进行 commit
    */
    rob_to_commit.Reset();
    if (signal.clear_all) {
      rob_next.clear();
      return;
    }
    if (issue_to_rob) {
      rob_next.push(issue_to_rob.val);
    }
    if (ex_to_rob) {
      rob_next[ex_to_rob->rob_id].result = ex_to_rob->result;
      rob_next[ex_to_rob->rob_id].ready = true;
    }
    if (slb_to_rob_prev) {
      if (slb_to_rob_prev->is_load) rob_next[slb_to_rob_prev->rob_id].result = slb_to_rob_prev->result;
      rob_next[slb_to_rob_prev->rob_id].ready = true;
    }

    if (!rob_prev.empty() && rob_prev.front().ready) {
      const auto &it = rob_prev.front();
      rob_to_commit.Set(rob_to_commit_t{it.inst, it.reg_id, rob_prev.head, it.slb_id, it.result});
      rob_next.pop();
    }
  }

  void Commit() {
    /*
    在这一部分你需要完成的工作：
    1. 根据 ROB 发出的信息更新寄存器的值，包括对应的 ROB 和是否被占用状态（注意考虑 issue 和 commit 同一个寄存器的情况）
    2. 遇到跳转指令更新 pc 值，并发出信号清空所有部分的信息存储（这条对于很多部分都有影响，需要慎重考虑）
    */
    commit_to_reg.Reset(), commit_to_slb.Reset();
    signal.clear_all = 0;
    if (rob_to_commit) {
      if (rob_to_commit->inst.NeedWriteReg()) {
        commit_to_reg.Set(commit_to_reg_t{rob_to_commit->reg_id, rob_to_commit->rob_id, rob_to_commit->result.value});
#ifdef LTC_REG
        for (u32 i = 0; i < kRegisterSize; ++i) {
          printf("%d ", reg_prev[i].val);
        }
        puts("");
#endif  // LTC_REG
      }
      if (rob_to_commit->inst.IsBranch()) {
        if (rob_prev[rob_to_commit->rob_id].predict_jump != rob_to_commit->result.jump) {
          signal.clear_all = 1;
          pc = rob_to_commit->result.pc;
          predictor.CountResult(false);
        } else {
          predictor.CountResult(true);
        }
      } else if (rob_to_commit->inst.inst_type == Instruction::JALR) {
        signal.clear_all = 1;
        pc = rob_to_commit->result.pc;
      }
      if (rob_to_commit->inst.IsStore()) {
        commit_to_slb.Set(commit_to_slb_t{rob_to_commit->slb_id});
      }
    }
  }

  void Update() {
    /*
    在这一部分你需要完成的工作：
    对于模拟中未完成同步的变量（即同时需记下两个周期的新/旧信息的变量），进行数据更新。
    */
    inst_queue_prev = inst_queue_next;
    rs_prev = rs_next;
    slb_prev = slb_next;
    rob_prev = rob_next;
    reg_prev = reg_next;
    slb_to_rs_prev = slb_to_rs_next;
    slb_to_slb_prev = slb_to_slb_next;
    slb_to_rob_prev = slb_to_rob_next;
  }
};
