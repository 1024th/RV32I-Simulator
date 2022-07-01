#include "tomasulo.hpp"

int main() {
  // freopen("testcases/array_test2.data", "r", stdin);
  // freopen("tmp.out", "w", stdout);
  Simulator simulator;
  simulator.InitializeMemory(std::cin);
  simulator.Run();
  return 0;
}