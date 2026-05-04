#include <casadi/casadi.hpp>
#include <iostream>
using namespace casadi;

int main() {
  SX x = SX::sym("x");
  SX fexpr = x*x;
  Function f("f", {x}, {fexpr});

  DM res = f(DM(2.0))[0];            // call with DM, get DM
  double v = double(res);            // convert to double
  std::cout << "res = " << double(res) << "\n";
  return 0;
}