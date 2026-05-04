#pragma once
#include <casadi/casadi.hpp>
using namespace casadi;

std::pair<MX, MX> EOM_kin_ca(const MX& X, double dt, double r);
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> EOM_kin_np(const std::vector<double>& X, double dt, double r);
