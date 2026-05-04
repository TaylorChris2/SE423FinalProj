#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "dynamics.hpp"
using namespace casadi;

int main(){
  // Params
  const int NX = 3;
  const int NU = 2;
  const int N = 30;
  double dt = 0.5;
  double r = 0.05;
  double v_l_max = 0.1, v_r_max = 0.1;
  int obst_num = 4;
  double safe_radius = 0.1;
  double weight = 50.0;

  Opti opti;

  MX X = opti.variable(NX, N+1);
  MX U = opti.variable(NU, N);
  MX x0 = opti.parameter(NX);
  MX xd = opti.parameter(NX, N+1);
  MX obst_pos = opti.parameter(2, obst_num);

  // dynamics constraints
  for(int k=0;k<N;k++){
    MX Xk = X(Slice(), k);
    auto [A,B] = EOM_kin_ca(Xk, dt, r);
    MX x_next = mtimes(A, Xk) + mtimes(B, U(Slice(), k));
    opti.subject_to(X(Slice(), k+1) == x_next);
  }
  opti.subject_to(X(Slice(),0) == x0);
  opti.subject_to(opti.bounded(-v_l_max, U(Slice(0,1), Slice()), v_l_max));
  opti.subject_to(opti.bounded(-v_r_max, U(Slice(1,2), Slice()), v_r_max));

  // cost
  DM Q = DM::diag(DM{1000.0,1000.0,10.0});
  DM R = DM::diag(DM{5.0,5.0});
  MX cost = MX::zeros(1,1);
  for(int k=0;k<N;k++){
    MX err = X(Slice(),k) - xd(Slice(),k);
    cost += mtimes(mtimes(err.T(), Q), err);
    MX uk = U(Slice(),k);
    cost += mtimes(mtimes(uk.T(), R), uk);
  }

  // terminal cost: placeholder P = Q (replace with DARE result if available)
  MX errT = X(Slice(),N) - xd(Slice(),N);
  MX P = DM::diag(DM{1000.0,1000.0,10.0});
//   cost += mtimes(mtimes(errT.T(), P), errT);

  // obstacle cost (vectorized)
  MX obst_cost = MX::zeros(1,1);
  for(int k=0;k<=N;k++){
    MX pos_k = X(Slice(0,2), k);            // 2x1
    MX pos_k_rep = repmat(pos_k, 1, obst_num); // 2 x obst_num
    MX diff = obst_pos - pos_k_rep;         // 2 x obst_num
    MX dist_sq = sum1(diff*diff);           // 1 x obst_num
    MX dist = sqrt(dist_sq);
    MX margin = fmax(dist - safe_radius, 1e-4);
    obst_cost += sum2(exp(1.0/(weight*margin)) - 1);
  }
  cost += obst_cost;

  opti.minimize(cost);

  Dict opts;
  opts["ipopt.max_iter"] = 1e3;
  opts["ipopt.tol"] = 1e-4;
  opts["ipopt.print_level"] = 5;
//   std::cout >> print_options() >> std::endl;
  opts["ipopt.linear_solver"] = "mumps";
  opti.solver("ipopt", opts);

  // create reference xd (constant)
  std::vector<double> xd_val = {1.0, 0.5, M_PI/4.0};
  DM xd_mat = DM::zeros(NX, N+1);
  for(int k=0;k<=N;k++){
    xd_mat(0,k) = xd_val[0];
    xd_mat(1,k) = xd_val[1];
    xd_mat(2,k) = xd_val[2];
  }

  // initial state
  std::vector<double> x_next = {0.0,0.0,0.0};

  // obstacles
  DM obst_mat = DM::zeros(2, obst_num);
  for(int i=0;i<obst_num;i++){ obst_mat(0,i)=0.0; obst_mat(1,i)=0.5; }
  std::vector<double> obsx = {0.4,0.6,0.82,1.15};
  std::vector<double> obsy = {0.1,0.4,0.29,0.4};
  for(int i=0;i<4;i++){ obst_mat(0,i)=obsx[i]; obst_mat(1,i)=obsy[i]; }

  bool first_iter = true;
  int iter_count = 1;
  std::vector<std::vector<double>> X_opt; // placeholder
  std::vector<double> U0(NU,0.0);

  while (sqrt((x_next[0]-xd_val[0])*(x_next[0]-xd_val[0]) + (x_next[1]-xd_val[1])*(x_next[1]-xd_val[1])) > 3e-2 && iter_count <= 100){
    if(first_iter){
      opti.set_value(x0, DM{0,0,0});
      opti.set_value(xd, xd_mat);
      first_iter = false;
    } else {
      opti.set_value(x0, DM{x_next[0], x_next[1], x_next[2]});
      opti.set_value(xd, xd_mat);
    }
    opti.set_value(obst_pos, obst_mat);

    auto sol = opti.solve();

    DM Xsol = sol.value(X);
    DM Usol = sol.value(U);

    // extract first control
    U0[0] = double(Usol(0,0));
    U0[1] = double(Usol(1,0));

    // RK4 numpy equivalent
    auto rk4_np = [&](const std::vector<double>& x, const std::vector<double>& u){
      auto f = [&](const std::vector<double>& xx, const std::vector<double>& uu){
        return std::vector<double>{ (uu[0]+uu[1])/2.0*std::cos(xx[2]),
                                    (uu[0]+uu[1])/2.0*std::sin(xx[2]),
                                    (uu[1]-uu[0])/(2.0*r) };
      };
      std::vector<double> k1 = f(x,u);
      std::vector<double> x2(3), x3(3), x4(3);
      for(int i=0;i<3;i++) x2[i] = x[i] + dt/2.0*k1[i];
      std::vector<double> k2 = f(x2,u);
      for(int i=0;i<3;i++) x3[i] = x[i] + dt/2.0*k2[i];
      std::vector<double> k3 = f(x3,u);
      for(int i=0;i<3;i++) x4[i] = x[i] + dt*k3[i];
      std::vector<double> k4 = f(x4,u);
      std::vector<double> xn(3);
      for(int i=0;i<3;i++) xn[i] = x[i] + (dt/6.0)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
      return xn;
    };

    x_next = rk4_np(x_next, U0);

    std::cout << "iter " << iter_count << " x_next: " << x_next[0] << " " << x_next[1] << " " << x_next[2] << "\n";

    iter_count++;
  }

  return 0;
}