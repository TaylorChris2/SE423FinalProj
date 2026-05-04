#include "dynamics.hpp"
using namespace casadi;

std::pair<MX, MX> EOM_kin_ca(const MX& X, double dt, double r) {
    MX theta = X(2);
    MX A = MX::eye(3);
    // build B columns as vectors then horzcat via vertcat+transpose or use hcat via MX::horzcat
    MX col0 = vertcat(MX(0.5*dt*cos(theta)), MX(0.5*dt*sin(theta)), MX(-0.5*dt/r));
    MX col1 = vertcat(MX(0.5*dt*cos(theta)), MX(0.5*dt*sin(theta)), MX(0.5*dt/r));
    MX B = horzcat(col0, col1); // use horzcat available in casadi::MX
    return {A, B};
}

std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> EOM_kin_np(const std::vector<double>& X, double dt, double r){
    double theta = X[2];
    std::vector<std::vector<double>> A(3, std::vector<double>(3,0.0));
    for(int i=0;i<3;i++) A[i][i]=1.0;
    std::vector<std::vector<double>> B = {
        {0.5*dt*std::cos(theta), 0.5*dt*std::cos(theta)},
        {0.5*dt*std::sin(theta), 0.5*dt*std::sin(theta)},
        {-0.5*dt/r,              0.5*dt/r}
    };
    return {A,B};
}
