#include "solver_cvxgen.hpp"
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
  Eigen::Matrix<double, 3, 3> Minterv = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> MintervP1 = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> MintervP2 = Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Vector3d viM1(0.0, 0.0, 0.0);
  Eigen::Vector3d viM2(0.0, 0.0, 0.0);
  double v_max = 7.0;

  SolverCvxgen my_solver;
  bool converged = my_solver.solveOptimization(Minterv, MintervP1, MintervP2, viM1, viM2, v_max);

  Eigen::Vector3d bound1;
  Eigen::Vector3d bound2;

  my_solver.getSolution(bound1, bound2);

  std::cout << "bound1= " << bound1.transpose() << std::endl;
  std::cout << "bound2= " << bound2.transpose() << std::endl;

  return 0;
}