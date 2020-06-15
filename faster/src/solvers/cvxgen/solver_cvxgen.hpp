#ifndef SOLVER_CVXGEN_HPP
#define SOLVER_CVXGEN_HPP
#pragma once

#include "cvxgen/solver.h"
#include <iostream>
#include <Eigen/Dense>

class SolverCvxgen
{
public:
  SolverCvxgen();
  bool solveOptimization(const Eigen::Matrix<double, 3, 3>& Minterv, const Eigen::Matrix<double, 3, 3>& MintervP1,
                         const Eigen::Matrix<double, 3, 3>& MintervP2, const Eigen::Vector3d& viM2,
                         const Eigen::Vector3d& viM1, const double& v_max);

  void getSolution(Eigen::Vector3d& bound1_, Eigen::Vector3d& bound2_);

  void applySolutionTo(double& constraint_xL, double& constraint_xU, double& constraint_yL, double& constraint_yU,
                       double& constraint_zL, double& constraint_zU);

private:
  double MAcol0_[3];
  double MAcol1_[3];
  double MAcol2_[3];
  double MBcol0_[3];
  double MBcol1_[3];
  double MBcol2_[3];
  double MCcol0_[3];
  double MCcol1_[3];
  double MCcol2_[3];
  double viM1_[3];
  double viM2_[3];
  Eigen::Vector3d bound1_;
  Eigen::Vector3d bound2_;
};

#endif