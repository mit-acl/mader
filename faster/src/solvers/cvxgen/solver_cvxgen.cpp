#include "solver_cvxgen.hpp"

SolverCvxgen::SolverCvxgen()
{
  initialize_optimizer();
}

bool SolverCvxgen::solveOptimization(const Eigen::Matrix<double, 3, 3>& Minterv,
                                     const Eigen::Matrix<double, 3, 3>& MintervP1,
                                     const Eigen::Matrix<double, 3, 3>& MintervP2, const Eigen::Vector3d& viM2,
                                     const Eigen::Vector3d& viM1, const double& v_max)
{
  //
  MAcol0_[0] = Minterv(0, 0);
  MAcol0_[1] = Minterv(1, 0);
  MAcol0_[2] = Minterv(2, 0);

  //
  MAcol1_[0] = Minterv(0, 1);
  MAcol1_[1] = Minterv(1, 1);
  MAcol1_[2] = Minterv(2, 1);

  //
  MAcol2_[0] = Minterv(0, 2);
  MAcol2_[1] = Minterv(1, 2);
  MAcol2_[2] = Minterv(2, 2);

  //
  MBcol0_[0] = MintervP1(0, 0);
  MBcol0_[1] = MintervP1(1, 0);
  MBcol0_[2] = MintervP1(2, 0);

  //
  MBcol1_[0] = MintervP1(0, 1);
  MBcol1_[1] = MintervP1(1, 1);
  MBcol1_[2] = MintervP1(2, 1);

  //
  MBcol2_[0] = MintervP1(0, 2);
  MBcol2_[1] = MintervP1(1, 2);
  MBcol2_[2] = MintervP1(2, 2);

  //
  MCcol0_[0] = MintervP2(0, 0);
  MCcol0_[1] = MintervP2(1, 0);
  MCcol0_[2] = MintervP2(2, 0);

  //
  MCcol1_[0] = MintervP2(0, 1);
  MCcol1_[1] = MintervP2(1, 1);
  MCcol1_[2] = MintervP2(2, 1);

  //
  MCcol2_[0] = MintervP2(0, 2);
  MCcol2_[1] = MintervP2(1, 2);
  MCcol2_[2] = MintervP2(2, 2);

  //
  viM2_[0] = viM2(0);
  viM2_[1] = viM2(1);
  viM2_[2] = viM2(2);

  //
  viM1_[0] = viM1(0);
  viM1_[1] = viM1(1);
  viM1_[2] = viM1(2);

  load_default_data(MAcol0_, MAcol1_, MAcol2_, MBcol0_, MBcol1_, MBcol2_, MCcol0_, MCcol1_, MCcol2_, viM2_, viM1_,
                    v_max);

  // tic();
  bool converged_bound1 = optimize();
  // double time = tocq();
  double* bound1 = get_vel();
  bound1_ << bound1[0], bound1[1], bound1[2];

  /////////////////////////////////

  MAcol0_[2] = -MAcol0_[2];
  MAcol1_[2] = -MAcol1_[2];
  MAcol2_[2] = -MAcol2_[2];

  MAcol0_[2] = -MAcol0_[2];
  MAcol1_[2] = -MAcol1_[2];
  MAcol2_[2] = -MAcol2_[2];

  MAcol0_[2] = -MAcol0_[2];
  MAcol1_[2] = -MAcol1_[2];
  MAcol2_[2] = -MAcol2_[2];

  MBcol0_[1] = -MBcol0_[1];
  MBcol1_[1] = -MBcol1_[1];
  MBcol2_[1] = -MBcol2_[1];

  MBcol0_[1] = -MBcol0_[1];
  MBcol1_[1] = -MBcol1_[1];
  MBcol2_[1] = -MBcol2_[1];

  MBcol0_[1] = -MBcol0_[1];
  MBcol1_[1] = -MBcol1_[1];
  MBcol2_[1] = -MBcol2_[1];

  MCcol0_[0] = -MCcol0_[0];
  MCcol1_[0] = -MCcol1_[0];
  MCcol2_[0] = -MCcol2_[0];

  MCcol0_[0] = -MCcol0_[0];
  MCcol1_[0] = -MCcol1_[0];
  MCcol2_[0] = -MCcol2_[0];

  MCcol0_[0] = -MCcol0_[0];
  MCcol1_[0] = -MCcol1_[0];
  MCcol2_[0] = -MCcol2_[0];

  load_default_data(MAcol0_, MAcol1_, MAcol2_, MBcol0_, MBcol1_, MBcol2_, MCcol0_, MCcol1_, MCcol2_, viM2_, viM1_,
                    v_max);

  bool converged_bound2 = optimize();
  double* bound2 = get_vel();
  bound2_ << -bound2[0], -bound2[1], -bound2[2];

  ///////////////////////////////
  // std::cout << "converged_bound1 = " << converged_bound1 << std::endl;
  // std::cout << "converged_bound2 = " << converged_bound2 << std::endl;

  // printf("Time taken: %.3g us.\n", 1e6 * time);

  return (converged_bound1 && converged_bound2);
}

void SolverCvxgen::getSolution(Eigen::Vector3d& bound1, Eigen::Vector3d& bound2)
{
  bound1 = bound1_;
  bound2 = bound2_;
}

void SolverCvxgen::applySolutionTo(double& constraint_xL, double& constraint_xU, double& constraint_yL,
                                   double& constraint_yU, double& constraint_zL, double& constraint_zU)
{
  Eigen::Vector3d lower_bound;
  lower_bound << std::min(bound1_.x(), bound2_.x()),  /////////// TODO: use Eigen ElementWise max
      std::min(bound1_.y(), bound2_.y()),             ////////
      std::min(bound1_.z(), bound2_.z());

  Eigen::Vector3d upper_bound;
  upper_bound << std::max(bound1_.x(), bound2_.x()),  ///////////
      std::max(bound1_.y(), bound2_.y()),             ////////
      std::max(bound1_.z(), bound2_.z());

  constraint_xL = std::max(constraint_xL, lower_bound.x());
  constraint_xU = std::min(constraint_xU, upper_bound.x());

  constraint_yL = std::max(constraint_yL, lower_bound.y());
  constraint_yU = std::min(constraint_yU, upper_bound.y());

  constraint_zL = std::max(constraint_zL, lower_bound.z());
  constraint_zU = std::min(constraint_zU, upper_bound.z());
}