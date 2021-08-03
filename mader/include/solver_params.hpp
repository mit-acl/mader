#ifndef SOLVER_PARAMS_HPP
#define SOLVER_PARAMS_HPP

#include <Eigen/Dense>

namespace ms  // mader solver
{
struct par_solver
{
  ///// Will not change between iterations
  double x_min = -std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::max();

  double y_min = -std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::max();

  double z_min = -std::numeric_limits<double>::max();
  double z_max = std::numeric_limits<double>::max();
  Eigen::Vector3d v_max;
  Eigen::Vector3d a_max;
  Eigen::Vector3d j_max;
  double dc;
  double dist_to_use_straight_guess;
  int a_star_samp_x;
  int a_star_samp_y;
  int a_star_samp_z;
  double a_star_fraction_voxel_size;
  int num_pol;
  int deg_pol;
  double weight;
  double epsilon_tol_constraints;
  double xtol_rel;
  double ftol_rel;
  std::string solver;
  std::string basis;
  double a_star_bias;
  bool allow_infeasible_guess;
  double Ra;

  double alpha_shrink;
};
}  // namespace ms

#endif