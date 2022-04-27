/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */
#include <vector>
#include <random>
#include "ros/ros.h"

#include "timer.hpp"
#include "termcolor.hpp"
#include "octopus_search.hpp"
#include "bspline_utils.hpp"
#include "utils.hpp"
using namespace termcolor;

typedef MADER_timers::Timer MyTimer;

// template <typename T>
// int mu::sgn(T val)
// {
//   return (T(0) < val) - (val < T(0));
// }

OctopusSearch::OctopusSearch(std::string basis, int num_pol, int deg_pol, double alpha_shrink)
{
  p_ = deg_pol;
  M_ = num_pol + 2 * p_;
  N_ = M_ - p_ - 1;
  num_pol_ = num_pol;

  mt::basisConverter basis_converter;

  if (basis == "MINVO")
  {
    // std::cout << green << bold << "A* is using MINVO" << reset << std::endl;
    M_pos_bs2basis_ = basis_converter.getMinvoPosConverters(num_pol);
    M_vel_bs2basis_ = basis_converter.getBSplineVelConverters(num_pol);  // getMinvoVelConverters TODO!!
    basis_ = MINVO;
  }
  else if (basis == "BEZIER")
  {
    // std::cout << green << bold << "A* is using BEZIER" << reset << std::endl;
    M_pos_bs2basis_ = basis_converter.getBezierPosConverters(num_pol);
    M_vel_bs2basis_ = basis_converter.getBSplineVelConverters(num_pol);  // getBezierVelConverters TODO!!
    basis_ = BEZIER;
  }
  else if (basis == "B_SPLINE")
  {
    // std::cout << green << bold << "A* is using B_SPLINE" << reset << std::endl;

    M_pos_bs2basis_ = basis_converter.getBSplinePosConverters(num_pol);
    M_vel_bs2basis_ = basis_converter.getBSplineVelConverters(num_pol);

    basis_ = B_SPLINE;
  }
  else
  {
    std::cout << bold << red << "Basis not implemented yet" << std::endl;
    abort();
  }

  M_pos_bs2basis_inverse_.clear();
  for (auto matrix_i : M_pos_bs2basis_)
  {
    M_pos_bs2basis_inverse_.push_back(matrix_i.inverse());
  }

  separator_solver_ = new separator::Separator();  // 0.0, 0.0, 0.0

  alpha_shrink_ = alpha_shrink;

  time_solving_lps_ = 0.0;
  time_expanding_ = 0.0;

  // Mbs2basis_inverse_ = Mbs2basis_.inverse();
}

void OctopusSearch::setUp(double t_min, double t_max, const mt::ConvexHullsOfCurves_Std& hulls)
{
  num_of_segments_ = (M_ - 2 * p_);

  num_of_obst_ = hulls.size();
  num_of_normals_ = num_of_segments_ * num_of_obst_;

  hulls_ = hulls;

  double deltaT = (t_max - t_min) / (1.0 * (M_ - 2 * p_ - 1 + 1));

  Eigen::RowVectorXd knots(M_ + 1);
  for (int i = 0; i <= p_; i++)
  {
    knots[i] = t_min;
  }

  for (int i = (p_ + 1); i <= M_ - p_ - 1; i++)
  {
    knots[i] = knots[i - 1] + deltaT;  // Assumming a uniform b-spline (internal knots are equally spaced)
  }

  for (int i = (M_ - p_); i <= M_; i++)
  {
    knots[i] = t_max;
  }

  knots_ = knots;
  // std::cout << "knots_=" << knots_ << std::endl;
}

OctopusSearch::~OctopusSearch()
{
}

int OctopusSearch::getNumOfLPsRun()
{
  return separator_solver_->getNumOfLPsRun();
}

void OctopusSearch::setVisual(bool visual)
{
  visual_ = visual;
}

void OctopusSearch::getBestTrajFound(mt::trajectory& best_traj_found, mt::PieceWisePol& pwp, double dc)
{
  mt::trajectory traj;
  CPs2TrajAndPwp(result_, best_traj_found, pwp, N_, p_, num_pol_, knots_, dc);
}

void OctopusSearch::getEdgesConvexHulls(mt::Edges& edges_convex_hulls)
{
  Eigen::Matrix<double, 3, 4> last4Cps;
  Eigen::Matrix<double, 3, 4> last4Cps_new_basis;

  for (int i = 3; i < result_.size(); i++)
  {
    last4Cps.col(0) = result_[i - 3];
    last4Cps.col(1) = result_[i - 2];
    last4Cps.col(2) = result_[i - 1];
    last4Cps.col(3) = result_[i];

    int interval = i - 3;

    last4Cps_new_basis = transformBSpline2otherBasis(last4Cps, interval);

    for (int j = 0; j < 4; j++)
    {  // For every point in the convex hull
      mt::Edge edge;
      edge.first = last4Cps_new_basis.col(j);
      for (int i = 0; i < 4; i++)
      {  // generate an edge from that point j to the other points i!=j
        if (i == j)
        {
          continue;
        }
        else
        {
          edge.second = last4Cps_new_basis.col(i);
          edges_convex_hulls.push_back(edge);
        }
      }
    }
  }
}

void OctopusSearch::getAllTrajsFound(std::vector<mt::trajectory>& all_trajs_found)
{
  all_trajs_found.clear();

  for (auto node : expanded_valid_nodes_)
  {
    std::vector<Eigen::Vector3d> cps;

    Node* tmp = &node;

    while (tmp != NULL)
    {
      cps.push_back(Eigen::Vector3d(tmp->qi.x(), tmp->qi.y(), tmp->qi.z()));
      tmp = tmp->previous;
    }

    cps.push_back(q1_);
    cps.push_back(q0_);  // cps = [....q4 q3 q2 q1 q0}

    std::reverse(std::begin(cps), std::end(cps));  // cps=[q0 q1 q2 q3 q4 ...]

    mt::trajectory traj;
    mt::PieceWisePol pwp;
    CPs2TrajAndPwp(cps, traj, pwp, N_, p_, num_pol_, knots_, 0.01);  // Last number is the resolution

    all_trajs_found.push_back(traj);
  }
}

void OctopusSearch::setBBoxSearch(double x, double y, double z)
{
  bbox_x_ = x;
  bbox_y_ = y;
  bbox_z_ = z;
}

void OctopusSearch::setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int num_samples_x,
                                           int num_samples_y, int num_samples_z, double fraction_voxel_size)
{
  all_combinations_.clear();
  indexes_samples_x_.clear();
  indexes_samples_y_.clear();
  indexes_samples_z_.clear();

  v_max_ = v_max;
  a_max_ = a_max;

  // ensure they are odd numbers (so that vx=0 is included in the samples)
  num_samples_x_ = (num_samples_x % 2 == 0) ? ceil(num_samples_x) : num_samples_x;
  num_samples_y_ = (num_samples_y % 2 == 0) ? ceil(num_samples_y) : num_samples_y;
  num_samples_z_ = (num_samples_z % 2 == 0) ? ceil(num_samples_z) : num_samples_z;

  for (int i = 0; i < num_samples_x; i++)
  {
    indexes_samples_x_.push_back(i);
  }

  for (int i = 0; i < num_samples_y; i++)
  {
    indexes_samples_y_.push_back(i);
  }

  for (int i = 0; i < num_samples_z; i++)
  {
    indexes_samples_z_.push_back(i);
  }

  for (int jx : indexes_samples_x_)
  {
    for (int jy : indexes_samples_y_)
    {
      for (int jz : indexes_samples_z_)
      {
        // std::cout << "Pushing combination " << jx << ", " << jy << ", " << jz << std::endl;
        all_combinations_.push_back(std::tuple<int, int, int>(jx, jy, jz));
      }
    }
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  shuffle(all_combinations_.begin(), all_combinations_.end(), std::default_random_engine(seed));

  double min_voxel_size;
  double max_voxel_size;
  computeLimitsVoxelSize(min_voxel_size, max_voxel_size);

  // Ensure  fraction_voxel_size is in [0,1]
  fraction_voxel_size = (fraction_voxel_size > 1) ? 1 : fraction_voxel_size;
  fraction_voxel_size = (fraction_voxel_size < 0) ? 0 : fraction_voxel_size;

  voxel_size_ = min_voxel_size + fraction_voxel_size * (max_voxel_size - min_voxel_size);

  orig_ = q2_ - Eigen::Vector3d(bbox_x_ / 2.0, bbox_y_ / 2.0, bbox_z_ / 2.0);
}

void OctopusSearch::setXYZMinMaxAndRa(double x_min, double x_max, double y_min, double y_max, double z_min,
                                      double z_max, double Ra)
{
  x_min_ = x_min;
  x_max_ = x_max;

  y_min_ = y_min;
  y_max_ = y_max;

  z_min_ = z_min;
  z_max_ = z_max;
  Ra_ = Ra;
}

void OctopusSearch::setBias(double bias)
{
  bias_ = bias;
}

void OctopusSearch::setGoal(Eigen::Vector3d& goal)
{
  goal_ = goal;
}

void OctopusSearch::setRunTime(double max_runtime)
{
  max_runtime_ = max_runtime;
}
void OctopusSearch::setGoalSize(double goal_size)
{
  goal_size_ = goal_size;
}

// returns the minimum voxel_size needed (if bigger, all the neighbous from q2_ are inside the voxel of q2_)
// min_voxel_size: if voxel_size < min_voxel_size, all the neighbous from q2_ will be expanded correctly
// max_voxel_size: if voxel_size > max_voxel_size, there will be no nodes expanded from q2_ (they are on the same cell
// as q2_)

void OctopusSearch::computeLimitsVoxelSize(double& min_voxel_size, double& max_voxel_size)
{
  int i = 2;
  double constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU;

  computeUpperAndLowerConstraints(i, q0_, q1_, q2_, constraint_xL, constraint_xU, constraint_yL, constraint_yU,
                                  constraint_zL, constraint_zU);

  min_voxel_size = std::numeric_limits<double>::max();
  max_voxel_size = std::numeric_limits<double>::min();

  Eigen::Vector3d neighbor_of_q2;

  for (int jx : indexes_samples_x_)
  {
    for (int jy : indexes_samples_y_)
    {
      for (int jz : indexes_samples_z_)
      {
        // sample a velocity
        Eigen::Vector3d vi;
        vi << constraint_xL + jx * ((constraint_xU - constraint_xL) / (num_samples_x_ - 1)),  /////////
            constraint_yL + jy * ((constraint_yU - constraint_yL) / (num_samples_y_ - 1)),    /////////
            constraint_zL + jz * ((constraint_zU - constraint_zL) / (num_samples_z_ - 1));    /////////

        if (vi.norm() < 0.000001)  // remove the cases where vi is very small
        {
          continue;
        }

        //  std::cout << "vx= " << vi.x() << ", vy= " << vi.y() << ", vz= " << vi.z() << std::endl;
        //  std::cout << "(neighbor_of_q2 - q2_).norm()= " << (neighbor_of_q2 - q2_).norm() << std::endl;

        neighbor_of_q2 = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + q2_;

        min_voxel_size = std::min(min_voxel_size, (neighbor_of_q2 - q2_).norm());
        max_voxel_size = std::max(max_voxel_size, (neighbor_of_q2 - q2_).norm());
      }
    }
  }
}

// // compute constraints so that it satisfies interval i-1
// // axis=0 (x), 1(y) or 2(z)
// bool OctopusSearch::computeAxisForNextInterval(const int i, const Eigen::Vector3d& viM1, int axis, double& constraint_L,
//                                                double& constraint_U)
// {
//   Eigen::Matrix<double, 3, 3> M_interv_next = M_vel_bs2basis_[i - 1];
//   constraint_L = -std::numeric_limits<double>::max();
//   constraint_U = std::numeric_limits<double>::max();

//   for (int i = 0; i < 3; i++)
//   {
//     for (int j = 0; j < 3; j++)
//     {
//       double g1 = viM1(axis);
//       double tmp = -(M_interv_next(1, j) / M_interv_next(2, j)) + (M_interv_next(1, i) / M_interv_next(2, i));
//       double qli = (-mu::sgn(M_interv_next(2, i)) * v_max_.x() - g1 * M_interv_next(0, i)) / M_interv_next(2, i);
//       double quj = (mu::sgn(M_interv_next(2, j)) * v_max_.x() - g1 * M_interv_next(0, j)) / M_interv_next(2, j);
//       double bound = (qli - quj) / tmp;
//       if (tmp >= 0)
//       {
//         constraint_L = std::max(constraint_L, bound);
//       }
//       else
//       {
//         constraint_U = std::min(constraint_U, bound);
//       }
//     }
//   }
// }

// Compute the lower and upper bounds on the velocity based on the velocity and acceleration constraints
// return false if any of the intervals, the lower bound is > the upper bound
bool OctopusSearch::computeUpperAndLowerConstraints(const int i, const Eigen::Vector3d& qiM2,
                                                    const Eigen::Vector3d& qiM1, const Eigen::Vector3d& qi,
                                                    double& constraint_xL, double& constraint_xU, double& constraint_yL,
                                                    double& constraint_yU, double& constraint_zL, double& constraint_zU)
{
  // std::cout << "_______________________ " << std::endl;
  // std::cout << "i= " << i << std::endl;
  // std::cout << "qiM2= " << qiM2.transpose() << std::endl;
  // std::cout << "qiM1= " << qiM1.transpose() << std::endl;
  // std::cout << "qi= " << qi.transpose() << std::endl;

  int interv = i - 2;  // index of the interval (i is the index of the control point)

  Eigen::Vector3d viM2 =
      p_ * (qiM1 - qiM2) / (knots_(i - 2 + p_ + 1) - knots_(i - 2 + 1));  // velocity_{current.index -2}
  Eigen::Vector3d viM1 =
      p_ * (qi - qiM1) / (knots_(i - 1 + p_ + 1) - knots_(i - 1 + 1));  // velocity_{current.index -1}

  constraint_xL = -std::numeric_limits<double>::max();
  constraint_xU = std::numeric_limits<double>::max();
  constraint_yL = -std::numeric_limits<double>::max();
  constraint_yU = std::numeric_limits<double>::max();
  constraint_zL = -std::numeric_limits<double>::max();
  constraint_zU = std::numeric_limits<double>::max();

  ////////////////////////IMPOSE VELOCITY CONSTRAINTS

  Eigen::Matrix<double, 3, 3> M_interv = M_vel_bs2basis_[interv];

  // std::cout << "M_interv= \n" << M_interv << std::endl;

  double eps = 1e-5;
  auto isNotZero = [&](double a) { return (fabs(a) > eps); };

  constraint_xL = std::max(constraint_xL, -v_max_.x());  // lower bound
  constraint_xU = std::min(constraint_xU, v_max_.x());   // upper bound

  constraint_yL = std::max(constraint_yL, -v_max_.y());  // lower bound
  constraint_yU = std::min(constraint_yU, v_max_.y());   // upper bound

  constraint_zL = std::max(constraint_zL, -v_max_.z());  // lower bound
  constraint_zU = std::min(constraint_zU, v_max_.z());   // upper bound

  // // Now, if were are sampling vNm3, check that velocities are satisfied
  if (i == (N_ - 3))
  {
    // std::cout << "doing this check!!!!, N=" << N_ << std::endl;
    ////////////////  [viM1 vi 0]
    Eigen::Matrix<double, 3, 3> tmp2 = viM1 * M_interv.row(0);

    M_interv = M_vel_bs2basis_[interv + 1];

    // For x
    for (int j = 0; j < 3; j++)  // For the three velocity control points
    {
      if (isNotZero(M_interv(1, j)))  // If it's zero, no need to impose this constraint
      {
        double vi_bound1 = (v_max_.x() - tmp2(0, j)) / M_interv(1, j);
        double vi_bound2 = (-v_max_.x() - tmp2(0, j)) / M_interv(1, j);
        constraint_xL = std::max(constraint_xL, std::min(vi_bound1, vi_bound2));
        constraint_xU = std::min(constraint_xU, std::max(vi_bound1, vi_bound2));
      }
    }

    // For y
    for (int j = 0; j < 3; j++)  // For the three velocity control points
    {
      if (isNotZero(M_interv(1, j)))  // If it's zero, no need to impose this constraint
      {
        double vi_bound1 = (v_max_.y() - tmp2(1, j)) / M_interv(1, j);
        double vi_bound2 = (-v_max_.y() - tmp2(1, j)) / M_interv(1, j);
        constraint_yL = std::max(constraint_yL, std::min(vi_bound1, vi_bound2));
        constraint_yU = std::min(constraint_yU, std::max(vi_bound1, vi_bound2));
      }
    }

    // For z
    for (int j = 0; j < 3; j++)  // For the three velocity control points
    {
      if (isNotZero(M_interv(1, j)))  // If it's zero, no need to impose this constraint
      {
        double vi_bound1 = (v_max_.z() - tmp2(2, j)) / M_interv(1, j);
        double vi_bound2 = (-v_max_.z() - tmp2(2, j)) / M_interv(1, j);
        constraint_zL = std::max(constraint_zL, std::min(vi_bound1, vi_bound2));
        constraint_zU = std::min(constraint_zU, std::max(vi_bound1, vi_bound2));
      }
    }

    M_interv = M_vel_bs2basis_[interv + 2];

    ////////////////  [vi 0 0]
    // For x
    for (int j = 0; j < 3; j++)  // For the three velocity control points
    {
      if (isNotZero(M_interv(0, j)))  // If it's zero, no need to impose this constraint
      {
        double vi_bound1 = v_max_.x() / M_interv(0, j);
        double vi_bound2 = -v_max_.x() / M_interv(0, j);
        constraint_xL = std::max(constraint_xL, std::min(vi_bound1, vi_bound2));
        constraint_xU = std::min(constraint_xU, std::max(vi_bound1, vi_bound2));
      }
    }

    // For y
    for (int j = 0; j < 3; j++)  // For the three velocity control points
    {
      if (isNotZero(M_interv(0, j)))  // If it's zero, no need to impose this constraint
      {
        double vi_bound1 = v_max_.y() / M_interv(0, j);
        double vi_bound2 = -v_max_.y() / M_interv(0, j);
        constraint_yL = std::max(constraint_yL, std::min(vi_bound1, vi_bound2));
        constraint_yU = std::min(constraint_yU, std::max(vi_bound1, vi_bound2));
      }
    }

    // For z
    for (int j = 0; j < 3; j++)  // For the three velocity control points
    {
      if (isNotZero(M_interv(0, j)))  // If it's zero, no need to impose this constraint
      {
        double vi_bound1 = v_max_.z() / M_interv(0, j);
        double vi_bound2 = -v_max_.z() / M_interv(0, j);
        constraint_zL = std::max(constraint_zL, std::min(vi_bound1, vi_bound2));
        constraint_zU = std::min(constraint_zU, std::max(vi_bound1, vi_bound2));
      }
    }
  }

  if (basis_ == MINVO)
  {
    double mean_x = (constraint_xL + constraint_xU) / 2.0;
    double dist_x = fabs(mean_x - constraint_xL);
    constraint_xL = mean_x - alpha_shrink_ * dist_x;
    constraint_xU = mean_x + alpha_shrink_ * dist_x;

    double mean_y = (constraint_yL + constraint_yU) / 2.0;
    double dist_y = fabs(mean_y - constraint_yL);
    constraint_yL = mean_y - alpha_shrink_ * dist_y;
    constraint_yU = mean_y + alpha_shrink_ * dist_y;

    double mean_z = (constraint_zL + constraint_zU) / 2.0;
    double dist_z = fabs(mean_z - constraint_zL);
    constraint_zL = mean_z - alpha_shrink_ * dist_z;
    constraint_zU = mean_z + alpha_shrink_ * dist_z;
  }

  ////////////////////////IMPOSE ACCELERATION CONSTRAINTS
  ////////////////////////

  // // // |vi - viM1|<=a_max_*d
  // // //  <=>
  // // // (vi - viM1)<=a_max_*d   AND   (vi - viM1)>=-a_max_*d
  // // //  <=>
  // // //  vi<=a_max_*d + viM1   AND     vi>=-a_max_*d + viM1

  double d = (knots_(i + p_) - knots_(i + 1)) / (1.0 * (p_ - 1));

  Eigen::Vector3d max_vel = a_max_ * d + viM1;   // this is to ensure that aiM1 is inside the bounds
  Eigen::Vector3d min_vel = -a_max_ * d + viM1;  // this is to ensure that aiM1 is inside the bounds

  constraint_xL = std::max(constraint_xL, min_vel.x());  // lower bound
  constraint_xU = std::min(constraint_xU, max_vel.x());  // upper bound

  constraint_yL = std::max(constraint_yL, min_vel.y());  // lower bound
  constraint_yU = std::min(constraint_yU, max_vel.y());  // upper bound

  constraint_zL = std::max(constraint_zL, min_vel.z());  // lower bound
  constraint_zU = std::min(constraint_zU, max_vel.z());  // upper bound

  if (i == (N_ - 3))
  {                                       // impose also that aNm3 is satisfied
    Eigen::Vector3d vNm2(0.0, 0.0, 0.0);  // Due to the stop condition

    double c = (knots_(N_ - 3 + p_ + 1) - knots_(N_ - 3 + 2)) / (p_ - 1);

    Eigen::Vector3d vNm3_max = vNm2 + c * a_max_;
    Eigen::Vector3d vNm3_min = vNm2 - c * a_max_;

    constraint_xL = std::max(constraint_xL, vNm3_min.x());  // lower bound
    constraint_xU = std::min(constraint_xU, vNm3_max.x());  // upper bound

    constraint_yL = std::max(constraint_yL, vNm3_min.y());  // lower bound
    constraint_yU = std::min(constraint_yU, vNm3_max.y());  // upper bound

    constraint_zL = std::max(constraint_zL, vNm3_min.z());  // lower bound
    constraint_zU = std::min(constraint_zU, vNm3_max.z());  // upper bound
  }

  double eps_delta = 1e-4;
  auto areVeryClose = [&](double a, double b) { return (fabs(a - b) < eps_delta); };

  if (constraint_xL > constraint_xU || constraint_yL > constraint_yU || constraint_zL > constraint_zU ||
      areVeryClose(constraint_xL, constraint_xU) ||  /////////////////
      areVeryClose(constraint_yL, constraint_yU) ||  /////////////////
      areVeryClose(constraint_zL, constraint_zU)     /////////////////
  )
  {  // can happen when i==(N_-3), but never happens when i<(N_-3)

    // std::cout << red << "x: " << constraint_xL << " --> " << constraint_xU << " || "
    //           << "y: " << constraint_yL << " --> " << constraint_yU << " || "
    //           << "z: " << constraint_zL << " --> " << constraint_zU << reset << std::endl;
    // std::cout << "Interval is zero" << std::endl;
    return false;
  }
  else
  {
    // std::cout << green << "x: " << constraint_xL << " --> " << constraint_xU << " || "
    //           << "y: " << constraint_yL << " --> " << constraint_yU << " || "
    //           << "z: " << constraint_zL << " --> " << constraint_zU << reset << std::endl;
    return true;  // constraint_xL<=constraint_xU (same with other axes)
  }

  // if (neighbor.index == (N_ - 2))
  // {  // Check that aNm2 is satisfied

  //   Eigen::Vector3d vNm1(0.0, 0.0, 0.0);  // Due to the stop condition
  //   Eigen::Vector3d vNm2 = vi;
  //   Eigen::Vector3d aNm2 = (p_ - 1) * (vNm1 - vNm2) / (knots_(N_ - 2 + p_ + 1) - knots_(N_ - 2 + 2));

  //   if ((aNm2.array() > a_max_.array()).any() || (aNm2.array() < -a_max_.array()).any())
  //   {
  //     continue;
  //   }
  // }
}

void OctopusSearch::computeInverses()
{
  Ainverses_.clear();
  Ainverses_.push_back(Eigen::MatrixXd::Zero(1, 1));  // dimension 0
  Ainverses_.push_back(Eigen::MatrixXd::Zero(1, 1));  // dimension 1
  Ainverses_.push_back(Eigen::MatrixXd::Zero(2, 2));  // dimension 2
  for (int dim = 3; dim < N_; dim++)
  {
    Eigen::MatrixXd A = 12 * Eigen::MatrixXd::Identity(dim, dim);
    Eigen::Matrix<double, -1, 1> tmp1 = -8 * Eigen::MatrixXd::Ones(dim - 1, 1);
    Eigen::Matrix<double, -1, 1> tmp2 = 2 * Eigen::MatrixXd::Ones(dim - 2, 1);
    A.diagonal(1) = tmp1;
    A.diagonal(-1) = tmp1;
    A.diagonal(2) = tmp2;
    A.diagonal(-2) = tmp2;

    Ainverses_.push_back(A.inverse());
  }
}

void OctopusSearch::setq0q1q2(Eigen::Vector3d& q0, Eigen::Vector3d& q1, Eigen::Vector3d& q2)
{
  q0_ = q0;
  q1_ = q1;
  q2_ = q2;
}

double OctopusSearch::h(Node& node)
{
  return (node.qi - goal_).norm();
}

double OctopusSearch::g(Node& node)
{
  double cost = 0;
  Node* tmp = &node;
  while (tmp->previous != NULL)
  {
    cost = cost + weightEdge(*tmp->previous, *tmp);
    tmp = tmp->previous;
  }

  return cost;
}

double OctopusSearch::weightEdge(Node& node1, Node& node2)  // edge cost when adding node 2
{
  return (node2.qi - node1.qi).norm();
}

void OctopusSearch::printPath(Node& node1)
{
  Node tmp = node1;
  while (tmp.previous != NULL)
  {
    // std::cout << tmp.index << ", ";  // qi.transpose().x()
    std::cout << tmp.previous->qi.transpose() << std::endl;
    tmp = *tmp.previous;
  }
  std::cout << std::endl;
}

void OctopusSearch::recoverPath(Node* result_ptr)
{
  // std::cout << "Recovering path" << std::endl;
  result_.clear();

  if (result_ptr == NULL)
  {
    result_.push_back(q0_);
    result_.push_back(q1_);
    return;
  }

  Node* tmp = result_ptr;

  // std::cout << "Pushing qN= " << tmp->qi.transpose() << std::endl;
  // std::cout << "Pushing qN-1= " << tmp->qi.transpose() << std::endl;

  // std::cout << "qi is" << std::endl;
  // std::cout << tmp->qi.transpose() << std::endl;

  //  std::cout << "qi is" << tmp->qi.transpose() << std::endl;
  result_.push_back(tmp->qi);  // qN
  result_.push_back(tmp->qi);  // qN-1

  while (tmp != NULL)
  {
    result_.push_back(tmp->qi);

    tmp = tmp->previous;
  }

  result_.push_back(q1_);
  result_.push_back(q0_);

  // std::cout << "reverse" << std::endl;

  std::reverse(std::begin(result_), std::end(result_));  // result_ is [q0 q1 q2 q3 ...]
}

Eigen::Matrix<double, 3, 4> OctopusSearch::transformBSpline2otherBasis(const Eigen::Matrix<double, 3, 4>& Qbs,
                                                                       int interval)
{
  return Qbs * M_pos_bs2basis_[interval];
}

Eigen::Matrix<double, 3, 4> OctopusSearch::transformOtherBasis2BSpline(const Eigen::Matrix<double, 3, 4>& Qmv,
                                                                       int interval)
{
  return Qmv * M_pos_bs2basis_inverse_[interval];
}

bool OctopusSearch::checkFeasAndFillND(std::vector<Eigen::Vector3d>& q, std::vector<Eigen::Vector3d>& n,
                                       std::vector<double>& d)
{
  n.resize(std::max(num_of_normals_, 0), Eigen::Vector3d::Zero());
  d.resize(std::max(num_of_normals_, 0), 0.0);

  Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point

  bool isFeasible = true;

  // Check obstacles constraints (and compute n and d)
  for (int index_interv = 0; index_interv < (q.size() - 3); index_interv++)
  {
    last4Cps.col(0) = q[index_interv];
    last4Cps.col(1) = q[index_interv + 1];
    last4Cps.col(2) = q[index_interv + 2];
    last4Cps.col(3) = q[index_interv + 3];

    Eigen::Matrix<double, 3, 4> last4Cps_new_basis = transformBSpline2otherBasis(last4Cps, index_interv);

    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      Eigen::Vector3d n_i;
      double d_i;

      bool solved = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][index_interv], last4Cps_new_basis);

      // std::cout << "index_interv= " << index_interv << std::endl;
      if (solved == false)
      {
        ////////////////////////////// For debugging
        // std::cout << red << "\nThis does NOT satisfy the LP:  obstacle= " << obst_index
        //          << ", last index=" << index_interv + 3 << reset << std::endl;

        // std::cout << " (in basis_ form)" << std::endl;

        // std::cout << last4Cps_new_basis[0].transpose() << std::endl;
        // std::cout << last4Cps_new_basis[1].transpose() << std::endl;
        // std::cout << last4Cps_new_basis[2].transpose() << std::endl;
        // std::cout << last4Cps_new_basis[3].transpose() << std::endl;

        // std::cout << " (in B-Spline form)" << std::endl;

        // std::cout << last4Cps[0].transpose() << std::endl;
        // std::cout << last4Cps[1].transpose() << std::endl;
        // std::cout << last4Cps[2].transpose() << std::endl;
        // std::cout << last4Cps[3].transpose() << std::endl;

        // std::cout << bold << red << "[A*] The node provided doesn't satisfy LPs" << reset << std::endl;

        ///////////////////////////

        isFeasible = false;
      }
      else
      {
        ////////////////////////////// For debugging
        // std::cout << "\nThis satisfies the LP:  obstacle= " << obst_index << ", last index=" << index_interv + 3
        //           << std::endl;

        // std::cout << " (in basis_ form)" << std::endl;

        // std::cout << last4Cps_new_basis[0].transpose() << std::endl;
        // std::cout << last4Cps_new_basis[1].transpose() << std::endl;
        // std::cout << last4Cps_new_basis[2].transpose() << std::endl;
        // std::cout << last4Cps_new_basis[3].transpose() << std::endl;

        // std::cout << " (in B-Spline form)" << std::endl;

        // std::cout << last4Cps[0].transpose() << std::endl;
        // std::cout << last4Cps[1].transpose() << std::endl;
        // std::cout << last4Cps[2].transpose() << std::endl;
        // std::cout << last4Cps[3].transpose() << std::endl;
        ///////////////////////////
      }

      /*      std::cout << "solved with ni=" << n_i.transpose() << std::endl;
            std::cout << "solved with d_i=" << d_i << std::endl;*/

      n[obst_index * num_of_segments_ + index_interv] = n_i;
      d[obst_index * num_of_segments_ + index_interv] = d_i;
    }
  }

  // Check velocity and accel constraints
  Eigen::Vector3d vi, vip1, vip2;
  Eigen::Vector3d ai;
  double epsilon = 1.0001;
  for (int i = 0; i <= (N_ - 3); i++)
  {
    vi = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
    vip1 = p_ * (q[i + 2] - q[i + 1]) / (knots_(i + 1 + p_ + 1) - knots_(i + 1 + 1));
    vip2 = p_ * (q[i + 3] - q[i + 2]) / (knots_(i + 1 + p_ + 1 + 1) - knots_(i + 1 + 1 + 1));

    if (i == (N_ - 2))
    {
      std::cout << "knots_= " << knots_ << std::endl;
      std::cout << "knots_(i + 1 + 1 + 1)= " << knots_(i + 1 + 1 + 1) << std::endl;
      std::cout << "knots_(i + 1 + p_ + 1 + 1)= " << knots_(i + 1 + p_ + 1 + 1) << std::endl;
      std::cout << "i + 1 + 1 + 1= " << i + 1 + 1 + 1 << std::endl;
      std::cout << "i + 1 + p_ + 1 + 1= " << i + 1 + p_ + 1 + 1 << std::endl;
    }

    ai = (p_ - 1) * (vip1 - vi) / (knots_(i + p_ + 1) - knots_(i + 2));

    Eigen::Matrix<double, 3, 3> Vbs;
    Vbs.col(0) = vi;
    Vbs.col(1) = vip1;
    Vbs.col(2) = vip2;

    Eigen::Matrix<double, 3, 3> V_newbasis = Vbs * M_vel_bs2basis_[i];

    // if ((vi.array() > epsilon * v_max_.array()).any() || (vi.array() < -epsilon * v_max_.array()).any())

    // Assumming here that all the elements of v_max_ are the same
    if (V_newbasis.maxCoeff() > epsilon * v_max_.x() || V_newbasis.minCoeff() < -epsilon * v_max_.x())
    {
      std::cout << red << "velocity constraint for vi is not satisfied, i=" << i << reset << std::endl;

      std::cout << "N_=" << N_ << std::endl;

      std::cout << "qa= " << q[i].transpose() << std::endl;
      std::cout << "qb= " << q[i + 1].transpose() << std::endl;
      std::cout << "qc= " << q[i + 2].transpose() << std::endl;
      std::cout << "qd= " << q[i + 3].transpose() << std::endl;

      std::cout << "Vbs= \n" << Vbs << std::endl;
      std::cout << "V_newbasis= \n" << V_newbasis << std::endl;
      std::cout << "v_max_= \n" << v_max_ << std::endl;
      std::cout << "Using matrix \n" << M_vel_bs2basis_[i] << std::endl;
      isFeasible = false;
    }

    // if (i == N_ - 2)
    // {  // Check also vNm1 (which should be zero)
    //   if ((vip1.array() > epsilon * v_max_.array()).any() || (vip1.array() < -epsilon * v_max_.array()).any())
    //   {
    //     isFeasible = false;
    //     std::cout << red << "velocity constraint for vNm1 is not satisfied" << reset << std::endl;
    //   }
    // }
    if ((ai.array() > epsilon * a_max_.array()).any() || (ai.array() < -epsilon * a_max_.array()).any())
    {
      // std::cout << red << "acceleration constraints are not satisfied" << reset << std::endl;
      // std::cout << "ai= " << ai.transpose() << std::endl;
      // std::cout << "i= " << i << std::endl;
      // std::cout << "N_=" << N_ << std::endl;

      // std::cout << "a_max_= " << a_max_.transpose() << std::endl;
      isFeasible = false;
    }
  }

  return isFeasible;
}

bool OctopusSearch::collidesWithObstacles(Node& current)
{
  Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point

  if (current.index >= 3)
  {
    if (current.index == 3)
    {
      last4Cps.col(0) = q0_;
      last4Cps.col(1) = q1_;
      last4Cps.col(2) = current.previous->qi;
      last4Cps.col(3) = current.qi;
    }
    else if (current.index == 4)
    {
      last4Cps.col(0) = q1_;
      last4Cps.col(1) = current.previous->previous->qi;
      last4Cps.col(2) = current.previous->qi;
      last4Cps.col(3) = current.qi;
    }
    else
    {
      last4Cps.col(0) = current.previous->previous->previous->qi;
      last4Cps.col(1) = current.previous->previous->qi;
      last4Cps.col(2) = current.previous->qi;
      last4Cps.col(3) = current.qi;
    }

    bool collides = collidesWithObstaclesGivenVertexes(last4Cps, current.index);

    ////////
    if (current.index == (N_ - 2))
    {
      // Check for the convex hull qNm4, qNm3, qNm2, qNm1 (where qNm1==qNm2)

      Eigen::Matrix<double, 3, 4> last4Cps_tmp;  // Each column contains a control point

      last4Cps_tmp.col(0) = last4Cps.col(1);
      last4Cps_tmp.col(1) = last4Cps.col(2);
      last4Cps_tmp.col(2) = last4Cps.col(3);
      last4Cps_tmp.col(3) = last4Cps.col(3);

      collides = (collides || collidesWithObstaclesGivenVertexes(last4Cps_tmp, N_ - 1));

      // Check for the convex hull qNm3, qNm2, qNm1, qN (where qN==qNm1==qNm2)

      last4Cps_tmp.col(0) = last4Cps.col(2);
      last4Cps_tmp.col(1) = last4Cps.col(3);
      last4Cps_tmp.col(2) = last4Cps.col(3);
      last4Cps_tmp.col(3) = last4Cps.col(3);

      collides = (collides || collidesWithObstaclesGivenVertexes(last4Cps_tmp, N_));
    }
    ////////

    return collides;
  }
  else
  {
    return false;  // I don't have enough control points yet to determine if the first interval collides or not
  }
}

void OctopusSearch::expandAndAddToQueue(Node& current, double constraint_xL, double constraint_xU, double constraint_yL,
                                        double constraint_yU, double constraint_zL, double constraint_zU)
{
  MyTimer timer_expand(true);

  if (current.index == (N_ - 2))
  {
    // std::cout << "can't expand more in this direction" << std::endl;
    return;  // neighbors = empty vector
  }

  int i = current.index;

  Eigen::Vector3d vi;

  int jx, jy, jz;

  Node neighbor;

  neighbor.index = current.index + 1;
  neighbor.previous = &current;

  double delta_x = ((constraint_xU - constraint_xL) / (num_samples_x_ - 1));
  double delta_y = ((constraint_yU - constraint_yL) / (num_samples_y_ - 1));
  double delta_z = ((constraint_zU - constraint_zL) / (num_samples_z_ - 1));

  // MyTimer timer_forLoop(true);
  for (auto comb : all_combinations_)
  {
    jx = std::get<0>(comb);
    jy = std::get<1>(comb);
    jz = std::get<2>(comb);

    vi << constraint_xL + jx * delta_x, constraint_yL + jy * delta_y, constraint_zL + jz * delta_z;

    neighbor.qi = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + current.qi;

    if (vi.norm() < 1e-5  ||  // Not wanna use v=[0,0,0]
        neighbor.qi.x() > x_max_ || neighbor.qi.x() < x_min_ ||                 /// Outside the limits
        neighbor.qi.y() > y_max_ || neighbor.qi.y() < y_min_ ||                 /// Outside the limits
        neighbor.qi.z() > z_max_ || neighbor.qi.z() < z_min_ ||                 /// Outside the limits
        (neighbor.qi - q0_).norm() >= Ra_                                       // ||  /// Outside the limits
        // (ix >= bbox_x_ / voxel_size_ ||                            // Out. the search box
        //  iy >= bbox_y_ / voxel_size_ ||                            // Out. the search box
        //  iz >= bbox_z_ / voxel_size_)                              // Out. the search box
    )

    {
      continue;
    }

    neighbor.g = current.g + weightEdge(current, neighbor);
    neighbor.h = h(neighbor);

    // std::cout << green << neighbor.qi.transpose() << " cost=" << neighbor.g + bias_ * neighbor.h << reset <<
    // std::endl;
    openList_.push(neighbor);
  }
  // std::cout << "pushing to openList  took " << time_openList << std::endl;
  // std::cout << "openList size= " << openList_.size() << std::endl;
  // std::cout << "for loop took " << timer_forLoop << std::endl;
  // std::cout << "time_solving_lps_ " << time_solving_lps_ << std::endl;
  // std::cout << bold << "expanding took " << timer_expand << reset << std::endl;

  //  time_solving_lps_ = 0.0;

  // time_expanding_ += timer_expand.ElapsedMs();
  // std::cout << "End of expand Function" << std::endl;
}

bool OctopusSearch::collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps, int index_lastCP)
{
  // MyTimer timer_function(true);
  // std::cout << "In collidesWithObstaclesGivenVertexes, index_lastCP= " << index_lastCP << std::endl;

  int interval = index_lastCP - 3;

  Eigen::Matrix<double, 3, 4> last4Cps_new_basis = transformBSpline2otherBasis(last4Cps, interval);

  bool satisfies_LP = true;
  Eigen::Vector3d n_i;
  double d_i;

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][interval], last4Cps_new_basis);
    if (satisfies_LP == false)
    {
      goto exit;
    }
  }

exit:

  // time_solving_lps_ += timer_function.ElapsedUs() / 1000.0;
  return (!satisfies_LP);
}

bool OctopusSearch::run(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d)
{
  /////////// reset some stuff
  // stores the closest node found
  closest_dist_so_far_ = std::numeric_limits<double>::max();
  closest_result_so_far_ptr_ = NULL;

  // stores the closest node found that has full length (i.e. its index == (N_ - 2) )
  complete_closest_dist_so_far_ = std::numeric_limits<double>::max();
  complete_closest_result_so_far_ptr_ = NULL;

  expanded_valid_nodes_.clear();
  result_.clear();
  map_open_list_.clear();

  ////////////////////////////

  CompareCost comparer;
  comparer.bias = bias_;

  std::priority_queue<Node, std::vector<Node>, CompareCost> openList_tmp(comparer);
  openList_ = openList_tmp;

  std::cout << "[A*] Running..." << std::endl;

  MyTimer timer_astar(true);

  Node nodeq2;
  nodeq2.index = 0;
  nodeq2.previous = NULL;
  nodeq2.g = 0;
  nodeq2.qi = q2_;
  nodeq2.index = 2;
  nodeq2.h = h(nodeq2);  // f=g+h

  openList_.push(nodeq2);

  Node* current_ptr;

  int status;

  int RUNTIME_REACHED = 0;
  int GOAL_REACHED = 1;
  int EMPTY_OPENLIST = 2;

  while (openList_.size() > 0)
  {
    // Check if runtime is over
    if (timer_astar.ElapsedMs() > (max_runtime_ * 1000))
    {
      std::cout << "[A*] Max Runtime was reached" << std::endl;
      status = RUNTIME_REACHED;
      goto exitloop;
    }

    current_ptr = new Node;

    *current_ptr = openList_.top();  // copy the first element onto (*current_ptr)
    openList_.pop();                 // remove it from the list

    double dist = ((*current_ptr).qi - goal_).norm();

    if (closest_result_so_far_ptr_ == NULL)
    {
      closest_dist_so_far_ = dist;
      closest_result_so_far_ptr_ = current_ptr;
    }

    unsigned int ix, iy, iz;
    ix = round(((*current_ptr).qi.x() - orig_.x()) / voxel_size_);
    iy = round(((*current_ptr).qi.y() - orig_.y()) / voxel_size_);
    iz = round(((*current_ptr).qi.z() - orig_.z()) / voxel_size_);
    auto ptr_to_voxel = map_open_list_.find(Eigen::Vector3i(ix, iy, iz));
    bool already_exist = (ptr_to_voxel != map_open_list_.end());
    // if (already_exist)
    // {
    //   already_exist = (already_exist) && (map_open_list_[Eigen::Vector3i(ix, iy, iz)] == true);
    // }
    // already_exist = false;
    if (already_exist || (*current_ptr).qi.x() > x_max_ || (*current_ptr).qi.x() < x_min_ ||  /// Outside the limits
        (*current_ptr).qi.y() > y_max_ || (*current_ptr).qi.y() < y_min_ ||                   /// Outside the limits
        (*current_ptr).qi.z() > z_max_ || (*current_ptr).qi.z() < z_min_ ||                   /// Outside the limits
        ((*current_ptr).qi - q0_).norm() >= Ra_)
    {
      continue;
    }

    /////////////////////
    int i = (*current_ptr).index;

    Eigen::Vector3d qiM2, qiM1;

    if ((*current_ptr).index == 2)
    {
      qiM2 = q0_;
      qiM1 = q1_;
    }
    else if ((*current_ptr).index == 3)
    {
      qiM2 = q1_;
      qiM1 = (*current_ptr).previous->qi;
    }
    else
    {
      qiM2 = (*current_ptr).previous->previous->qi;
      qiM1 = (*current_ptr).previous->qi;
    }

    double constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU;

    bool intervalIsNotZero = true;

    if (i < (N_ - 2))
    {  // for qNm2 I'm not going to sample velocities (not going to call expandAndAddToQueue) --> don't do this
      intervalIsNotZero =
          computeUpperAndLowerConstraints(i, qiM2, qiM1, (*current_ptr).qi, constraint_xL, constraint_xU, constraint_yL,
                                          constraint_yU, constraint_zL, constraint_zU);
    }
    if (intervalIsNotZero == false)  // constraintxL>constraint_xU (or with other axes)
    {
      continue;
    }

    /////////////////////

    MyTimer timer_collision_check(true);
    bool collides = collidesWithObstacles(*current_ptr);
    // std::cout << "collision check took " << timer_collision_check << std::endl;

    // already_exist = false;
    if (collides)
    {
      // Node* tmp = current_ptr;
      // while (tmp != NULL)
      // {
      //   // cps.push_back(Eigen::Vector3d(tmp->qi.x(), tmp->qi.y(), tmp->qi.z()));
      //   unsigned int ix2, iy2, iz2;
      //   ix2 = round((tmp->qi.x() - orig_.x()) / voxel_size_);
      //   iy2 = round((tmp->qi.y() - orig_.y()) / voxel_size_);
      //   iz2 = round((tmp->qi.z() - orig_.z()) / voxel_size_);
      //   map_open_list_[Eigen::Vector3i(ix2, iy2, iz2)] = false;
      //   tmp = tmp->previous;
      // }

      // map_open_list_[Eigen::Vector3i(ix, iy, iz)] = false;
      continue;
    }
    else
    {
      // std::cout << green << bold << "does not collide: " << (*current_ptr).qi.transpose() << "(i= " << i << ")" <<
      // reset
      //           << std::endl;

      // std::cout << "pushing, index= " << (*current_ptr).index << std::endl;
      map_open_list_[Eigen::Vector3i(ix, iy, iz)] = true;
      expanded_valid_nodes_.push_back(*current_ptr);
    }

    if ((*current_ptr).index == (N_ - 2) &&
        dist < std::max(0.0, complete_closest_dist_so_far_ - 1e-4))  // the 1e-4 is to avoid numerical issues of paths
                                                                     // essentially with the same dist to the goal. In
                                                                     // those cases, this gives priority to the
                                                                     // trajectories found first
    {
      complete_closest_dist_so_far_ = dist;
      complete_closest_result_so_far_ptr_ = current_ptr;

      // std::cout << bold << blue << "complete_closest_dist_so_far_= " << std::setprecision(10)
      //           << complete_closest_dist_so_far_ << reset << std::endl;
    }

    if (dist < closest_dist_so_far_)
    {
      closest_dist_so_far_ = dist;
      closest_result_so_far_ptr_ = current_ptr;
    }

    // check if we are already in the goal
    if ((dist < goal_size_) && (*current_ptr).index == (N_ - 2))
    {
      std::cout << "[A*] Goal was reached!" << std::endl;
      status = GOAL_REACHED;
      goto exitloop;
    }
    if ((*current_ptr).index == (N_ - 2))
    {
      continue;
    }
    expandAndAddToQueue(*current_ptr, constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL,
                        constraint_zU);
  }

  std::cout << "[A*] openList_ is empty" << std::endl;
  status = EMPTY_OPENLIST;
  goto exitloop;

exitloop:

  // std::cout << "status= " << status << std::endl;
  // std::cout << "expanded_nodes_.size()= " << expanded_nodes_.size() << std::endl;
  // std::cout << "complete_closest_dist_so_far_= " << complete_closest_dist_so_far_ << std::endl;

  Node* best_node_ptr = NULL;

  bool have_a_solution = (complete_closest_result_so_far_ptr_ != NULL) || (closest_result_so_far_ptr_ != NULL);

  if (status == GOAL_REACHED)
  {
    std::cout << "[A*] choosing current_ptr as solution" << std::endl;
    best_node_ptr = current_ptr;
  }
  else if ((status == RUNTIME_REACHED || status == EMPTY_OPENLIST) && have_a_solution)
  {
    // if (complete_closest_result_so_far_ptr_ != NULL)
    // {
    //   std::cout << "THERE EXISTS AT LEAST ONE COMPLETE PATH, with distance= " << complete_closest_dist_so_far_
    //             << std::endl;
    // }

    if (complete_closest_result_so_far_ptr_ != NULL)
    {
      std::cout << "[A*] choosing closest complete path as solution" << std::endl;
      best_node_ptr = complete_closest_result_so_far_ptr_;
      std::cout << bold << blue << "complete_closest_dist_so_far_= " << complete_closest_dist_so_far_ << reset
                << std::endl;
    }
    else
    {
      std::cout << "[A*] choosing closest path as solution" << std::endl;
      best_node_ptr = closest_result_so_far_ptr_;
    }
  }
  else
  {
    std::cout << red << "This state should never occur" << reset << std::endl;
    abort();
    return false;
  }

  // Fill (until we arrive to N_-2), with the same qi
  // Note that, by doing this, it's not guaranteed feasibility wrt a dynamic obstacle
  // and hence the need of the function checkFeasAndFillND()

  bool path_found_is_not_complete = (best_node_ptr->index < (N_ - 2));

  if (path_found_is_not_complete)
  {
    for (int j = (best_node_ptr->index) + 1; j <= N_ - 2; j++)
    {
      // return false;

      Node* node_ptr = new Node;
      node_ptr->qi = best_node_ptr->qi;
      node_ptr->index = j;

      std::cout << red << "Filled " << j << ", " << reset;

      // << node_ptr->qi.transpose() << std::endl;
      node_ptr->previous = best_node_ptr;
      best_node_ptr = node_ptr;
    }
  }

  recoverPath(best_node_ptr);  // saved in result_

  // std::cout << "____________" << std::endl;
  // for (auto qi : result_)
  // {
  //   std::cout << qi.transpose() << std::endl;
  // }
  result = result_;

  bool isFeasible = checkFeasAndFillND(result_, n, d);

  if (isFeasible == false && path_found_is_not_complete == false)
  {
    // std::cout << red << "This should never happen: All complete paths are guaranteed to be feasible" << reset
    //           << std::endl;

    // std::cout << red << "=====================================================" << std::endl;

    // abort();
  }

  return isFeasible;
}