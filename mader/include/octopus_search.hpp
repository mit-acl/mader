/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef OCTOPUS_SEARCH_HPP
#define OCTOPUS_SEARCH_HPP

#include <vector>
#include <Eigen/Dense>
#include "mader_types.hpp"

#include "separator.hpp"
#include "cgal_utils.hpp"

#include <unordered_map>
#include <queue>
#include <tuple>

//#include "solvers/cvxgen/solver_cvxgen.hpp"

typedef struct Node Node;  // needed to be able to have a pointer inside the struct

struct Node
{
  Eigen::Vector3d qi;
  Node* previous = NULL;
  double g = 0;
  double h = 0;
  int index = 2;  // Start with q2_
  int idx;
  int idy;
  int idz;
};

// Taken from https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    // Note that it is obvious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class OctopusSearch
{
public:
  OctopusSearch(std::string basis, int num_pol, int deg_pol, double alpha_shrink);
  void setUp(double t_min, double t_max, const mt::ConvexHullsOfCurves_Std& hulls);
  ~OctopusSearch();

  void setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int num_samples_x, int num_samples_y,
                              int num_samples_z, double fraction_voxel_size);
  void setq0q1q2(Eigen::Vector3d& q0, Eigen::Vector3d& q1, Eigen::Vector3d& q2);
  void setGoal(Eigen::Vector3d& goal);
  void setSamples(int num_samples_x, int num_samples_y, int num_samples_z);

  void setRunTime(double max_runtime);
  void setGoalSize(double goal_size);

  void setBasisUsedForCollision(int basis);

  void setBias(double bias);

  bool run(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d);

  void recoverPath(Node* node1_ptr);

  void getAllTrajsFound(std::vector<mt::trajectory>& all_trajs_found);

  void computeInverses();

  void setBBoxSearch(double x, double y, double z);

  void setVisual(bool visual);

  void setXYZMinMaxAndRa(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, double Ra);

  int getNumOfLPsRun();

  void getBestTrajFound(mt::trajectory& best_traj_found, mt::PieceWisePol& pwp, double dc);
  void getEdgesConvexHulls(mt::Edges& edges_convex_hulls);

  int B_SPLINE = 1;  // B-Spline Basis
  int MINVO = 2;     // Minimum volume basis
  int BEZIER = 3;    // Bezier basis

  struct CompareCost
  {
    double bias;
    bool operator()(const Node& left, const Node& right)
    {
      double cost_left = left.g + bias * left.h;
      double cost_right = right.g + bias * right.h;
      if (fabs(cost_left - cost_right) < 1e-5)
      {
        return left.h > right.h;  // If two costs are ~the same, decide only upon heuristic
      }
      else
      {
        return cost_left > cost_right;
      }
    }
  };

  bool collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps, int index_lastCP);
  bool collidesWithObstacles(Node& current);
  double getCost();

protected:
private:
  // bool computeAxisForNextInterval(const int i, const Eigen::Vector3d& viM1, int axis, double& constraint_L,
  //                                 double& constraint_U);

  Eigen::Matrix<double, 3, 4> transformBSpline2otherBasis(const Eigen::Matrix<double, 3, 4>& Qbs, int interval);
  Eigen::Matrix<double, 3, 4> transformOtherBasis2BSpline(const Eigen::Matrix<double, 3, 4>& Qmv, int interval);

  void computeLimitsVoxelSize(double& min_voxel_size, double& max_voxel_size);
  bool computeUpperAndLowerConstraints(const int i, const Eigen::Vector3d& qiM2, const Eigen::Vector3d& qiM1,
                                       const Eigen::Vector3d& qi, double& constraint_xL, double& constraint_xU,
                                       double& constraint_yL, double& constraint_yU, double& constraint_zL,
                                       double& constraint_zU);

  void plotExpandedNodesAndResult(std::vector<Node>& expanded_nodes, Node* result_ptr);
  void expandAndAddToQueue(Node& current, double constraint_xL, double constraint_xU, double constraint_yL,
                           double constraint_yU, double constraint_zL, double constraint_zU);
  void printPath(Node& node1);
  double h(Node& node);
  double g(Node& node);
  double weightEdge(Node& node1, Node& node2);

  bool checkFeasAndFillND(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n,
                          std::vector<double>& d);

  // bias should be >=1.0
  double bias_;  // page 34 of https://www.cs.cmu.edu/~motionplanning/lecture/Asearch_v8.pdf

  int basis_ = B_SPLINE;

  Eigen::Vector3d goal_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;

  mt::ConvexHullsOfCurves_Std hulls_;
  separator::Separator* separator_solver_;

  int num_samples_x_ = 3;
  int num_samples_y_ = 3;
  int num_samples_z_ = 3;

  std::vector<int> indexes_samples_x_;
  std::vector<int> indexes_samples_y_;
  std::vector<int> indexes_samples_z_;

  std::vector<std::tuple<int, int, int>> all_combinations_;

  int p_;
  int N_;
  int M_;
  int num_of_obst_;
  int num_of_segments_;
  int num_of_normals_;

  Eigen::Vector3d orig_;  // origin of the search box (left bottom corner)

  Eigen::Vector3d q0_;
  Eigen::Vector3d q1_;
  Eigen::Vector3d q2_;

  Eigen::RowVectorXd knots_;

  // stores the closest node found
  double closest_dist_so_far_ = std::numeric_limits<double>::max();
  Node* closest_result_so_far_ptr_ = NULL;

  // stores the closest node found that has full length (i.e. its index == (N_ - 2) )
  double complete_closest_dist_so_far_ = std::numeric_limits<double>::max();
  Node* complete_closest_result_so_far_ptr_ = NULL;

  double goal_size_ = 0.5;    //[m]
  double max_runtime_ = 0.5;  //[s]

  std::vector<Eigen::MatrixXd> Ainverses_;

  double time_solving_lps_ = 0.0;
  double time_expanding_ = 0.0;

  double voxel_size_;

  double bbox_x_;
  double bbox_y_;
  double bbox_z_;

  bool visual_ = false;

  double x_min_ = -std::numeric_limits<double>::max();
  double x_max_ = std::numeric_limits<double>::max();

  double y_min_ = -std::numeric_limits<double>::max();
  double y_max_ = std::numeric_limits<double>::max();

  double z_min_ = -std::numeric_limits<double>::max();
  double z_max_ = std::numeric_limits<double>::max();

  // transformation between the B-spline control points and other basis (MINVO or Bezier)
  std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2basis_;
  std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2basis_;
  std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2basis_inverse_;  // Mbs2basis_

  int num_pol_;

  std::vector<Eigen::Vector3d> result_;

  std::vector<Node> expanded_valid_nodes_;

  std::unordered_map<Eigen::Vector3i, bool, matrix_hash<Eigen::Vector3i>> map_open_list_;

  std::priority_queue<Node, std::vector<Node>, CompareCost> openList_;  //= OpenSet, = Q

  double Ra_ = 1e10;

  // SolverCvxgen cvxgen_solver_;

  double alpha_shrink_;
};

#endif