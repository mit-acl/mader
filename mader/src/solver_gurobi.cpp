/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "solver_nlopt.hpp"
#include "solver_gurobi.hpp"
#include "termcolor.hpp"
#include "bspline_utils.hpp"
#include "ros/ros.h"
#include "solver_gurobi_utils.hpp"

#include <decomp_util/ellipsoid_decomp.h>  //For Polyhedron definition
#include <unsupported/Eigen/Splines>
#include <iostream>
#include <list>
#include <random>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace termcolor;

SolverGurobi::SolverGurobi(par_sgurobi &par)
{
  deg_pol_ = par.deg_pol;
  num_pol_ = par.num_pol;

  p_ = deg_pol_;
  M_ = num_pol_ + 2 * p_;
  N_ = M_ - p_ - 1;
  num_of_segments_ = (M_ - 2 * p_);  // this is the same as num_pol_

  ///////////////////////////////////////
  ///////////////////////////////////////

  basisConverter basis_converter;

  std::cout << "In the SolverGurobi Constructor\n";

  a_star_bias_ = par.a_star_bias;
  // basis used for collision
  if (par.basis == "MINVO")
  {
    basis_ = MINVO;
    M_pos_bs2basis_ = basis_converter.getMinvoPosConverters(num_of_segments_);
    M_vel_bs2basis_ = basis_converter.getMinvoVelConverters(num_of_segments_);
  }
  else if (par.basis == "BEZIER")
  {
    basis_ = BEZIER;
    M_pos_bs2basis_ = basis_converter.getBezierPosConverters(num_of_segments_);
    M_vel_bs2basis_ = basis_converter.getBezierVelConverters(num_of_segments_);
  }
  else if (par.basis == "B_SPLINE")
  {
    basis_ = B_SPLINE;
    M_pos_bs2basis_ = basis_converter.getBSplinePosConverters(num_of_segments_);
    M_vel_bs2basis_ = basis_converter.getBSplineVelConverters(num_of_segments_);
  }
  else
  {
    std::cout << red << "Basis " << par.basis << " not implemented yet" << reset << std::endl;
    std::cout << red << "============================================" << reset << std::endl;
    abort();
  }

  A_pos_bs_ = basis_converter.getABSpline(num_of_segments_);

  ///////////////////////////////////////
  ///////////////////////////////////////

  x_min_ = par.x_min;
  x_max_ = par.x_max;

  y_min_ = par.y_min;
  y_max_ = par.y_max;

  z_min_ = par.z_min;
  z_max_ = par.z_max;
  Ra_ = par.Ra;
  a_star_samp_x_ = par.a_star_samp_x;
  a_star_samp_y_ = par.a_star_samp_y;
  a_star_samp_z_ = par.a_star_samp_z;
  a_star_fraction_voxel_size_ = par.a_star_fraction_voxel_size;
  dist_to_use_straight_guess_ = par.dist_to_use_straight_guess;
  dc_ = par.dc;
  v_max_ = par.v_max;
  a_max_ = par.a_max;
  allow_infeasible_guess_ = par.allow_infeasible_guess;
  // solver_ = getSolver(par.solver);                         // getSolver(nlopt::LD_VAR2);
  epsilon_tol_constraints_ = par.epsilon_tol_constraints;  // 1e-1;
  xtol_rel_ = par.xtol_rel;                                // 1e-1;
  ftol_rel_ = par.ftol_rel;                                // 1e-1;

  weight_ = par.weight;

  separator_solver_ = new separator::Separator();

  myAStarSolver_ = new OctopusSearch(par.basis, num_pol_, deg_pol_, par.alpha_shrink);
}

SolverGurobi::~SolverGurobi()
{
  // delete opt_;
  // delete local_opt_;
}

void SolverGurobi::getGuessForPlanes(std::vector<Hyperplane3D> &planes)
{
  planes = planes_;
  /*  planes.clear();
    std::cout << "GettingGuessesForPlanes= " << n_guess_.size() << std::endl;
    for (auto n_i : n_guess_)
    {
      Eigen::Vector3d p_i;
      p_i << 0.0, 0.0, -1.0 / n_i.z();  // TODO deal with n_i.z()=0
      Hyperplane3D plane(p_i, n_i);
      planes.push_back(plane);
    }*/
}

int SolverGurobi::getNumOfLPsRun()
{
  return num_of_LPs_run_;
}

int SolverGurobi::getNumOfQCQPsRun()
{
  return num_of_QCQPs_run_;
}

void SolverGurobi::setMaxRuntimeKappaAndMu(double max_runtime, double kappa, double mu)
{
  kappa_ = kappa;
  mu_ = mu;
  max_runtime_ = max_runtime;
}

void SolverGurobi::fillPlanesFromNDQ(std::vector<Hyperplane3D> &planes_, const std::vector<Eigen::Vector3d> &n,
                                     const std::vector<double> &d, const std::vector<Eigen::Vector3d> &q)
{
  planes_.clear();

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      Eigen::Vector3d centroid_hull;
      // findCentroidHull(hulls_[obst_index][i], centroid_hull);
      // Eigen::Vector3d tmp = (centroid_hull + q[i]) / 2.0;

      Eigen::Vector3d tmp = q[i];  //(centroid_hull + q[i]) / 2.0;

      Eigen::Vector3d point_in_plane = Eigen::Vector3d::Zero();

      // std::cout << "tmp= " << tmp.transpose() << std::endl;
      // std::cout << "centroid_hull= " << centroid_hull.transpose() << std::endl;

      if (fabs(n[i].x()) != 0)
      {
        point_in_plane << -(n[i].y() * tmp.y() + n[i].z() * tmp.z() + d[i]) / (n[i].x()), tmp.y(), tmp.z();
      }
      else if (fabs(n[i].y()) != 0)
      {
        point_in_plane << tmp.x(), -(n[i].x() * tmp.x() + n[i].z() * tmp.z() + d[i]) / (n[i].y()), tmp.z();
      }
      else
      {
        point_in_plane << tmp.x(), tmp.y(), -(n[i].x() * tmp.x() + n[i].y() * tmp.y() + d[i]) / (n[i].z());
      }
      Hyperplane3D plane(point_in_plane, n[i]);
      planes_.push_back(plane);
    }
  }
}

void SolverGurobi::generateAStarGuess()
{
  std::cout << "[NL] Running A* from" << q0_.transpose() << " to " << final_state_.pos.transpose()
            << ", allowing time = " << kappa_ * max_runtime_ * 1000 << " ms" << std::endl;

  //  std::cout << bold << blue << "z_max_= " << z_max_ << reset << std::endl;

  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();
  planes_.clear();

  generateStraightLineGuess();  // If A* doesn't succeed --> use straight lineGuess
  /*  generateRandomN(n_guess_);
    generateRandomD(d_guess_);
    generateRandomQ(q_guess_);*/

  // std::cout << "The StraightLineGuess is" << std::endl;
  // printStd(q_guess_);
  // std::cout << "************" << std::endl;

  myAStarSolver_->setUp(t_init_, t_final_, hulls_);

  // std::cout << "q0_=" << q0_.transpose() << std::endl;
  // std::cout << "q1_=" << q1_.transpose() << std::endl;
  // std::cout << "q2_=" << q2_.transpose() << std::endl;

  myAStarSolver_->setq0q1q2(q0_, q1_, q2_);
  myAStarSolver_->setGoal(final_state_.pos);

  // double runtime = 0.05;   //[seconds]
  double goal_size = 0.05;  //[meters]

  myAStarSolver_->setXYZMinMaxAndRa(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
                                    Ra_);                 // z limits for the search, in world frame
  myAStarSolver_->setBBoxSearch(2000.0, 2000.0, 2000.0);  // limits for the search, centered on q2
  myAStarSolver_->setMaxValuesAndSamples(v_max_, a_max_, a_star_samp_x_, a_star_samp_y_, a_star_samp_z_,
                                         a_star_fraction_voxel_size_);

  myAStarSolver_->setRunTime(kappa_ * max_runtime_);  // hack, should be kappa_ * max_runtime_
  myAStarSolver_->setGoalSize(goal_size);

  myAStarSolver_->setBias(a_star_bias_);
  // if (basis_ == MINVO)
  // {
  //   // std::cout << green << bold << "snlopt is using MINVO" << reset << std::endl;
  //   myAStarSolver_->setBasisUsedForCollision(myAStarSolver_->MINVO);
  // }
  // else if (basis_ == BEZIER)
  // {
  //   // std::cout << green << bold << "snlopt is using BEZIER" << reset << std::endl;
  //   myAStarSolver_->setBasisUsedForCollision(myAStarSolver_->BEZIER);
  // }
  // else
  // {
  //   // std::cout << green << bold << "snlopt is using B_SPLINE" << reset << std::endl;
  //   myAStarSolver_->setBasisUsedForCollision(myAStarSolver_->B_SPLINE);
  // }

  myAStarSolver_->setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool is_feasible = myAStarSolver_->run(q, n, d);

  num_of_LPs_run_ = myAStarSolver_->getNumOfLPsRun();
  // std::cout << "After Running solved, n= " << std::endl;
  // printStd(n);

  fillPlanesFromNDQ(planes_, n_guess_, d_guess_, q_guess_);

  if (is_feasible)
  {
    ROS_INFO_STREAM("[NL] A* found a feasible solution!");
  }
  else
  {
    ROS_ERROR_STREAM("[NL] A* didn't find a feasible solution!");
  }

  if (is_feasible == true || (is_feasible == false && allow_infeasible_guess_ == true))
  {
    q_guess_ = q;
    n_guess_ = n;
    d_guess_ = d;

    ROS_INFO_STREAM("[NL] Using the A* guess");
  }
  else
  {
    std::cout << "[NL] Using straight line guess" << std::endl;
  }

  return;
}

void SolverGurobi::generateRandomD(std::vector<double> &d)
{
  d.clear();
  for (int k = k_min_; k <= k_max_; k++)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    d.push_back(r1);
  }
}

void SolverGurobi::generateRandomN(std::vector<Eigen::Vector3d> &n)
{
  n.clear();
  for (int j = j_min_; j < j_max_; j = j + 3)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    double r2 = ((double)rand() / (RAND_MAX));
    double r3 = ((double)rand() / (RAND_MAX));
    n.push_back(Eigen::Vector3d(r1, r2, r3));
  }

  // std::cout << "After Generating RandomN, n has size= " << n.size() << std::endl;
}

void SolverGurobi::generateRandomQ(std::vector<Eigen::Vector3d> &q)
{
  q.clear();

  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist_x(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_y(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_z(z_min_, z_max_);

  for (int i = 0; i <= N_; i++)
  {
    q.push_back(Eigen::Vector3d(dist_x(generator), dist_y(generator), dist_z(generator)));
  }

  saturateQ(q);  // make sure is inside the bounds specified
}

void SolverGurobi::findCentroidHull(const Polyhedron_Std &hull, Eigen::Vector3d &centroid)
{
  centroid = Eigen::Vector3d::Zero();

  for (int i = 0; i < hull.cols(); i++)
  {
    centroid += hull.col(i);
  }
  if (hull.cols() > 0)
  {
    centroid = centroid / hull.cols();
  }
}

void SolverGurobi::generateGuessNDFromQ(const std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                        std::vector<double> &d)
{
  n.clear();
  d.clear();

  planes_.clear();

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      Eigen::Vector3d centroid_hull;
      findCentroidHull(hulls_[obst_index][i], centroid_hull);

      Eigen::Vector3d n_i =
          (centroid_hull - q[i]).normalized();  // n_i should point towards the obstacle (i.e. towards the hull)

      double alpha = 0.01;  // the smaller, the higher the chances the plane is outside the obstacle. Should be <1

      Eigen::Vector3d point_in_middle = q[i] + (centroid_hull - q[i]) * alpha;

      double d_i = -n_i.dot(point_in_middle);  // n'x + d = 0

      n.push_back(n_i);  // n'x + 1 = 0
      d.push_back(d_i);  // n'x + 1 = 0

      Hyperplane3D plane(point_in_middle, n_i);
      planes_.push_back(plane);

      // d.push_back(d_i);
    }
  }
}

void SolverGurobi::assignEigenToVector(double *result, int var_gindex, const Eigen::Vector3d &tmp)

{
  /*  std::cout << "i= " << var_gindex << std::endl;
    std::cout << "i= " << var_gindex + 1 << std::endl;
    std::cout << "i= " << var_gindex + 2 << std::endl;*/
  result[var_gindex] = tmp(0);
  result[var_gindex + 1] = tmp(1);
  result[var_gindex + 2] = tmp(2);
}

void SolverGurobi::setHulls(ConvexHullsOfCurves_Std &hulls)

{
  hulls_.clear();
  hulls_ = hulls;

  num_of_obst_ = hulls_.size();

  i_min_ = 0;
  i_max_ =
      3 * (N_ + 1) - 1 - 9 - 6;  // because pos, vel and accel at t_init_ and t_final_ are fixed (not dec variables)
  j_min_ = i_max_ + 1;
  j_max_ = j_min_ + 3 * (M_ - 2 * p_) * num_of_obst_ - 1;
  k_min_ = j_max_ + 1;
  k_max_ = k_min_ + (M_ - 2 * p_) * num_of_obst_ - 1;

  num_of_variables_ = k_max_ + 1;  // k_max_ + 1;

  std::vector<std::string> coords = { "x", "y", "z" };

  // Create the variables: control points
  q_var_.clear();
  for (int i = 0; i <= N_; i++)
  {
    std::vector<GRBVar> q_i;
    for (int j = 0; j < 3; j++)  // x,y,z
    {
      q_i.push_back(m.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "q_" + std::to_string(i) + coords[j]));
    }
    q_var_.push_back(q_i);
    std::cout << "Creating variables!" << std::endl;
  }

  int num_of_cpoints = N_ + 1;

  num_of_normals_ = num_of_segments_ * num_of_obst_;
}

void SolverGurobi::prepareObjective()
{
  GRBQuadExpr cost = 0.0;

  // Cost. See Lyx derivation (notes.lyx)

  Eigen::Matrix<double, 4, 1> tmp;
  tmp << 6.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < num_of_segments_; i++)
  {
    Eigen::Matrix<double, -1, 1> A_i_times_tmp = A_pos_bs_[i] * tmp;  // TODO (this is 4x1)

    std::vector<std::vector<GRBVar>> Q;

    std::vector<GRBVar> tmp_x = { q_var_[i][0], q_var_[i + 1][0], q_var_[i + 2][0], q_var_[i + 3][0] };
    std::vector<GRBVar> tmp_y = { q_var_[i][1], q_var_[i + 1][1], q_var_[i + 2][1], q_var_[i + 3][1] };
    std::vector<GRBVar> tmp_z = { q_var_[i][2], q_var_[i + 1][2], q_var_[i + 2][2], q_var_[i + 3][2] };

    Q.push_back(tmp_x);  // row0
    Q.push_back(tmp_y);  // row1
    Q.push_back(tmp_z);  // row2

    // Eigen::Matrix<double, 3, 4> Q;
    // Q.col(0) = q[i];
    // Q.col(1) = q[i + 1];
    // Q.col(2) = q[i + 2];
    // Q.col(3) = q[i + 3];
    // cost += (Q * A_i_times_tmp).squaredNorm();

    std::vector<GRBLinExpr> tmp = matrixMultiply(Q, eigenVector2std(A_i_times_tmp));
    cost += getNorm2(tmp);

    // std::cout << "cost=" << cost << std::endl;
  }

  double weight = 1.5;

  cost += weight * getNorm2(q_var_[N_] - eigenVector2std(final_state_.pos));

  m.setObjective(cost, GRB_MINIMIZE);
}

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

// Note that t_final will be updated in case the saturation in deltaT_ has had effect
bool SolverGurobi::setInitStateFinalStateInitTFinalT(state initial_state, state final_state, double t_init,
                                                     double &t_final)
{
  ///////////////////////////
  Eigen::Vector3d p0 = initial_state.pos;
  Eigen::Vector3d v0 = initial_state.vel;
  Eigen::Vector3d a0 = initial_state.accel;

  Eigen::Vector3d pf = final_state.pos;
  Eigen::Vector3d vf = final_state.vel;
  Eigen::Vector3d af = final_state.accel;

  // here we saturate the value to ensure it is within the limits
  // the reason for this is the epsilon_tol_constraints (in the previous iteration, it may be slightly unfeasible)
  saturate(v0, -v_max_, v_max_);
  saturate(a0, -a_max_, a_max_);
  saturate(vf, -v_max_, v_max_);
  saturate(af, -a_max_, a_max_);

  initial_state_ = initial_state;
  final_state_ = final_state;

  std::cout << "initial_state= " << std::endl;
  initial_state.printHorizontal();

  std::cout << "final_state= " << std::endl;
  final_state.printHorizontal();

  //////////////////////////////

  deltaT_ = (t_final - t_init) / (1.0 * (M_ - 2 * p_ - 1 + 1));

  //////////////////////////////
  // Now make sure deltaT in knots_ is such that -v_max<=v1<=v_max is satisfied:

  // std::cout << bold << "deltaT_ before= " << deltaT_ << reset << std::endl;

  for (int axis = 0; axis < 3; axis++)
  {
    double upper_bound, lower_bound;
    if (fabs(a0(axis)) > 1e-7)
    {
      upper_bound = ((p_ - 1) * (sgn(a0(axis)) * v_max_(axis) - v0(axis)) / (a0(axis)));
      lower_bound = ((p_ - 1) * (-sgn(a0(axis)) * v_max_(axis) - v0(axis)) / (a0(axis)));

      // std::cout << "axis= " << axis << std::endl;
      // std::cout << "lower_bound= " << lower_bound << std::endl;
      // std::cout << "upper_bound= " << upper_bound << std::endl;

      ////////////////// Just for debugging
      if (upper_bound < lower_bound)
      {
        std::cout << red << bold << "This should never happen, aborting" << std::endl;
        abort();
      }
      //////////////////

      if (upper_bound <= 0)
      {
        std::cout << red << bold << "There is no way to satisfy v1" << std::endl;  //(deltat will be zero)
        return false;
      }

      saturate(deltaT_, std::max(0.0, lower_bound), upper_bound);
    }
    else
    {
      // do nothing: a0 ==0 for that axis, so that means that v1==v0, and therefore v1 satisfies constraints for that
      // axis
    }
  }

  // Eigen::Vector3d bound1 = ((p_ - 1) * (v_max_ - v0).array() / (a0.array()));
  // Eigen::Vector3d bound2 = ((p_ - 1) * (-v_max_ - v0).array() / (a0.array()));

  // // note that if any element of a0 is ==0.0, then its corresponding element in bound1 (or bound2) is +-infinity, but
  // // valid  for the saturation below

  // saturate(deltaT_, std::min(bound1.x(), bound2.x()), std::max(bound1.x(), bound2.x()));
  // saturate(deltaT_, std::min(bound1.y(), bound2.y()), std::max(bound1.y(), bound2.y()));
  // saturate(deltaT_, std::min(bound1.z(), bound2.z()), std::max(bound1.z(), bound2.z()));

  // std::cout << "std::min(bound1.x(), bound2.x()= " << std::min(bound1.x(), bound2.x()) << std::endl;
  // std::cout << "std::max(bound1.x(), bound2.x()= " << std::max(bound1.x(), bound2.x()) << std::endl;

  // std::cout << "std::min(bound1.y(), bound2.y()= " << std::min(bound1.y(), bound2.y()) << std::endl;
  // std::cout << "std::max(bound1.y(), bound2.y()= " << std::max(bound1.y(), bound2.y()) << std::endl;

  // std::cout << "std::min(bound1.z(), bound2.z()= " << std::min(bound1.z(), bound2.z()) << std::endl;
  // std::cout << "std::max(bound1.z(), bound2.z()= " << std::max(bound1.z(), bound2.z()) << std::endl;

  // std::cout << bold << "deltaT_ after= " << deltaT_ << reset << std::endl;

  t_final = t_init + (1.0 * (M_ - 2 * p_ - 1 + 1)) * deltaT_;

  t_init_ = t_init;
  t_final_ = t_final;

  /////////////////////////

  Eigen::RowVectorXd knots(M_ + 1);
  for (int i = 0; i <= p_; i++)
  {
    knots[i] = t_init_;
  }

  for (int i = (p_ + 1); i <= M_ - p_ - 1; i++)
  {
    knots[i] = knots[i - 1] + deltaT_;  // Assumming a uniform b-spline (internal knots are equally spaced)
  }

  for (int i = (M_ - p_); i <= M_; i++)
  {
    knots[i] = t_final_;
  }

  knots_ = knots;

  // std::cout << "knots_" << knots_ << std::endl;

  //////////////////

  // See https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
  // I think equation (7) of the paper "Robust and Efficent quadrotor..." has a typo, p_ is missing there (compare
  // with equation 15 of that paper)

  weight_modified_ = weight_ * (final_state_.pos - initial_state_.pos).norm();

  double t1 = knots_(1);
  double t2 = knots_(2);
  double tpP1 = knots_(p_ + 1);
  double t1PpP1 = knots_(1 + p_ + 1);

  double tN = knots_(N_);
  double tNm1 = knots_(N_ - 1);
  double tNPp = knots_(N_ + p_);
  double tNm1Pp = knots_(N_ - 1 + p_);

  // See Mathematica Notebook
  q0_ = p0;
  q1_ = p0 + (-t1 + tpP1) * v0 / p_;
  q2_ = (p_ * p_ * q1_ - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) - p_ * (q1_ + (-t1PpP1 + t2) * v0)) / ((-1 + p_) * p_);

  qN_ = pf;
  qNm1_ = pf + ((tN - tNPp) * vf) / p_;
  qNm2_ = (p_ * p_ * qNm1_ - (tNm1 - tNm1Pp) * (af * (-tN + tNm1Pp) + vf) - p_ * (qNm1_ + (-tNm1 + tNm1Pp) * vf)) /
          ((-1 + p_) * p_);

  /////////////////////////////////////////// FOR DEBUGGING

  // Eigen::MatrixXd knots_0 = knots_;
  // for (int i = 0; i < knots_0.cols(); i++)
  // {
  //   knots_0(0, i) = knots_(0, 0);
  // }
  // std::cout << "knots_ - knots[0]= " << std::endl;
  // std::cout << std::setprecision(13) << (knots_ - knots_0).transpose() << reset << std::endl;

  // int i = 1;
  // Eigen::Vector3d vcomputed_1 = p_ * (q2_ - q1_) / (knots_(i + p_ + 1) - knots_(i + 1));

  // i = 0;
  // Eigen::Vector3d vcomputed_0 = p_ * (q1_ - q0_) / (knots_(i + p_ + 1) - knots_(i + 1));
  // Eigen::Vector3d acomputed_0 = (p_ - 1) * (vcomputed_1 - vcomputed_0) / (knots_(i + p_ + 1) - knots_(i + 2));

  // // std::cout << "vcomputed_0= " << vcomputed_0.transpose() << std::endl;
  // // std::cout << "vcomputed_1= " << vcomputed_1.transpose() << std::endl;
  // // std::cout << "acomputed_0= " << acomputed_0.transpose() << std::endl;

  // double epsilon = 1.0001;

  // // TODO: remove this (it's not valid for Bezier/MINVO)
  // if ((vcomputed_1.array() > epsilon * v_max_.array()).any() || (vcomputed_1.array() < -epsilon *
  // v_max_.array()).any())
  // {
  //   std::cout << bold << red << "vel constraint for v1 is not satisfied" << reset << std::endl;

  //   abort();
  // }
  ///////////////////////////////////////////

  return true;
}

bool SolverGurobi::isADecisionCP(int i)

{  // If Q[i] is a decision variable
  return ((i >= 3) && i <= (N_ - 2));
}

double SolverGurobi::getTimeNeeded()
{
  return time_needed_;
}

int SolverGurobi::lastDecCP()
{
  return (N_ - 2);
}

void SolverGurobi::transformPosBSpline2otherBasis(const std::vector<std::vector<GRBVar>> &Qbs,
                                                  std::vector<std::vector<GRBLinExpr>> &Qmv, int interval)
{
  Qmv = matrixMultiply(Qbs, eigenMatrix2std(M_pos_bs2basis_[interval]));

  // Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverGurobi::transformPosBSpline2otherBasis(const std::vector<std::vector<GRBLinExpr>> &Qbs,
                                                  std::vector<std::vector<GRBLinExpr>> &Qmv, int interval)
{
  Qmv = matrixMultiply(Qbs, eigenMatrix2std(M_pos_bs2basis_[interval]));

  // Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverGurobi::transformPosBSpline2otherBasis(const Eigen::Matrix<double, 3, 4> &Qbs,
                                                  Eigen::Matrix<double, 3, 4> &Qmv, int interval)
{
  Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverGurobi::transformVelBSpline2otherBasis(const Eigen::Matrix<double, 3, 3> &Qbs,
                                                  Eigen::Matrix<double, 3, 3> &Qmv, int interval)
{
  Qmv = Qbs * M_vel_bs2basis_[interval];
}

// void SolverGurobi::transformVelBSpline2otherBasis(const std::vector<std::vector<Eigen::Vector3d>> &Qbs,
//                                                   std::vector<std::vector<Eigen::Vector3d>> &Qmv, int interval)
// {
//   Qmv = matrixMultiply(Qbs, M_vel_bs2basis_[interval]);
//   // Qmv = Qbs * M_vel_bs2basis_[interval];
// }

void SolverGurobi::addConstraints()
{
  // See here why we can use an epsilon of 1.0:
  // http://www.joyofdata.de/blog/testing-linear-separability-linear-programming-r-glpk/
  double epsilon = 1.0;

  /////////////////////////////////////////////////
  //////////// PLANES CONSTRAINTS    //////////////
  /////////////////////////////////////////////////

  for (int i = 0; i <= (N_ - 3); i++)  // i  is the interval (\equiv segment)
  {
    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      int ip = obst_index * num_of_segments_ + i;  // index plane

      // impose that all the vertexes of the obstacle are on one side of the plane
      // for (int j = 0; j < hulls_[obst_index][i].cols(); j++)  // Eigen::Vector3d vertex : hulls_[obst_index][i]
      // {
      //   Eigen::Vector3d vertex = hulls_[obst_index][i].col(j);
      //   m.addConstr((-(n_[ip].dot(vertex) + d_[ip] - epsilon)) <= 0, "plane" + std::to_string(j));
      //   // constraints[r] = -(n[ip].dot(vertex) + d[ip] - epsilon);  // f<=0
      // }

      // and the control points on the other side
      std::vector<std::vector<GRBVar>> Qbs;
      std::vector<std::vector<GRBLinExpr>> Qmv;  // other basis "basis_" (minvo, bezier,...).

      std::vector<GRBVar> tmp_x = { q_var_[i][0], q_var_[i + 1][0], q_var_[i + 2][0], q_var_[i + 3][0] };
      std::vector<GRBVar> tmp_y = { q_var_[i][1], q_var_[i + 1][1], q_var_[i + 2][1], q_var_[i + 3][1] };
      std::vector<GRBVar> tmp_z = { q_var_[i][2], q_var_[i + 1][2], q_var_[i + 2][2], q_var_[i + 3][2] };

      Qbs.push_back(tmp_x);  // row0
      Qbs.push_back(tmp_y);  // row1
      Qbs.push_back(tmp_z);  // row2

      transformPosBSpline2otherBasis(Qbs, Qmv, i);  // each row of Qmv will contain a "basis_" control point

      for (int u = 0; u <= 3; u++)
      {
        // q_ipu = Qmv.col(u);
        std::vector<GRBLinExpr> q_ipu = getColumn(Qmv, u);
        GRBLinExpr dot = n_[ip].x() * q_ipu[0] + n_[ip].y() * q_ipu[1] + n_[ip].z() * q_ipu[2];
        m.addConstr(dot + d_[ip] + epsilon <= 0);
        // constraints[r] = (n[ip].dot(q_ipu) + d[ip] + epsilon);  //  // fi<=0
      }
    }
  }

  /////////////////////////////////////////////////
  ////////// VELOCITY CONSTRAINTS    //////////////
  /////////////////////////////////////////////////

  for (int i = 2; i <= (N_ - 2); i++)  // If using BSpline basis, v0 and v1 are already determined by initial_state
  {
    double ciM2 = p_ / (knots_(i + p_ + 1 - 2) - knots_(i + 1 - 2));
    double ciM1 = p_ / (knots_(i + p_ + 1 - 1) - knots_(i + 1 - 1));
    double ci = p_ / (knots_(i + p_ + 1) - knots_(i + 1));

    std::vector<double> c;
    c.push_back(ciM2);
    c.push_back(ciM1);
    c.push_back(ci);

    auto v_iM2 = ciM2 * (q_var_[i - 1] - q_var_[i - 2]);
    auto v_iM1 = ciM1 * (q_var_[i] - q_var_[i - 1]);
    auto v_i = ci * (q_var_[i + 1] - q_var_[i]);

    std::vector<std::vector<GRBLinExpr>> Qbs;
    std::vector<std::vector<GRBLinExpr>> Qmv;  // other basis "basis_" (minvo, bezier,...).

    std::vector<GRBLinExpr> tmp_x = { v_iM2[0], v_iM1[0], v_i[0] };
    std::vector<GRBLinExpr> tmp_y = { v_iM2[1], v_iM1[1], v_i[1] };
    std::vector<GRBLinExpr> tmp_z = { v_iM2[2], v_iM1[2], v_i[2] };

    Qbs.push_back(tmp_x);  // row0
    Qbs.push_back(tmp_y);  // row1
    Qbs.push_back(tmp_z);  // row2

    // Qbs.col(0) = v_iM2;
    // Qbs.col(1) = v_iM1;
    // Qbs.col(2) = v_i;

    transformVelBSpline2otherBasis(Qbs, Qmv, i - 2);

    for (int j = 0; j < 3; j++)
    {  // loop over each of the velocity control points (v_{i-2+j}) of the new basis
      //|v_{i-2+j}| <= v_max ////// v_{i-2}, v_{i-1}, v_{i}

      // Eigen::Vector3d v_iM2Pj = Qmv.col(j);  // v_{i-2+j};
      // // Constraint v_{i-2+j} - vmax <= 0
      // assignEigenToVector(constraints, r, v_iM2Pj - v_max_);  // f<=0

      // // Constraint -v_{i-2+j} - vmax <= 0
      // assignEigenToVector(constraints, r, -v_iM2Pj - v_max_);  // f<=0

      std::vector<GRBExpr> v_iM2Pj = getColumn(Qmv, j);
      for (auto tmp : v_iM2Pj)
      {
        m.addConstr(tmp - v_max_ <= 0);
        m.addConstr(-tmp - v_max_ <= 0);
      }
    }
  }
}

// // m is the number of constraints, nn is the number of variables
// void SolverGurobi::computeConstraints(unsigned m, double *constraints, unsigned nn, double *grad,
//                                       const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
//                                       const std::vector<double> &d)
// {
//   Eigen::Vector3d ones = Eigen::Vector3d::Ones();
//   int r = 0;

//   index_const_obs_ = r;

//   /////////////////////////////////////////////////
//   ////////// ACCELERATION CONSTRAINTS    //////////
//   /////////////////////////////////////////////////

//   for (int i = 1; i <= (N_ - 3); i++)  // a0 is already determined by the initial state

//   {
//     double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
//     double c2 = p_ / (knots_(i + p_ + 1 + 1) - knots_(i + 1 + 1));
//     double c3 = (p_ - 1) / (knots_(i + p_ + 1) - knots_(i + 2));

//     Eigen::Vector3d v_i = c1 * (q[i + 1] - q[i]);
//     Eigen::Vector3d v_iP1 = c2 * (q[i + 2] - q[i + 1]);
//     Eigen::Vector3d a_i = c3 * (v_iP1 - v_i);

//     // a<=amax  ==  a_i - amax <= 0  ==  c3 * (v_iP1 - v_i)<=0 ==
//     // c3*c2 *q[i + 2] - c3*c2* q[i + 1]  -  c3*c1*q[i + 1] + c3*c1*q[i]   - amax <= 0

//     // if (((a_i - a_max_).array() > Eigen::Vector3d::Zero().array()).any())
//     // {
//     //   std::cout << "__________________ i= " << i << " _________________" << std::endl;

//     //   std::cout << "v_i= " << v_i.transpose() << std::endl;
//     //   std::cout << "a_i= " << a_i.transpose() << std::endl;
//     //   std::cout << "v_iP1= " << v_iP1.transpose() << std::endl;

//     //   std::cout << "r= " << r << ", a_i= " << a_i.transpose() << "  a_max_=" << a_max_.transpose() << std::endl;
//     // }

//     assignEigenToVector(constraints, r, a_i - a_max_);  // f<=0
//     if (grad)
//     {
//       if (isADecisionCP(i))  // If Q[i] is a decision variable
//       {
//         toGradDiffConstraintsDiffVariables(gIndexQ(i), c3 * c1 * ones, grad, r, nn);
//       }

//       if (i == (N_ - 3))
//       {  // this is needed because qN=qNm1=qNm2 (and therefore the gradient of qNm2 should take into account also
//       qNm1)
//         toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), (-c3 * c2 - c3 * c1) * ones + c3 * c2 * ones, grad, r,
//         nn);
//       }
//       else
//       {
//         if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
//         {
//           toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), (-c3 * c2 - c3 * c1) * ones, grad, r, nn);
//         }
//         if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
//         {
//           toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), c3 * c2 * ones, grad, r, nn);
//         }
//       }
//     }
//     r = r + 3;

//     assignEigenToVector(constraints, r, -a_i - a_max_);  // f<=0
//     if (grad)
//     {
//       if (isADecisionCP(i))  // If Q[i] is a decision variable
//       {
//         toGradDiffConstraintsDiffVariables(gIndexQ(i), -c3 * c1 * ones, grad, r, nn);
//       }

//       if (i == (N_ - 3))
//       {
//         toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -(-c3 * c2 - c3 * c1) * ones - c3 * c2 * ones, grad, r,
//         nn);
//       }
//       else
//       {
//         if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
//         {
//           toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -(-c3 * c2 - c3 * c1) * ones, grad, r, nn);
//         }
//         if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
//         {
//           toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), -c3 * c2 * ones, grad, r, nn);
//         }
//       }
//     }
//     r = r + 3;
//   }

//   index_const_normals_ = r;

//   /////////////////////////////////////////////////
//   //////////// SPHERE CONSTRAINTS    /////////////
//   /////////////////////////////////////////////////
//   // Impose that all the control points are inside an sphere of radius Ra

//   // for (int i = 2; i <= N_ - 2; i++)  // Asumming q0_, q1_ and q2_ are inside the sphere TODO: Add this check
//   //                                    // when computing q0_, q1_ and q2_ (and return false if not)
//   // {
//   //   constraints[r] = (q[i] - q0_).squaredNorm() - Ra_ * Ra_;  // fi<=0
//   //   if (grad)
//   //   {
//   //     toGradSameConstraintDiffVariables(gIndexQ(i), 2 * (q[i] - q0_), grad, r, nn);
//   //   }

//   for (int i = 0; i <= (N_ - 3); i++)  // i  is the interval (\equiv segment)
//   {
//     Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
//     Eigen::Matrix<double, 3, 4> Qmv;  // other basis "basis_" (minvo, bezier,...).
//     Qbs.col(0) = q[i];
//     Qbs.col(1) = q[i + 1];
//     Qbs.col(2) = q[i + 2];
//     Qbs.col(3) = q[i + 3];

//     transformPosBSpline2otherBasis(Qbs, Qmv,
//                                    i);  // Now Qmv is a matrix whose each row contains a "basis_" control point

//     Eigen::Vector3d q_ipu;
//     for (int u = 0; u <= 3; u++)
//     {
//       q_ipu = Qmv.col(u);  // if using the MINVO basis

//       constraints[r] = (q_ipu - q0_).squaredNorm() - Ra_ * Ra_;  // fi<=0

//       if (grad)
//       {
//         for (int k = 0; (k <= 3); k++)
//         // This loop is needed because each q in MINVO depends on every q in BSpline
//         {
//           if ((i + 3) == (N_ - 1) && k == 2)
//           {  // The last control point of the interval is qNm1, and the variable is qNm2
//             // Needed because qN=qNm1=qNm2
//             toGradSameConstraintDiffVariables(
//                 gIndexQ(i + k), 2 * (q_ipu - q0_) * (M_pos_bs2basis_[i](k, u) + M_pos_bs2basis_[i](k + 1, u)), grad,
//                 r, nn);
//           }

//           else if ((i + 3) == (N_) && k == 1)
//           {  // The last 2 control points of the interval are qNm1 and qN, and the variable is qNm2
//             // Needed because qN=qNm1=qNm2
//             toGradSameConstraintDiffVariables(
//                 gIndexQ(i + k),
//                 2 * (q_ipu - q0_) *
//                     (M_pos_bs2basis_[i](k, u) + M_pos_bs2basis_[i](k + 1, u) + M_pos_bs2basis_[i](k + 2, u)),
//                 grad, r, nn);
//           }

//           else
//           {
//             if (isADecisionCP(i + k))  // If Q[i] is a decision variable
//             {
//               toGradSameConstraintDiffVariables(gIndexQ(i + k), 2 * (q_ipu - q0_) * M_pos_bs2basis_[i](k, u), grad,
//               r,
//                                                 nn);
//             }
//           }
//         }
//       }
//       r++;
//     }
//   }

//   num_of_constraints_ = r;  // + 1 has already been added in the last loop of the previous for loop;
// }

void SolverGurobi::printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  std::cout << std::setprecision(5) << std::endl;
  std::cout << "   control points:" << std::endl;
  for (Eigen::Vector3d q_i : q)
  {
    std::cout << q_i.transpose() << std::endl;
  }
  std::cout << "   normals:" << std::endl;
  for (Eigen::Vector3d n_i : n)
  {
    std::cout << n_i.transpose() << std::endl;
  }
  std::cout << "   d coeffs:" << std::endl;
  for (double d_i : d)
  {
    std::cout << d_i << std::endl;
  }
  std::cout << reset << std::endl;
}

void SolverGurobi::generateRandomGuess()
{
  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();

  generateRandomN(n_guess_);
  generateRandomD(d_guess_);
  generateRandomQ(q_guess_);
}

void SolverGurobi::printStd(const std::vector<double> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i << std::endl;
  }
}

void SolverGurobi::printStd(const std::vector<Eigen::Vector3d> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i.transpose() << std::endl;
  }
}

bool SolverGurobi::optimize()
{
  generateAStarGuess();
  n_ = n_guess_;
  d_ = d_guess_;

  prepareObjective();
  addConstraints();

  m.update();
  m.write("/home/jtorde/Desktop/ws/src/mader/model.lp");
  m.optimize();
}

// bool SolverGurobi::optimize()

// {
//   // reset some stuff
//   traj_solution_.clear();
//   best_cost_so_far_ = std::numeric_limits<double>::max();
//   best_feasible_sol_so_far_.clear();
//   got_a_feasible_solution_ = false;
//   best_feasible_sol_so_far_.resize(num_of_variables_);

//   // note that, for a v0 and a0 given, q2_ is not guaranteed to lie within the bounds. If that's the case --> keep
//   // executing previous trajectory
//   if (q2_.x() > x_max_ || q2_.x() < x_min_ ||  //////////////
//       q2_.y() > y_max_ || q2_.y() < y_min_ ||  /////////////////
//       q2_.z() > z_max_ || q2_.z() < z_min_)
//   {
//     std::cout << bold << red << "q2_ is not in [min, max]" << reset << std::endl;
//     std::cout << "q2_= " << q2_.transpose() << std::endl;
//     std::cout << "x_min_= " << x_min_ << ", x_max_=" << x_max_ << std::endl;
//     std::cout << "y_min_= " << y_min_ << ", y_max_=" << y_max_ << std::endl;
//     std::cout << "z_min_= " << z_min_ << ", z_max_=" << z_max_ << std::endl;
//     return false;
//   }

//   generateAStarGuess();

//   // https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/#augmented-lagrangian-algorithm
//   // The augmented Lagrangian method is specified in NLopt as NLOPT_AUGLAG. We also provide a variant,
//   NLOPT_AUGLAG_EQ,
//   // that only uses penalty functions for equality constraints, while inequality constraints are passed through to
//   the
//   // subsidiary algorithm to be handled directly; in this case, the subsidiary algorithm must handle inequality
//   // constraints (e.g. MMA or COBYLA)

//   nlopt::opt opt(nlopt::AUGLAG, num_of_variables_);  // need to create it here because I need the # of variables
//   nlopt::opt local_opt(solver_, num_of_variables_);  // need to create it here because I need the # of variables

//   // opt_ = new nlopt::opt(nlopt::AUGLAG_EQ, num_of_variables_);  // nlopt::AUGLAG
//   // local_opt_ = new nlopt::opt(solver_, num_of_variables_);

//   local_opt.set_xtol_rel(xtol_rel_);  // stopping criteria.
//   local_opt.set_ftol_rel(ftol_rel_);  // stopping criteria.
//   local_opt.set_maxtime(std::max(mu_ * max_runtime_, 0.001));

//   opt.set_local_optimizer(local_opt);
//   opt.set_xtol_rel(xtol_rel_);  // Stopping criteria.
//   opt.set_ftol_rel(ftol_rel_);  // Stopping criteria.

//   // opt.set_maxeval(1e6);  // maximum number of evaluations. Negative --> don't use this criterion

//   opt.set_maxtime(std::max(mu_ * max_runtime_, 0.001));  // 0.001 to make sure this criterion is used  // maximum
//   time
//                                                          // in seconds. Negative --> don't use this criterion

//   // opt.set_maxtime(max_runtime_);
//   initializeNumOfConstraints();

//   // see https://github.com/stevengj/nlopt/issues/168
//   std::vector<double> tol_constraints;
//   for (int i = 0; i < num_of_constraints_; i++)
//   {
//     tol_constraints.push_back(epsilon_tol_constraints_);
//   }

//   // andd lower and upper bounds
//   std::vector<double> lb;
//   std::vector<double> ub;
//   // control points q
//   for (int i = 0; i <= i_max_; i = i + 3)
//   {
//     // x component
//     lb.push_back(x_min_);
//     ub.push_back(x_max_);
//     // y component
//     lb.push_back(y_min_);
//     ub.push_back(y_max_);
//     // z component
//     lb.push_back(z_min_);
//     ub.push_back(z_max_);
//   }
//   // normals n
//   for (int j = j_min_; j <= j_max_; j++)
//   {
//     lb.push_back(-1e9);
//     ub.push_back(1e9);
//   }
//   // coefficients d
//   for (int k = k_min_; k <= k_max_; k++)
//   {
//     lb.push_back(-1e9);
//     ub.push_back(1e9);
//   }
//   // opt.set_lower_bounds(lb);
//   // opt.set_upper_bounds(ub);
//   // // if (local_opt_)
//   // // {
//   // local_opt.set_lower_bounds(lb);
//   // local_opt.set_upper_bounds(ub);
//   // }

//   // set constraint and objective
//   opt.add_inequality_mconstraint(SolverGurobi::myIneqConstraints, this, tol_constraints);
//   opt.set_min_objective(SolverGurobi::myObjFunc,
//                         this);  // this is passed as a parameter (the obj function has to be static)

//   qnd2x(q_guess_, n_guess_, d_guess_, x_);

//   double obj_guess = computeObjFunctionJerk(num_of_variables_, nullptr, q_guess_, n_guess_, d_guess_);

//   double second_term_obj_guess = weight_modified_ * (q_guess_[N_] - final_state_.pos).squaredNorm();
//   double first_term_obj_guess = obj_guess - second_term_obj_guess;

//   // std::cout << "The guess is the following one:" << std::endl;
//   // printQVA(q_guess_);
//   // printQND(q_guess_, n_guess_, d_guess_);

//   // x2qnd(x_, q_guess_, n_guess_);

//   std::cout << bold << "The infeasible constraints of the initial Guess" << reset << std::endl;

//   printInfeasibleConstraints(q_guess_, n_guess_, d_guess_);

//   printIndexesConstraints();
//   printIndexesVariables();

//   double obj_obtained;

//   opt_timer_.Reset();
//   std::cout << "[NL] Optimizing now, allowing time = " << mu_ * max_runtime_ * 1000 << "ms" << std::endl;

//   int result;
//   try
//   {
//     result = opt.optimize(x_, obj_obtained);
//   }
//   catch (...)
//   {
//     std::cout << bold << red << "An exception occurred in the solver" << reset << std::endl;
//     return false;
//   }
//   // std::cout << "[NL] Finished optimizing " << std::endl;

//   time_needed_ = opt_timer_.ElapsedMs() / 1000;

//   num_of_QCQPs_run_++;

//   // See codes in https://github.com/JuliaOpt/NLopt.jl/blob/master/src/NLopt.jl
//   // got_a_feasible_solution_ = got_a_feasible_solution_;
//   bool optimal = (result == nlopt::SUCCESS);
//   bool failed = (!optimal) && (!got_a_feasible_solution_);
//   bool feasible_but_not_optimal = (!optimal) && (got_a_feasible_solution_);

//   // Store the results here
//   std::vector<Eigen::Vector3d> q;
//   std::vector<Eigen::Vector3d> n;
//   std::vector<double> d;

//   // std::cout << "[NL] result= " << getResultCode(result) << std::endl;

//   if (failed == false)
//   {
//   }

//   if (failed)
//   {
//     ROS_ERROR_STREAM("[NL] Failed, code=" << getResultCode(result));

//     printf("[NL] nlopt failed or maximum time was reached!\n");

//     std::cout << bold << red << "[NL] Solution not found" << opt_timer_ << reset << std::endl;  // on_red

//     // x2qnd(x_, q, n, d);
//     // printInfeasibleConstraints(q, n, d);

//     return false;
//   }
//   else if (optimal)
//   {
//     ROS_INFO_STREAM("[NL] Optimal, code=" << getResultCode(result));

//     std::cout << on_green << bold << "[NL] Optimal Solution found in" << opt_timer_ << reset << std::endl;
//     x2qnd(x_, q, n, d);

//     // std::cout << "q[N_]= " << q[N_].transpose() << std::endl;

//     double second_term_obj_obtained = weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
//     double first_term_obj_obtained = obj_obtained - second_term_obj_obtained;
//     // std::cout << "obj_guess= " << std::setprecision(7) << obj_guess << reset << "(" << first_term_obj_guess << " +
//     "
//     //           << second_term_obj_guess << ")" << std::endl;
//     // std::cout << "obj_obtained= " << std::setprecision(7) << obj_obtained << reset << "(" <<
//     first_term_obj_obtained
//     //           << " + " << second_term_obj_obtained << ")" << std::endl;

//     improvement_ = 100 * (1.0 - (obj_obtained / obj_guess));
//     // std::cout << green << std::setprecision(7) << "Improvement: " << 100 * (1.0 - (obj_obtained / obj_guess)) <<
//     "%"
//     //           << reset << std::endl;
//   }
//   else if (feasible_but_not_optimal)
//   {
//     ROS_INFO_STREAM("[NL] Feasible, code=" << getResultCode(result));

//     std::cout << on_green << bold << "[NL] Feasible Solution found" << opt_timer_ << reset << std::endl;
//     x2qnd(best_feasible_sol_so_far_, q, n, d);  // was

//     std::cout << "q[N_]= " << q[N_].transpose() << std::endl;

//     double second_term_obj_obtained = weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
//     double first_term_obj_obtained = best_cost_so_far_ - second_term_obj_obtained;
//     std::cout << "obj_guess= " << std::setprecision(7) << obj_guess << reset << "(" << first_term_obj_guess << " + "
//               << second_term_obj_guess << ")" << std::endl;
//     std::cout << "obj_obtained= " << std::setprecision(7) << best_cost_so_far_ << reset << "("
//               << first_term_obj_obtained << " + " << second_term_obj_obtained << ")" << std::endl;

//     improvement_ = 100 * (1.0 - (obj_obtained / obj_guess));

//     // std::cout << green << std::setprecision(7) << "Improvement: " << improvement_ << "%" << reset << std::endl;
//   }
//   else
//   {
//     std::cout << on_red << bold << "[NL] not implemented yet" << opt_timer_ << reset << std::endl;
//     abort();
//     return false;
//   }

//   // printQND(q, n, d);

//   /*  std::cout << on_green << bold << "Solution found: " << time_first_feasible_solution_ << "/" << opt_timer_ <<
//      reset
//               << std::endl;*/

//   CPs2TrajAndPwp(q, traj_solution_, solution_, N_, p_, num_pol_, knots_, dc_);
//   ///////////////

//   // Force the last position to be the final_state_ (it's not guaranteed to be because of the discretization with
//   dc_) traj_solution_.back().vel = final_state_.vel; traj_solution_.back().accel = final_state_.accel;
//   traj_solution_.back().jerk = Eigen::Vector3d::Zero();

//   return true;
// }

void SolverGurobi::getSolution(PieceWisePol &solution)
{
  solution = solution_;
}

void SolverGurobi::saturateQ(std::vector<Eigen::Vector3d> &q)
{
  for (int i = 0; i < q.size(); i++)
  {
    q[i].z() = std::max(q[i].z(), z_min_);  // Make sure it's within the limits
    q[i].z() = std::min(q[i].z(), z_max_);  // Make sure it's within the limits
  }
}

void SolverGurobi::generateStraightLineGuess()
{
  // std::cout << "Using StraightLineGuess" << std::endl;
  q_guess_.clear();
  n_guess_.clear();
  d_guess_.clear();

  q_guess_.push_back(q0_);  // Not a decision variable
  q_guess_.push_back(q1_);  // Not a decision variable
  q_guess_.push_back(q2_);  // Not a decision variable

  for (int i = 1; i < (N_ - 2 - 2); i++)
  {
    Eigen::Vector3d q_i = q2_ + i * (final_state_.pos - q2_) / (N_ - 2 - 2);
    q_guess_.push_back(q_i);
  }

  q_guess_.push_back(qNm2_);  // three last cps are the same because of the vel/accel final conditions
  q_guess_.push_back(qNm1_);
  q_guess_.push_back(qN_);
  // Now q_guess_ should have (N_+1) elements
  saturateQ(q_guess_);  // make sure is inside the bounds specified

  // std::vector<Eigen::Vector3d> q_guess_with_qNm1N = q_guess_;
  // q_guess_with_qNm1N.push_back(qNm1_);
  // q_guess_with_qNm1N.push_back(qN_);
  //////////////////////

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      std::vector<Eigen::Vector3d> last4Cps(4);

      Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
      Eigen::Matrix<double, 3, 4> Qmv;  // minvo. each column contains a MINVO control point
      Qbs.col(0) = q_guess_[i];
      Qbs.col(1) = q_guess_[i + 1];
      Qbs.col(2) = q_guess_[i + 2];
      Qbs.col(3) = q_guess_[i + 3];

      transformPosBSpline2otherBasis(Qbs, Qmv, i);

      Eigen::Vector3d n_i;
      double d_i;

      bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][i], Qmv);

      n_guess_.push_back(n_i);
      d_guess_.push_back(d_i);
    }
  }
}

void SolverGurobi::printIndexesVariables()
{
  std::cout << "_______________________" << std::endl;
  std::cout << "Indexes variables" << std::endl;
  std::cout << "q: " << i_min_ << "-->" << i_max_ << std::endl;
  std::cout << "n: " << j_min_ << "-->" << j_max_ << std::endl;
  std::cout << "d: " << k_min_ << "-->" << k_max_ << std::endl;

  std::cout << "Total number of Variables: " << num_of_variables_ << std::endl;
  std::cout << "_______________________" << std::endl;
}

void SolverGurobi::printIndexesConstraints()
{
  std::cout << "_______________________" << std::endl;
  std::cout << "Indexes constraints" << std::endl;
  std::cout << "Obstacles: " << index_const_obs_ << "-->" << index_const_vel_ - 1 << std::endl;
  std::cout << "Velocity: " << index_const_vel_ << "-->" << index_const_accel_ - 1 << std::endl;
  std::cout << "Accel: " << index_const_accel_ << "-->" << index_const_normals_ - 1 << std::endl;
  // std::cout << "Normals: " << index_const_normals_ << "-->" << num_of_constraints_ << std::endl;
  std::cout << "Total number of Constraints: " << num_of_constraints_ << std::endl;
  std::cout << "_______________________" << std::endl;
}

// void SolverGurobi::printQVA(const std::vector<Eigen::Vector3d> &q)
// {
//   std::cout << red << "Position cps, " << blue << "Velocity cps, " << green << "Accel cps" << reset << std::endl;

//   std::cout << "BSPLINE Basis: " << std::endl;

//   std::vector<Eigen::Vector3d> cps_vel;

//   for (int i = 0; i <= (N_ - 2); i++)
//   {
//     Eigen::Vector3d vi = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
//     Eigen::Vector3d vip1 = p_ * (q[i + 1 + 1] - q[i + 1]) / (knots_(i + 1 + p_ + 1) - knots_(i + 1 + 1));
//     Eigen::Vector3d ai = (p_ - 1) * (vip1 - vi) / (knots_(i + p_ + 1) - knots_(i + 2));

//     cps_vel.push_back(vi);

//     std::cout << "***i= " << i << red << q[i].transpose() << blue << "   " << vi.transpose() << green << "   "
//               << ai.transpose() << reset << std::endl;
//   }

//   int i = N_ - 1;

//   Eigen::Vector3d vi = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
//   std::cout << "***i= " << i << red << q[i].transpose() << blue << "   " << vi.transpose() << reset << std::endl;
//   cps_vel.push_back(vi);

//   i = N_;

//   std::cout << "***i= " << i << red << q[i].transpose() << reset << std::endl;

//   std::cout << std::endl;
//   if (basis_ == MINVO)
//   {
//     std::cout << "MINVO Basis: " << std::endl;

//     std::cout << blue << "Position cps" << reset << std::endl;

//     for (int j = 3; j < q.size(); j++)
//     {  // Note that for each interval there are 3 cps, but in the next interval they'll be different
//       int interval = j - 3;

//       Eigen::Matrix<double, 3, 4> Qbs;
//       Eigen::Matrix<double, 3, 4> Qmv;

//       Qbs.col(0) = q[j - 3];
//       Qbs.col(1) = q[j - 2];
//       Qbs.col(2) = q[j - 1];
//       Qbs.col(3) = q[j];

//       transformPosBSpline2otherBasis(Qbs, Qmv, interval);

//       std::cout << "***i= " << interval << std::endl;
//       std::cout << Qmv << std::endl;

//       std::cout << "norm= " << std::endl;
//       for (int jj = 0; jj < Qmv.cols(); jj++)
//       {
//         std::cout << (Qmv.col(jj) - q0_).norm() << ", ";
//       }
//       std::cout << std::endl;
//     }

//     /////

//     std::cout << blue << "Velocity cps" << reset << std::endl;

//     for (int j = 2; j < cps_vel.size(); j++)
//     {  // Note that for each interval there are 3 cps, but in the next interval they'll be different
//       int interval = j - 2;

//       Eigen::Matrix<double, 3, 3> Qbs;
//       Eigen::Matrix<double, 3, 3> Qmv;

//       Qbs.col(0) = cps_vel[j - 2];
//       Qbs.col(1) = cps_vel[j - 1];
//       Qbs.col(2) = cps_vel[j];

//       transformVelBSpline2otherBasis(Qbs, Qmv, interval);

//       std::cout << "***i= " << interval << std::endl;
//       std::cout << Qmv << std::endl;
//     }
//   }
// }