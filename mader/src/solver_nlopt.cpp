/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "solver_nlopt.hpp"
#include "termcolor.hpp"
#include "bspline_utils.hpp"
#include "ros/ros.h"

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

SolverNlopt::SolverNlopt(ms::par_solver &par)
{
  deg_pol_ = par.deg_pol;
  num_pol_ = par.num_pol;

  p_ = deg_pol_;
  M_ = num_pol_ + 2 * p_;
  N_ = M_ - p_ - 1;
  num_of_segments_ = (M_ - 2 * p_);  // this is the same as num_pol_

  ///////////////////////////////////////
  ///////////////////////////////////////

  mt::basisConverter basis_converter;

  std::cout << "In the SolverNlopt Constructor\n";

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
  solver_ = getSolver(par.solver);                         // getSolver(nlopt::LD_VAR2);
  epsilon_tol_constraints_ = par.epsilon_tol_constraints;  // 1e-1;
  xtol_rel_ = par.xtol_rel;                                // 1e-1;
  ftol_rel_ = par.ftol_rel;                                // 1e-1;

  weight_ = par.weight;

  separator_solver_ = new separator::Separator();

  octopusSolver_ = new OctopusSearch(par.basis, num_pol_, deg_pol_, par.alpha_shrink);
}

SolverNlopt::~SolverNlopt()
{
  // delete opt_;
  // delete local_opt_;
}

void SolverNlopt::getPlanes(std::vector<Hyperplane3D> &planes)
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

int SolverNlopt::getNumOfLPsRun()
{
  return num_of_LPs_run_;
}

int SolverNlopt::getNumOfQCQPsRun()
{
  return num_of_QCQPs_run_;
}

/*bool SolverNlopt::intersects()
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point;
  typedef K::Plane_3 Plane;
  typedef K::Vector_3 Vector;
  typedef K::Segment_3 Segment;
  typedef K::Ray_3 Ray;
  typedef CGAL::Polyhedron_3<K> Polyhedron;
  typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type> Segment_intersection;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type> Plane_intersection;
  typedef Tree::Primitive_id Primitive_id;

  Point p(1.0, 0.0, 0.0);
  Point q(0.0, 1.0, 0.0);
  Point r(0.0, 0.0, 1.0);
  Point s(0.0, 0.0, 0.0);
  Polyhedron polyhedron;
  polyhedron.make_tetrahedron(p, q, r, s);
  // constructs AABB tree
  Tree tree(faces(polyhedron).first, faces(polyhedron).second, polyhedron);
  // constructs segment query
  Point a(-0.2, 0.2, -0.2);
  Point b(1.3, 0.2, 1.3);
  Segment segment_query(a, b);
  // tests intersections with segment query
  if (tree.do_intersect(segment_query))
    std::cout << "intersection(s)" << std::endl;
  else
    std::cout << "no intersection" << std::endl;
}*/

/*
Correlated 3D Gaussian distribution
#include "./../../eigenmvn.hpp"
Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d covar = rot * Eigen::DiagonalMatrix<double, 3, 3>(0.005, 0.005, 0.001) * rot.transpose();
    Eigen::EigenMultivariateNormal<double> normX_solver(mean, covar);  // or normX_cholesk(mean, covar, true);
    tmp = normX_solver.samples(1).transpose();*/

// std::cout << "obst index=" << obst_index << "sample= " << tmp.transpose() << std::endl;

/*        for (int j = i; j >= (i - 3); j--)  // Q3 needs to check against polyh0, polyh1 and polyh2 of all the
   obstacles
        {
          // std::cout << "j=" << j << std::endl;

          int ip = obst_index * num_of_segments_ + j;  // index poly

          if (polyhedra[ip].inside(tmp) == true)
          {
            goto initloop;
          }
        }*/

void SolverNlopt::setMaxRuntimeKappaAndMu(double max_runtime, double kappa, double mu)
{
  kappa_ = kappa;
  mu_ = mu;
  max_runtime_ = max_runtime;
}

void SolverNlopt::fillPlanesFromNDQ(std::vector<Hyperplane3D> &planes_, const std::vector<Eigen::Vector3d> &n,
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

void SolverNlopt::generateAStarGuess()
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

  octopusSolver_->setUp(t_init_, t_final_, hulls_);

  // std::cout << "q0_=" << q0_.transpose() << std::endl;
  // std::cout << "q1_=" << q1_.transpose() << std::endl;
  // std::cout << "q2_=" << q2_.transpose() << std::endl;

  octopusSolver_->setq0q1q2(q0_, q1_, q2_);
  octopusSolver_->setGoal(final_state_.pos);

  // double runtime = 0.05;   //[seconds]
  double goal_size = 0.05;  //[meters]

  octopusSolver_->setXYZMinMaxAndRa(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
                                    Ra_);                 // z limits for the search, in world frame
  octopusSolver_->setBBoxSearch(2000.0, 2000.0, 2000.0);  // limits for the search, centered on q2
  octopusSolver_->setMaxValuesAndSamples(v_max_, a_max_, a_star_samp_x_, a_star_samp_y_, a_star_samp_z_,
                                         a_star_fraction_voxel_size_);

  octopusSolver_->setRunTime(kappa_ * max_runtime_);  // hack, should be kappa_ * max_runtime_
  octopusSolver_->setGoalSize(goal_size);

  octopusSolver_->setBias(a_star_bias_);
  // if (basis_ == MINVO)
  // {
  //   // std::cout << green << bold << "snlopt is using MINVO" << reset << std::endl;
  //   octopusSolver_->setBasisUsedForCollision(octopusSolver_->MINVO);
  // }
  // else if (basis_ == BEZIER)
  // {
  //   // std::cout << green << bold << "snlopt is using BEZIER" << reset << std::endl;
  //   octopusSolver_->setBasisUsedForCollision(octopusSolver_->BEZIER);
  // }
  // else
  // {
  //   // std::cout << green << bold << "snlopt is using B_SPLINE" << reset << std::endl;
  //   octopusSolver_->setBasisUsedForCollision(octopusSolver_->B_SPLINE);
  // }

  octopusSolver_->setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool is_feasible = octopusSolver_->run(q, n, d);

  num_of_LPs_run_ = octopusSolver_->getNumOfLPsRun();
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

void SolverNlopt::generateRandomD(std::vector<double> &d)
{
  d.clear();
  for (int k = k_min_; k <= k_max_; k++)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    d.push_back(r1);
  }
}

void SolverNlopt::generateRandomN(std::vector<Eigen::Vector3d> &n)
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

void SolverNlopt::generateRandomQ(std::vector<Eigen::Vector3d> &q)
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

void SolverNlopt::findCentroidHull(const mt::Polyhedron_Std &hull, Eigen::Vector3d &centroid)
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

void SolverNlopt::generateGuessNDFromQ(const std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
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

void SolverNlopt::assignEigenToVector(double *result, int var_gindex, const Eigen::Vector3d &tmp)

{
  /*  std::cout << "i= " << var_gindex << std::endl;
    std::cout << "i= " << var_gindex + 1 << std::endl;
    std::cout << "i= " << var_gindex + 2 << std::endl;*/
  result[var_gindex] = tmp(0);
  result[var_gindex + 1] = tmp(1);
  result[var_gindex + 2] = tmp(2);
}

// r is the constraint index
// nn is the number of variables
// var_gindex is the index of the variable of the first element of the vector
void SolverNlopt::toGradDiffConstraintsDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,
                                                     int nn)

{
  grad[r * nn + var_gindex] = tmp(0);
  grad[(r + 1) * nn + var_gindex + 1] = tmp(1);
  grad[(r + 2) * nn + var_gindex + 2] = tmp(2);
}

void SolverNlopt::toGradSameConstraintDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,
                                                    int nn)

{
  grad[r * nn + var_gindex] = tmp(0);
  grad[r * nn + var_gindex + 1] = tmp(1);
  grad[r * nn + var_gindex + 2] = tmp(2);
}

void SolverNlopt::setHulls(mt::ConvexHullsOfCurves_Std &hulls)

{
  hulls_.clear();
  hulls_ = hulls;

  /*  for (int i = 0; i < hulls_.size(); i++)  // i  is the interval (\equiv segment)  //hulls_.size()
    {
      // impose that all the vertexes are on one side of the plane

      // std::cout << "Interval " << i << " has" << std::endl;
      for (int vertex_index = 0; vertex_index < hulls_[i].size(); vertex_index++)  // hulls_[i][vertex_index].size()
      {
        Eigen::Vector3d vertex = hulls_[i][vertex_index];
        std::cout << "Vertex = " << vertex.transpose() << std::endl;
      }
    }*/

  num_of_obst_ = hulls_.size();

  i_min_ = 0;
  i_max_ =
      3 * (N_ + 1) - 1 - 9 - 6;  // because pos, vel and accel at t_init_ and t_final_ are fixed (not dec variables)
  j_min_ = i_max_ + 1;
  j_max_ = j_min_ + 3 * (M_ - 2 * p_) * num_of_obst_ - 1;
  k_min_ = j_max_ + 1;
  k_max_ = k_min_ + (M_ - 2 * p_) * num_of_obst_ - 1;

  num_of_variables_ = k_max_ + 1;  // k_max_ + 1;

  int num_of_cpoints = N_ + 1;

  num_of_normals_ = num_of_segments_ * num_of_obst_;
}

template <class T>
void SolverNlopt::printInfeasibleConstraints(const T &constraints)
{
  std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints[i] << std::endl;
    }
  }
}

template <class T>
bool SolverNlopt::areTheseConstraintsFeasible(const T &constraints)
{
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      return false;
    }
  }
  return true;
}

template <class T>
int SolverNlopt::getNumberOfInfeasibleConstraints(const T &constraints)
{
  int result = 0;

  for (int i = 0; i < num_of_constraints_; i++)
  {
    std::cout << "i= " << i << std::endl;
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      result = result + 1;
    }
  }
  return result;
}

template <class T>
bool SolverNlopt::isFeasible(const T x)
{
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  x2qnd(x, q, n, d);
  // computeConstraints(0, constraints_, num_of_variables_, nullptr, q, n, d);

  // // std::cout << "The Infeasible Constraints are these ones:\n";
  // for (int i = 0; i < num_of_constraints_; i++)
  // {
  //   if (constraints_[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
  //   {
  //     return false;
  //     // std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
  //   }
  // }
  return isFeasible(q, n, d);
}

bool SolverNlopt::isFeasible(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                             const std::vector<double> &d)
{
  double constraints[num_of_constraints_];
  computeConstraints(0, constraints, num_of_variables_, nullptr, q, n, d);

  // std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      return false;
      // std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
    }
  }
  return true;
}

/*template <class T>
void SolverNlopt::printInfeasibleConstraints(const T x)
{
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  x2qnd(x, q, n, d);
  computeConstraints(0, constraints_, num_of_variables_, nullptr, q, n, d);

  std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints_[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
    }
  }
}*/

void SolverNlopt::printInfeasibleConstraints(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                             std::vector<double> &d)
{
  double constraints[num_of_constraints_];
  computeConstraints(0, constraints, num_of_variables_, nullptr, q, n, d);

  std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints[i] << std::endl;
    }
  }
}

void SolverNlopt::qnd2x(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                        const std::vector<double> &d, std::vector<double> &x)
{
  x.clear();

  // std::cout << "q has size= " << q.size() << std::endl;
  // std::cout << "n has size= " << n.size() << std::endl;

  for (int i = 3; i <= lastDecCP(); i++)
  {
    x.push_back(q[i](0));
    x.push_back(q[i](1));
    x.push_back(q[i](2));
  }

  for (auto n_i : n)
  {
    x.push_back(n_i(0));
    x.push_back(n_i(1));
    x.push_back(n_i(2));
  }

  for (auto d_i : d)
  {
    x.push_back(d_i);
  }
}

void SolverNlopt::initializeNumOfConstraints()
{
  // hack to get the number of constraints, calling once computeConstraints(...)
  std::vector<double> xx;
  for (int i = 0; i < num_of_variables_; i++)
  {
    xx.push_back(0.0);
  }
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;

  double constraints[10000];  // this number should be very big!! (hack, TODO)
  x2qnd(xx, q, n, d);
  computeConstraints(0, constraints, num_of_variables_, nullptr, q, n, d);
  // end of hack
}

// Note that t_final will be updated in case the saturation in deltaT_ has had effect
bool SolverNlopt::setInitStateFinalStateInitTFinalT(mt::state initial_state, mt::state final_state, double t_init,
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
  mu::saturate(v0, -v_max_, v_max_);
  mu::saturate(a0, -a_max_, a_max_);
  mu::saturate(vf, -v_max_, v_max_);
  mu::saturate(af, -a_max_, a_max_);

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
      upper_bound = ((p_ - 1) * (mu::sgn(a0(axis)) * v_max_(axis) - v0(axis)) / (a0(axis)));
      lower_bound = ((p_ - 1) * (-mu::sgn(a0(axis)) * v_max_(axis) - v0(axis)) / (a0(axis)));

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

      mu::saturate(deltaT_, std::max(0.0, lower_bound), upper_bound);
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

  // mu::saturate(deltaT_, std::min(bound1.x(), bound2.x()), std::max(bound1.x(), bound2.x()));
  // mu::saturate(deltaT_, std::min(bound1.y(), bound2.y()), std::max(bound1.y(), bound2.y()));
  // mu::saturate(deltaT_, std::min(bound1.z(), bound2.z()), std::max(bound1.z(), bound2.z()));

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
  // See also eq. 15 of of the paper "Robust and Efficent quadrotor..."

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

// check if there is a normal vector = [0 0 0]
bool SolverNlopt::isDegenerate(const std::vector<double> &x)
{
  for (int j = j_min_; j <= (j_max_ - 2); j = j + 3)
  {
    Eigen::Vector3d normal(x[j], x[j + 1], x[j + 2]);
    if ((normal.norm() < 1e-5))
    {
      return true;
    }
  }
  return false;
}

template <class T>
void SolverNlopt::x2qnd(T &x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  q.clear();
  n.clear();
  d.clear();

  q.push_back(q0_);  // Not a decision variable
  q.push_back(q1_);  // Not a decision variable
  q.push_back(q2_);  // Not a decision variable

  // Control points (3x1)
  for (int i = i_min_; i <= i_max_ - 2; i = i + 3)
  {
    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }

  Eigen::Vector3d qNm2(x[i_max_ - 2], x[i_max_ - 1], x[i_max_]);
  q.push_back(qNm2);  // qn-1 Not a decision variable
  q.push_back(qNm2);  // qn Not a decision variable

  // Normals vectors (3x1)
  for (int j = j_min_; j <= j_max_ - 2; j = j + 3)
  {
    n.push_back(Eigen::Vector3d(x[j], x[j + 1], x[j + 2]));
  }

  // d values (1x1)
  for (int k = k_min_; k <= k_max_; k = k + 1)
  {
    d.push_back(x[k]);
  }
}

// global index of the first element of the control point i
int SolverNlopt::gIndexQ(int i)
{
  return 3 * i - 9;  // Q0, Q1, Q2 are always fixed (not decision variables)
}

// global index of the first element of the normal i
int SolverNlopt::gIndexN(int i)
{
  return 3 * i + j_min_;
}

int SolverNlopt::gIndexD(int i)
{
  return i + k_min_;
}

void SolverNlopt::assignValueToGradConstraints(int var_gindex, const double &tmp, double *grad, int r, int nn)

{
  grad[r * nn + var_gindex] = tmp;
}

// nn is the number of variables
double SolverNlopt::computeObjFunctionJerk(unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q,
                                           std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  // std::cout << "[Ineq] Time so far= " << (opt_timer_.ElapsedMs() / 1000) << std::endl;

  // Cost. See Lyx derivation (notes.lyx)
  double cost = 0.0;

  Eigen::Matrix<double, 4, 1> tmp;
  tmp << 6.0, 0.0, 0.0, 0.0;

  std::vector<Eigen::Vector3d> gradient;
  for (int i = 0; i <= N_; i++)
  {
    gradient.push_back(Eigen::Vector3d::Zero());
  }

  for (int i = 0; i < num_of_segments_; i++)
  {
    Eigen::Matrix<double, 4, 1> A_i_times_tmp = A_pos_bs_[i] * tmp;

    Eigen::Matrix<double, 3, 4> Q;
    Q.col(0) = q[i];
    Q.col(1) = q[i + 1];
    Q.col(2) = q[i + 2];
    Q.col(3) = q[i + 3];

    cost += (Q * A_i_times_tmp).squaredNorm();

    // I only need to do this if grad!=NULL
    for (int u = 0; u <= 3; u++)
    {
      gradient[i + u] += 2 * (Q * A_i_times_tmp) * A_i_times_tmp(u);
    }
    ///////////////
  }

  cost += weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();

  if (grad)
  {
    // Initialize to zero all the elements, IT IS NEEDED (if not it doesn't converge)
    for (int i = 0; i < nn; i++)
    {
      grad[i] = 0.0;
    }

    gradient[N_ - 2] = gradient[N_ - 2] + 2 * weight_modified_ * (q[N_ - 2] - final_state_.pos);
    gradient[N_ - 2] = gradient[N_ - 2] + gradient[N_ - 1] + gradient[N_];  // because qNm2=qNm1=qN

    for (int i = 3; i <= (N_ - 2); i++)  // q0_. q1_ and q2_ are already fixed by p0, v0, a0
    {
      assignEigenToVector(grad, gIndexQ(i), gradient[i]);
    }
  }

  ////////////This part below is valid when it is a uniform Non-clamped Bspline. I.e. when the matrices A don't change
  /// every interval
  // for (int i = p_; i <= N_; i++)  // i is the index of the control point
  // {
  //   cost += (-q[i - 3] + 3 * q[i - 2] - 3 * q[i - 1] + q[i]).squaredNorm();  // the jerk of the interval i-4
  //                                                                            // ///////////////////
  // }

  // if (grad)
  // {
  //   // Initialize to zero all the elements, IT IS NEEDED (if not it doesn't converge)
  //   for (int i = 0; i < nn; i++)
  //   {
  //     grad[i] = 0.0;
  //   }

  //   // Gradient for the control points that are decision variables
  //   for (int i = p_; i <= (lastDecCP() - 1); i++)
  //   // q0,q1,q2,qNm1,qN are not decision variables. //qNm2 is done outside the loop
  //   {
  //     Eigen::Vector3d gradient;

  //     gradient = 2 * (-q[i - 3] + 3 * q[i - 2] - 3 * q[i - 1] + q[i])     // Right
  //                - 6 * (-q[i - 2] + 3 * q[i - 1] - 3 * q[i] + q[i + 1])   // Center-right
  //                + 6 * (-q[i - 1] + 3 * q[i] - 3 * q[i + 1] + q[i + 2])   // Center-right
  //                - 2 * (-q[i] + 3 * q[i + 1] - 3 * q[i + 2] + q[i + 3]);  // Left

  //     assignEigenToVector(grad, gIndexQ(i), gradient);
  //   }

  //   // and now do qNm2. Note that qN=qNm1=qNm2
  //   int i = N_ - 2;
  //   Eigen::Vector3d gradient = 2 * (-q[i - 3] + 3 * q[i - 2] - 3 * q[i - 1] + q[i])  // Right
  //                              - 4 * (-q[i - 2] + 3 * q[i - 1] - 2 * q[i])           // Center-right
  //                              + 2 * (-q[i - 1] + q[i])                              // Center-right
  //                              + 2 * weight_modified_ * (q[i] - final_state_.pos);

  //   assignEigenToVector(grad, gIndexQ(i), gradient);
  // }

  return cost;
}

// double SolverNlopt::computeObjFunction(unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q,
//                                        std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
// {
//   // Cost
//   double cost = 0.0;
//   for (int i = 1; i <= (N_ - 1); i++)
//   {
//     cost += (q[i + 1] - 2 * q[i] + q[i - 1]).squaredNorm();
//   }

//   if (force_final_state_ == false)
//   {
//     cost += weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
//   }

//   if (grad)
//   {
//     // Initialize to zero all the elements, IT IS NEEDED (if not it doesn't converge)
//     for (int i = 0; i < nn; i++)
//     {
//       grad[i] = 0.0;
//     }

//     // Gradient for the control points that are decision variables
//     for (int i = 3; i <= lastDecCP(); i++)
//     {
//       Eigen::Vector3d gradient;

//       if (i == (N_ - 2))  // not reached when force_final_state_==true
//       {
//         gradient = -2 * (q[i - 1] - q[i]) + 2 * (q[i] - 2 * q[i - 1] + q[i - 2]);
//         gradient += 2 * weight_modified_ * (q[i] - final_state_.pos);
//       }
//       else
//       {
//         gradient = 2 * (q[i] - 2 * q[i - 1] + q[i - 2]) +     ///////////// Right
//                    (-4 * (q[i + 1] - 2 * q[i] + q[i - 1])) +  // Center
//                    (2 * (q[i + 2] - 2 * q[i + 1] + q[i]));    //// Left
//       }
//       assignEigenToVector(grad, gIndexQ(i), gradient);
//     }
//   }

//   // std::cout << "cost= " << cost << std::endl;

//   return cost;
// }

double SolverNlopt::myObjFunc(unsigned nn, const double *x, double *grad, void *my_func_data)
{
  SolverNlopt *opt = reinterpret_cast<SolverNlopt *>(my_func_data);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->x2qnd(x, q, n, d);

  double cost = opt->computeObjFunctionJerk(nn, grad, q, n, d);

  // if (isFeasible(x) && cost < lowest_cost_so_far_)
  // {
  //   lowest_cost_so_far_ = cost;
  //   best_x_so_far_ = x_
  // }

  return cost;
}

bool SolverNlopt::isADecisionCP(int i)

{  // If Q[i] is a decision variable
  return ((i >= 3) && i <= (N_ - 2));
}

double SolverNlopt::getTimeNeeded()
{
  return time_needed_;
}

int SolverNlopt::lastDecCP()
{
  return (N_ - 2);
}

void SolverNlopt::transformPosBSpline2otherBasis(const Eigen::Matrix<double, 3, 4> &Qbs,
                                                 Eigen::Matrix<double, 3, 4> &Qmv, int interval)
{
  Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverNlopt::transformVelBSpline2otherBasis(const Eigen::Matrix<double, 3, 3> &Qbs,
                                                 Eigen::Matrix<double, 3, 3> &Qmv, int interval)
{
  Qmv = Qbs * M_vel_bs2basis_[interval];
}

bool SolverNlopt::checkGradientsUsingFiniteDiff()
{
  optimize();  // we need to call optimize first so that all the variables and constraints are set up

  double weight_modified_original = weight_modified_;
  weight_modified_ =
      10;  // For the gradient check, we set the weight to a small value (if not you run into numerical issues)

  printIndexesConstraints();
  printIndexesVariables();

  std::vector<Eigen::Vector3d> n;  // normals
  std::vector<Eigen::Vector3d> q;  // control points
  std::vector<double> d;           // d

  n.clear();
  q.clear();
  d.clear();

  generateRandomN(n);
  generateRandomD(d);
  generateRandomQ(q);

  std::vector<double> x;
  qnd2x(q, n, d, x);
  x2qnd(x, q, n, d);  // This is needed to ensure q0,q1,q2,qNm1,qN are right

  int m = num_of_constraints_;
  int nn = num_of_variables_;

  double grad_constraints[nn * m];
  // grad is a vector with nn *m elements

  double constraints[num_of_constraints_];

  std::cout << "CALLING computeConstraints!!" << std::endl;

  computeConstraints(m, constraints, num_of_variables_, grad_constraints, q, n, d);

  std::cout << "CALLed computeConstraints!!" << std::endl;

  double grad_f[nn];

  std::cout << "CALLING computeObjFunctionJerk!!" << std::endl;

  double f = computeObjFunctionJerk(nn, grad_f, q, n, d);

  std::cout << "CALLed computeObjFunctionJerk!!" << std::endl;

  double epsilon = 1e-6;

  bool gradients_f_are_right = true;  // objective function
  bool gradients_c_are_right = true;  // constraints

  for (int i = 0; i < num_of_variables_; i++)
  {
    std::vector<double> x_perturbed = x;
    x_perturbed[i] = x_perturbed[i] + epsilon;

    std::cout << "******Checking variable " << i << std::endl;
    // "*******, that has changed from" << x[i] << " to " << x_perturbed[i]    << std::endl;

    std::vector<Eigen::Vector3d> n_perturbed;  // normals
    std::vector<Eigen::Vector3d> q_perturbed;  // control points
    std::vector<double> d_perturbed;           // d

    x2qnd(x_perturbed, q_perturbed, n_perturbed, d_perturbed);

    double f_perturbed = computeObjFunctionJerk(nn, nullptr, q_perturbed, n_perturbed, d_perturbed);

    double grad_f_is = grad_f[i];
    double grad_f_should_be = (f_perturbed - f) / epsilon;

    std::cout << "Obj Func: ";

    if (fabs(grad_f_is - grad_f_should_be) > 1e-4)
    {
      gradients_f_are_right = false;
      std::cout << red << "ERROR" << reset << "(partial f)/(partial var_" << i << ") = " << grad_f_is << ", should be "
                << grad_f_should_be << std::endl;
    }
    else
    {
      std::cout << green << "OK" << reset << std::endl;
    }
    ///////////////////////
    std::cout << "Constraints: ";
    double constraints_perturbed[num_of_constraints_];  // this number should be very big!! (hack, TODO)
    computeConstraints(m, constraints_perturbed, num_of_variables_, nullptr, q_perturbed, n_perturbed, d_perturbed);

    // And now check the check on that column (variable i)
    // ci is the constraint index
    bool check_satisfied = true;
    for (int ci = 0; ci < num_of_constraints_; ci++)
    {
      double grad_c_should_be = (constraints_perturbed[ci] - constraints[ci]) / epsilon;

      double grad_c_is = grad_constraints[ci * nn + i];
      if (fabs(grad_c_is - grad_c_should_be) > 1e-4)
      {
        gradients_c_are_right = false;
        // std::cout << "constraints_perturbed[ci]= " << constraints_perturbed[ci] << std::endl;
        // std::cout << "constraints[ci]= " << constraints[ci] << std::endl;
        check_satisfied = false;
        std::cout << red << "ERROR: " << reset << "(partial constraint_" << ci << ")/(partial var_" << i
                  << ") = " << grad_c_is << ", should be " << grad_c_should_be << std::endl;
      }
    }

    if (check_satisfied == true)
    {
      std::cout << green << "OK" << reset << std::endl;
    }
  }

  weight_modified_ = weight_modified_original;

  return (gradients_f_are_right && gradients_c_are_right);
}

// m is the number of constraints, nn is the number of variables
void SolverNlopt::computeConstraints(unsigned m, double *constraints, unsigned nn, double *grad,
                                     const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                                     const std::vector<double> &d)
{
  Eigen::Vector3d ones = Eigen::Vector3d::Ones();
  int r = 0;
  // grad is a vector with nn*m elements
  // Initialize grad to 0 all the elements, is needed I think
  if (grad)
  {
    for (int i = 0; i < nn * m; i++)
    {
      grad[i] = 0.0;
    }
  }

  index_const_obs_ = r;

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
      for (int j = 0; j < hulls_[obst_index][i].cols(); j++)  // Eigen::Vector3d vertex : hulls_[obst_index][i]
      {
        Eigen::Vector3d vertex = hulls_[obst_index][i].col(j);
        constraints[r] = -(n[ip].dot(vertex) + d[ip] - epsilon);  //+d[ip] // f<=0

        if (grad)
        {
          toGradSameConstraintDiffVariables(gIndexN(ip), -vertex, grad, r, nn);
          assignValueToGradConstraints(gIndexD(ip), -1, grad, r, nn);
        }
        r++;
      }

      // and the control points on the other side

      // if (basis_ == MINVO || basis_ == BEZIER)
      // {
      Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
      Eigen::Matrix<double, 3, 4> Qmv;  // other basis "basis_" (minvo, bezier,...).
      Qbs.col(0) = q[i];
      Qbs.col(1) = q[i + 1];
      Qbs.col(2) = q[i + 2];
      Qbs.col(3) = q[i + 3];
      transformPosBSpline2otherBasis(Qbs, Qmv,
                                     i);  // Now Qmv is a matrix whose each row contains a "basis_" control point

      Eigen::Vector3d q_ipu;
      for (int u = 0; u <= 3; u++)
      {
        q_ipu = Qmv.col(u);                                     // if using the MINVO basis
        constraints[r] = (n[ip].dot(q_ipu) + d[ip] + epsilon);  //  // fi<=0

        if (grad)
        {
          toGradSameConstraintDiffVariables(gIndexN(ip), q_ipu, grad, r, nn);

          // If using the MINVO basis
          for (int k = 0; (k <= 3); k++)
          // This loop is needed because each q in MINVO depends on every q in BSpline
          {
            if ((i + 3) == (N_ - 1) && k == 2)
            {  // The last control point of the interval is qNm1, and the variable is qNm2
              // Needed because qN=qNm1=qNm2
              toGradSameConstraintDiffVariables(
                  gIndexQ(i + k), (M_pos_bs2basis_[i](k, u) + M_pos_bs2basis_[i](k + 1, u)) * n[ip], grad, r, nn);
            }

            else if ((i + 3) == (N_) && k == 1)
            {  // The last 2 control points of the interval are qNm1 and qN, and the variable is qNm2
              // Needed because qN=qNm1=qNm2

              toGradSameConstraintDiffVariables(
                  gIndexQ(i + k),
                  (M_pos_bs2basis_[i](k, u) + M_pos_bs2basis_[i](k + 1, u) + M_pos_bs2basis_[i](k + 2, u)) * n[ip],
                  grad, r, nn);
            }

            else
            {
              if (isADecisionCP(i + k))  // If Q[i] is a decision variable
              {
                toGradSameConstraintDiffVariables(gIndexQ(i + k), M_pos_bs2basis_[i](k, u) * n[ip], grad, r, nn);
              }
            }
            //}
          }
          assignValueToGradConstraints(gIndexD(ip), 1, grad, r, nn);
        }
        r++;
      }
      // }
      // else  // NOT using the MINVO Basis
      // {
      //   // std::cout << "Not using MINVO" << std::endl;
      //   for (int u = 0; (u <= 3 && (i + u) <= (N_ - 2)); u++)

      //   {
      //     constraints[r] = (n[ip].dot(q[i + u]) + d[ip] + epsilon);  //  // f<=0

      //     if (grad)
      //     {
      //       toGradSameConstraintDiffVariables(gIndexN(ip), q[i + u], grad, r, nn);
      //       if (isADecisionCP(i + u))  // If Q[i] is a decision variable
      //       {
      //         toGradSameConstraintDiffVariables(gIndexQ(i + u), n[ip], grad, r, nn);
      //       }

      //       assignValueToGradConstraints(gIndexD(ip), 1, grad, r, nn);
      //     }

      //     r++;
      //   }
      // }
    }
  }

  index_const_vel_ = r;

  /////////////////////////////////////////////////
  ////////// VELOCITY CONSTRAINTS    //////////////
  /////////////////////////////////////////////////

  for (int i = 2; i <= (N_ - 2); i++)  // If using BSpline basis, v0 and v1 are already determined by initial_state
  {
    double ciM2 = p_ / (knots_(i + p_ + 1 - 2) - knots_(i + 1 - 2));
    double ciM1 = p_ / (knots_(i + p_ + 1 - 1) - knots_(i + 1 - 1));
    double ci = p_ / (knots_(i + p_ + 1) - knots_(i + 1));

    // std::cout << "ciM2= " << ciM2 << std::endl;
    // std::cout << "ciM1= " << ciM1 << std::endl;
    // std::cout << "ci= " << ci << std::endl;

    std::vector<double> c;
    c.push_back(ciM2);
    c.push_back(ciM1);
    c.push_back(ci);

    Eigen::Vector3d v_iM2 = ciM2 * (q[i - 1] - q[i - 2]);
    Eigen::Vector3d v_iM1 = ciM1 * (q[i] - q[i - 1]);
    Eigen::Vector3d v_i = ci * (q[i + 1] - q[i]);

    Eigen::Matrix<double, 3, 3> Qbs;  // b-spline
    Eigen::Matrix<double, 3, 3> Qmv;  // other basis "basis_" (minvo, bezier,...).

    Qbs.col(0) = v_iM2;
    Qbs.col(1) = v_iM1;
    Qbs.col(2) = v_i;

    transformVelBSpline2otherBasis(Qbs, Qmv, i - 2);

    ///////////////////////// ANY BASIS //////////////////////////////

    // std::cout << blue << "****Using q_" << i - 2 << ", " << i - 1 << ", " << i << ", " << i + 1 << reset <<
    // std::endl; std::cout << "Qbs=\n" << Qbs << std::endl; std::cout << "Qmv=\n" << Qmv << std::endl;
    for (int j = 0; j < 3; j++)
    {  // loop over each of the velocity control points (v_{i-2+j}) of the new basis
      //|v_{i-2+j}| <= v_max ////// v_{i-2}, v_{i-1}, v_{i}

      Eigen::Vector3d v_iM2Pj = Qmv.col(j);  // v_{i-2+j};

      // Constraint v_{i-2+j} - vmax <= 0
      assignEigenToVector(constraints, r, v_iM2Pj - v_max_);  // f<=0

      if (grad)
      {
        // Each column of partials has partial constraint / partial q_{i-2+u}
        Eigen::Matrix<double, 3, 4> partials = Eigen::Matrix<double, 3, 4>::Zero();
        for (int u = 0; u < 3; u++)
        {  // v_{i-2+j} depends on v_{i-2}, v_{i-1}, v_{i} of the old basis

          partials.col(u) += -M_vel_bs2basis_[i - 2](u, j) * c[u] * ones;
          partials.col(u + 1) += M_vel_bs2basis_[i - 2](u, j) * c[u] * ones;
        }

        if (i == (N_ - 2))
        {
          partials.col(2) = partials.col(2) + partials.col(3);
        }

        // and now assign it to the vector grad
        for (int u = 0; u < 3; u++)
        {
          if (isADecisionCP(i - 2 + u))  // If Q[i] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i - 2 + u), partials.col(u), grad, r, nn);
          }

          // v_{i-2+u} depends on q_{i-2+u+1}
          if (isADecisionCP(i - 2 + u + 1))  // If Q[i+1] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i - 2 + u + 1), partials.col(u + 1), grad, r, nn);
          }
        }
      }
      r = r + 3;

      // Constraint -v_{i-2+j} - vmax <= 0
      assignEigenToVector(constraints, r, -v_iM2Pj - v_max_);  // f<=0

      if (grad)
      {
        // Each column of partials has partial constraint / partial q_{i-2+u}
        Eigen::Matrix<double, 3, 4> partials = Eigen::Matrix<double, 3, 4>::Zero();
        for (int u = 0; u < 3; u++)
        {  // v_{i-2+j} depends on v_{i-2}, v_{i-1}, v_{i} of the old basis

          partials.col(u) += M_vel_bs2basis_[i - 2](u, j) * c[u] * ones;
          partials.col(u + 1) += -M_vel_bs2basis_[i - 2](u, j) * c[u] * ones;
        }

        if (i == (N_ - 2))
        {
          partials.col(2) = partials.col(2) + partials.col(3);
        }

        // and now assign it to the vector grad
        for (int u = 0; u < 3; u++)
        {
          if (isADecisionCP(i - 2 + u))  // If Q[i] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i - 2 + u), partials.col(u), grad, r, nn);
          }

          // v_{i-2+u} depends on q_{i-2+u+1}
          if (isADecisionCP(i - 2 + u + 1))  // If Q[i+1] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i - 2 + u + 1), partials.col(u + 1), grad, r, nn);
          }
        }
      }
      r = r + 3;
    }

    // The code below is valid only for the B-Spline (for which Mbs2basis_vel_ is the identity matrix)

    ///////////////////////// B-SPLINE //////////////////////////////
    // // v<=vmax  \equiv  v_i - vmax <= 0
    // assignEigenToVector(constraints, r, v_i - v_max_);  // f<=0
    // if (grad)
    // {
    //   if (isADecisionCP(i))  // If Q[i] is a decision variable
    //   {
    //     toGradDiffConstraintsDiffVariables(gIndexQ(i), -c1 * ones, grad, r, nn);
    //   }
    //   if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
    //   {
    //     toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), c1 * ones, grad, r, nn);
    //   }
    // }
    // r = r + 3;

    // // v>=-vmax  \equiv  -v_i - vmax <= 0
    // assignEigenToVector(constraints, r, -v_i - v_max_);  // f<=0
    // if (grad)
    // {
    //   if (isADecisionCP(i))  // If Q[i] is a decision variable
    //   {
    //     toGradDiffConstraintsDiffVariables(gIndexQ(i), c1 * ones, grad, r, nn);
    //   }
    //   if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
    //   {
    //     toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -c1 * ones, grad, r, nn);
    //   }
    // }
    // r = r + 3;
    ///////////////////////// B-SPLINE //////////////////////////////
  }

  index_const_accel_ = r;

  /////////////////////////////////////////////////
  ////////// ACCELERATION CONSTRAINTS    //////////
  /////////////////////////////////////////////////

  for (int i = 1; i <= (N_ - 3); i++)  // a0 is already determined by the initial state

  {
    double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
    double c2 = p_ / (knots_(i + p_ + 1 + 1) - knots_(i + 1 + 1));
    double c3 = (p_ - 1) / (knots_(i + p_ + 1) - knots_(i + 2));

    Eigen::Vector3d v_i = c1 * (q[i + 1] - q[i]);
    Eigen::Vector3d v_iP1 = c2 * (q[i + 2] - q[i + 1]);
    Eigen::Vector3d a_i = c3 * (v_iP1 - v_i);

    // a<=amax  ==  a_i - amax <= 0  ==  c3 * (v_iP1 - v_i)<=0 ==
    // c3*c2 *q[i + 2] - c3*c2* q[i + 1]  -  c3*c1*q[i + 1] + c3*c1*q[i]   - amax <= 0

    // if (((a_i - a_max_).array() > Eigen::Vector3d::Zero().array()).any())
    // {
    //   std::cout << "__________________ i= " << i << " _________________" << std::endl;

    //   std::cout << "v_i= " << v_i.transpose() << std::endl;
    //   std::cout << "a_i= " << a_i.transpose() << std::endl;
    //   std::cout << "v_iP1= " << v_iP1.transpose() << std::endl;

    //   std::cout << "r= " << r << ", a_i= " << a_i.transpose() << "  a_max_=" << a_max_.transpose() << std::endl;
    // }

    assignEigenToVector(constraints, r, a_i - a_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), c3 * c1 * ones, grad, r, nn);
      }

      if (i == (N_ - 3))
      {  // this is needed because qN=qNm1=qNm2 (and therefore the gradient of qNm2 should take into account also qNm1)
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), (-c3 * c2 - c3 * c1) * ones + c3 * c2 * ones, grad, r, nn);
      }
      else
      {
        if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), (-c3 * c2 - c3 * c1) * ones, grad, r, nn);
        }
        if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), c3 * c2 * ones, grad, r, nn);
        }
      }
    }
    r = r + 3;

    assignEigenToVector(constraints, r, -a_i - a_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), -c3 * c1 * ones, grad, r, nn);
      }

      if (i == (N_ - 3))
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -(-c3 * c2 - c3 * c1) * ones - c3 * c2 * ones, grad, r, nn);
      }
      else
      {
        if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -(-c3 * c2 - c3 * c1) * ones, grad, r, nn);
        }
        if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), -c3 * c2 * ones, grad, r, nn);
        }
      }
    }
    r = r + 3;
  }

  index_const_normals_ = r;

  /////////////////////////////////////////////////
  //////////// SPHERE CONSTRAINTS    /////////////
  /////////////////////////////////////////////////
  // Impose that all the control points are inside an sphere of radius Ra

  // for (int i = 2; i <= N_ - 2; i++)  // Asumming q0_, q1_ and q2_ are inside the sphere TODO: Add this check
  //                                    // when computing q0_, q1_ and q2_ (and return false if not)
  // {
  //   constraints[r] = (q[i] - q0_).squaredNorm() - Ra_ * Ra_;  // fi<=0
  //   if (grad)
  //   {
  //     toGradSameConstraintDiffVariables(gIndexQ(i), 2 * (q[i] - q0_), grad, r, nn);
  //   }

  for (int i = 0; i <= (N_ - 3); i++)  // i  is the interval (\equiv segment)
  {
    Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
    Eigen::Matrix<double, 3, 4> Qmv;  // other basis "basis_" (minvo, bezier,...).
    Qbs.col(0) = q[i];
    Qbs.col(1) = q[i + 1];
    Qbs.col(2) = q[i + 2];
    Qbs.col(3) = q[i + 3];

    transformPosBSpline2otherBasis(Qbs, Qmv,
                                   i);  // Now Qmv is a matrix whose each row contains a "basis_" control point

    Eigen::Vector3d q_ipu;
    for (int u = 0; u <= 3; u++)
    {
      q_ipu = Qmv.col(u);  // if using the MINVO basis

      constraints[r] = (q_ipu - q0_).squaredNorm() - Ra_ * Ra_;  // fi<=0

      if (grad)
      {
        for (int k = 0; (k <= 3); k++)
        // This loop is needed because each q in MINVO depends on every q in BSpline
        {
          if ((i + 3) == (N_ - 1) && k == 2)
          {  // The last control point of the interval is qNm1, and the variable is qNm2
            // Needed because qN=qNm1=qNm2
            toGradSameConstraintDiffVariables(
                gIndexQ(i + k), 2 * (q_ipu - q0_) * (M_pos_bs2basis_[i](k, u) + M_pos_bs2basis_[i](k + 1, u)), grad, r,
                nn);
          }

          else if ((i + 3) == (N_) && k == 1)
          {  // The last 2 control points of the interval are qNm1 and qN, and the variable is qNm2
            // Needed because qN=qNm1=qNm2
            toGradSameConstraintDiffVariables(
                gIndexQ(i + k),
                2 * (q_ipu - q0_) *
                    (M_pos_bs2basis_[i](k, u) + M_pos_bs2basis_[i](k + 1, u) + M_pos_bs2basis_[i](k + 2, u)),
                grad, r, nn);
          }

          else
          {
            if (isADecisionCP(i + k))  // If Q[i] is a decision variable
            {
              toGradSameConstraintDiffVariables(gIndexQ(i + k), 2 * (q_ipu - q0_) * M_pos_bs2basis_[i](k, u), grad, r,
                                                nn);
            }
          }
        }
      }
      r++;
    }
  }

  num_of_constraints_ = r;  // + 1 has already been added in the last loop of the previous for loop;
}

// See example https://github.com/stevengj/nlopt/issues/168
// m is the number of constraints (which is computed from the lentgh of the vector of tol_constraint)
// nn is the number of variables
void SolverNlopt::myIneqConstraints(unsigned m, double *constraints, unsigned nn, const double *x, double *grad,
                                    void *f_data)
{
  SolverNlopt *opt = reinterpret_cast<SolverNlopt *>(f_data);

  // std::cout << "[Ineq] Time so far= " << (opt->opt_timer_.ElapsedMs() / 1000) << std::endl;
  // sometimes max_runtime in nlopt doesn't work --> force stop execution
  // if ((opt->opt_timer_.ElapsedMs() / 1000) > (opt->mu_ * opt->max_runtime_))
  // {
  //   //nlopt::set_force_stop();
  //   // try
  //   // {
  //   //   throw 20;
  //   // }
  //   // catch (int e)
  //   // {
  //   //   // std::cout << "An exception occurred. Exception Nr. " << e << '\n';
  //   // }
  //   // // https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/
  // }

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->x2qnd(x, q, n, d);
  // opt->printQND(q, n, d);
  opt->computeConstraints(m, constraints, nn, grad, q, n, d);

  // opt->printInfeasibleConstraints(constraints);
  if (opt->areTheseConstraintsFeasible(constraints))
  {
    double cost_now = opt->computeObjFunctionJerk(nn, nullptr, q, n, d);
    if (cost_now < opt->best_cost_so_far_)
    {
      // std::cout << blue << "Updating best_cost to " << cost_now << reset << std::endl;
      opt->best_cost_so_far_ = cost_now;
      // Copy onto the std::vector)
      for (int i = 0; i < nn; i++)
      {
        opt->best_feasible_sol_so_far_[i] = x[i];
      }
      opt->got_a_feasible_solution_ = true;
    }
  }

  return;
}

void SolverNlopt::printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
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

void SolverNlopt::generateRandomGuess()
{
  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();

  generateRandomN(n_guess_);
  generateRandomD(d_guess_);
  generateRandomQ(q_guess_);
}

void SolverNlopt::printStd(const std::vector<double> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i << std::endl;
  }
}

void SolverNlopt::printStd(const std::vector<Eigen::Vector3d> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i.transpose() << std::endl;
  }
}

void *func(void *arg)
{
  // detach the current thread
  // from the calling thread
  pthread_detach(pthread_self());

  printf("Inside the thread\n");

  // exit the current thread
  pthread_exit(NULL);
}

bool SolverNlopt::optimize()

{
  // reset some stuff
  traj_solution_.clear();
  best_cost_so_far_ = std::numeric_limits<double>::max();
  best_feasible_sol_so_far_.clear();
  got_a_feasible_solution_ = false;
  best_feasible_sol_so_far_.resize(num_of_variables_);

  // note that, for a v0 and a0 given, q2_ is not guaranteed to lie within the bounds. If that's the case --> keep
  // executing previous trajectory
  if (q2_.x() > x_max_ || q2_.x() < x_min_ ||  //////////////
      q2_.y() > y_max_ || q2_.y() < y_min_ ||  /////////////////
      q2_.z() > z_max_ || q2_.z() < z_min_)
  {
    std::cout << bold << red << "q2_ is not in [min, max]" << reset << std::endl;
    std::cout << "q2_= " << q2_.transpose() << std::endl;
    std::cout << "x_min_= " << x_min_ << ", x_max_=" << x_max_ << std::endl;
    std::cout << "y_min_= " << y_min_ << ", y_max_=" << y_max_ << std::endl;
    std::cout << "z_min_= " << z_min_ << ", z_max_=" << z_max_ << std::endl;
    return false;
  }

  generateAStarGuess();

  // https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/#augmented-lagrangian-algorithm
  // The augmented Lagrangian method is specified in NLopt as NLOPT_AUGLAG. We also provide a variant, NLOPT_AUGLAG_EQ,
  // that only uses penalty functions for equality constraints, while inequality constraints are passed through to the
  // subsidiary algorithm to be handled directly; in this case, the subsidiary algorithm must handle inequality
  // constraints (e.g. MMA or COBYLA)

  nlopt::opt opt(nlopt::AUGLAG, num_of_variables_);  // need to create it here because I need the # of variables
  nlopt::opt local_opt(solver_, num_of_variables_);  // need to create it here because I need the # of variables

  // opt_ = new nlopt::opt(nlopt::AUGLAG_EQ, num_of_variables_);  // nlopt::AUGLAG
  // local_opt_ = new nlopt::opt(solver_, num_of_variables_);

  local_opt.set_xtol_rel(xtol_rel_);  // stopping criteria.
  local_opt.set_ftol_rel(ftol_rel_);  // stopping criteria.
  local_opt.set_maxtime(std::max(mu_ * max_runtime_, 0.001));

  opt.set_local_optimizer(local_opt);
  opt.set_xtol_rel(xtol_rel_);  // Stopping criteria.
  opt.set_ftol_rel(ftol_rel_);  // Stopping criteria.

  // opt.set_maxeval(1e6);  // maximum number of evaluations. Negative --> don't use this criterion

  opt.set_maxtime(std::max(mu_ * max_runtime_, 0.001));  // 0.001 to make sure this criterion is used  // maximum time
                                                         // in seconds. Negative --> don't use this criterion

  // opt.set_maxtime(max_runtime_);
  initializeNumOfConstraints();

  // see https://github.com/stevengj/nlopt/issues/168
  std::vector<double> tol_constraints;
  for (int i = 0; i < num_of_constraints_; i++)
  {
    tol_constraints.push_back(epsilon_tol_constraints_);
  }

  // andd lower and upper bounds
  std::vector<double> lb;
  std::vector<double> ub;
  // control points q
  for (int i = 0; i <= i_max_; i = i + 3)
  {
    // x component
    lb.push_back(x_min_);
    ub.push_back(x_max_);
    // y component
    lb.push_back(y_min_);
    ub.push_back(y_max_);
    // z component
    lb.push_back(z_min_);
    ub.push_back(z_max_);
  }
  // normals n
  for (int j = j_min_; j <= j_max_; j++)
  {
    lb.push_back(-1e9);
    ub.push_back(1e9);
  }
  // coefficients d
  for (int k = k_min_; k <= k_max_; k++)
  {
    lb.push_back(-1e9);
    ub.push_back(1e9);
  }
  // opt.set_lower_bounds(lb);
  // opt.set_upper_bounds(ub);
  // // if (local_opt_)
  // // {
  // local_opt.set_lower_bounds(lb);
  // local_opt.set_upper_bounds(ub);
  // }

  // set constraint and objective
  opt.add_inequality_mconstraint(SolverNlopt::myIneqConstraints, this, tol_constraints);
  opt.set_min_objective(SolverNlopt::myObjFunc,
                        this);  // this is passed as a parameter (the obj function has to be static)

  qnd2x(q_guess_, n_guess_, d_guess_, x_);

  double obj_guess = computeObjFunctionJerk(num_of_variables_, nullptr, q_guess_, n_guess_, d_guess_);

  double second_term_obj_guess = weight_modified_ * (q_guess_[N_] - final_state_.pos).squaredNorm();
  double first_term_obj_guess = obj_guess - second_term_obj_guess;

  // std::cout << "The guess is the following one:" << std::endl;
  // printQVA(q_guess_);
  // printQND(q_guess_, n_guess_, d_guess_);

  // x2qnd(x_, q_guess_, n_guess_);

  std::cout << bold << "The infeasible constraints of the initial Guess" << reset << std::endl;

  printInfeasibleConstraints(q_guess_, n_guess_, d_guess_);

  printIndexesConstraints();
  printIndexesVariables();

  double obj_obtained;

  opt_timer_.Reset();
  std::cout << "[NL] Optimizing now, allowing time = " << mu_ * max_runtime_ * 1000 << "ms" << std::endl;

  int result;
  try
  {
    result = opt.optimize(x_, obj_obtained);
  }
  catch (...)
  {
    std::cout << bold << red << "An exception occurred in the solver" << reset << std::endl;
    return false;
  }
  // std::cout << "[NL] Finished optimizing " << std::endl;

  time_needed_ = opt_timer_.ElapsedMs() / 1000;

  num_of_QCQPs_run_++;

  // See codes in https://github.com/JuliaOpt/NLopt.jl/blob/master/src/NLopt.jl
  // got_a_feasible_solution_ = got_a_feasible_solution_;
  bool optimal = (result == nlopt::SUCCESS);
  bool failed = (!optimal) && (!got_a_feasible_solution_);
  bool feasible_but_not_optimal = (!optimal) && (got_a_feasible_solution_);

  // Store the results here
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;

  // std::cout << "[NL] result= " << getResultCode(result) << std::endl;

  if (failed == false)
  {
  }

  if (failed)
  {
    ROS_ERROR_STREAM("[NL] Failed, code=" << getResultCode(result));

    printf("[NL] nlopt failed or maximum time was reached!\n");

    std::cout << bold << red << "[NL] Solution not found" << opt_timer_ << reset << std::endl;  // on_red

    // x2qnd(x_, q, n, d);
    // printInfeasibleConstraints(q, n, d);

    return false;
  }
  else if (optimal)
  {
    ROS_INFO_STREAM("[NL] Optimal, code=" << getResultCode(result));

    std::cout << on_green << bold << "[NL] Optimal Solution found in" << opt_timer_ << reset << std::endl;
    x2qnd(x_, q, n, d);

    // std::cout << "q[N_]= " << q[N_].transpose() << std::endl;

    double second_term_obj_obtained = weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
    double first_term_obj_obtained = obj_obtained - second_term_obj_obtained;
    // std::cout << "obj_guess= " << std::setprecision(7) << obj_guess << reset << "(" << first_term_obj_guess << " + "
    //           << second_term_obj_guess << ")" << std::endl;
    // std::cout << "obj_obtained= " << std::setprecision(7) << obj_obtained << reset << "(" << first_term_obj_obtained
    //           << " + " << second_term_obj_obtained << ")" << std::endl;

    improvement_ = 100 * (1.0 - (obj_obtained / obj_guess));
    // std::cout << green << std::setprecision(7) << "Improvement: " << 100 * (1.0 - (obj_obtained / obj_guess)) << "%"
    //           << reset << std::endl;
  }
  else if (feasible_but_not_optimal)
  {
    ROS_INFO_STREAM("[NL] Feasible, code=" << getResultCode(result));

    std::cout << on_green << bold << "[NL] Feasible Solution found" << opt_timer_ << reset << std::endl;
    x2qnd(best_feasible_sol_so_far_, q, n, d);  // was

    std::cout << "q[N_]= " << q[N_].transpose() << std::endl;

    double second_term_obj_obtained = weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
    double first_term_obj_obtained = best_cost_so_far_ - second_term_obj_obtained;
    std::cout << "obj_guess= " << std::setprecision(7) << obj_guess << reset << "(" << first_term_obj_guess << " + "
              << second_term_obj_guess << ")" << std::endl;
    std::cout << "obj_obtained= " << std::setprecision(7) << best_cost_so_far_ << reset << "("
              << first_term_obj_obtained << " + " << second_term_obj_obtained << ")" << std::endl;

    improvement_ = 100 * (1.0 - (obj_obtained / obj_guess));

    // std::cout << green << std::setprecision(7) << "Improvement: " << improvement_ << "%" << reset << std::endl;
  }
  else
  {
    std::cout << on_red << bold << "[NL] not implemented yet" << opt_timer_ << reset << std::endl;
    abort();
    return false;
  }

  // printQND(q, n, d);

  /*  std::cout << on_green << bold << "Solution found: " << time_first_feasible_solution_ << "/" << opt_timer_ <<
     reset
              << std::endl;*/

  CPs2TrajAndPwp(q, traj_solution_, solution_, N_, p_, num_pol_, knots_, dc_);
  ///////////////

  ///////////////For debugging

  // double epsilon = 0.1;  // note that epsilon_tol_constraints_ applies to the bspline control points

  // for (auto xi : traj_solution_)
  // {
  //   if (fabs(xi.vel.x()) > (v_max_.x() + epsilon) || fabs(xi.vel.y()) > (v_max_.y() + epsilon) ||
  //       (fabs(xi.vel.z()) > v_max_.z() + epsilon) || (fabs(xi.accel.x()) > a_max_.x() + epsilon) ||
  //       (fabs(xi.accel.y()) > a_max_.y() + epsilon) || (fabs(xi.accel.z()) > a_max_.z() + epsilon))
  //   {
  //     std::cout << bold << red << "Velocity or Accel constraints are not satisfied,v_max_= " << v_max_.transpose()
  //               << " a_max_=" << a_max_.transpose() << reset << std::endl;
  //     std::cout << "Velocity_i=" << xi.vel.transpose() << std::endl;
  //     std::cout << "Accel_i=" << xi.accel.transpose() << std::endl;

  //     std::cout << "These are the control points" << std::endl;
  //     printQVA(q);

  //     bool is_feasible = isFeasible(q, n, d);
  //     std::cout << "is Feasible= " << is_feasible << std::endl;

  //     std::cout << red << "====================================" << reset << std::endl;
  //     // abort();
  //   }

  //   if ((xi.pos - q0_).norm() > (Ra_ + epsilon))
  //   {
  //     std::cout << "norm(xi.pos-q0_)=" << (traj_solution_.back().pos - q0_).norm() << std::endl;
  //     std::cout << "These are the control points" << std::endl;
  //     printQVA(q);

  //     bool is_feasible = isFeasible(q, n, d);
  //     std::cout << "is Feasible= " << is_feasible << std::endl;
  //     std::cout << "got_a_feasible_solution_= " << got_a_feasible_solution_ << std::endl;

  //     abort();
  //   }
  // }

  //////////////////////////////

  // Force the last position to be the final_state_ (it's not guaranteed to be because of the discretization with dc_)
  traj_solution_.back().vel = final_state_.vel;
  traj_solution_.back().accel = final_state_.accel;
  traj_solution_.back().jerk = Eigen::Vector3d::Zero();

  return true;
}

void SolverNlopt::getSolution(mt::PieceWisePol &solution)
{
  solution = solution_;
}

void SolverNlopt::saturateQ(std::vector<Eigen::Vector3d> &q)
{
  for (int i = 0; i < q.size(); i++)
  {
    q[i].z() = std::max(q[i].z(), z_min_);  // Make sure it's within the limits
    q[i].z() = std::min(q[i].z(), z_max_);  // Make sure it's within the limits
  }
}

void SolverNlopt::generateStraightLineGuess()
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

nlopt::algorithm SolverNlopt::getSolver(std::string &solver)
{
  // nlopt::algorithm_from_string("LD_MMA"); //doesn't work in c++

  if (solver == "LD_MMA")
  {
    return nlopt::LD_MMA;
  }
  else if (solver == "LN_NELDERMEAD")
  {
    return nlopt::LN_NELDERMEAD;
  }
  else if (solver == "LN_SBPLX")
  {
    return nlopt::LN_SBPLX;
  }
  else if (solver == "LN_PRAXIS")
  {
    return nlopt::LN_PRAXIS;
  }
  else if (solver == "LD_AUGLAG")
  {
    return nlopt::LD_AUGLAG;
  }
  else if (solver == "LD_AUGLAG_EQ")
  {
    return nlopt::LD_AUGLAG_EQ;
  }
  else if (solver == "LN_BOBYQA")
  {
    return nlopt::LN_BOBYQA;
  }
  else if (solver == "LD_SLSQP")
  {
    return nlopt::LD_SLSQP;
  }
  else if (solver == "LN_NEWUOA")
  {
    return nlopt::LN_NEWUOA;
  }
  else if (solver == "LN_NEWUOA_BOUND")
  {
    return nlopt::LN_NEWUOA_BOUND;
  }
  else if (solver == "LD_TNEWTON_PRECOND_RESTART")
  {
    return nlopt::LD_TNEWTON_PRECOND_RESTART;
  }
  else if (solver == "LD_TNEWTON_RESTART")
  {
    return nlopt::LD_TNEWTON_RESTART;
  }
  else if (solver == "LD_TNEWTON_PRECOND")
  {
    return nlopt::LD_TNEWTON_PRECOND;
  }
  else if (solver == "LD_VAR1")
  {
    return nlopt::LD_VAR1;
  }
  else if (solver == "LD_VAR2")
  {
    return nlopt::LD_VAR2;
  }
  else if (solver == "LD_LBFGS_NOCEDAL")
  {
    return nlopt::LD_LBFGS_NOCEDAL;
  }
  else if (solver == "LD_LBFGS")
  {
    return nlopt::LD_LBFGS;
  }
  else if (solver == "LD_CCSAQ")
  {
    return nlopt::LD_CCSAQ;
  }
  else
  {
    std::cout << "Are you sure this solver exists?" << std::endl;
    abort();
  }
}

std::string SolverNlopt::getResultCode(int &result)
{
  switch (result)
  {
    case -5:
      return "Forced_Stop";
    case -4:
      return "Roundoff_limited";
    case -3:
      return "Out_of_memory";
    case -2:
      return "Invalid_args";
    case -1:
      return "Failure";
    case 1:
      return "Success";
    case 2:
      return "Stopval_reached";
    case 3:
      return "Ftol_reached";
    case 4:
      return "Xtol_reached";
    case 5:
      return "Maxeval_reached";
    case 6:
      return "Maxtime_reached";
    default:
      return "Result_Code_unknown";
  }
}

void SolverNlopt::printIndexesVariables()
{
  std::cout << "_______________________" << std::endl;
  std::cout << "Indexes variables" << std::endl;
  std::cout << "q: " << i_min_ << "-->" << i_max_ << std::endl;
  std::cout << "n: " << j_min_ << "-->" << j_max_ << std::endl;
  std::cout << "d: " << k_min_ << "-->" << k_max_ << std::endl;

  std::cout << "Total number of Variables: " << num_of_variables_ << std::endl;
  std::cout << "_______________________" << std::endl;
}

void SolverNlopt::printIndexesConstraints()
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

void SolverNlopt::printQVA(const std::vector<Eigen::Vector3d> &q)
{
  std::cout << red << "Position cps, " << blue << "Velocity cps, " << green << "Accel cps" << reset << std::endl;

  std::cout << "BSPLINE Basis: " << std::endl;

  std::vector<Eigen::Vector3d> cps_vel;

  for (int i = 0; i <= (N_ - 2); i++)
  {
    Eigen::Vector3d vi = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
    Eigen::Vector3d vip1 = p_ * (q[i + 1 + 1] - q[i + 1]) / (knots_(i + 1 + p_ + 1) - knots_(i + 1 + 1));
    Eigen::Vector3d ai = (p_ - 1) * (vip1 - vi) / (knots_(i + p_ + 1) - knots_(i + 2));

    cps_vel.push_back(vi);

    std::cout << "***i= " << i << red << q[i].transpose() << blue << "   " << vi.transpose() << green << "   "
              << ai.transpose() << reset << std::endl;
  }

  int i = N_ - 1;

  Eigen::Vector3d vi = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
  std::cout << "***i= " << i << red << q[i].transpose() << blue << "   " << vi.transpose() << reset << std::endl;
  cps_vel.push_back(vi);

  i = N_;

  std::cout << "***i= " << i << red << q[i].transpose() << reset << std::endl;

  std::cout << std::endl;
  if (basis_ == MINVO)
  {
    std::cout << "MINVO Basis: " << std::endl;

    std::cout << blue << "Position cps" << reset << std::endl;

    for (int j = 3; j < q.size(); j++)
    {  // Note that for each interval there are 3 cps, but in the next interval they'll be different
      int interval = j - 3;

      Eigen::Matrix<double, 3, 4> Qbs;
      Eigen::Matrix<double, 3, 4> Qmv;

      Qbs.col(0) = q[j - 3];
      Qbs.col(1) = q[j - 2];
      Qbs.col(2) = q[j - 1];
      Qbs.col(3) = q[j];

      transformPosBSpline2otherBasis(Qbs, Qmv, interval);

      std::cout << "***i= " << interval << std::endl;
      std::cout << Qmv << std::endl;

      std::cout << "norm= " << std::endl;
      for (int jj = 0; jj < Qmv.cols(); jj++)
      {
        std::cout << (Qmv.col(jj) - q0_).norm() << ", ";
      }
      std::cout << std::endl;
    }

    /////

    std::cout << blue << "Velocity cps" << reset << std::endl;

    for (int j = 2; j < cps_vel.size(); j++)
    {  // Note that for each interval there are 3 cps, but in the next interval they'll be different
      int interval = j - 2;

      Eigen::Matrix<double, 3, 3> Qbs;
      Eigen::Matrix<double, 3, 3> Qmv;

      Qbs.col(0) = cps_vel[j - 2];
      Qbs.col(1) = cps_vel[j - 1];
      Qbs.col(2) = cps_vel[j];

      transformVelBSpline2otherBasis(Qbs, Qmv, interval);

      std::cout << "***i= " << interval << std::endl;
      std::cout << Qmv << std::endl;
    }
  }
}