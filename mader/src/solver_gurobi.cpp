/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

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

using namespace termcolor;

SolverGurobi::SolverGurobi(par_solver &par)
{
  deg_pol_ = par.deg_pol;
  num_pol_ = par.num_pol;

  p_ = deg_pol_;
  M_ = num_pol_ + 2 * p_;
  N_ = M_ - p_ - 1;
  num_of_segments_ = (M_ - 2 * p_);  // this is the same as num_pol_

  ///////////////////////////////////////
  mt::basisConverter basis_converter;

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

  dc_ = par.dc;
  v_max_ = par.v_max;
  mv_max_ = -v_max_;

  a_max_ = par.a_max;
  ma_max_ = -a_max_;

  //  allow_infeasible_guess_ = par.allow_infeasible_guess;

  weight_ = par.weight;

  separator_solver_ = new separator::Separator();
  octopusSolver_ = new OctopusSearch(par.basis, num_pol_, deg_pol_, par.alpha_shrink);
}

SolverGurobi::~SolverGurobi()
{
}

void SolverGurobi::addObjective()
{
  GRBQuadExpr cost = 0.0;

  Eigen::Matrix<double, 4, 1> tmp;
  tmp << 6.0, 0.0, 0.0, 0.0;

  for (int i = 0; i < num_of_segments_; i++)
  {
    Eigen::Matrix<double, -1, 1> A_i_times_tmp = A_pos_bs_[i] * tmp;  // TODO (this is 4x1)

    GRBMatrix Q;
    Q.push_back(GRBVector{ q_exp_[i][0], q_exp_[i + 1][0], q_exp_[i + 2][0], q_exp_[i + 3][0] });  // row0
    Q.push_back(GRBVector{ q_exp_[i][1], q_exp_[i + 1][1], q_exp_[i + 2][1], q_exp_[i + 3][1] });  // row1
    Q.push_back(GRBVector{ q_exp_[i][2], q_exp_[i + 1][2], q_exp_[i + 2][2], q_exp_[i + 3][2] });  // row2

    cost += getNorm2(matrixMultiply(Q, eigenVector2std(A_i_times_tmp)));
  }

  cost += weight_modified_ * getNorm2(q_exp_[N_] - eigenVector2std(final_state_.pos));

  m_.setObjective(cost, GRB_MINIMIZE);
}

void SolverGurobi::addConstraints()
{
  // double epsilon = 1.0;  // See http://www.joyofdata.de/blog/testing-linear-separability-linear-programming-r-glpk/

  addVectorEqConstraint(m_, q_exp_[0], q0_);
  addVectorEqConstraint(m_, q_exp_[1], q1_);
  addVectorEqConstraint(m_, q_exp_[2], q2_);

  Eigen::Vector3d zeros = Eigen::Vector3d::Zero();

  // Final velocity and acceleration are zero \equiv q_exp_[N_]=q_exp_[N_-1]=q_exp_[N_-2]
  addVectorEqConstraint(m_, q_exp_[N_] - q_exp_[N_ - 1], zeros);
  addVectorEqConstraint(m_, q_exp_[N_ - 1] - q_exp_[N_ - 2], zeros);

  /////////////////////////////////////////////////
  ////// PLANES AND SPHERE CONSTRAINTS    /////////
  /////////////////////////////////////////////////

  for (int i = 0; i <= (N_ - 3); i++)  // i  is the interval (\equiv segment)
  {
    GRBMatrix Qbs;
    GRBMatrix Qmv;  // other basis "basis_" (minvo, bezier,...).

    Qbs.push_back(GRBVector{ q_exp_[i][0], q_exp_[i + 1][0], q_exp_[i + 2][0], q_exp_[i + 3][0] });  // row0
    Qbs.push_back(GRBVector{ q_exp_[i][1], q_exp_[i + 1][1], q_exp_[i + 2][1], q_exp_[i + 3][1] });  // row1
    Qbs.push_back(GRBVector{ q_exp_[i][2], q_exp_[i + 1][2], q_exp_[i + 2][2], q_exp_[i + 3][2] });  // row2

    transformPosBSpline2otherBasis(Qbs, Qmv, i);  // each row of Qmv will contain a "basis_" control point

    /////// Plane constraints
    /////////////////////////////////////////////////

    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      int ip = obst_index * num_of_segments_ + i;  // index plane

      // impose that all the vertexes of the obstacle are on one side of the plane
      // for (int j = 0; j < hulls_[obst_index][i].cols(); j++)  // Eigen::Vector3d vertex : hulls_[obst_index][i]
      // {
      //   Eigen::Vector3d vertex = hulls_[obst_index][i].col(j);
      //   m_.addConstr((-(n_[ip].dot(vertex) + d_[ip] - epsilon)) <= 0, "plane" + std::to_string(j));
      //   // constraints[r] = -(n[ip].dot(vertex) + d[ip] - epsilon);  // f<=0
      // }

      // and the control points on the other side

      for (int u = 0; u <= 3; u++)
      {
        GRBVector q_ipu = getColumn(Qmv, u);
        GRBLinExpr dot = n_[ip].x() * q_ipu[0] + n_[ip].y() * q_ipu[1] + n_[ip].z() * q_ipu[2];
        m_.addConstr(dot + d_[ip] + 1 <= 0);
      }
    }

    /////// Sphere constraints
    ///////////////////////////////////////////////
    Eigen::Vector3d q_ipu;
    for (int u = 0; u <= 3; u++)
    {
      GRBVector q_ipu = getColumn(Qmv, u);
      GRBQuadExpr tmp = getNorm2(q_ipu - eigenVector2std(q0_));
      m_.addQConstr(tmp <= Ra_ * Ra_);
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

    GRBVector v_iM2 = ciM2 * (q_exp_[i - 1] - q_exp_[i - 2]);
    GRBVector v_iM1 = ciM1 * (q_exp_[i] - q_exp_[i - 1]);
    GRBVector v_i = ci * (q_exp_[i + 1] - q_exp_[i]);

    GRBMatrix Qbs;
    GRBMatrix Qmv;  // other basis "basis_" (minvo, bezier,...).

    Qbs.push_back(GRBVector{ v_iM2[0], v_iM1[0], v_i[0] });  // row0
    Qbs.push_back(GRBVector{ v_iM2[1], v_iM1[1], v_i[1] });  // row1
    Qbs.push_back(GRBVector{ v_iM2[2], v_iM1[2], v_i[2] });  // row2

    transformVelBSpline2otherBasis(Qbs, Qmv, i - 2);

    for (int j = 0; j < 3; j++)
    {  // loop over each of the velocity control points (v_{i-2+j}) of the new basis
       //|v_{i-2+j}| <= v_max ////// v_{i-2}, v_{i-1}, v_{i}

      GRBVector v_iM2Pj = getColumn(Qmv, j);
      addVectorLessEqualConstraint(m_, v_iM2Pj, v_max_);      // v_max_
      addVectorGreaterEqualConstraint(m_, v_iM2Pj, mv_max_);  // mv_max_
    }
  }

  /////////////////////////////////////////////////
  ////////// ACCELERATION CONSTRAINTS    //////////
  /////////////////////////////////////////////////

  for (int i = 1; i <= (N_ - 3); i++)  // a0 is already determined by the initial state
  {
    double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
    double c2 = p_ / (knots_(i + p_ + 1 + 1) - knots_(i + 1 + 1));
    double c3 = (p_ - 1) / (knots_(i + p_ + 1) - knots_(i + 2));

    GRBVector v_i = c1 * (q_exp_[i + 1] - q_exp_[i]);
    GRBVector v_iP1 = c2 * (q_exp_[i + 2] - q_exp_[i + 1]);
    GRBVector a_i = c3 * (v_iP1 - v_i);

    addVectorLessEqualConstraint(m_, a_i, a_max_);      // v_max_
    addVectorGreaterEqualConstraint(m_, a_i, ma_max_);  // mv_max_
  }
}

void SolverGurobi::getPlanes(std::vector<Hyperplane3D> &planes)
{
  planes = planes_;
}

int SolverGurobi::getNumOfLPsRun()
{
  return octopusSolver_->getNumOfLPsRun();
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

void SolverGurobi::fillPlanesFromNDQ(const std::vector<Eigen::Vector3d> &n, const std::vector<double> &d,
                                     const std::vector<Eigen::Vector3d> &q)
{
  planes_.clear();

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      int ip = obst_index * num_of_segments_ + i;  // index plane
      Eigen::Vector3d centroid_hull;
      findCentroidHull(hulls_[obst_index][i], centroid_hull);

      Eigen::Vector3d point_in_plane;

      Eigen::Matrix<double, 3, 4> Qmv, Qbs;  // minvo. each column contains a MINVO control point
      Qbs.col(0) = q[i];
      Qbs.col(1) = q[i + 1];
      Qbs.col(2) = q[i + 2];
      Qbs.col(3) = q[i + 3];

      transformPosBSpline2otherBasis(Qbs, Qmv, i);

      Eigen::Vector3d centroid_cps = Qmv.rowwise().mean();

      // the colors refer to the second figure of
      // https://github.com/mit-acl/separator/tree/06c0ddc6e2f11dbfc5b6083c2ea31b23fd4fa9d1

      // Equation of the blue planes is n'x+d == -1
      // Convert here to equation [A B C]'x+D ==0
      double A = n[ip].x();
      double B = n[ip].y();
      double C = n[ip].z();
      double D = d[ip] + 1;

      /////////////////// OPTION 1: point_in_plane = intersection between line  centroid_cps --> centroid_hull
      // bool intersects = getIntersectionWithPlane(centroid_cps, centroid_hull, Eigen::Vector4d(A, B, C, D),
      //                                            point_in_plane);  // result saved in point_in_plane

      //////////////////////////

      /////////////////// OPTION 2: point_in_plane = intersection between line  centroid_cps --> closest_vertex
      double dist_min = std::numeric_limits<double>::max();  // delta_min will contain the minimum distance between
                                                             // the centroid_cps and the vertexes of the obstacle
      int index_closest_vertex = 0;
      for (int j = 0; j < hulls_[obst_index][i].cols(); j++)
      {
        Eigen::Vector3d vertex = hulls_[obst_index][i].col(j);

        double distance_to_vertex = (centroid_cps - vertex).norm();
        if (distance_to_vertex < dist_min)
        {
          dist_min = distance_to_vertex;
          index_closest_vertex = j;
        }
      }

      Eigen::Vector3d closest_vertex = hulls_[obst_index][i].col(index_closest_vertex);

      bool intersects = getIntersectionWithPlane(centroid_cps, closest_vertex, Eigen::Vector4d(A, B, C, D),
                                                 point_in_plane);  // result saved in point_in_plane

      //////////////////////////

      if (intersects == false)
      {
        // TODO: this msg is printed sometimes in Multi-Agent simulations. Find out why
        std::cout << red << "There is no intersection, this should never happen (TODO)" << reset << std::endl;
        continue;  // abort();
      }

      Hyperplane3D plane(point_in_plane, n[i]);
      planes_.push_back(plane);
    }
  }
}

// returns 1 if there is an intersection between the segment P1-P2 and the plane given by coeff=[A B C D]
// (Ax+By+Cz+D==0)  returns 0 if there is no intersection.
// The intersection point is saved in "intersection"
bool SolverGurobi::getIntersectionWithPlane(const Eigen::Vector3d &P1, const Eigen::Vector3d &P2,
                                            const Eigen::Vector4d &coeff, Eigen::Vector3d &intersection)
{
  double A = coeff[0];
  double B = coeff[1];
  double C = coeff[2];
  double D = coeff[3];
  // http://www.ambrsoft.com/TrigoCalc/Plan3D/PlaneLineIntersection_.htm
  double x1 = P1[0];
  double a = (P2[0] - P1[0]);
  double y1 = P1[1];
  double b = (P2[1] - P1[1]);
  double z1 = P1[2];
  double c = (P2[2] - P1[2]);
  double t = -(A * x1 + B * y1 + C * z1 + D) / (A * a + B * b + C * c);

  (intersection)[0] = x1 + a * t;
  (intersection)[1] = y1 + b * t;
  (intersection)[2] = z1 + c * t;

  bool result =
      (t < 0 || t > 1) ? false : true;  // False if the intersection is with the line P1-P2, not with the segment P1-P2

  return result;
}

// Intersection between the ray p0<-->p1 and the plane n'x+f==0
// Eigen::Vector3d SolverGurobi::intersectPoint(Eigen::Vector3d p0, Eigen::Vector3d p1, Eigen::Vector3d n, double f)
// {
//   Eigen::Vector3d planePoint = Eigen::Vector3d::Zero();

//   if (fabs(n.x()) != 0)
//   {
//     planePoint << -f / (n.x()), 0.0, 0.0;
//   }
//   else if (fabs(n.y()) != 0)
//   {
//     planePoint << 0.0, -f / (n.y()), 0.0;
//   }
//   else
//   {
//     planePoint << 0, 0, -f / (n.z());
//   }

//   // https://rosettacode.org/wiki/Find_the_intersection_of_a_line_with_a_plane#C.2B.2B

//   Eigen::Vector3d planeNormal = n.normalize();
//   Vector3D diff = p0 - planePoint;
//   Eigen::Vector3d rayVector = p1 - p0;

//   return p0 - rayVector * (diff.dot(planeNormal)) / (rayVector.dot(planeNormal));
// }

void SolverGurobi::setHulls(mt::ConvexHullsOfCurves_Std &hulls)

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

  std::vector<std::string> coords = { "x", "y", "z" };
  std::vector<double> mins = { x_min_, y_min_, z_min_ };
  std::vector<double> maxs = { x_max_, y_max_, z_max_ };

  // Create the variables: control points
  q_var_.clear();
  q_exp_.clear();
  for (int i = 0; i <= N_; i++)
  {
    std::vector<GRBVar> q_var_i;
    GRBVector q_exp_i;
    for (int j = 0; j < 3; j++)  // x,y,z
    {
      GRBVar tmp;
      if (fabs(maxs[j] - mins[j]) > 100)
      {  // this is to prevent numerical issues with the solver (happen when the bounds are too large)
        // Here we let the variable be free (by using -GRB_INFINITY and GRB_INFINITY)
        tmp = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "q_" + std::to_string(i) + coords[j]);
      }
      else
      {
        tmp = m_.addVar(mins[j], maxs[j], 0, GRB_CONTINUOUS, "q_" + std::to_string(i) + coords[j]);
      }
      q_var_i.push_back(tmp);
      q_exp_i.push_back(GRBLinExpr(tmp));
    }
    q_var_.push_back(q_var_i);
    q_exp_.push_back(q_exp_i);
  }

  int num_of_cpoints = N_ + 1;

  num_of_normals_ = num_of_segments_ * num_of_obst_;
}

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
  //////////////////

  weight_modified_ = weight_ * (final_state_.pos - initial_state_.pos).norm();

  // See https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
  // See also eq. 15 of the paper "Robust and Efficent quadrotor..."

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

  return true;
}

bool SolverGurobi::optimize()
{
  // reset some stuff
  traj_solution_.clear();
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
  bool guess_is_feasible = generateAStarGuess();  // I obtain q_quess_, n_guess_, d_guess_
  if (guess_is_feasible == false)
  {
    std::cout << "Planes haven't been found" << std::endl;
    return false;
  }
  n_ = n_guess_;
  d_ = d_guess_;

  resetCompleteModel(m_);
  m_.set("OutputFlag", std::to_string(0));  // verbose (1) or not (0)
  // See https://www.gurobi.com/documentation/9.0/refman/cpp_parameter_examples.html
  m_.set("TimeLimit", std::to_string(mu_ * max_runtime_));
  addObjective();
  addConstraints();
  m_.update();  // needed due to the lazy evaluation
  // m_.write("/home/jtorde/Desktop/ws/src/mader/model.lp");
  std::cout << "Starting optimization, allowing time = " << mu_ * max_runtime_ * 1000 << " ms" << std::endl;
  m_.optimize();

  int optimstatus = m_.get(GRB_IntAttr_Status);

  printGurobiStatus(optimstatus);

  int number_of_stored_solutions = m_.get(GRB_IntAttr_SolCount);

  std::vector<Eigen::Vector3d> q;

  //        optimstatus == GRB_SUBOPTIMAL //Don't include this one (sometimes it generates very long/suboptimal
  //        trajectories)

  // See https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html#sec:StatusCodes
  if ((optimstatus == GRB_OPTIMAL || optimstatus == GRB_TIME_LIMIT ||
       optimstatus == GRB_USER_OBJ_LIMIT ||                                    ///////////////
       optimstatus == GRB_ITERATION_LIMIT || optimstatus == GRB_NODE_LIMIT ||  ///////////////
       optimstatus == GRB_SOLUTION_LIMIT) &&
      number_of_stored_solutions > 0)

  {
    std::cout << green << "Gurobi found a solution" << reset << std::endl;
    // copy the solution
    for (auto tmp : q_exp_)
    {
      q.push_back(Eigen::Vector3d(tmp[0].getValue(), tmp[1].getValue(), tmp[2].getValue()));
    }
  }
  else
  {
    std::cout << red << "Gurobi failed to find a solution, using initial guess (which is feasible)" << reset
              << std::endl;
    q = q_guess_;
  }

  CPs2TrajAndPwp(q, traj_solution_, solution_, N_, p_, num_pol_, knots_, dc_);
  // Force last position =final_state_ (which it's not guaranteed because of the discretization with dc_)
  traj_solution_.back().vel = final_state_.vel;
  traj_solution_.back().accel = final_state_.accel;
  traj_solution_.back().jerk = Eigen::Vector3d::Zero();

  // std::cout << blue << "traj_solution_.size()=" << traj_solution_.size() <<reset<< std::endl;

  // Uncomment the following line if you wanna visualize the planes
  // fillPlanesFromNDQ(n_, d_, q);  // TODO: move this outside the SolverGurobi class

  return true;
}

void SolverGurobi::getSolution(mt::PieceWisePol &solution)
{
  solution = solution_;
}

/*  std::cout << "result" << std::endl;

  std::cout << "N_=" << N_ << std::endl;

  for (auto q_i : q_exp_)
  {
    std::cout << q_i[0].getValue() << ", " << q_i[1].getValue() << ", " << q_i[2].getValue() << std::endl;
  }*/

// bool SolverGurobi::optimize()

// {

//   best_cost_so_far_ = std::numeric_limits<double>::max();
//   best_feasible_sol_so_far_.clear();
//   got_a_feasible_solution_ = false;
//   best_feasible_sol_so_far_.resize(num_of_variables_);

//   qnd2x(q_guess_, n_guess_, d_guess_, x_);

//   // std::cout << "The guess is the following one:" << std::endl;
//   // printQVA(q_guess_);
//   // printQND(q_guess_, n_guess_, d_guess_);

//   // x2qnd(x_, q_guess_, n_guess_);

//   std::cout << bold << "The infeasible constraints of the initial Guess" << reset << std::endl;

//   printInfeasibleConstraints(q_guess_, n_guess_, d_guess_);

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
//     // std::cout << "obj_guess= " << std::setprecision(7) << obj_guess << reset << "(" << first_term_obj_guess << "
//     +
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
//     std::cout << "obj_guess= " << std::setprecision(7) << obj_guess << reset << "(" << first_term_obj_guess << " +
//     "
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