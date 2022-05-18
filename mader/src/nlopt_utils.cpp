/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

// This file simply creates a solverNlopt object, sets its params and some random obstacles, and then checks all the
// gradients numerically

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>
#include <random>
#include "timer.hpp"

#include "solver_nlopt.hpp"
#include "nlopt_utils.hpp"

bool nlopt_utils::checkGradientsNlopt(std::string basis)
{
  double x_min = -10.0;
  double x_max = 10.0;

  double y_min = -10.0;
  double y_max = 10.0;

  double z_min = -4.0;
  double z_max = 4.0;
  double dc = 0.01;
  double Ra = 4.0;
  int deg_pol = 3;
  int samples_per_interval = 1;
  double weight = 10000.0;  // Note that internally, this weight will be changed to other value for the check (to get
                            // rid of numerical issues)
  double epsilon_tol_constraints = 0.001;
  double xtol_rel = 1e-07;
  double ftol_rel = 1e-07;
  std::string solver = "LD_MMA";
  double kappa = 0.0;
  double mu = 1.0;
  int a_star_samp_x = 7;
  int a_star_samp_y = 7;
  int a_star_samp_z = 7;
  double increment = 0.3;  // grid used to prune nodes that are on the same cell
  double runtime = 1.0;    //(not use this criterion)  //[seconds]
  double v_max = 10;
  double a_max = 60;
  mt::state initial;
  initial.pos = Eigen::Vector3d(-4.0, 1.0, -2.0);
  initial.vel = Eigen::Vector3d(0.0, 0.0, 0.0);
  initial.accel = Eigen::Vector3d(0.0, 0.0, 0.0);
  mt::state final;
  final.pos = Eigen::Vector3d(4.0, 2.0, 3.0);
  double dist_to_use_straight_guess = 1.0;
  double a_star_fraction_voxel_size = 0.0;
  int num_pol = 8;

  ms::par_solver param;

  param.x_min = x_min;
  param.x_max = x_max;

  param.y_min = y_min;
  param.y_max = y_max;

  param.z_min = z_min;
  param.z_max = z_max;
  param.v_max = Eigen::Vector3d(v_max, v_max, v_max);
  param.a_max = Eigen::Vector3d(a_max, a_max, a_max);
  param.dc = dc;
  param.dist_to_use_straight_guess = dist_to_use_straight_guess;
  param.a_star_samp_x = a_star_samp_x;
  param.a_star_samp_y = a_star_samp_y;
  param.a_star_samp_z = a_star_samp_z;
  param.a_star_fraction_voxel_size = a_star_fraction_voxel_size;
  param.num_pol = num_pol;
  param.deg_pol = deg_pol;
  param.weight = weight;
  param.epsilon_tol_constraints = epsilon_tol_constraints;
  param.xtol_rel = xtol_rel;
  param.ftol_rel = ftol_rel;
  param.solver = solver;
  param.basis = basis;

  double t_min = 0.0;
  double t_max = t_min + (final.pos - initial.pos).norm() / (0.3 * v_max);

  mt::Polyhedron_Std hull(3, 8);

  hull.col(0) = Eigen::Vector3d(-0.5, -0.5, -70.0);
  hull.col(1) = Eigen::Vector3d(-0.5, 0.5, 70.0);
  hull.col(2) = Eigen::Vector3d(0.5, -0.5, 70.0);
  hull.col(3) = Eigen::Vector3d(0.5, 0.5, -70.0);
  hull.col(4) = Eigen::Vector3d(-0.5, -0.5, 70.0);
  hull.col(5) = Eigen::Vector3d(0.5, -0.5, -70.0);
  hull.col(6) = Eigen::Vector3d(-0.5, 0.5, -70.0);
  hull.col(7) = Eigen::Vector3d(0.5, 0.5, 70.0);

  mt::ConvexHullsOfCurves_Std hulls_curves;
  mt::ConvexHullsOfCurve_Std hulls_curve;
  // Assummes static obstacle
  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull);
  }

  hulls_curves.push_back(hulls_curve);

  SolverNlopt snlopt(param);  // snlopt(a,g) a polynomials of degree 3
  snlopt.setHulls(hulls_curves);
  snlopt.setMaxRuntimeKappaAndMu(runtime, kappa, mu);

  snlopt.setInitStateFinalStateInitTFinalT(initial, final, t_min, t_max);

  return snlopt.checkGradientsUsingFiniteDiff();
}