// Jesus Tordesillas Torres, jtorde@mit.edu, May 2020

// This file simply creates a solverNlopt object, sets its params and some random obstacles, and then checks all the
// gradients numerically

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>
#include <random>
#include "./../../timer.hpp"

#include "solverNlopt.hpp"
#include "nlopt_utils.hpp"

bool nlopt_utils::checkGradientsNlopt()
{
  double z_ground = -2.0;
  double z_max = 2.0;
  double dc = 0.01;
  double Ra = 4.0;
  int deg = 3;
  int samples_per_interval = 1;
  double weight = 10000.0;  // Note that internally, this weight will be changed to other value for the check (to get
                            // rid of numerical issues)
  double epsilon_tol_constraints = 0.1;
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
  Eigen::Vector3d v_max(10.0, 10.0, 10.0);
  Eigen::Vector3d a_max(60.0, 60.0, 60.0);
  state initial;
  initial.pos = Eigen::Vector3d(-4.0, 0.0, 0.0);
  state final;
  final.pos = Eigen::Vector3d(4.0, 0.0, 0.0);

  double t_min = 0.0;
  double t_max = t_min + (final.pos - initial.pos).norm() / (0.3 * v_max(0));

  Polyhedron_Std hull;

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, -70.0));

  hull.push_back(Eigen::Vector3d(-0.5, 0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, 0.5, -70.0));

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, -70.0));
  hull.push_back(Eigen::Vector3d(-0.5, 0.5, -70.0));

  hull.push_back(Eigen::Vector3d(0.5, 0.5, 70.0));

  int n_pol = 8;

  ConvexHullsOfCurves_Std hulls_curves;
  ConvexHullsOfCurve_Std hulls_curve;
  // Assummes static obstacle
  for (int i = 0; i < n_pol; i++)
  {
    hulls_curve.push_back(hull);
  }

  hulls_curves.push_back(hulls_curve);

  SolverNlopt snlopt(n_pol, deg, hulls_curves.size(), weight, epsilon_tol_constraints, xtol_rel, ftol_rel,
                     solver);  // snlopt(a,g) a polynomials of degree 3
  snlopt.setBasisUsedForCollision(snlopt.B_SPLINE);
  snlopt.setHulls(hulls_curves);
  snlopt.setDistanceToUseStraightLine(Ra / 2.0);
  snlopt.setKappaAndMu(kappa, mu);
  snlopt.setZminZmax(z_ground, z_max);
  snlopt.setAStarSamplesAndFractionVoxel(a_star_samp_x, a_star_samp_y, a_star_samp_z, 0.5);
  snlopt.setMaxValues(v_max.x(), a_max.x());  // v_max and a_max
  snlopt.setDC(dc);                           // dc
  snlopt.setTminAndTmax(t_min, t_max);
  snlopt.setMaxRuntime(runtime);
  snlopt.setInitAndFinalStates(initial, final);

  return snlopt.checkGradientsUsingFiniteDiff();
}