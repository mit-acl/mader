/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <iomanip>

#include <Eigen/Dense>
#include <random>
#include "timer.hpp"

#include "solver_gurobi.hpp"
#include "solver_params.hpp"
#include "termcolor.hpp"

#include <fstream>

using namespace termcolor;

typedef MADER_timers::Timer MyTimer;

ConvexHullsOfCurve createStaticObstacle(double x, double y, double z, int num_pol, double bbox_x, double bbox_y,
                                        double bbox_z)
{
  ConvexHullsOfCurve hulls_curve;
  std::vector<Point_3> points;

  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));

  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));

  CGAL_Polyhedron_3 hull_interval = cu::convexHullOfPoints(points);

  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull_interval);  // static obstacle
  }

  return hulls_curve;
}

int main()
{
  double bbox_x = 0.4;
  double bbox_y = 0.4;
  double bbox_z = 0.4;
  int num_pol = 4;

  ConvexHullsOfCurve hulls_curve = createStaticObstacle(0.0, 0.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z);
  ConvexHullsOfCurves hulls_curves;
  hulls_curves.push_back(hulls_curve);
  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls_curves);

  ms::par_solver parameters;
  parameters.v_max = 2 * Eigen::Vector3d::Ones();
  parameters.a_max = 2 * Eigen::Vector3d::Ones();
  parameters.dc = 0.01;
  parameters.dist_to_use_straight_guess = 1;
  parameters.a_star_samp_x = 5;
  parameters.a_star_samp_y = 5;
  parameters.a_star_samp_z = 5;
  parameters.a_star_fraction_voxel_size = 0.5;
  parameters.num_pol = num_pol;
  parameters.deg_pol = 3;
  parameters.weight = 1.0;
  parameters.epsilon_tol_constraints = 0.001;
  parameters.xtol_rel = 0.0000000000001;
  parameters.ftol_rel = 0.0000000000001;
  parameters.solver = "LD_MMA";
  parameters.basis = "B_SPLINE";
  parameters.a_star_bias = 1.0;
  parameters.allow_infeasible_guess = true;
  parameters.Ra = 4.0;

  parameters.alpha_shrink = 0.95;

  SolverGurobi my_solver(parameters);  // my_solver(a,g) a polynomials of degree 3
  my_solver.setMaxRuntimeKappaAndMu(0.2, 0.5, 0.5);
  mt::state initial_state;
  initial_state.pos = Eigen::Vector3d(-4.0, 0.0, 0.0);

  mt::state final_state;
  final_state.pos = Eigen::Vector3d(4.0, 0.0, 0.0);

  double t_min = 0.0;
  double t_max = t_min + (final_state.pos - initial_state.pos).norm() / (0.3 * parameters.v_max(0));

  std::cout << green << "t_final" << t_min << reset << std::endl;
  std::cout << green << "t_init_" << t_max << reset << std::endl;

  my_solver.setInitStateFinalStateInitTFinalT(initial_state, final_state, t_min, t_max);

  my_solver.setHulls(hulls_std);

  std::cout << "Calling optimize" << std::endl;
  bool converged = my_solver.optimize();

  // double time_needed = my_solver.getTimeNeeded();
  double delta = (t_max - t_min) / num_pol;
}
// std::ofstream myfile;
// myfile.open("/home/jtorde/Desktop/ws/src/mader/mader/src/solvers/nlopt/example.txt");

// double tmp = 8.0;
//  for (double tmp = 3; tmp < 50; tmp = tmp + 0.05)
// {

// if (converged)
// {
//   myfile << num_pol << ", " << delta << ", " << time_needed << std::endl;
// }
// //  }
// myfile.close();