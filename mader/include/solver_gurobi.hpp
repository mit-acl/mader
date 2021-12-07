/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef SOLVER_GUROBI_HPP
#define SOLVER_GUROBI_HPP
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <Eigen/StdVector>

#include <iomanip>  //set precision
#include "mader_types.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include <decomp_geometry/polyhedron.h>  //For Polyhedron  and Hyperplane definition
#include "separator.hpp"
#include "octopus_search.hpp"
#include "solver_params.hpp"

typedef MADER_timers::Timer MyTimer;

class SolverGurobi
{
public:
  SolverGurobi(ms::par_solver &par);

  ~SolverGurobi();

  bool optimize();

  // setters
  void setMaxRuntimeKappaAndMu(double runtime, double kappa, double mu);
  bool setInitStateFinalStateInitTFinalT(mt::state initial_state, mt::state final_state, double t_init,
                                         double &t_final);
  void setHulls(mt::ConvexHullsOfCurves_Std &hulls);

  mt::trajectory traj_solution_;

  // getters
  void getPlanes(std::vector<Hyperplane3D> &planes);
  int getNumOfLPsRun();
  int getNumOfQCQPsRun();
  void getSolution(mt::PieceWisePol &solution);
  double getTimeNeeded();

  int B_SPLINE = 1;  // B-Spline Basis
  int MINVO = 2;     // Minimum volume basis
  int BEZIER = 3;    // Bezier basis

  bool checkGradientsUsingFiniteDiff();

protected:
private:
  bool getIntersectionWithPlane(const Eigen::Vector3d &P1, const Eigen::Vector3d &P2, const Eigen::Vector4d &coeff,
                                Eigen::Vector3d &intersection);

  void addObjective();
  void addConstraints();

  void saturateQ(std::vector<Eigen::Vector3d> &q);

  // transform functions (with Eigen)
  void transformPosBSpline2otherBasis(const Eigen::Matrix<double, 3, 4> &Qbs, Eigen::Matrix<double, 3, 4> &Qmv,
                                      int interval);
  void transformVelBSpline2otherBasis(const Eigen::Matrix<double, 3, 3> &Qbs, Eigen::Matrix<double, 3, 3> &Qmv,
                                      int interval);

  // transform functions (with std)
  void transformPosBSpline2otherBasis(const std::vector<std::vector<GRBLinExpr>> &Qbs,
                                      std::vector<std::vector<GRBLinExpr>> &Qmv, int interval);

  void transformVelBSpline2otherBasis(const std::vector<std::vector<GRBLinExpr>> &Qbs,
                                      std::vector<std::vector<GRBLinExpr>> &Qmv, int interval);

  void generateRandomGuess();
  bool generateAStarGuess();
  void generateStraightLineGuess();

  void printStd(const std::vector<Eigen::Vector3d> &v);
  void printStd(const std::vector<double> &v);
  void generateGuessNDFromQ(const std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                            std::vector<double> &d);

  void fillPlanesFromNDQ(const std::vector<Eigen::Vector3d> &n, const std::vector<double> &d,
                         const std::vector<Eigen::Vector3d> &q);

  void generateRandomD(std::vector<double> &d);
  void generateRandomN(std::vector<Eigen::Vector3d> &n);
  void generateRandomQ(std::vector<Eigen::Vector3d> &q);

  void printQVA(const std::vector<Eigen::Vector3d> &q);

  void printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d);

  GRBEnv *env_ = new GRBEnv();
  GRBModel m_ = GRBModel(*env_);

  std::vector<std::vector<GRBVar>> q_var_;      // Each q_var_[i] has 3 elements (x,y,z)
  std::vector<std::vector<GRBLinExpr>> q_exp_;  // Each q_exp_[i] has 3 elements (x,y,z)

  std::vector<Eigen::Vector3d> n_;  // Each n_[i] has 3 elements (nx,ny,nz)
  std::vector<double> d_;           // d_[i] has 1 element

  void findCentroidHull(const mt::Polyhedron_Std &hull, Eigen::Vector3d &centroid);

  // void printIndexesConstraints();
  // void printIndexesVariables();

  mt::PieceWisePol solution_;

  int basis_ = B_SPLINE;

  int p_ = 5;
  int i_min_;
  int i_max_;
  int j_min_;
  int j_max_;
  int k_min_;
  int k_max_;
  int M_;
  int N_;

  int num_of_normals_;

  int num_of_obst_;
  int num_of_segments_;

  std::vector<Hyperplane3D> planes_;

  double dc_;
  Eigen::RowVectorXd knots_;
  double t_init_;
  double t_final_;
  double deltaT_;

  // double weight_ = 10000;
  double weight_modified_ = 10000;

  mt::state initial_state_;
  mt::state final_state_;

  Eigen::Vector3d q0_, q1_, q2_, qNm2_, qNm1_, qN_;

  mt::ConvexHullsOfCurves_Std hulls_;

  MyTimer opt_timer_;

  double max_runtime_ = 2;  //[seconds]

  // Guesses
  std::vector<Eigen::Vector3d> n_guess_;  // Guesses for the normals
  std::vector<Eigen::Vector3d> q_guess_;  // Guesses for the normals
  std::vector<double> d_guess_;           // Guesses for the normals

  double kappa_ = 0.2;  // kappa_*max_runtime_ is spent on the initial guess
  double mu_ = 0.5;     // mu_*max_runtime_ is spent on the optimization

  int num_of_QCQPs_run_ = 0;

  // transformation between the B-spline control points and other basis
  std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2basis_;
  std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2basis_;
  std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs_;

  separator::Separator *separator_solver_;
  OctopusSearch *octopusSolver_;

  ms::par_solver par_;

  GRBQuadExpr control_cost_ = 0.0;
  GRBQuadExpr terminal_cost_ = 0.0;
  GRBQuadExpr cost_ = 0.0;

  // double x_min_ = -std::numeric_limits<double>::max();
  // double x_max_ = std::numeric_limits<double>::max();

  // double y_min_ = -std::numeric_limits<double>::max();
  // double y_max_ = std::numeric_limits<double>::max();

  // double z_min_ = -std::numeric_limits<double>::max();
  // double z_max_ = std::numeric_limits<double>::max();

  // int deg_pol_ = 3;
  // int num_pol_ = 5;

  // int a_star_samp_x_ = 7;
  // int a_star_samp_y_ = 7;
  // int a_star_samp_z_ = 7;

  // double a_star_bias_ = 1.0;
  // double a_star_fraction_voxel_size_ = 0.5;
  // double Ra_ = 1e10;
};
#endif