/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef SOLVER_NLOPT_HPP
#define SOLVER_NLOPT_HPP
#include <Eigen/Dense>

#include <iomanip>  //set precision
#include <nlopt.hpp>
#include "mader_types.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include <decomp_geometry/polyhedron.h>  //For Polyhedron  and Hyperplane definition
#include "separator.hpp"
#include "octopus_search.hpp"
#include "solver_params.hpp"

typedef MADER_timers::Timer MyTimer;

class SolverNlopt
{
public:
  SolverNlopt(ms::par_solver &par);

  ~SolverNlopt();

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

  double improvement_ = 0.0;

protected:
private:
  void saturateQ(std::vector<Eigen::Vector3d> &q);

  bool isDegenerate(const std::vector<double> &x);

  void transformPosBSpline2otherBasis(const Eigen::Matrix<double, 3, 4> &Qbs, Eigen::Matrix<double, 3, 4> &Qmv,
                                      int interval);
  void transformVelBSpline2otherBasis(const Eigen::Matrix<double, 3, 3> &Qbs, Eigen::Matrix<double, 3, 3> &Qmv,
                                      int interval);

  void generateRandomGuess();
  void generateAStarGuess();
  void generateStraightLineGuess();

  void sampleFeasible(Eigen::Vector3d &qiP1, std::vector<Eigen::Vector3d> &q);

  void printStd(const std::vector<Eigen::Vector3d> &v);
  void printStd(const std::vector<double> &v);
  void generateGuessNDFromQ(const std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                            std::vector<double> &d);

  void fillPlanesFromNDQ(std::vector<Hyperplane3D> &planes_, const std::vector<Eigen::Vector3d> &n,
                         const std::vector<double> &d, const std::vector<Eigen::Vector3d> &q);

  void generateRandomD(std::vector<double> &d);
  void generateRandomN(std::vector<Eigen::Vector3d> &n);
  void generateRandomQ(std::vector<Eigen::Vector3d> &q);

  nlopt::algorithm getSolver(std::string &solver);

  bool isADecisionCP(int i);

  template <class T>
  bool isFeasible(const T x);

  bool isFeasible(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                  const std::vector<double> &d);

  void printQVA(const std::vector<Eigen::Vector3d> &q);

  void assignEigenToVector(double *grad, int index, const Eigen::Vector3d &tmp);

  template <class T>
  void x2qnd(T &x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d);

  void qnd2x(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n, const std::vector<double> &d,
             std::vector<double> &x);

  int gIndexQ(int i);  // Element jth of control point ith
  int gIndexN(int i);  // Element jth of normal ith
  int gIndexD(int i);

  void printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d);

  // r is the constraint index
  // nn is the number of variables
  // var_gindex is the index of the variable of the first element of the vector
  void toGradDiffConstraintsDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r, int nn);

  void toGradSameConstraintDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r, int nn);

  void assignValueToGradConstraints(int var_gindex, const double &tmp, double *grad, int r, int nn);

  // This function has to be static, see example
  // https://github.com/HKUST-Aerial-Robotics/Fast-Planner/blob/master/fast_planner/bspline_opt/src/bspline_optimizer.cpp
  // and here: https://github.com/stevengj/nlopt/issues/246
  static double myObjFunc(unsigned nn, const double *x, double *grad, void *my_func_data);

  // See example https://github.com/stevengj/nlopt/issues/168
  static void myIneqConstraints(unsigned m, double *result, unsigned nn, const double *x, double *grad, void *f_data);

  // double computeObjFunction(unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d>
  // &n,
  //                           std::vector<double> &d);

  double computeObjFunctionJerk(unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q,
                                std::vector<Eigen::Vector3d> &n, std::vector<double> &d);

  void computeConstraints(unsigned m, double *constraints, unsigned nn, double *grad,
                          const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                          const std::vector<double> &d);

  void initializeNumOfConstraints();

  void printInfeasibleConstraints(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                  std::vector<double> &d);

  // template <class T>
  // void printInfeasibleConstraints(const T x);

  template <class T>
  int getNumberOfInfeasibleConstraints(const T &constraints);

  template <class T>
  bool areTheseConstraintsFeasible(const T &constraints);

  template <class T>
  void printInfeasibleConstraints(const T &constraints);

  std::string getResultCode(int &result);

  void findCentroidHull(const mt::Polyhedron_Std &hull, Eigen::Vector3d &centroid);

  int lastDecCP();

  // bool intersects();

  // void computeVeli(Eigen::Vector3d &vel, std::vector<Eigen::Vector3d> &q);

  // void computeAcceli(Eigen::Vector3d &accel, std::vector<Eigen::Vector3d> &q);

  // bool satisfiesVmaxAmax(std::vector<Eigen::Vector3d> &q);

  void printIndexesConstraints();
  void printIndexesVariables();

  mt::PieceWisePol solution_;

  int basis_ = B_SPLINE;

  int deg_pol_ = 3;
  int num_pol_ = 5;
  int p_ = 5;
  int i_min_;
  int i_max_;
  int j_min_;
  int j_max_;
  int k_min_;
  int k_max_;
  int M_;
  int N_;

  int num_of_variables_;
  int num_of_normals_;
  int num_of_constraints_;
  int num_of_obst_;
  int num_of_segments_;

  nlopt::algorithm solver_;

  bool got_a_feasible_solution_ = false;
  double time_first_feasible_solution_ = 0.0;

  double best_cost_so_far_ = std::numeric_limits<double>::max();

  std::vector<double> best_feasible_sol_so_far_;

  std::vector<Hyperplane3D> planes_;

  double epsilon_tol_constraints_;
  double xtol_rel_;
  double ftol_rel_;

  std::vector<double> x_;  // Here the initial guess, and the solution, are stored

  double lowest_cost_so_far_;
  double dc_;
  Eigen::RowVectorXd knots_;
  double t_init_;
  double t_final_;
  double deltaT_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;

  double weight_ = 10000;
  double weight_modified_ = 10000;

  // bool force_final_state_ = true;

  mt::state initial_state_;
  mt::state final_state_;

  // double constraints_[10000];  // this number should be very big!! (hack, TODO)

  Eigen::Vector3d q0_, q1_, q2_, qNm2_, qNm1_, qN_;

  mt::ConvexHullsOfCurves_Std hulls_;

  MyTimer opt_timer_;

  double max_runtime_ = 2;  //[seconds]

  // Eigen::Vector3d initial_point_;
  // Eigen::Vector3d final_point_;
  // nlopt::opt *opt_ = nullptr;
  // nlopt::opt *local_opt_ = nullptr;

  // nlopt::opt opt_;
  // nlopt::opt local_opt_;

  // Guesses
  std::vector<Eigen::Vector3d> n_guess_;  // Guesses for the normals
  std::vector<Eigen::Vector3d> q_guess_;  // Guesses for the normals
  std::vector<double> d_guess_;           // Guesses for the normals

  Eigen::MatrixXd R_;  // This matrix is [r0, r1, r2, r3, r0, r1, r2, r3] (for two segments)

  // separator::Separator *separator_solver;

  int index_const_obs_ = 0;
  int index_const_vel_ = 0;
  int index_const_accel_ = 0;
  int index_const_normals_ = 0;

  double kappa_ = 0.2;  // kappa_*max_runtime_ is spent on the initial guess
  double mu_ = 0.5;     // mu_*max_runtime_ is spent on the optimization

  double x_min_ = -std::numeric_limits<double>::max();
  double x_max_ = std::numeric_limits<double>::max();

  double y_min_ = -std::numeric_limits<double>::max();
  double y_max_ = std::numeric_limits<double>::max();

  double z_min_ = -std::numeric_limits<double>::max();
  double z_max_ = std::numeric_limits<double>::max();

  int num_of_LPs_run_ = 0;
  int num_of_QCQPs_run_ = 0;

  int a_star_samp_x_ = 7;
  int a_star_samp_y_ = 7;
  int a_star_samp_z_ = 7;

  double time_needed_;
  double dist_to_use_straight_guess_ = std::numeric_limits<double>::max();

  // transformation between the B-spline control points and other basis
  std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2basis_;
  std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2basis_;
  std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs_;

  double a_star_bias_ = 1.0;
  double a_star_fraction_voxel_size_ = 0.5;
  bool allow_infeasible_guess_ = false;

  separator::Separator *separator_solver_;
  OctopusSearch *octopusSolver_;

  double Ra_ = 1e10;
};
#endif