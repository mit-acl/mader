
#ifndef SOLVER_NLOPT_HPP
#define SOLVER_NLOPT_HPP
#include <Eigen/Dense>

#include <iomanip>
#include <nlopt.hpp>
#include "./../../faster_types.hpp"
#include <sstream>

/*#include <Eigen/Dense>
#include <type_traits>
#include <fstream>
#include "./../termcolor.hpp"

#include <decomp_ros_utils/data_ros_utils.h>
#include <unsupported/Eigen/Polynomials>
#include "./../faster_types.hpp"
using namespace termcolor;*/

class SolverNlopt
{
public:
  SolverNlopt(int num_pol, int deg_pol, bool force_final_state);

  ~SolverNlopt();

  bool optimize();

  void setInitAndFinalStates(state &initial_state, state &final_state);

  void setHulls(std::vector<std::vector<Eigen::Vector3d>> &hulls);
  void setTminAndTmax(double t_min, double t_max);

  void setMaxValues(double v_max, double a_max);

  void setDC(double dc);

  std::vector<state> X_temp_;

protected:
private:
  bool isADecisionCP(int i);

  void assignEigenToVector(double *grad, int index, const Eigen::Vector3d &tmp);

  template <class T>
  void toEigen(T x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d);

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
  static void multi_ineq_constraint(unsigned m, double *result, unsigned nn, const double *x, double *grad,
                                    void *f_data);

  void add_ineq_constraints(unsigned m, double *constraints, unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q,
                            std::vector<Eigen::Vector3d> &n, std::vector<double> &d);

  void initializeNumOfConstraints();

  void qndtoX(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
              const std::vector<double> &d, std::vector<double> &x);

  void printInfeasibleConstraints(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                  std::vector<double> &d);

  template <class T>
  void printInfeasibleConstraints(const T x);

  int lastDecCP();

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

  int num_of_segments_;

  double dc_;
  Eigen::RowVectorXd knots_;
  double t_min_;
  double t_max_;
  double deltaT_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;

  double weight_ = 10000;

  bool force_final_state_ = true;

  state initial_state_;
  state final_state_;

  double constraints_[10000];  // this number should be very big!! (hack)

  Eigen::Vector3d q0_, q1_, q2_;  //, qNm2_, qNm1_, qN_;

  std::vector<std::vector<Eigen::Vector3d>> hulls_;

  // Eigen::Vector3d initial_point_;
  // Eigen::Vector3d final_point_;
  nlopt::opt *opt_ = NULL;
  nlopt::opt *local_opt_ = NULL;

  Eigen::MatrixXd R_;  // This matrix is [r0, r1, r2, r3, r0, r1, r2, r3] (for two segments)
};
#endif