
#ifndef SOLVER_NLOPT_HPP
#define SOLVER_NLOPT_HPP
#include <Eigen/Dense>

#include <iomanip>
#include <nlopt.hpp>

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
  SolverNlopt(int num_pol, int deg_pol_);
  void setDegAndNumPol(int deg_pol_, int num_pol_);

  void optimize();

  void setInitAndFinalPoints(Eigen::Vector3d &initial_point, Eigen::Vector3d &final_point);

  void setHulls(std::vector<std::vector<Eigen::Vector3d>> hulls);
  void setTminAndTmax(double t_min, double t_max);

  void setMaxValues(double v_max, double a_max);

protected:
private:
  void assignEigenToVector(double *grad, int index, const Eigen::Vector3d &tmp);

  void toEigen(const double *x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
               std::vector<double> &d);

  void toEigen(const std::vector<double> &x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
               std::vector<double> &d);

  int gIndexQ(int i);  // Element jth of control point ith
  int gIndexN(int i);  // Element jth of normal ith
  int gIndexD(int i);

  void printQND(std::vector<Eigen::Vector3d> q, std::vector<Eigen::Vector3d> n, std::vector<double> d);

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

  double t_min_;
  double t_max_;
  double deltaT_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;

  std::vector<std::vector<Eigen::Vector3d>> hulls_;

  Eigen::Vector3d initial_point_;
  Eigen::Vector3d final_point_;
  nlopt::opt *opt_;
  nlopt::opt *local_opt_;

  Eigen::MatrixXd R_;  // This matrix is [r0, r1, r2, r3, r0, r1, r2, r3] (for two segments)
};
#endif