// Jesus Tordesillas Torres, jtorde@mit.edu, January 2020

#include "solverNlopt.hpp"

#include <iostream>
#include <vector>
#include "timer.hpp"

#include <unsupported/Eigen/Splines>

typedef Timer MyTimer;

SolverNlopt::SolverNlopt(int num_pol, int deg_pol_)
{
  std::cout << "In the SolverNlopt Constructor\n";

  p_ = deg_pol_;
  M_ = num_pol_ + 2 * p_;
  N_ = M_ - p_ - 1;
  num_of_variables_ = 3 * (N_ + 1) + 3 * (M_ - 2 * p_) + (M_ - 2 * p_);  // total number of variables

  i_min_ = 0;
  i_max_ = 3 * (N_ + 1) - 1;
  j_min_ = i_max_ + 1;
  j_max_ = j_min_ + 3 * (M_ - 2 * p_) - 1;
  k_min_ = j_max_ + 1;
  k_max_ = k_min_ + (M_ - 2 * p_) - 1;

  std::cout << "M_= " << M_ << std::endl;
  std::cout << "N_= " << N_ << std::endl;

  std::cout << "i_min_= " << i_min_ << std::endl;
  std::cout << "i_max_= " << i_max_ << std::endl;
  std::cout << "j_min_= " << j_min_ << std::endl;
  std::cout << "j_max_= " << j_max_ << std::endl;
  std::cout << "k_min_= " << k_min_ << std::endl;
  std::cout << "k_max_= " << k_max_ << std::endl;

  std::cout << "num_of_variables_= " << num_of_variables_ << std::endl;

  int num_of_segments = M_ - 2 * p_;
  int num_of_cpoints = N_ + 1;

  std::cout << "gIndexQ(0)=" << gIndexQ(0) << std::endl;
  std::cout << "gIndexQ(num_of_segments-1)=" << gIndexQ(num_of_cpoints - 1) << std::endl;

  std::cout << "gIndexN(0)=" << gIndexN(0) << std::endl;
  std::cout << "gIndexN(num_of_segments-1)=" << gIndexN(num_of_segments - 1) << std::endl;

  std::cout << "gIndexD(0)=" << gIndexD(0) << std::endl;
  std::cout << "gIndexD(num_of_segments-1)=" << gIndexD(num_of_segments - 1) << std::endl;

  std::vector<double> x;
  for (int i = 0; i < num_of_variables_; i++)
  {
    x.push_back(i);
  }

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);
  printQND(q, n, d);

  opt_ = new nlopt::opt(nlopt::AUGLAG, num_of_variables_);
  local_opt_ = new nlopt::opt(nlopt::LD_MMA, num_of_variables_);
  local_opt_->set_xtol_rel(1e-4);
  opt_->set_local_optimizer(*local_opt_);
  opt_->set_xtol_rel(1e-4);  // Stopping criteria
  std::cout << "Done the SolverNlopt Constructor\n";

  opt_->set_maxeval(1e6);  // maximum number of evaluations. Negative --> don't use this criterion
  opt_->set_maxtime(5);    // maximum time in seconds. Negative --> don't use this criterion
}

void SolverNlopt::setTminAndTmax(double t_min, double t_max)
{
  t_min_ = t_min;
  t_max_ = t_max;

  deltaT_ = (t_max_ - t_min_) / (1.0 * (M_ - 2 * p_ - 1 + 1));
}

void SolverNlopt::setDC(double dc)
{
  dc_ = dc;
}

void SolverNlopt::setMaxValues(double v_max, double a_max)
{
  v_max_ = v_max * Eigen::Vector3d::Ones();
  a_max_ = a_max * Eigen::Vector3d::Ones();
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

void SolverNlopt::setHulls(std::vector<std::vector<Eigen::Vector3d>> hulls)

{
  hulls_ = hulls;

  for (int i = 0; i < hulls_.size(); i++)  // i  is the interval (\equiv segment)  //hulls_.size()
  {
    // impose that all the vertexes are on one side of the plane

    std::cout << "Interval " << i << " has" << std::endl;
    for (int vertex_index = 0; vertex_index < hulls_[i].size(); vertex_index++)  // hulls_[i][vertex_index].size()
    {
      Eigen::Vector3d vertex = hulls_[i][vertex_index];
      std::cout << "Vertex = " << vertex.transpose() << std::endl;
    }
  }
}

void SolverNlopt::setInitAndFinalPoints(Eigen::Vector3d &initial_point, Eigen::Vector3d &final_point)
{
  initial_point_ = initial_point;
  final_point_ = final_point;
}

void SolverNlopt::toEigen(const double *x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                          std::vector<double> &d)
{
  // std::cout << "Entering toEigen" << std::endl;
  q.clear();
  n.clear();
  d.clear();

  /*  std::cout << "k_min_=" << k_min_ << std::endl;

    std::cout << "j_min_=" << j_min_ << std::endl;

    std::cout << "cleared" << std::endl;
    std::cout << "i_min_=" << i_min_ << std::endl;*/

  // Control points (3x1)
  for (int i = i_min_; i <= i_max_ - 2; i = i + 3)
  {
    // std::cout << "i= " << i << std::endl;

    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }

  // Normals vectors (3x1)
  for (int j = j_min_; j <= j_max_ - 2; j = j + 3)
  {
    // std::cout << "j= " << j << std::endl;
    n.push_back(Eigen::Vector3d(x[j], x[j + 1], x[j + 2]));
  }

  // d values (1x1)
  for (int k = k_min_; k <= k_max_; k = k + 1)
  {
    // std::cout << "k= " << k << std::endl;

    d.push_back(x[k]);
  }

  // std::cout << "done with toEigen" << std::endl;
}

void SolverNlopt::toEigen(const std::vector<double> &x, std::vector<Eigen::Vector3d> &q,
                          std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  q.clear();
  n.clear();
  d.clear();
  // Control points (3x1)
  for (int i = i_min_; i <= i_max_ - 2; i = i + 3)
  {
    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }

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
  return 3 * i;
}

// global index of the first element of the norml i
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

double SolverNlopt::myObjFunc(unsigned nn, const double *x, double *grad, void *my_func_data)
{
  SolverNlopt *opt = reinterpret_cast<SolverNlopt *>(my_func_data);

  // Creation of the control points, normal vectors and d values
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->toEigen(x, q, n, d);
  /*  std::cout << "*******************" << std::endl;
    std::cout << "In the cost function" << std::endl;
    std::cout << "CPoints" << std::endl;*/
  // Cost
  double cost = 0.0;
  for (int i = 1; i <= (opt->N_ - 1); i++)
  {
    cost += (q[i + 1] - 2 * q[i] + q[i - 1]).squaredNorm();
  }

  if (grad)
  {
    // Initialize to zero all the elements, not sure if needed
    for (int i = 0; i < nn; i++)
    {
      grad[i] = 0.0;
    }

    // Gradient for the control points 0 ,..., N
    for (int i = 0; i <= opt->N_; i++)  // i = 0,...,N (q.size()==N+1)
    {
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

      if (i != 0 && i != 1)
      {
        gradient += 2 * (q[i] - 2 * q[i - 1] + q[i - 2]);
      }
      if (i != 0 && i != opt->N_)
      {
        gradient += -4 * (q[i + 1] - 2 * q[i] + q[i - 1]);
      }
      if (i != opt->N_ && i != (opt->N_ - 1))
      {
        gradient += 2 * (q[i + 2] - 2 * q[i + 1] + q[i]);
      }

      // std::cout << "gradient= " << gradient.transpose() << std::endl;

      opt->assignEigenToVector(grad, opt->gIndexQ(i), gradient);
    }

    // Gradient for the normals
    // std::cout << "Normals" << std::endl;
    for (int j = opt->j_min_; j <= opt->j_max_; j++)
    {
      // std::cout << "j= " << j << std::endl;
      grad[j] = 0;
    }

    // Gradient for d
    // std::cout << "Coeffs D" << std::endl;
    for (int k = opt->k_min_; k <= opt->k_max_; k++)
    {
      // std::cout << "k= " << k << std::endl;
      grad[k] = 0;
    }
  }

  // std::cout << "Cost=" << cost << std::endl;
  // std::cout << "*******************" << std::endl;
  // std::cout << "*******************" << std::endl;

  return cost;
}

void SolverNlopt::add_ineq_constraints(unsigned m, double *constraints, unsigned nn, double *grad,
                                       std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                       std::vector<double> &d)
{
  Eigen::Vector3d ones = Eigen::Vector3d::Ones();
  int r = 0;
  // grad is a vector with nn*m elements
  // Initialize grad to 0 all the elements, not sure if needed
  if (grad)
  {
    for (int i = 0; i < nn * m; i++)
    {
      grad[i] = 0.0;
    }
  }

  for (int i = 0; i <= N_ - 3; i++)  // i  is the interval (\equiv segment)
  {
    // impose that all the vertexes are on one side of the plane

    for (int vertex_index = 0; vertex_index < hulls_[i].size(); vertex_index++)  // opt->hulls_[i].size()
    {
      Eigen::Vector3d vertex = hulls_[i][vertex_index];

      constraints[r] = -(n[i].dot(vertex) + d[i]);
      if (grad)
      {
        toGradSameConstraintDiffVariables(gIndexN(i), -vertex, grad, r, nn);
        assignValueToGradConstraints(gIndexD(i), -1, grad, r, nn);
      }
      r++;
    }

    // and the control points on the other
    for (int u = 0; u <= 3; u++)
    {
      constraints[r] = n[i].dot(q[i + u]) + d[i];  //<= 0
      if (grad)
      {
        toGradSameConstraintDiffVariables(gIndexN(i), q[i + u], grad, r, nn);
        toGradSameConstraintDiffVariables(gIndexQ(i + u), n[i], grad, r, nn);
        assignValueToGradConstraints(gIndexD(i), 1, grad, r, nn);
      }
      r++;
    }
  }

  // std::cout << "r before the equality constraints= " << r << std::endl;

  // Now add the "equality" constraints using inequalities with an epsilon:

  double epsilon = 0.1;
  Eigen::Vector3d eps_vector(epsilon, epsilon, epsilon);

  // Initial position
  assignEigenToVector(constraints, r, q[0] - initial_point_ - eps_vector);  // f<=0
  if (grad)
  {
    toGradDiffConstraintsDiffVariables(gIndexQ(0), ones, grad, r, nn);
  }
  r = r + 3;

  assignEigenToVector(constraints, r, initial_point_ - q[0] - eps_vector);  // f<=0
  if (grad)
  {
    toGradDiffConstraintsDiffVariables(gIndexQ(0), -ones, grad, r, nn);
  }
  r = r + 3;

  // Final position
  // std::cout << "N_= " << N_ << std::endl;
  assignEigenToVector(constraints, r, q[N_] - final_point_ - eps_vector);  // f<=0
  if (grad)
  {
    toGradDiffConstraintsDiffVariables(gIndexQ(N_), ones, grad, r, nn);
  }
  r = r + 3;

  // std::cout << "N_=" << N_ << std::endl;

  assignEigenToVector(constraints, r, final_point_ - q[N_] - eps_vector);  // f<=0
  if (grad)
  {
    toGradDiffConstraintsDiffVariables(gIndexQ(N_), -ones, grad, r, nn);
  }
  r = r + 3;

  // VELOCITY CONSTRAINTS:
  for (int i = 0; i <= N_ - 1; i++)
  {
    double inv_deltaT = (1 / deltaT_);

    Eigen::Vector3d v_i = inv_deltaT * (q[i + 1] - q[i]);

    // v<=vmax
    assignEigenToVector(constraints, r, v_i - v_max_);  // f<=0
    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(i), -inv_deltaT * ones, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), inv_deltaT * ones, grad, r, nn);
    }
    r = r + 3;

    // v>=-vmax
    assignEigenToVector(constraints, r, -v_i - v_max_);  // f<=0
    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(i), -inv_deltaT * ones, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), inv_deltaT * ones, grad, r, nn);
    }
    r = r + 3;
  }

  // ACCELERATION CONSTRAINTS:
  for (int i = 0; i <= N_ - 2; i++)
  {
    double inv_deltaT2 = 1 / (deltaT_ * deltaT_);

    // a<=amax
    Eigen::Vector3d a_i = inv_deltaT2 * (q[i + 2] - 2 * q[i + 1] + q[i]);

    assignEigenToVector(constraints, r, a_i - a_max_);  // f<=0
    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(i), inv_deltaT2 * ones, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -2 * inv_deltaT2 * ones, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), inv_deltaT2 * ones, grad, r, nn);
    }
    r = r + 3;

    // a>=-amax
    assignEigenToVector(constraints, r, -a_i - a_max_);  // f<=0
    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(i), -inv_deltaT2 * ones, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), 2 * inv_deltaT2 * ones, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), -inv_deltaT2 * ones, grad, r, nn);
    }
    r = r + 3;
  }

  // Impose that the normals are not [0 0 0]
  /*  for (int i = 0; i < n.size(); i++)
    {
      double epsilon_normals = 1;

      std::cout << "n[i]= " << n[i].transpose() << std::endl;
      result[r] = epsilon_normals - n[i].dot(n[i]);  // f<=0
      if (grad)
      {
        toGradSameConstraintDiffVariables(gIndexN(i), -2 * n[i], grad, r, nn);
      }
      r++;
    }
  */
  num_of_constraints_ = r + 1;

  /*  std::cout << "There are =" << num_of_constraints_ << " constraints in total" << std::endl;
    std::cout << "There are =" << m << " constraints in total" << std::endl;*/
}

// See example https://github.com/stevengj/nlopt/issues/168
// m is the number of constraints (which is computed from the lentgh of the vector of tol_constraint)
// nn is the number of variables
void SolverNlopt::multi_ineq_constraint(unsigned m, double *constraints, unsigned nn, const double *x, double *grad,
                                        void *f_data)
{
  SolverNlopt *opt = reinterpret_cast<SolverNlopt *>(f_data);

  // std::cout << "in multi_ineq_constraint, m=" << m << ", n=" << nn << std::endl;

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->toEigen(x, q, n, d);
  opt->add_ineq_constraints(m, constraints, nn, grad, q, n, d);

  /**/
  return;
}

void SolverNlopt::printQND(std::vector<Eigen::Vector3d> q, std::vector<Eigen::Vector3d> n, std::vector<double> d)
{
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
}

void SolverNlopt::optimize()

{
  // Hack to get the number of constraints in the problem
  double constraints[10000];  // this number should be very big!! (hack)
  double xx[num_of_variables_];

  for (int i = 0; i < num_of_variables_; i++)
  {
    xx[i] = 0.0;
  }
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(xx, q, n, d);
  add_ineq_constraints(0, constraints, num_of_variables_, NULL, q, n, d);
  // end of hack

  std::cout << "num_of_constraints_=" << num_of_constraints_ << std::endl;

  // see https://github.com/stevengj/nlopt/issues/168
  double data[4] = { 2, 0, -1, 1 };
  std::vector<double> tol_constraint(num_of_constraints_);  // This number should be the num of constraints I think
  for (int i = 0; i < tol_constraint.size(); i++)
  {
    tol_constraint[i] = 1e-4;
  }

  std::cout << "The size of tol_constraint= " << tol_constraint.size() << std::endl;

  // andd lower and upper bounds
  /*  std::vector<double> lb;
    std::vector<double> ub;
    for (int i = 0; i < num_of_variables_; i++)
    {
      lb.push_back(-HUGE_VAL);
      ub.push_back(HUGE_VAL);
    }
    opt_->set_lower_bounds(lb);
    opt_->set_upper_bounds(ub);*/

  // set constraint and objective
  opt_->add_inequality_mconstraint(SolverNlopt::multi_ineq_constraint, this, tol_constraint);
  opt_->set_min_objective(SolverNlopt::myObjFunc,
                          this);  // this is passed as a parameter (the obj function has to be static)

  std::vector<double> x(num_of_variables_);  // initial guess

  // guesses for the normals
  for (int i = j_min_; i <= j_max_; i++)
  {
    x[i] = (((double)rand() / (RAND_MAX)) + 1);  // TODO Change this
  }

  x[0] = initial_point_(0);
  x[1] = initial_point_(1);
  x[2] = initial_point_(2);
  x[gIndexQ(N_)] = final_point_(0);
  x[gIndexQ(N_) + 1] = final_point_(1);
  x[gIndexQ(N_) + 2] = final_point_(2);
  double minf;

  MyTimer opt_timer(true);
  int result = opt_->optimize(x, minf);
  std::cout << "Solve time = " << opt_timer << std::endl;

  if (result < 0)
  {
    printf("nlopt failed!\n");
  }
  else
  {
    std::cout << "************SOLUTION FOUND**********" << std::endl;
    std::cout << "opt value= " << minf << std::endl;

    std::vector<Eigen::Vector3d> q;
    std::vector<Eigen::Vector3d> n;
    std::vector<double> d;
    toEigen(x, q, n, d);
    printQND(q, n, d);

    /*    std::cout << "Checking if these constraints are satisfied" << std::endl;
        for (int i = 0; i <= N_ - 3; i++)  // i  is the interval (\equiv segment)
        {
          // impose that all the vertexes are on one side of the plane
          for (int vertex_index = 0; vertex_index < hulls_[i].size(); vertex_index++)
          {
            Eigen::Vector3d vertex = hulls_[i][vertex_index];
            // std::cout << vertex.transpose() << std::endl;
            std::cout << -(n[i].dot(vertex) + d[i]) << std::endl;
            ;
          }

          // and the control points on the other
          for (int u = 0; u <= 3; u++)
          {
            std::cout << n[i].dot(q[i + u]) + d[i] << std::endl;
          }
        }*/

    // Construct now the B-Spline
    // See example at https://github.com/libigl/eigen/blob/master/unsupported/test/splines.cpp#L37
    Eigen::RowVectorXd knots(M_ + 1);
    for (int i = 0; i <= p_; i++)
    {
      knots[i] = t_min_;
    }

    for (int i = (p_ + 1); i <= M_ - p_ - 1; i++)
    {
      knots[i] = knots[i - 1] + deltaT_;  // Assumming a uniform b-spline (internal knots are equally spaced)
    }

    for (int i = (M_ - p_); i <= M_; i++)
    {
      knots[i] = t_max_;
    }

    Eigen::MatrixXd control_points(q.size(), 3);

    for (int i = 0; i < q.size(); i++)
    {
      control_points.row(i) = q[i].transpose();
    }
    control_points.transposeInPlace();

    Eigen::Spline<double, 3, Eigen::Dynamic> spline(knots, control_points);

    std::cout << "Knots= " << knots << std::endl;
    std::cout << "dc_= " << dc_ << std::endl;

    X_temp_.clear();

    for (double t = t_min_; t <= t_max_; t = t + dc_)
    {
      // std::cout << "t= " << t << std::endl;
      Eigen::MatrixXd derivatives = spline.derivatives(t, 4);  // Compute all the derivatives up to order 4

      state state_i;

      state_i.setPos(derivatives.col(0));  // First column
      state_i.setVel(derivatives.col(1));
      state_i.setAccel(derivatives.col(2));
      state_i.setJerk(derivatives.col(3));
      X_temp_.push_back(state_i);
    }
  }

  // AVAILABLE ALGORITHMS:
  //   G/L denotes global/local optimization and N/D denotes derivative-free/gradient-based
  /*%  GD_MLSL_LDS,  GD_MLSL,  GD_STOGO,  GD_STOGO_RAND,
  %  GN_CRS2_LM,  GN_DIRECT_L,  GN_DIRECT_L_NOSCAL,
  %  GN_DIRECT_L_RAND,  GN_DIRECT_L_RAND_NOSCAL,  GN_DIRECT,
  %  GN_DIRECT_NOSCAL,  GN_ISRES,  GN_MLSL_LDS,  GN_MLSL,
  %  GN_ORIG_DIRECT_L,  GN_ORIG_DIRECT,  LD_AUGLAG_EQ,
  %  LD_AUGLAG,   LD_LBFGS_NOCEDAL,  LD_MMA,
  %   LD_TNEWTON_PRECOND,
  %  LD_TNEWTON_PRECOND_RESTART,  LD_TNEWTON_RESTART,
  %  LD_VAR1,  LD_VAR2,  LN_AUGLAG_EQ,  LN_AUGLAG,
  %  LN_BOBYQA,  LN_COBYLA,  ,
  %  LN_NEWUOA_BOUND,  LN_NEWUOA,  LN_PRAXIS,  LN_SBPLX LD_SLSQP*/
  // LD_MMA goes really fast
  // LD_AUGLAG accepts equality constraints, but does not converge
  // LD_LBFGS does NOT accept equality constraints
  // LD_SLSQP supports equality constraints, see
  // http://ab-initio.mit.edu/wiki/index.php?title=NLopt_Algorithms&printable=yes
  // LD_LBFGS, LD_TNEWTON, LN_NELDERMEAD, LN_BOBYQA No ineq const
  // Only some of the NLopt algorithms (AUGLAG, COBYLA, and ISRES) currently support nonlinear equality constraints
  // LD_MMA doesn't work with equality constraints
  // LN_BOBYQA

  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  // see https://github.com/stevengj/nlopt/issues/168
  // Set constraints and objective

  // AVAILABLE ALGORITHMS:
  //   G/L denotes global/local optimization and N/D denotes derivative-free/gradient-based
  /*%  GD_MLSL_LDS,  GD_MLSL,  GD_STOGO,  GD_STOGO_RAND,
  %  GN_CRS2_LM,  GN_DIRECT_L,  GN_DIRECT_L_NOSCAL,
  %  GN_DIRECT_L_RAND,  GN_DIRECT_L_RAND_NOSCAL,  GN_DIRECT,
  %  GN_DIRECT_NOSCAL,  GN_ISRES,  GN_MLSL_LDS,  GN_MLSL,
  %  GN_ORIG_DIRECT_L,  GN_ORIG_DIRECT,  LD_AUGLAG_EQ,
  %  LD_AUGLAG,   LD_LBFGS_NOCEDAL,  LD_MMA,
  %   LD_TNEWTON_PRECOND,
  %  LD_TNEWTON_PRECOND_RESTART,  LD_TNEWTON_RESTART,
  %  LD_VAR1,  LD_VAR2,  LN_AUGLAG_EQ,  LN_AUGLAG,
  %  LN_BOBYQA,  LN_COBYLA,  ,
  %  LN_NEWUOA_BOUND,  LN_NEWUOA,  LN_PRAXIS,  LN_SBPLX LD_SLSQP*/

  // LD_MMA goes really fast
  // LD_AUGLAG accepts equality constraints, but does not converge
  // LD_LBFGS does NOT accept equality constraints
  // LD_SLSQP supports equality constraints, see
  // http://ab-initio.mit.edu/wiki/index.php?title=NLopt_Algorithms&printable=yes

  // LD_LBFGS, LD_TNEWTON, LN_NELDERMEAD, LN_BOBYQA No ineq const

  // Only some of the NLopt algorithms (AUGLAG, COBYLA, and ISRES) currently support nonlinear equality constraints

  /*  opt_ = new nlopt::opt(nlopt::AUGLAG, num_of_variables_);  // LD_MMA doesn't work with equality constraints
                                                              // LN_BOBYQA

    std::vector<double> lb;
    std::vector<double> ub;
    for (int i = 0; i < num_of_variables_; i++)
    {
      lb.push_back(-HUGE_VAL);
      ub.push_back(HUGE_VAL);
    }
    opt_->set_lower_bounds(lb);
    opt_->set_upper_bounds(ub);

    opt_->set_xtol_rel(1e-4);  // Stopping criteria

    nlopt::opt local_opt(nlopt::LD_MMA, num_of_variables_);
    local_opt.set_xtol_rel(1e-4);
    opt_->set_local_optimizer(local_opt);

    std::vector<double> tol_constraint(8 * (N_ - 2) + 12);  // This number should be the num of constraints I think
    for (int i = 0; i < tol_constraint.size(); i++)
    {
      tol_constraint[i] = 1e-4;
    }

    opt_->add_inequality_mconstraint(SolverNlopt::multi_ineq_constraint, this, tol_constraint);
    opt_->set_min_objective(SolverNlopt::myObjFunc,
                            this);  // this is passed as a parameter (the obj function has to be static)

    std::cout << "here" << std::endl;

    opt_->set_xtol_rel(1e-4);                  // Stopping criteria
    std::vector<double> x(num_of_variables_);  // initial guess
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
    x[gIndexQ(N_)] = 10;
    x[gIndexQ(N_) + 1] = 10;
    x[gIndexQ(N_) + 2] = 10;
    double minf;

    if (opt_->optimize(x, minf) < 0)
    {
      printf("nlopt failed!\n");
    }
    else
    {
      std::cout << "SOLVED!" << std::endl;
      std::cout << "gIndexQ(N)=" << gIndexQ(N_) << std::endl;
      printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);

      std::cout << "Solution found, opt value= " << minf << std::endl;
      std::cout << "   control points:" << std::endl;
      for (int i = 0; i <= i_max_; i = i + 3)
      {
        std::cout << x[i] << ", " << x[i + 1] << ", " << x[i + 2] << std::endl;
      }
      std::cout << "   normals:" << std::endl;
      for (int j = j_min_; j < j_max_; j = j + 3)
      {
        std::cout << x[j] << ", " << x[j + 1] << ", " << x[j + 2] << std::endl;
      }
      std::cout << "   d coeffs:" << std::endl;
      for (int k = k_min_; k < k_max_; k = k + 1)
      {
        std::cout << x[k] << std::endl;
      }
    }*/

  /*
    local_opt_ = new nlopt::opt(nlopt::LD_MMA, num_of_variables_);
    local_opt_->set_xtol_rel(1e-4);
    opt_->set_local_optimizer(*local_opt_);

    // Set initial guess
    std::vector<double> x(num_of_variables_);
    x[0] = 0.2;
    x[1] = 0.2;
    x[2] = 0.2;
    x[gIndexQ(N_)] = 9.8;
    x[gIndexQ(N_) + 1] = 9.8;
    x[gIndexQ(N_) + 2] = 9.8;

    // run the optimization
    double minf;
    MyTimer opt_timer(true);
    std::cout << "Calling optimize!!\n";
    int result = opt_->optimize(x, minf);
    std::cout << "Done with optimize!!\n";
    std::cout << "Solve time = " << opt_timer << std::endl;

    // print results
    if (result < 0)
    {
      printf("nlopt failed!\n");
    }
    else
    {
      std::cout << "Solution found, opt value= " << minf << std::endl;
      std::cout << "   control points:" << std::endl;
      for (int i = 0; i <= i_max_; i = i + 3)
      {
        std::cout << x[i] << ", " << x[i + 1] << ", " << x[i + 2] << std::endl;
      }
      std::cout << "   normals:" << std::endl;
      for (int j = j_min_; j < j_max_; j = j + 3)
      {
        std::cout << x[j] << ", " << x[j + 1] << ", " << x[j + 2] << std::endl;
      }
      std::cout << "   d coeffs:" << std::endl;
      for (int k = k_min_; k < k_max_; k = k + 1)
      {
        std::cout << x[k] << std::endl;
      }
    }*/
}

/*void SolverNlopt::multi_eq_constraint(unsigned m, double *result, unsigned nn, const double *x, double *grad, void
*f_data)
{
  std::cout << "in multi_eq_constraint" << std::endl;

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);

  int r = 0;
  assignEigenToVector(result, r, q[0] - initial_point);
  // std::cout << "going to assign the gradient" << std::endl;
  if (grad)
  {
    assignEigenToVector(grad, r + gIndexQ(0, 0), Eigen::Vector3d::Ones());
  }
  r = r + 3;  // Note that the previous assignment to result is 3x1

  // std::cout << "in the middle of multi_eq_constraint" << std::endl;

  assignEigenToVector(result, r, q[N] - final_point);  // f1==0
  if (grad)
  {
    assignEigenToVector(grad, r + gIndexQ(q.size() - 1, 0), Eigen::Vector3d::Ones());
  }

  return;
}*/