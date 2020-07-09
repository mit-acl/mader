// Jesus Tordesillas Torres, jtorde@mit.edu, January 2020

// NOTE THAT THIS EXAMPLE RUNS, BUT IT HAS SOME BUGS IN THE GRADIENTS. SEE THE CLASS solverNlopt.cpp for THE GOOD
// VERSION

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>

#define DEG_POL 3
#define NUM_POL 5

int i_min;
int i_max;
int j_min;
int j_max;
int k_min;
int k_max;

int p;
int M;
int N;
int num_of_variables;

Eigen::MatrixXd R(3, 20);  // This matrix is [r0, r1, r2, r3, r0, r1, r2, r3] (for two segments)

Eigen::Vector3d initial_point;
Eigen::Vector3d final_point;

void assignEigenToVector(double *grad, int index, const Eigen::Vector3d &tmp)

{
  /*  std::cout << "Going to assign, index=" << index << "and vector=" << tmp(0) << ", " << tmp(1) << ", " << tmp(2)
              << std::endl;*/

  // std::cout << "Result: " << index << ", " << index + 1 << ", " << index + 2 << std::endl;
  // std::cout << "index= " << index << std::endl;

  grad[index] = tmp(0);
  grad[index + 1] = tmp(1);
  grad[index + 2] = tmp(2);
}

void toEigen(const double *x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  q.clear();
  n.clear();
  d.clear();
  // Control points (3x1)
  for (int i = i_min; i <= i_max; i = i + 3)
  {
    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }

  // Normals vectors (3x1)
  for (int j = j_min; j <= j_max; j = j + 3)
  {
    n.push_back(Eigen::Vector3d(x[j], x[j + 1], x[j + 2]));
  }

  // d values (1x1)
  for (int k = k_min; k <= k_max; k = k + 1)
  {
    d.push_back(x[k]);
  }
}

int gIndexQ(int i)  // Element jth of control point ith
{
  return 3 * i;
}

int gIndexN(int i)  // Element jth of normal ith
{
  return 3 * i + j_min;
}

int gIndexD(int i)
{
  return i + k_min;
}

double myvfunc(unsigned nn, const double *x, double *grad, void *my_func_data)
{
  // Creation of the control points, normal vectors and d values

  // Control points (3x1)

  // std::cout << "in myvfunc" << std::endl;

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);

  // Cost

  double cost = 0.0;
  for (int i = 1; i <= (N - 1); i++)
  {
    cost += (q[i + 1] - 2 * q[i] + q[i - 1]).squaredNorm();
  }

  if (grad)
  {
    // Initialize to zero all the elements
    for (int i = 0; i < nn; i++)
    {
      grad[i] = 0.0;
    }

    // Gradient for the control points 0 ,..., N
    for (int i = 0; i <= N; i++)  // i = 0,...,N (q.size()==N+1)
    {
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

      if (i != 0 && i != 1)
      {
        gradient += 2 * (q[i] - 2 * q[i - 1] + q[i - 2]);
      }
      if (i != 0 && i != N)
      {
        gradient += -4 * (q[i + 1] - 2 * q[i] + q[i - 1]);
      }
      if (i != N && i != (N - 1))
      {
        gradient += +2 * (q[i + 2] - 2 * q[i + 1] + q[i]);
      }

      // std::cout << "gradient= " << gradient.transpose() << std::endl;

      grad[gIndexQ(i)] = gradient(0);
      grad[gIndexQ(i) + 1] = gradient(1);
      grad[gIndexQ(i) + 2] = gradient(2);
    }

    // Gradient for the normals
    for (int j = j_min; j <= j_max; j++)
    {
      grad[j] = 0;
    }

    // Gradient for d
    for (int k = k_min; k <= k_max; k++)
    {
      grad[k] = 0;
    }
  }

  // std::cout << "end of myvfunc" << std::endl;

  // Delete this afterwards
  /*  cost = 0.0;
    if (grad)
    {
      // Initialize to zero all the elements
      for (int i = 0; i < nn; i++)
      {
        grad[i] = 0.0;
      }
    }*/

  return cost;
}

typedef struct
{
  double a, b;
} my_constraint_data;

void printElementsStdEigen(std::vector<Eigen::Vector3d> tmp)
{
  for (int i = 0; i < tmp.size(); i++)
  {
    std::cout << tmp[i].transpose() << std::endl;
  }
}

// r is the constraint index
// nn is the number of variables
// var_gindex is the index of the variable of the first element of the vector
void assignEigenToGradConstraints(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r, int nn)

{
  grad[r * nn + var_gindex] = tmp(0);
  grad[(r + 1) * nn + var_gindex + 1] = tmp(1);
  grad[(r + 2) * nn + var_gindex + 2] = tmp(2);
}

void assignEigenToGradUnitConstraint(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r, int nn)

{
  grad[r * nn + var_gindex] = tmp(0);
  grad[r * nn + var_gindex + 1] = tmp(1);
  grad[r * nn + var_gindex + 2] = tmp(2);
}

void assignValueToGradConstraints(int var_gindex, const double &tmp, double *grad, int r, int nn)

{
  grad[r * nn + var_gindex] = tmp;
}

// See example https://github.com/stevengj/nlopt/issues/168
void multi_ineq_constraint(unsigned m, double *result, unsigned nn, const double *x, double *grad, void *f_data)
{
  // std::cout << "in multi_ineq_constraint, m=" << m << ", n=" << nn << std::endl;

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);

  // std::cout << "n is " << std::endl;
  // printElementsStdEigen(n);

  // std::cout << "in multi_ineq_constraint, after eigen" << std::endl;
  // n is the length of x, m is the length of result
  /*  double *df_data = static_cast<double *>(f_data);
    double a1 = df_data[0];
    double b1 = df_data[1];
    double a2 = df_data[2];
    double b2 = df_data[3];*/

  // std::cout << "gonna start the loop" << std::endl;

  // Initialize grad to 0 all the elements
  if (grad)
  {
    for (int i = 0; i < nn * m; i++)
    {
      grad[i] = 0.0;
    }
  }

  int r = 0;
  for (int i = 0; i <= N - 3; i++)  // i here is the interval
  {
    Eigen::Vector3d r0 = R.col(3 * i + 0);
    Eigen::Vector3d r1 = R.col(3 * i + 1);
    Eigen::Vector3d r2 = R.col(3 * i + 2);
    Eigen::Vector3d r3 = R.col(3 * i + 3);

    result[r] = -(n[i].dot(r0) + d[i]);
    if (grad)
    {
      assignEigenToGradUnitConstraint(gIndexN(i), -r0, grad, r, nn);
      assignValueToGradConstraints(gIndexD(i), -1, grad, r, nn);
    }
    r++;

    result[r] = -(n[i].dot(r1) + d[i]);
    if (grad)
    {
      assignEigenToGradUnitConstraint(gIndexN(i), -r1, grad, r, nn);
      assignValueToGradConstraints(gIndexD(i), -1, grad, r, nn);
    }
    r++;

    result[r] = -(n[i].dot(r2) + d[i]);
    if (grad)
    {
      assignEigenToGradUnitConstraint(gIndexN(i), -r2, grad, r, nn);
      assignValueToGradConstraints(gIndexD(i), -1, grad, r, nn);
    }
    r++;

    result[r] = -(n[i].dot(r3) + d[i]);
    if (grad)
    {
      assignEigenToGradUnitConstraint(gIndexN(i), -r3, grad, r, nn);
      assignValueToGradConstraints(gIndexD(i), -1, grad, r, nn);
    }
    r++;

    for (int u = 0; u <= 3; u++)
    {
      result[r] = n[i].dot(q[i + u]) + d[i];  //<= 0
      if (grad)
      {
        assignEigenToGradUnitConstraint(gIndexN(i), q[i + u], grad, r, nn);
        assignEigenToGradUnitConstraint(gIndexD(i + u), n[i], grad, r, nn);
        assignValueToGradConstraints(gIndexD(i), 1, grad, r, nn);
      }
      r++;
    }
  }

  // Now add the "equality" constraints using inequalities with an epsilon:

  double epsilon = 0.005;
  Eigen::Vector3d eps_vector(epsilon, epsilon, epsilon);

  // Initial position
  assignEigenToVector(result, r, q[0] - initial_point - eps_vector);  // f<=0
  if (grad)
  {
    assignEigenToGradConstraints(gIndexQ(0), Eigen::Vector3d::Ones(), grad, r, nn);
  }
  r = r + 3;
  assignEigenToVector(result, r, initial_point - q[0] - eps_vector);  // f<=0
  if (grad)
  {
    assignEigenToGradConstraints(gIndexQ(0), -Eigen::Vector3d::Ones(), grad, r, nn);
  }
  r = r + 3;

  // Final position
  assignEigenToVector(result, r, q[N] - final_point - eps_vector);  // f<=0
  if (grad)
  {
    assignEigenToGradConstraints(gIndexQ(N), Eigen::Vector3d::Ones(), grad, r, nn);
  }
  r = r + 3;
  assignEigenToVector(result, r, final_point - q[N] - eps_vector);  // f<=0
  if (grad)
  {
    assignEigenToGradConstraints(gIndexQ(N), -Eigen::Vector3d::Ones(), grad, r, nn);
  }
  r = r + 3;

  // std::cout << "There are =" << r + 3 << " constraints in total" << std::endl;

  return;
}

/*void multi_eq_constraint(unsigned m, double *result, unsigned nn, const double *x, double *grad, void *f_data)
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

int main()
{
  p = DEG_POL;
  M = NUM_POL + 2 * p;
  N = M - p - 1;
  num_of_variables = (3 * N + 1) + 3 * (M - 2 * p) + (M - 2 * p);  // total number of variables

  i_min = 0;
  i_max = 3 * N;
  j_min = i_max + 1;
  j_max = j_min + 3 * (M - 2 * p) - 1;
  k_min = j_max + 1;
  k_max = k_min + (M - 2 * p) - 1;

  initial_point << 0, 0, 0;
  final_point << 10, 10, 10;

  // Matrix containing the control points (see script test_to_create_splines.m)
  R << 1.3784, 1.9082, 2.2617, 2.4963, 2.4963, 2.7309, 2.8467, 2.9011, 2.9011, 2.9554, 2.9484, 3.1676, 3.1676, 3.3869,
      3.8326, 4.4580, 4.4580, 5.0833, 5.8885, 6.8267,  /////////////////////////
      0.3048, 2.0841, 2.5280, 2.6516, 2.6516, 2.7752, 2.5786, 3.0767, 3.0767, 3.5748, 4.7678, 4.9923, 4.9923, 5.2168,
      4.4728, 4.1564, 4.1564, 3.8399, 3.9509, 5.8854,  /////////////////////////
      0.2133, 2.9102, 3.2191, 2.9238, 2.9238, 2.6285, 1.7290, 2.0093, 2.0093, 2.2895, 3.7495, 4.5498, 4.5498, 5.3502,
      5.4909, 5.6346, 5.6346, 5.7783, 5.9250, 6.7374;  /////////////////////////

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

  nlopt::opt opt(nlopt::AUGLAG, num_of_variables);  // LD_MMA doesn't work with equality constraints
                                                    // LN_BOBYQA

  nlopt::opt local_opt(nlopt::LD_MMA, num_of_variables);
  local_opt.set_xtol_rel(1e-4);
  opt.set_local_optimizer(local_opt);
  // LD_MMA goes really fast
  // LD_AUGLAG accepts equality constraints, but does not converge
  // LD_LBFGS does NOT accept equality constraints
  // LD_SLSQP supports equality constraints, see
  // http://ab-initio.mit.edu/wiki/index.php?title=NLopt_Algorithms&printable=yes

  // LD_LBFGS, LD_TNEWTON, LN_NELDERMEAD, LN_BOBYQA No ineq const

  // Only some of the NLopt algorithms (AUGLAG, COBYLA, and ISRES) currently support nonlinear equality constraints

  // see https://github.com/stevengj/nlopt/issues/168
  double data[4] = { 2, 0, -1, 1 };
  std::vector<double> tol_constraint(8 * (N - 2) + 12);  // This number should be the num of constraints I think
  for (int i = 0; i < tol_constraint.size(); i++)
  {
    tol_constraint[i] = 1e-4;
  }

  std::cout << "here" << std::endl;

  opt.add_inequality_mconstraint(multi_ineq_constraint, data, tol_constraint);
  // opt.add_equality_mconstraint(multi_eq_constraint, data, tol_constraint);  // This is a linear constraint!!

  /*  std::vector<double> lb(num_of_variables);
    lb[0] = -HUGE_VAL;
    lb[1] = 0;*/

  std::vector<double> lb;
  std::vector<double> ub;
  for (int i = 0; i < num_of_variables; i++)
  {
    lb.push_back(-HUGE_VAL);
    ub.push_back(HUGE_VAL);
  }
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(myvfunc, NULL);
  // my_constraint_data data[2] = { { 2, 0 }, { -1, 1 } };

  /*  opt.add_inequality_constraint(myvconstraint, &data[0], 1e-8);
    opt.add_inequality_constraint(myvconstraint, &data[1], 1e-8);*/

  opt.set_xtol_rel(1e-4);                   // Stopping criteria
  std::vector<double> x(num_of_variables);  // initial guess
  x[0] = 0.2;
  x[1] = 0.2;
  x[2] = 0.2;
  x[gIndexQ(N)] = 9.8;
  x[gIndexQ(N) + 1] = 9.8;
  x[gIndexQ(N) + 2] = 9.8;
  double minf;

  if (opt.optimize(x, minf) < 0)
  {
    printf("nlopt failed!\n");
  }
  else
  {
    std::cout << "gIndexQ(N)=" << gIndexQ(N) << std::endl;
    printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);

    std::cout << "Solution found, opt value= " << minf << std::endl;
    std::cout << "   control points:" << std::endl;
    for (int i = 0; i <= i_max; i = i + 3)
    {
      std::cout << x[i] << ", " << x[i + 1] << ", " << x[i + 2] << std::endl;
    }
    std::cout << "   normals:" << std::endl;
    for (int j = j_min; j < j_max; j = j + 3)
    {
      std::cout << x[j] << ", " << x[j + 1] << ", " << x[j + 2] << std::endl;
    }
    std::cout << "   d coeffs:" << std::endl;
    for (int k = k_min; k < k_max; k = k + 1)
    {
      std::cout << x[k] << std::endl;
    }
  }
  /*
    try
    {
      nlopt::result result = opt.optimize(x, minf);
      std::cout << "Solution found:" << std::endl;
      std::cout << "   control points:" << std::endl;
      for (int i = 0; i <= i_max; i = i + 3)
      {
        std::cout << x[i] << ", " << x[i + 1] << ", " << x[i + 2] << std::endl;
      }
      std::cout << "   normals:" << std::endl;
      for (int j = j_min; j < j_max; j = j + 3)
      {
        std::cout << x[j] << ", " << x[j + 1] << ", " << x[j + 2] << std::endl;
      }
      std::cout << "   d coeffs:" << std::endl;
      for (int k = k_min; k < k_max; k = k + 1)
      {
        std::cout << x[k] << std::endl;
      }
      // std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = " << std::setprecision(10) << minf <<
      // std::endl;
    }
    catch (std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }*/
}

/*double myvconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
  // Creation of the control points, normal vectors and d values
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);
  my_constraint_data *d = reinterpret_cast<my_constraint_data *>(data);
  double a = d->a, b = d->b;
  for (int m = 0; m <= N - 3; m++)
  {
    constraints.push_back(-(gn(i) * r0 + d(i)));
    constraints.push_back(-(gn(i) * r1 + d(i)));
    constraints.push_back(-(gn(i) * r2 + d(i)));
    constraints.push_back(-(gn(i) * r3 + d(i)));
    for (int u = 0; u <= 3; u++)
    {
      constraints.push_back(gn(m) * q(m + u) + d(m));  //<= 0
    }
  }
  constraints.push_back();
  for (int j = j_min; j <= j_max; j = j + 3)
  {
    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }
  if (!grad.empty())
  {
    grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
    grad[1] = -1.0;
  }
  return ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
}*/

// The n dimension of grad is stored contiguously, so that \partci/\partxj is stored in grad[i*n + j]
// Here you see take dCi/dx0...dxn and store it one by one, then repeat. grad is just an one dimensional array

/*  if (!grad.empty())
  {
    grad[0] = 3 * a1 * (a1 * x[0] + b1) * (a1 * x[0] + b1);  // partial f1/partial x1
    grad[1] = -1.0;                                          // partial f1/partial x2
    grad[2] = 3 * a2 * (a2 * x[0] + b2) * (a2 * x[0] + b2);  // partial f2/partial x1
    grad[3] = -1.0;                                          // partial f2/partial x2
  }*/

/*
result[0] = -(gn(i) * r0 + d(i));  // f1<=0
result[1] = -(gn(i) * r1 + d(i));  // f2<=0*/

/*  std::cout << "FIRST WAY" << std::endl;
  // Quick test
  result[0] = q[0](0) - epsilon;
  result[1] = q[0](1) - epsilon;
  result[2] = q[0](2) - epsilon;
  std::cout << "Result="
            << "0, 1, 2" << std::endl;
  if (grad)
  {
    std::cout << "First element normal is" << 0 * nn + 0 << std::endl;
    grad[0 * nn + 0] = 1;
    grad[1 * nn + 1] = 1;
    grad[2 * nn + 2] = 1;
    std::cout << "Grad=" << 0 * nn + 0 << ", " << 1 * nn + 1 << ", " << 2 * nn + 2 << std::endl;
  }
  // Quick test
  result[3] = epsilon - q[0](0);
  result[4] = epsilon - q[0](1);
  result[5] = epsilon - q[0](2);
  std::cout << "Result="
            << "3, 4, 5" << std::endl;
  if (grad)
  {
    grad[3 * nn + 0] = -1;
    grad[4 * nn + 1] = -1;
    grad[5 * nn + 2] = -1;
    std::cout << "Grad=" << 3 * nn + 0 << ", " << 4 * nn + 1 << ", " << 5 * nn + 2 << std::endl;
  }
*/
