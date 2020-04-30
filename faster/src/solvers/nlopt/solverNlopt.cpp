// Jesus Tordesillas Torres, jtorde@mit.edu, January 2020
#include "solverNlopt.hpp"

#include <iostream>
#include <vector>
#include "./../../termcolor.hpp"
#include "./../../spline_AStar.hpp"

#include <random>

#include <decomp_util/ellipsoid_decomp.h>  //For Polyhedron definition
#include <unsupported/Eigen/Splines>

#include "./../../bspline_utils.hpp"

// CGAL
#include <iostream>
#include <list>
//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/AABB_tree.h>
//#include <CGAL/AABB_traits.h>
//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/AABB_face_graph_triangle_primitive.h>

//#define DEBUG_MODE_NLOPT 1  // any value will make the debug output appear (comment line if you don't want debug)
using namespace termcolor;

SolverNlopt::SolverNlopt(int num_pol, int deg_pol, int num_obst, double weight, double epsilon_tol_constraints,
                         double xtol_rel, double ftol_rel, bool force_final_state, std::string &solver)
{
  // std::cout << "In the SolverNlopt Constructor\n";

  solver_ = getSolver(solver);

  force_final_state_ = force_final_state;

  epsilon_tol_constraints_ = epsilon_tol_constraints;  // 1e-1;
  xtol_rel_ = xtol_rel;                                // 1e-1;
  ftol_rel_ = ftol_rel;                                // 1e-1;

  num_of_obst_ = num_obst;
  weight_ = weight;
  deg_pol_ = deg_pol;
  num_pol_ = num_pol;

  p_ = deg_pol_;
  M_ = num_pol_ + 2 * p_;
  N_ = M_ - p_ - 1;
  // num_of_variables_ = (3 * (N_ + 1) - 18) + (3 * (M_ - 2 * p_)) + (M_ - 2 * p_);  // total number of variables

  i_min_ = 0;
  i_max_ =
      3 * (N_ + 1) - 1 - 9 - 6 * (!force_final_state_) -
      9 * force_final_state_;  //(9 * (force_final_state_));  // 18 is because pos, vel and accel at t_min_ and t_max_
                               // are fixed (not dec variables)
  j_min_ = i_max_ + 1;
  j_max_ = j_min_ + 3 * (M_ - 2 * p_) * num_of_obst_ - 1;
  k_min_ = j_max_ + 1;
  k_max_ = k_min_ + (M_ - 2 * p_) * num_of_obst_ - 1;

  num_of_variables_ = k_max_ + 1;  // k_max_ + 1;

  num_of_segments_ = (M_ - 2 * p_);  // this is the same as num_pol_
  int num_of_cpoints = N_ + 1;

  num_of_normals_ = num_of_segments_ * num_of_obst_;

  q0_ << 0, 0, 0;
  q1_ << 0, 0, 0;
  q2_ << 0, 0, 0;
  /*  qNm2_ << 0, 0, 0;
    qNm1_ << 0, 0, 0;
    qN_ << 0, 0, 0;*/
  std::cout << bold << "N_= " << N_ << reset << std::endl;
  /*  opt_ = new nlopt::opt(nlopt::AUGLAG, num_of_variables_);
    local_opt_ = new nlopt::opt(nlopt::LD_MMA, num_of_variables_);*/
  //#ifdef DEBUG_MODE_NLOPT
  // Debugging stuff
  /*  std::cout << "deg_pol_= " << deg_pol_ << std::endl;
    std::cout << "num_pol= " << num_pol << std::endl;
    std::cout << "num_of_obst_= " << num_of_obst_ << std::endl;
    std::cout << "p_= " << p_ << std::endl;
    std::cout << "M_= " << M_ << std::endl;
    std::cout << "N_= " << N_ << std::endl;
    std::cout << "num_of_cpoints= " << num_of_cpoints << std::endl;
    std::cout << "num_of_segments_= " << num_of_segments_ << std::endl;
    std::cout << "i_min_= " << i_min_ << std::endl;
    std::cout << "i_max_= " << i_max_ << std::endl;
    std::cout << "j_min_= " << j_min_ << std::endl;
    std::cout << "j_max_= " << j_max_ << std::endl;

    std::cout << "num_of_variables_= " << num_of_variables_ << std::endl;
    std::cout << "gIndexQ(3)=" << gIndexQ(3) << std::endl;
    std::cout << "gIndexQ(num_of_cpoints-3)=" << gIndexQ(num_of_cpoints - 3 - 1) << std::endl;
    std::cout << "gIndexN(0)=" << gIndexN(0) << std::endl;
    std::cout << "gIndexN(num_of_segments-1)=" << gIndexN(num_of_segments_ - 1) << std::endl;
  */
  // separator_solver = new separator::Separator(1.0, 1.0, 1.0);

  /*  x_.clear();
    for (int i = 0; i < num_of_variables_; i++)
    {
      x_.push_back(i);
    }*/

  /*  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);
  printQND(q, n, d);*/
  //#endif

  Mbs2mv_ << 182, 685, 100, -7,  //////////////////
      56, 640, 280, -16,         //////////////////
      -16, 280, 640, 56,         //////////////////
      -7, 100, 685, 182;
  Mbs2mv_ = (1.0 / 960.0) * Mbs2mv_;
  // Mbs2ov_ = Eigen::Matrix<double, 4, 4>::Identity();
  Mbs2mv_inverse_ = Mbs2mv_.inverse();

  separator_solver_ = new separator::Separator();
}

SolverNlopt::~SolverNlopt()
{
  delete opt_;
  delete local_opt_;
}

void SolverNlopt::getGuessForPlanes(std::vector<Hyperplane3D> &planes)
{
  planes = planes_;
  /*  planes.clear();
    std::cout << "GettingGuessesForPlanes= " << n_guess_.size() << std::endl;
    for (auto n_i : n_guess_)
    {
      Eigen::Vector3d p_i;
      p_i << 0.0, 0.0, -1.0 / n_i.z();  // TODO deal with n_i.z()=0
      Hyperplane3D plane(p_i, n_i);
      planes.push_back(plane);
    }*/
}

int SolverNlopt::getNumOfLPsRun()
{
  return num_of_LPs_run_;
}

int SolverNlopt::getNumOfQCQPsRun()
{
  return num_of_QCQPs_run_;
}

/*bool SolverNlopt::intersects()
{
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point;
  typedef K::Plane_3 Plane;
  typedef K::Vector_3 Vector;
  typedef K::Segment_3 Segment;
  typedef K::Ray_3 Ray;
  typedef CGAL::Polyhedron_3<K> Polyhedron;
  typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type> Segment_intersection;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Plane>::Type> Plane_intersection;
  typedef Tree::Primitive_id Primitive_id;

  Point p(1.0, 0.0, 0.0);
  Point q(0.0, 1.0, 0.0);
  Point r(0.0, 0.0, 1.0);
  Point s(0.0, 0.0, 0.0);
  Polyhedron polyhedron;
  polyhedron.make_tetrahedron(p, q, r, s);
  // constructs AABB tree
  Tree tree(faces(polyhedron).first, faces(polyhedron).second, polyhedron);
  // constructs segment query
  Point a(-0.2, 0.2, -0.2);
  Point b(1.3, 0.2, 1.3);
  Segment segment_query(a, b);
  // tests intersections with segment query
  if (tree.do_intersect(segment_query))
    std::cout << "intersection(s)" << std::endl;
  else
    std::cout << "no intersection" << std::endl;
}*/

/*
Correlated 3D Gaussian distribution
#include "./../../eigenmvn.hpp"
Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d covar = rot * Eigen::DiagonalMatrix<double, 3, 3>(0.005, 0.005, 0.001) * rot.transpose();
    Eigen::EigenMultivariateNormal<double> normX_solver(mean, covar);  // or normX_cholesk(mean, covar, true);
    tmp = normX_solver.samples(1).transpose();*/

// std::cout << "obst index=" << obst_index << "sample= " << tmp.transpose() << std::endl;

/*        for (int j = i; j >= (i - 3); j--)  // Q3 needs to check against polyh0, polyh1 and polyh2 of all the
   obstacles
        {
          // std::cout << "j=" << j << std::endl;

          int ip = obst_index * num_of_segments_ + j;  // index poly

          if (polyhedra[ip].inside(tmp) == true)
          {
            goto initloop;
          }
        }*/

void SolverNlopt::setKappaAndMu(double kappa, double mu)
{
  kappa_ = kappa;
  mu_ = mu;
}

void SolverNlopt::setZminZmax(double z_min, double z_max)
{
  z_ground_ = z_min;
  z_max_ = z_max;
}

void SolverNlopt::fillPlanesFromNDQ(std::vector<Hyperplane3D> &planes_, const std::vector<Eigen::Vector3d> &n,
                                    const std::vector<double> &d, const std::vector<Eigen::Vector3d> &q)
{
  planes_.clear();

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      Eigen::Vector3d centroid_hull;
      // findCentroidHull(hulls_[obst_index][i], centroid_hull);
      // Eigen::Vector3d tmp = (centroid_hull + q[i]) / 2.0;

      Eigen::Vector3d tmp = q[i];  //(centroid_hull + q[i]) / 2.0;

      Eigen::Vector3d point_in_plane = Eigen::Vector3d::Zero();

      // std::cout << "tmp= " << tmp.transpose() << std::endl;
      // std::cout << "centroid_hull= " << centroid_hull.transpose() << std::endl;

      if (fabs(n[i].x()) != 0)
      {
        point_in_plane << -(n[i].y() * tmp.y() + n[i].z() * tmp.z() + d[i]) / (n[i].x()), tmp.y(), tmp.z();
      }
      else if (fabs(n[i].y()) != 0)
      {
        point_in_plane << tmp.x(), -(n[i].x() * tmp.x() + n[i].z() * tmp.z() + d[i]) / (n[i].y()), tmp.z();
      }
      else
      {
        point_in_plane << tmp.x(), tmp.y(), -(n[i].x() * tmp.x() + n[i].y() * tmp.y() + d[i]) / (n[i].z());
      }
      Hyperplane3D plane(point_in_plane, n[i]);
      planes_.push_back(plane);
    }
  }
}

void SolverNlopt::setAStarSamplesAndFractionVoxel(int a_star_samp_x, int a_star_samp_y, int a_star_samp_z,
                                                  double a_star_fraction_voxel_size)
{
  a_star_samp_x_ = a_star_samp_x;
  a_star_samp_y_ = a_star_samp_y;
  a_star_samp_z_ = a_star_samp_z;

  a_star_fraction_voxel_size_ = a_star_fraction_voxel_size;
}

void SolverNlopt::generateAStarGuess()
{
  std::cout << "[NL] Running A* from" << q0_.transpose() << " to " << final_state_.pos.transpose()
            << ", allowing time = " << kappa_ * max_runtime_ * 1000 << " ms" << std::endl;

  std::cout << bold << blue << "z_max_= " << z_max_ << reset << std::endl;

  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();
  planes_.clear();

  generateStraightLineGuess();  // If A* doesn't succeed --> use straight lineGuess
  /*  generateRandomN(n_guess_);
    generateRandomD(d_guess_);
    generateRandomQ(q_guess_);*/

  std::cout << "The StraightLineGuess is" << std::endl;
  printStd(q_guess_);
  std::cout << "************" << std::endl;

  SplineAStar myAStarSolver(num_pol_, deg_pol_, hulls_.size(), t_min_, t_max_, hulls_);

  // std::cout << "q0_=" << q0_.transpose() << std::endl;
  // std::cout << "q1_=" << q1_.transpose() << std::endl;
  // std::cout << "q2_=" << q2_.transpose() << std::endl;

  myAStarSolver.setq0q1q2(q0_, q1_, q2_);
  myAStarSolver.setGoal(final_state_.pos);

  // double runtime = 0.05;   //[seconds]
  double goal_size = 0.05;  //[meters]

  myAStarSolver.setZminZmax(z_ground_, z_max_);         // z limits for the search, in world frame
  myAStarSolver.setBBoxSearch(2000.0, 2000.0, 2000.0);  // limits for the search, centered on q2
  myAStarSolver.setMaxValuesAndSamples(v_max_, a_max_, a_star_samp_x_, a_star_samp_y_, a_star_samp_z_,
                                       a_star_fraction_voxel_size_);

  myAStarSolver.setRunTime(kappa_ * max_runtime_);  // hack, should be kappa_ * max_runtime_
  myAStarSolver.setGoalSize(goal_size);

  myAStarSolver.setBias(a_star_bias_);
  if (basis_ == MINVO)
  {
    myAStarSolver.setBasisUsedForCollision(myAStarSolver.MINVO);
  }
  else
  {
    myAStarSolver.setBasisUsedForCollision(myAStarSolver.B_SPLINE);
  }

  myAStarSolver.setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool solved = myAStarSolver.run(q, n, d);

  num_of_LPs_run_ = myAStarSolver.getNumOfLPsRun();
  // std::cout << "After Running solved, n= " << std::endl;
  // printStd(n);

  fillPlanesFromNDQ(planes_, n_guess_, d_guess_, q_guess_);

  if (solved == true)
  {
    q_guess_ = q;
    n_guess_ = n;
    d_guess_ = d;
  }
  else
  {
    std::cout << bold << red << "[NL] A* didn't find a solution, using straight line guess" << reset << std::endl;
  }

  return;
}

void SolverNlopt::generateRandomD(std::vector<double> &d)
{
  d.clear();
  for (int k = k_min_; k <= k_max_; k++)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    d.push_back(r1);
  }
}

void SolverNlopt::generateRandomN(std::vector<Eigen::Vector3d> &n)
{
  n.clear();
  for (int j = j_min_; j < j_max_; j = j + 3)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    double r2 = ((double)rand() / (RAND_MAX));
    double r3 = ((double)rand() / (RAND_MAX));
    n.push_back(Eigen::Vector3d(r1, r2, r3));
  }

  // std::cout << "After Generating RandomN, n has size= " << n.size() << std::endl;
}

void SolverNlopt::generateRandomQ(std::vector<Eigen::Vector3d> &q)
{
  q.clear();

  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist_x(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_y(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_z(z_ground_, z_max_);

  for (int i = 0; i <= N_; i++)
  {
    q.push_back(Eigen::Vector3d(dist_x(generator), dist_y(generator), dist_z(generator)));
  }

  saturateQ(q);  // make sure is inside the bounds specified
}

void SolverNlopt::findCentroidHull(const std::vector<Eigen::Vector3d> &hull, Eigen::Vector3d &centroid)
{
  centroid = Eigen::Vector3d::Zero();

  for (auto vertex : hull)
  {
    centroid += vertex;
  }
  if (hull.size() > 0)
  {
    centroid = centroid / hull.size();
    int novale = 0;
  }
}

void SolverNlopt::generateGuessNDFromQ(const std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                       std::vector<double> &d)
{
  n.clear();
  d.clear();
  signs_.clear();

  planes_.clear();

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      Eigen::Vector3d centroid_hull;
      findCentroidHull(hulls_[obst_index][i], centroid_hull);

      Eigen::Vector3d n_i =
          (centroid_hull - q[i]).normalized();  // n_i should point towards the obstacle (i.e. towards the hull)

      double alpha = 0.01;  // the smaller, the higher the chances the plane is outside the obstacle. Should be <1

      Eigen::Vector3d point_in_middle = q[i] + (centroid_hull - q[i]) * alpha;

      double d_i = -n_i.dot(point_in_middle);  // n'x + d = 0

      int sign_d_i = (d_i >= 0) ? 1 : -1;

      signs_.push_back(sign_d_i);
      n.push_back(n_i);  // n'x + 1 = 0
      d.push_back(d_i);  // n'x + 1 = 0

      Hyperplane3D plane(point_in_middle, n_i);
      planes_.push_back(plane);

      // d.push_back(d_i);
    }
  }
}

void SolverNlopt::setTminAndTmax(double t_min, double t_max)
{
  t_min_ = t_min;
  t_max_ = t_max;

  // std::cout << "t_min_= " << t_min_ << std::endl;
  // std::cout << "t_max_= " << t_max_ << std::endl;

  deltaT_ = (t_max_ - t_min_) / (1.0 * (M_ - 2 * p_ - 1 + 1));

  // std::cout << "deltaT_" << deltaT_ << std::endl;

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

  knots_ = knots;
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
#ifdef DEBUG_MODE_NLOPT
  if (var_gindex >= nn)
  {
    std::cout << "Something is wrong!!" << std::endl;
  }
#endif
  grad[r * nn + var_gindex] = tmp(0);
  grad[(r + 1) * nn + var_gindex + 1] = tmp(1);
  grad[(r + 2) * nn + var_gindex + 2] = tmp(2);
}

void SolverNlopt::toGradSameConstraintDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,
                                                    int nn)

{
#ifdef DEBUG_MODE_NLOPT
  if (var_gindex >= nn)
  {
    std::cout << "Something is wrong!!" << std::endl;
  }
#endif
  grad[r * nn + var_gindex] = tmp(0);
  grad[r * nn + var_gindex + 1] = tmp(1);
  grad[r * nn + var_gindex + 2] = tmp(2);
}

void SolverNlopt::setHulls(ConvexHullsOfCurves_Std &hulls)

{
  /*#ifdef DEBUG_MODE_NLOPT
    if (hulls.size() != num_of_segments_)
    {
      std::cout << "There should be as many hulls as segments" << std::endl;
      std::cout << "hulls_.size()=" << hulls_.size() << std::endl;
      std::cout << "num_of_segments_=" << num_of_segments_ << std::endl;
      abort();
    }
  #endif*/

  hulls_.clear();
  hulls_ = hulls;

  /*  for (int i = 0; i < hulls_.size(); i++)  // i  is the interval (\equiv segment)  //hulls_.size()
    {
      // impose that all the vertexes are on one side of the plane

      // std::cout << "Interval " << i << " has" << std::endl;
      for (int vertex_index = 0; vertex_index < hulls_[i].size(); vertex_index++)  // hulls_[i][vertex_index].size()
      {
        Eigen::Vector3d vertex = hulls_[i][vertex_index];
        std::cout << "Vertex = " << vertex.transpose() << std::endl;
      }
    }*/
}

template <class T>
void SolverNlopt::printInfeasibleConstraints(const T &constraints)
{
  std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints_[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
    }
  }
}

template <class T>
bool SolverNlopt::areTheseConstraintsFeasible(const T &constraints)
{
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      return false;
    }
  }
  return true;
}

template <class T>
int SolverNlopt::getNumberOfInfeasibleConstraints(const T &constraints)
{
  int result = 0;

  for (int i = 0; i < num_of_constraints_; i++)
  {
    std::cout << "i= " << i << std::endl;
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      result = result + 1;
    }
  }
  return result;
}

template <class T>
bool SolverNlopt::isFeasible(const T x)
{
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);
  computeConstraints(0, constraints_, num_of_variables_, NULL, q, n, d);

  // std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints_[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      return false;
      // std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
    }
  }
  return true;
}

/*template <class T>
void SolverNlopt::printInfeasibleConstraints(const T x)
{
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  toEigen(x, q, n, d);
  computeConstraints(0, constraints_, num_of_variables_, NULL, q, n, d);

  std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints_[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
    }
  }
}*/

void SolverNlopt::printInfeasibleConstraints(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                             std::vector<double> &d)
{
  computeConstraints(0, constraints_, num_of_variables_, NULL, q, n, d);

  std::cout << "The Infeasible Constraints are these ones:\n";
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints_[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet
    {
      std::cout << std::setprecision(5) << "Constraint " << i << " = " << constraints_[i] << std::endl;
    }
  }
}

void SolverNlopt::qndtoX(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                         const std::vector<double> &d, std::vector<double> &x)
{
  x.clear();

  // std::cout << "q has size= " << q.size() << std::endl;
  // std::cout << "n has size= " << n.size() << std::endl;

  for (int i = 3; i <= lastDecCP(); i++)
  {
    x.push_back(q[i](0));
    x.push_back(q[i](1));
    x.push_back(q[i](2));
  }

  for (auto n_i : n)
  {
    x.push_back(n_i(0));
    x.push_back(n_i(1));
    x.push_back(n_i(2));
  }

  for (auto d_i : d)
  {
    x.push_back(d_i);
  }
}

void SolverNlopt::initializeNumOfConstraints()
{
  // hack to get the number of constraints, calling once computeConstraints(...)
  std::vector<double> xx;
  for (int i = 0; i < num_of_variables_; i++)
  {
    xx.push_back(0.0);
  }
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;

  toEigen(xx, q, n, d);
  computeConstraints(0, constraints_, num_of_variables_, NULL, q, n, d);
  // end of hack
}

void SolverNlopt::setInitAndFinalStates(state &initial_state, state &final_state)
{
  // See https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-derv.html
  // I think equation (7) of the paper "Robust and Efficent quadrotor..." has a typo, p_ is missing there (compare
  // with equation 15 of that paper)

  Eigen::Vector3d p0 = initial_state.pos;
  Eigen::Vector3d v0 = initial_state.vel;
  Eigen::Vector3d a0 = initial_state.accel;

  Eigen::Vector3d pf = final_state.pos;
  Eigen::Vector3d vf = final_state.vel;
  Eigen::Vector3d af = final_state.accel;

  initial_state_ = initial_state;
  final_state_ = final_state;

  weight_modified_ = weight_ * (final_state_.pos - initial_state_.pos).norm();

  /*  std::cout << "initial_state= " << std::endl;
    initial_state.printHorizontal();

    std::cout << "final_state= " << std::endl;
    final_state.printHorizontal();
  */
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
}

// check if there is a normal vector = [0 0 0]
bool SolverNlopt::isDegenerate(const std::vector<double> &x)
{
  for (int j = j_min_; j <= (j_max_ - 2); j = j + 3)
  {
    Eigen::Vector3d normal(x[j], x[j + 1], x[j + 2]);
    if ((normal.norm() < 1e-5))
    {
      return true;
    }
  }
  return false;
}

template <class T>
void SolverNlopt::toEigen(T &x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                          std::vector<double> &d)
{
  q.clear();
  n.clear();
  d.clear();

  q.push_back(q0_);  // Not a decision variable
  q.push_back(q1_);  // Not a decision variable
  q.push_back(q2_);  // Not a decision variable

  // Control points (3x1)
  for (int i = i_min_; i <= i_max_ - 2; i = i + 3)
  {
    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }

  if (force_final_state_ == true)
  {
    q.push_back(qNm2_);  // Not a decision variable
    q.push_back(qNm1_);  // Not a decision variable
    q.push_back(qN_);    // Not a decision variable
  }
  else
  {
    Eigen::Vector3d qNm2(x[i_max_ - 2], x[i_max_ - 1], x[i_max_]);
    q.push_back(qNm2);  // qn-1 Not a decision variable
    q.push_back(qNm2);  // qn Not a decision variable
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

// global index of the first element of the control point i
int SolverNlopt::gIndexQ(int i)
{
#ifdef DEBUG_MODE_NLOPT
  if (i <= 2 || i > i_max_)  // Q0, Q1, Q2 are fixed (not decision variables)
  {
    std::cout << "ERROR in gIndexQ!!" << std::endl;
  }
#endif
  return 3 * i - 9;  // Q0, Q1, Q2 are always fixed (not decision variables)
}

// global index of the first element of the normal i
int SolverNlopt::gIndexN(int i)
{
#ifdef DEBUG_MODE_NLOPT
  if ((i < 0) || (i >= num_of_segments_))
  {
    std::cout << "ERROR in gIndexN!!" << std::endl;
    std::cout << "asked for " << i << std::endl;
    std::cout << "But num_of_segments_= " << num_of_segments_ << std::endl;
  }
#endif

  return 3 * i + j_min_;
}

int SolverNlopt::gIndexD(int i)
{
#ifdef DEBUG_MODE_NLOPT
  if ((i < 0) || (i >= num_of_segments_))  // Q0, Q1, Q2 are fixed (not decision variables)
  {
    std::cout << "ERROR in gIndexD!!" << std::endl;
    std::cout << "asked for " << i << std::endl;
    std::cout << "But num_of_segments_= " << num_of_segments_ << std::endl;
  }
#endif

  return i + k_min_;
}

void SolverNlopt::assignValueToGradConstraints(int var_gindex, const double &tmp, double *grad, int r, int nn)

{
  grad[r * nn + var_gindex] = tmp;
}

double SolverNlopt::computeObjFunctionJerk(unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q,
                                           std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  // Cost. See Lyx derivation (notes.lyx)
  double cost = 0.0;
  for (int i = p_; i <= N_; i++)  // i is the index of the control point
  {
    cost += (-q[i - 3] + 3 * q[i - 2] - 3 * q[i - 1] + q[i]).squaredNorm();  // the jerk of the interval i-4
                                                                             // ///////////////////
  }

  if (force_final_state_ == false)
  {
    cost += weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
  }

  if (grad)
  {
    // Initialize to zero all the elements, IT IS NEEDED (if not it doesn't converge)
    for (int i = 0; i < nn; i++)
    {
      grad[i] = 0.0;
    }

    // Gradient for the control points that are decision variables
    for (int i = p_; i <= lastDecCP(); i++)  // q0,q1,q2 are not decision variables
    {
      Eigen::Vector3d gradient;

      gradient = 2 * (-q[i - 3] + 3 * q[i - 2] - 3 * q[i - 1] + q[i])                      // Right
                 - 6 * (-q[i - 2] + 3 * q[i - 1] - 3 * q[i] + q[i + 1])                    // Center-right
                 + 6 * (-q[i - 1] + 3 * q[i] - 3 * q[i + 1] + q[i + 2])                    // Center-right
                 - 2 * (i <= (N_ - 3)) * (-q[i] + 3 * q[i + 1] - 3 * q[i + 2] + q[i + 3])  // Left
                 + (i == (N_ - 2)) * 2 * weight_modified_ * (q[i] - final_state_.pos);

      assignEigenToVector(grad, gIndexQ(i), gradient);
    }
  }

  // std::cout << "cost= " << cost << std::endl;

  return cost;
}

double SolverNlopt::computeObjFunction(unsigned nn, double *grad, std::vector<Eigen::Vector3d> &q,
                                       std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  // Cost
  double cost = 0.0;
  for (int i = 1; i <= (N_ - 1); i++)
  {
    cost += (q[i + 1] - 2 * q[i] + q[i - 1]).squaredNorm();
  }

  if (force_final_state_ == false)
  {
    cost += weight_modified_ * (q[N_] - final_state_.pos).squaredNorm();
  }

  if (grad)
  {
    // Initialize to zero all the elements, IT IS NEEDED (if not it doesn't converge)
    for (int i = 0; i < nn; i++)
    {
      grad[i] = 0.0;
    }

    // Gradient for the control points that are decision variables
    for (int i = 3; i <= lastDecCP(); i++)
    {
      Eigen::Vector3d gradient;

      if (i == (N_ - 2))  // not reached when force_final_state_==true
      {
        gradient = -2 * (q[i - 1] - q[i]) + 2 * (q[i] - 2 * q[i - 1] + q[i - 2]);
        gradient += 2 * weight_modified_ * (q[i] - final_state_.pos);
      }
      else
      {
        gradient = 2 * (q[i] - 2 * q[i - 1] + q[i - 2]) +     ///////////// Right
                   (-4 * (q[i + 1] - 2 * q[i] + q[i - 1])) +  // Center
                   (2 * (q[i + 2] - 2 * q[i + 1] + q[i]));    //// Left
      }
      assignEigenToVector(grad, gIndexQ(i), gradient);
    }
  }

  // std::cout << "cost= " << cost << std::endl;

  return cost;
}

double SolverNlopt::myObjFunc(unsigned nn, const double *x, double *grad, void *my_func_data)
{
  SolverNlopt *opt = reinterpret_cast<SolverNlopt *>(my_func_data);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->toEigen(x, q, n, d);

  double cost = opt->computeObjFunctionJerk(nn, grad, q, n, d);

  return cost;
}

// 3,...,N_ are dec points if force_final_state_=false (0,1,2 aren't)
// 3,...,N_-3 are dec points if force_final_state_=true (0,1,2 aren't)
bool SolverNlopt::isADecisionCP(int i)

{  // If Q[i] is a decision variable
#ifdef DEBUG_MODE_NLOPT
  if (i > N_)  // N_ is the last cpoint
  {
    std::cout << "There is sth wrong, i=" << i << std::endl;
    std::cout << "But N_=" << N_ << std::endl;
  }
#endif
  return (force_final_state_ == true) ? (i >= 3 && i <= (N_ - 3)) : ((i >= 3) && i <= (N_ - 2));
}

//[seconds]
void SolverNlopt::setMaxRuntime(double deltaT)
{
  max_runtime_ = deltaT;
}

double SolverNlopt::getTimeNeeded()
{
  return time_needed_;
}

int SolverNlopt::lastDecCP()
{
  return (force_final_state_ == true) ? (N_ - 3) : (N_ - 2);
}

void SolverNlopt::transformBSpline2Minvo(Eigen::Matrix<double, 4, 3> &Qbs, Eigen::Matrix<double, 4, 3> &Qmv)
{
  /////////////////////
  // Eigen::Matrix<double, 4, 3> Qmv;  // minvo

  Qmv = Mbs2mv_ * Qbs;
}

void SolverNlopt::setBasisUsedForCollision(int basis)
{
  basis_ = basis;
}

void SolverNlopt::setAStarBias(double a_star_bias)
{
  a_star_bias_ = a_star_bias;
}

// m is the number of constraints, nn is the number of variables
void SolverNlopt::computeConstraints(unsigned m, double *constraints, unsigned nn, double *grad,
                                     std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n,
                                     std::vector<double> &d)
{
  Eigen::Vector3d ones = Eigen::Vector3d::Ones();
  int r = 0;
  // grad is a vector with nn*m elements
  // Initialize grad to 0 all the elements, not sure if needed
  /*  if (grad)
    {
      for (int i = 0; i < nn * m; i++)
      {
        grad[i] = 0.0;
      }
    }*/

  index_const_obs_ = r;
  /*#ifdef DEBUG_MODE_NLOPT
    // std::cout << "here1" << std::endl;*/
  // std::cout << "Going to add plane constraints, r= " << r << std::endl;
  /*  //#endif
   std::cout << "num_of_obst_ " << num_of_obst_ << std::endl;
   std::cout << "num_of_segments_ " << num_of_segments_ << std::endl;*/

  // See here why we can use an epsilon of 1.0:
  // http://www.joyofdata.de/blog/testing-linear-separability-linear-programming-r-glpk/
  double epsilon = 1.0;

  for (int i = 0; i <= (N_ - 3); i++)  // i  is the interval (\equiv segment)
  {
    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      int ip = obst_index * num_of_segments_ + i;  // index plane

      // int sign_d_i = signs_[ip];

      // impose that all the vertexes of the obstacle are on one side of the plane
      // std::cout << "Vertexes of Obstacle " << obst_index << std::endl;

      for (Eigen::Vector3d vertex : hulls_[obst_index][i])  // opt->hulls_[i].size()
      {
        constraints[r] = -(n[ip].dot(vertex) + d[ip] - epsilon);  //+d[ip] // f<=0

        /*        if (constraints[r] > epsilon_tol_constraints_)
                {
                  std::cout << bold << red << "sign_d_i= " << sign_d_i << reset << std::endl;
                  std::cout << "n[ip]= " << n[ip] << std::endl;
                  std::cout << "ip= " << ip << std::endl;
                  std::cout << "vertex= " << vertex << std::endl;
                  std::cout << "constraints[r]= " << constraints[r] << std::endl;
                }*/

        if (grad)
        {
          toGradSameConstraintDiffVariables(gIndexN(ip), -vertex, grad, r, nn);
          assignValueToGradConstraints(gIndexD(ip), -1, grad, r, nn);
        }
        r++;
      }

      if (basis_ == MINVO)
      {
        Eigen::Matrix<double, 4, 3> Qbs;  // b-spline
        Eigen::Matrix<double, 4, 3> Qmv;  // minvo
        Qbs.row(0) = q[i].transpose();
        Qbs.row(1) = q[i + 1].transpose();
        Qbs.row(2) = q[i + 2].transpose();
        Qbs.row(3) = q[i + 3].transpose();
        transformBSpline2Minvo(Qbs, Qmv);  // Now Qmv is a matrix whose each row contains a MINVO control point

        // std::cout << "Control Points" << std::endl;
        // and the control points on the other side
        Eigen::Vector3d q_ipu;
        for (int u = 0; u <= 3; u++)
        {
          q_ipu = Qmv.row(u).transpose();                         // if using the MINVO basis
          constraints[r] = (n[ip].dot(q_ipu) + d[ip] + epsilon);  //  // fi<=0

          if (grad)
          {
            toGradSameConstraintDiffVariables(gIndexN(ip), q_ipu, grad, r, nn);

            // If using the MINVO basis
            for (int k = 0; k <= 3; k++)  // This loop is needed because each q in MINVO depends on every q in BSpline
            {
              if (isADecisionCP(i + k))  // If Q[i] is a decision variable
              {
                toGradSameConstraintDiffVariables(gIndexQ(i + k), Mbs2mv_(u, k) * n[ip], grad, r, nn);
              }
            }
            assignValueToGradConstraints(gIndexD(ip), 1, grad, r, nn);
          }
          r++;
        }
      }
      else  // NOT using the MINVO Basis
      {
        for (int u = 0; u <= 3; u++)
        {
          constraints[r] = (n[ip].dot(q[i + u]) + d[ip] + epsilon);  //  // f<=0
          if (grad)
          {
            toGradSameConstraintDiffVariables(gIndexN(ip), q[i + u], grad, r, nn);
            if (isADecisionCP(i + u))  // If Q[i] is a decision variable
            {
              toGradSameConstraintDiffVariables(gIndexQ(i + u), n[ip], grad, r, nn);
            }
            assignValueToGradConstraints(gIndexD(ip), 1, grad, r, nn);
          }
          r++;
        }
      }
    }
  }

  index_const_vel_ = r;
  /*#ifdef DEBUG_MODE_NLOPT*/
  // std::cout << "Going to add velocity constraints, r= " << r << std::endl;
  //#endif
  // VELOCITY CONSTRAINTS:
  for (int i = 2; i <= (N_ - 3); i++)  // v0 and v1 are already determined by initial_state
  {
    double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
    Eigen::Vector3d v_i = c1 * (q[i + 1] - q[i]);

    // v<=vmax  \equiv  v_i - vmax <= 0
    assignEigenToVector(constraints, r, v_i - v_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), -c1 * ones, grad, r, nn);
      }
      if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), c1 * ones, grad, r, nn);
      }
    }
    r = r + 3;

    // v>=-vmax  \equiv  -v_i - vmax <= 0
    assignEigenToVector(constraints, r, -v_i - v_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), c1 * ones, grad, r, nn);
      }
      if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -c1 * ones, grad, r, nn);
      }
    }
    r = r + 3;
  }
  //#ifdef DEBUG_MODE_NLOPT
  // std::cout << "Going to add acceleration constraints, r= " << r << std::endl;
  //#endif
  index_const_accel_ = r;
  // ACCELERATION CONSTRAINTS:
  for (int i = 1; i <= (N_ - 3); i++)  // a0 is already determined by the initial state
  {
    double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
    double c2 = p_ / (knots_(i + p_ + 1 + 1) - knots_(i + 1 + 1));
    double c3 = (p_ - 1) / (knots_(i + p_ + 1) - knots_(i + 2));

    Eigen::Vector3d v_i = c1 * (q[i + 1] - q[i]);
    Eigen::Vector3d v_iP1 = c2 * (q[i + 2] - q[i + 1]);
    Eigen::Vector3d a_i = c3 * (v_iP1 - v_i);

    // a<=amax  ==  a_i - amax <= 0  ==  c3 * (v_iP1 - v_i)<=0 ==
    // c3*c2 *q[i + 2] - c3*c2* q[i + 1]  -  c3*c1*q[i + 1] + c3*c1*q[i]   - amax <= 0
    assignEigenToVector(constraints, r, a_i - a_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), c3 * c1 * ones, grad, r, nn);
      }
      if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), (-c3 * c2 - c3 * c1) * ones, grad, r, nn);
      }
      if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), c3 * c2 * ones, grad, r, nn);
      }
    }
    r = r + 3;

    // a>=-amax    \equiv  -a_i - amax <= 0
    assignEigenToVector(constraints, r, -a_i - a_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), -c3 * c1 * ones, grad, r, nn);
      }
      if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -(-c3 * c2 - c3 * c1) * ones, grad, r, nn);
      }
      if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), -c3 * c2 * ones, grad, r, nn);
      }
    }
    r = r + 3;
  }

  index_const_normals_ = r;
  // Impose that the normals are not [0 0 0]
  for (int i = 0; i < n.size(); i++)
  {
    double min_norm_squared = 1;  // normals should have at least module^2 min_norm_squared

    // std::cout << "n[i]= " << n[i].transpose() << std::endl;
    constraints[r] = min_norm_squared - n[i].dot(n[i]);  // f<=0
    if (grad)
    {
      toGradSameConstraintDiffVariables(gIndexN(i), -2 * n[i], grad, r, nn);
    }
    r++;
  }

#ifdef DEBUG_MODE_NLOPT

  for (int i = 0; i < m; i++)
  {
    std::cout << "Constraint " << i << " = " << constraints[i] << std::endl;
  }
  std::cout << "num_of_constraints_= " << num_of_constraints_ << std::endl;
  std::cout << "m= " << m << std::endl;
#endif

  num_of_constraints_ = r;  // + 1 has already been added in the last loop of the previous for loop;
}

// See example https://github.com/stevengj/nlopt/issues/168
// m is the number of constraints (which is computed from the lentgh of the vector of tol_constraint)
// nn is the number of variables
void SolverNlopt::myIneqConstraints(unsigned m, double *constraints, unsigned nn, const double *x, double *grad,
                                    void *f_data)
{
  SolverNlopt *opt = reinterpret_cast<SolverNlopt *>(f_data);

  // std::cout << "in myIneqConstraints, m=" << m << ", n=" << nn << std::endl;

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->toEigen(x, q, n, d);
  // opt->printQND(q, n, d);
  opt->computeConstraints(m, constraints, nn, grad, q, n, d);

  // Be careful cause this adds more runtime...
  // printInfeasibleConstraints(constraints);
  /*  if (opt->areTheseConstraintsFeasible(constraints))
    {
      opt->got_a_feasible_solution_ = true;
      double cost_now = opt->computeObjFunctionJerk(nn, NULL, q, n, d);
      if (cost_now < opt->best_cost_so_far_)
      {
        opt->best_cost_so_far_ = cost_now;
        // Copy onto the std::vector)
        for (int i = 0; i < nn; i++)
        {
          opt->best_feasible_sol_so_far_[i] = x[i];
        }
      }
    }*/

  /**/
  return;
}

void SolverNlopt::printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
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

void SolverNlopt::generateRandomGuess()
{
  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();

  generateRandomN(n_guess_);
  generateRandomD(d_guess_);
  generateRandomQ(q_guess_);

  /*  q_guess_.clear();
    n_guess_.clear();
    d_guess_.clear();

    q_guess_.push_back(q0_);  // Not a decision variable
    q_guess_.push_back(q1_);  // Not a decision variable
    q_guess_.push_back(q2_);  // Not a decision variable

    // Control points (3x1)
    for (int i = i_min_; i <= i_max_ - 2; i = i + 3)
    {
      double r1 = ((double)rand() / (RAND_MAX));
      double r2 = ((double)rand() / (RAND_MAX));
      double r3 = ((double)rand() / (RAND_MAX));
      q_guess_.push_back(Eigen::Vector3d(r1, r2, r3));
    }

    q_guess_.push_back(q_guess_.back());  // Not a decision variable
    q_guess_.push_back(q_guess_.back());  // Not a decision variable

    generateRandomN(n_guess_);
    generateRandomD(d_guess_);*/
}

void SolverNlopt::printStd(const std::vector<double> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i << std::endl;
  }
}

void SolverNlopt::printStd(const std::vector<Eigen::Vector3d> &v)
{
  for (auto v_i : v)
  {
    std::cout << v_i.transpose() << std::endl;
  }
}

void SolverNlopt::setDistanceToUseStraightLine(double dist_to_use_straight_guess)
{
  dist_to_use_straight_guess_ = dist_to_use_straight_guess;
}

void SolverNlopt::printIndexesConstraints()
{
  std::cout << "_______________________" << std::endl;
  std::cout << "Indexes constraints" << std::endl;
  std::cout << "Obstacles: " << index_const_obs_ << "-->" << index_const_vel_ - 1 << std::endl;
  std::cout << "Velocity: " << index_const_vel_ << "-->" << index_const_accel_ - 1 << std::endl;
  std::cout << "Accel: " << index_const_accel_ << "-->" << index_const_normals_ - 1 << std::endl;
  std::cout << "Normals: >" << index_const_normals_ << std::endl;
  std::cout << "_______________________" << std::endl;
}

bool SolverNlopt::optimize()

{
  // generateRandomGuess();
  // generateStraightLineGuess();
  if ((initial_state_.pos - final_state_.pos).norm() < dist_to_use_straight_guess_ || num_of_obst_ == 0)
  {
    generateStraightLineGuess();
  }
  else
  {
    generateAStarGuess();
  }

  // std::cout << "knots= " << knots_ << std::endl;

  // the creations of the solvers should be done here, and NOT on the constructor (not sure why, but if you do it in the
  // construtor of this class, and use the same ones forever, it gets stuck very often)
  if (opt_)
  {
    (*opt_).~opt();  // Call the destructor
    delete opt_;
  }
  if (local_opt_)
  {
    (*local_opt_).~opt();  // Call the destructor
    delete local_opt_;
  }

  opt_ = new nlopt::opt(nlopt::AUGLAG, num_of_variables_);
  local_opt_ = new nlopt::opt(solver_, num_of_variables_);

  local_opt_->set_xtol_rel(xtol_rel_);  // stopping criteria. If >=1e-1, it leads to weird trajectories
  local_opt_->set_ftol_rel(ftol_rel_);  // stopping criteria. If >=1e-1, it leads to weird trajectories
  opt_->set_local_optimizer(*local_opt_);
  opt_->set_xtol_rel(xtol_rel_);  // Stopping criteria. If >=1e-1, it leads to weird trajectories
  opt_->set_ftol_rel(ftol_rel_);  // Stopping criteria. If >=1e-1, it leads to weird trajectories

  // opt_->set_maxeval(1e6);  // maximum number of evaluations. Negative --> don't use this criterion
  // max_runtime_ = 0.2;               // hack
  opt_->set_maxtime(std::max(mu_ * max_runtime_, 0.001));  // 0.001 to make use this criterion is used  // maximum time
                                                           // in seconds. Negative --> don't use this criterion

  // opt_->set_maxtime(max_runtime_);
  initializeNumOfConstraints();

  // see https://github.com/stevengj/nlopt/issues/168
  std::vector<double> tol_constraints;
  for (int i = 0; i < num_of_constraints_; i++)
  {
    tol_constraints.push_back(epsilon_tol_constraints_);
  }

  // andd lower and upper bounds
  std::vector<double> lb;
  std::vector<double> ub;
  // control points q
  for (int i = 0; i <= i_max_; i = i + 3)
  {
    lb.push_back(-HUGE_VAL);
    ub.push_back(HUGE_VAL);
    // y conpoment
    lb.push_back(-HUGE_VAL);
    ub.push_back(HUGE_VAL);
    // z conpoment
    lb.push_back(z_ground_);
    ub.push_back(z_max_);
  }
  // normals n
  for (int j = j_min_; j <= j_max_; j++)
  {
    lb.push_back(-HUGE_VAL);
    ub.push_back(HUGE_VAL);
  }
  // coefficients d
  for (int k = k_min_; k <= k_max_; k++)
  {
    lb.push_back(-HUGE_VAL);
    ub.push_back(HUGE_VAL);
  }
  opt_->set_lower_bounds(lb);
  opt_->set_upper_bounds(ub);
  local_opt_->set_lower_bounds(lb);
  local_opt_->set_upper_bounds(ub);

  // set constraint and objective
  opt_->add_inequality_mconstraint(SolverNlopt::myIneqConstraints, this, tol_constraints);
  opt_->set_min_objective(SolverNlopt::myObjFunc,
                          this);  // this is passed as a parameter (the obj function has to be static)

  double minf;

  best_feasible_sol_so_far_.resize(num_of_variables_);
  got_a_feasible_solution_ = false;

  qndtoX(q_guess_, n_guess_, d_guess_, x_);

  // std::cout << bold << blue << "GUESSES: " << reset << std::endl;
  // std::cout << "q_guess_ is\n" << std::endl;
  // printStd(q_guess_);

  // std::cout << "n_guess_ is\n" << std::endl;
  // printStd(n_guess_);

  // std::cout << "d_guess_ is\n" << std::endl;
  // printStd(d_guess_);

  // // toEigen(x_, q_guess_, n_guess_);

  // std::cout << bold << "The infeasible constraints of the initial Guess" << reset << std::endl;
  // printInfeasibleConstraints(q_guess_, n_guess_, d_guess_);

  // printIndexesConstraints();

  opt_timer_.Reset();
  std::cout << "[NL] Optimizing now, allowing time = " << mu_ * max_runtime_ * 1000 << "ms" << std::endl;
  int result = opt_->optimize(x_, minf);
  time_needed_ = opt_timer_.ElapsedMs() / 1000;

  num_of_QCQPs_run_++;

  got_a_feasible_solution_ = isFeasible(x_);  // wasn't here
  bool x_is_deg = isDegenerate(x_);
  bool feas_is_deg = isDegenerate(x_);  // was best_feasible_sol_so_far_

  // See codes in https://github.com/JuliaOpt/NLopt.jl/blob/master/src/NLopt.jl
  got_a_feasible_solution_ = got_a_feasible_solution_ && (!feas_is_deg);
  bool optimal = (result == nlopt::SUCCESS) && (!x_is_deg);
  bool failed = (!optimal) && (!got_a_feasible_solution_);
  bool feasible_but_not_optimal = (!optimal) && (got_a_feasible_solution_);

  // Store the results here
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;

  // std::cout << "[NL] result= " << getResultCode(result) << std::endl;

  if (failed)
  {
    printf("[NL] nlopt failed or maximum time was reached!\n");

    std::cout << on_red << bold << "[NL] Solution not found" << opt_timer_ << reset << std::endl;

    toEigen(x_, q, n, d);
    // printInfeasibleConstraints(q, n, d);

    return false;
  }
  else if (optimal)
  {
    std::cout << on_green << bold << "[NL] Optimal Solution found" << opt_timer_ << reset << std::endl;
    toEigen(x_, q, n, d);
  }
  else if (feasible_but_not_optimal)
  {
    std::cout << on_green << bold << "[NL] Feasible Solution found" << opt_timer_ << reset << std::endl;
    toEigen(x_, q, n, d);  // was best_feasible_sol_so_far_
  }
  else
  {
    std::cout << on_red << bold << "[NL] not implemented yet" << opt_timer_ << reset << std::endl;
    return false;
  }

  // printQND(q, n, d);

  /*  std::cout << on_green << bold << "Solution found: " << time_first_feasible_solution_ << "/" << opt_timer_ << reset
              << std::endl;*/

  CPs2TrajAndPwp(q, X_temp_, solution_, N_, p_, num_pol_, knots_, dc_);

  //  fillXTempFromCPs(q);

  // Force the last position to be the final_state_ (it's not guaranteed to be because of the discretization with dc_)
  if (force_final_state_ == true)
  {
    X_temp_.back() = final_state_;
  }
  else
  {
    X_temp_.back().vel = final_state_.vel;
    X_temp_.back().accel = final_state_.accel;
  }

  // std::cout << "Done filling the solution" << std::endl;

  return true;
}

void SolverNlopt::getSolution(PieceWisePol &solution)
{
  solution = solution_;
}

void SolverNlopt::saturateQ(std::vector<Eigen::Vector3d> &q)
{
  for (int i = 0; i < q.size(); i++)
  {
    q[i].z() = std::max(q[i].z(), z_ground_);  // Make sure it's within the limits
    q[i].z() = std::min(q[i].z(), z_max_);     // Make sure it's within the limits
  }
}

void SolverNlopt::generateStraightLineGuess()
{
  // std::cout << "Using StraightLineGuess" << std::endl;
  q_guess_.clear();
  n_guess_.clear();
  d_guess_.clear();

  q_guess_.push_back(q0_);  // Not a decision variable
  q_guess_.push_back(q1_);  // Not a decision variable
  q_guess_.push_back(q2_);  // Not a decision variable

  for (int i = 1; i < (N_ - 2 - 2); i++)
  {
    Eigen::Vector3d q_i = q2_ + i * (final_state_.pos - q2_) / (N_ - 2 - 2);
    q_guess_.push_back(q_i);
  }

  q_guess_.push_back(qNm2_);  // three last cps are the same because of the vel/accel final conditions
  q_guess_.push_back(qNm1_);
  q_guess_.push_back(qN_);
  // Now q_guess_ should have (N_+1) elements
  saturateQ(q_guess_);  // make sure is inside the bounds specified

  // std::vector<Eigen::Vector3d> q_guess_with_qNm1N = q_guess_;
  // q_guess_with_qNm1N.push_back(qNm1_);
  // q_guess_with_qNm1N.push_back(qN_);
  //////////////////////

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      std::vector<Eigen::Vector3d> last4Cps(4);
      if (basis_ == MINVO)
      {
        Eigen::Matrix<double, 4, 3> Qbs;  // b-spline
        Eigen::Matrix<double, 4, 3> Qmv;  // minvo
        Qbs.row(0) = q_guess_[i].transpose();
        Qbs.row(1) = q_guess_[i + 1].transpose();
        Qbs.row(2) = q_guess_[i + 2].transpose();
        Qbs.row(3) = q_guess_[i + 3].transpose();

        transformBSpline2Minvo(Qbs, Qmv);  // Now Qmv is a matrix whose each row contains a MINVO control point

        std::vector<Eigen::Vector3d> last4Cps(4);
        last4Cps[0] = Qmv.row(0).transpose();
        last4Cps[1] = Qmv.row(1).transpose();
        last4Cps[2] = Qmv.row(2).transpose();
        last4Cps[3] = Qmv.row(3).transpose();
      }
      else
      {
        last4Cps[0] = q_guess_[i].transpose();
        last4Cps[1] = q_guess_[i + 1].transpose();
        last4Cps[2] = q_guess_[i + 2].transpose();
        last4Cps[3] = q_guess_[i + 3].transpose();
      }

      Eigen::Vector3d n_i;
      double d_i;
      // std::cout << "===================================" << std::endl;

      bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][i], last4Cps);
      /*      std::cout << "satisfies_LP= " << satisfies_LP << std::endl;
            std::cout << "last4Cps[0]=" << last4Cps[0].transpose() << std::endl;
            std::cout << "last4Cps[1]=" << last4Cps[1].transpose() << std::endl;
            std::cout << "last4Cps[2]=" << last4Cps[2].transpose() << std::endl;
            std::cout << "last4Cps[3]=" << last4Cps[3].transpose() << std::endl;

            std::cout << "hulls_[obst_index][i][0]= " << hulls_[obst_index][i][0].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][1]= " << hulls_[obst_index][i][1].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][2]= " << hulls_[obst_index][i][2].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][3]= " << hulls_[obst_index][i][3].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][4]= " << hulls_[obst_index][i][4].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][5]= " << hulls_[obst_index][i][5].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][6]= " << hulls_[obst_index][i][6].transpose() << std::endl;
            std::cout << "hulls_[obst_index][i][7]= " << hulls_[obst_index][i][7].transpose() << std::endl;*/

      n_guess_.push_back(n_i);
      d_guess_.push_back(d_i);

      /*      Eigen::Vector3d centroid_hull;
            findCentroidHull(hulls_[obst_index][i], centroid_hull);

            Eigen::Vector3d n_i =
                (centroid_hull - q[i]).normalized();  // n_i should point towards the obstacle (i.e. towards the hull)

            double alpha = 0.01;  // the smaller, the higher the chances the plane is outside the obstacle. Should be <1

            Eigen::Vector3d point_in_middle = q[i] + (centroid_hull - q[i]) * alpha;

            double d_i = -n_i.dot(point_in_middle);  // n'x + d = 0

            int sign_d_i = (d_i >= 0) ? 1 : -1;

            signs_.push_back(sign_d_i);
            n.push_back(n_i);  // n'x + 1 = 0
            d.push_back(d_i);  // n'x + 1 = 0

            Hyperplane3D plane(point_in_middle, n_i);
            planes_.push_back(plane);
      */
      // d.push_back(d_i);
    }
  }

  // for (int seg_index = 0; seg_index < num_of_segments_; seg_index++)
  // {
  //   for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  //   {
  //     Eigen::Vector3d n_i;
  //     double d_i;

  //     bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][current.index + 1 - 3],
  //     last4Cps);

  //     /*      if (satisfies_LP == false)
  //           {
  //             // std::cout << neighbor_va.qi.transpose() << " does not satisfy constraints" << std::endl;
  //             break;
  //           }*/
  //   }
  // }
  //////////////////////

  /*  generateRandomD(d_guess_);
    generateRandomN(n_guess_);*/
  // generateGuessNDFromQ(q_guess_, n_guess_, d_guess_);
  // generateRandomN(n_guess_);
  // Guesses for the planes
  /*
    std::cout << "This is the initial guess: " << std::endl;
    std::cout << "q.size()= " << q_guess_.size() << std::endl;
    std::cout << "n.size()= " << n_guess_.size() << std::endl;
    std::cout << "num_of_variables_= " << num_of_variables_ << std::endl;

    printQND(q_guess_, n_guess_, d_guess_);*/
}

nlopt::algorithm SolverNlopt::getSolver(std::string &solver)
{
  // nlopt::algorithm_from_string("LD_MMA"); //doesn't work in c++

  if (solver == "LD_MMA")
  {
    return nlopt::LD_MMA;
  }
  else if (solver == "LN_NELDERMEAD")
  {
    return nlopt::LN_NELDERMEAD;
  }
  else if (solver == "LN_SBPLX")
  {
    return nlopt::LN_SBPLX;
  }

  else if (solver == "LN_PRAXIS")
  {
    return nlopt::LN_PRAXIS;
  }

  else if (solver == "LD_AUGLAG")
  {
    return nlopt::LD_AUGLAG;
  }
  else if (solver == "LD_AUGLAG_EQ")
  {
    return nlopt::LD_AUGLAG_EQ;
  }
  else if (solver == "LN_BOBYQA")
  {
    return nlopt::LN_BOBYQA;
  }
  else if (solver == "LD_SLSQP")
  {
    return nlopt::LD_SLSQP;
  }
  else if (solver == "LN_NEWUOA")
  {
    return nlopt::LN_NEWUOA;
  }
  else if (solver == "LN_NEWUOA_BOUND")
  {
    return nlopt::LN_NEWUOA_BOUND;
  }
  else if (solver == "LD_TNEWTON_PRECOND_RESTART")
  {
    return nlopt::LD_TNEWTON_PRECOND_RESTART;
  }
  else if (solver == "LD_TNEWTON_RESTART")
  {
    return nlopt::LD_TNEWTON_RESTART;
  }
  else if (solver == "LD_TNEWTON_PRECOND")
  {
    return nlopt::LD_TNEWTON_PRECOND;
  }
  else if (solver == "LD_VAR1")
  {
    return nlopt::LD_VAR1;
  }
  else if (solver == "LD_VAR2")
  {
    return nlopt::LD_VAR2;
  }
  else if (solver == "LD_LBFGS_NOCEDAL")
  {
    return nlopt::LD_LBFGS_NOCEDAL;
  }
  else
  {
    std::cout << "Are you sure this solver exists?" << std::endl;
    abort();
  }
}

std::string SolverNlopt::getResultCode(int &result)
{
  switch (result)
  {
    case -5:
      return "Forced_Stop";
    case -4:
      return "Roundoff_limited";
    case -3:
      return "Out_of_memory";
    case -2:
      return "Invalid_args";
    case -1:
      return "Failure";
    case 1:
      return "Success";
    case 2:
      return "Stopval_reached";
    case 3:
      return "Ftol_reached";
    case 4:
      return "Xtol_reached";
    case 5:
      return "Maxeval_reached";
    case 6:
      return "Maxtime_reached";
    default:
      return "Result_Code_unknown";
  }
}

/*void SolverNlopt::computeVeli(Eigen::Vector3d &vel, std::vector<Eigen::Vector3d> &q)
{
  int i = q.size() - 2;

  if (i >= (q.size() - 1))
  {
    std::cout << "Velocity cannot be ccomputed for this index" << std::endl;
    return;
  }
  vel = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
}

void SolverNlopt::computeAcceli(Eigen::Vector3d &accel, std::vector<Eigen::Vector3d> &q)
{
  int i = q.size() - 3;

  std::vector<Eigen::Vector3d> q_reduced;
  q_reduced.push_back(q[q.size() - 4]);
  q_reduced.push_back(q[q.size() - 3]);
  q_reduced.push_back(q[q.size() - 2]);

  Eigen::Vector3d vi;
  computeVeli(vi, q_reduced);

  Eigen::Vector3d viP1;
  computeVeli(viP1, q);

  std::cout << "In computeAcceli q=" << std::endl;
  printStd(q);
  std::cout << "In computeAcceli vi=" << vi.transpose() << std::endl;
  std::cout << "In computeAcceli viP1" << viP1.transpose() << std::endl;

  accel = (p_ - 1) * (viP1 - vi) / (knots_(i + p_ + 1) - knots_(i + 2));
}

// Given a vector of control points q0,...,qj
// it checks if v_{j-1} satisfies vmax constraints
// and if  a_{j-2} satisfies amax constraints
bool SolverNlopt::satisfiesVmaxAmax(std::vector<Eigen::Vector3d> &q)
{
  if (q.size() <= 2)
  {
    std::cout << bold << red << "Velocity and accel cannot be computed for this q" << reset << std::endl;
    return false;
  }

  Eigen::Vector3d vi;
  computeVeli(vi, q);
  Eigen::Vector3d aiM1;
  computeAcceli(aiM1, q);

  std::cout << "vi= " << vi.transpose() << std::endl;
  std::cout << "ai= " << aiM1.transpose() << std::endl;

  return ((vi.array().abs() <= v_max_.array()).all() && (aiM1.array().abs() <= a_max_.array()).all());
}*/

// Given std::vector<Eigen::Vector3d> &q, this generates a sample that satisfies the vmax and amax constraints wrt the
// last 3 cpoints of q
/*void SolverNlopt::sampleFeasible(Eigen::Vector3d &qiP1, std::vector<Eigen::Vector3d> &q)
{
 if (q.size() <= 2)
 {
   std::cout << bold << red << "q should be bigger" << reset << std::endl;
 }

 int i = q.size() - 1;

 double tmp = (knots_(i + p_ + 1) - knots_(i + 2)) / (1.0 * (p_ - 1));

 double constraint_x = std::min(v_max_.x(), a_max_.x() * tmp);
 double constraint_y = std::min(v_max_.y(), a_max_.y() * tmp);
 double constraint_z = std::min(v_max_.z(), a_max_.z() * tmp);

 std::default_random_engine generator;
 generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
 std::uniform_real_distribution<double> distribution_x(-constraint_x, +constraint_x);
 std::uniform_real_distribution<double> distribution_y(-constraint_y, +constraint_y);
 std::uniform_real_distribution<double> distribution_z(-constraint_z, +constraint_z);

 Eigen::Vector3d vi(distribution_x(generator), distribution_y(generator),
                    distribution_z(generator));  // velocity sample

 std::cout << "q[i]= " << q[i].transpose() << std::endl;
 std::cout << "Velocity sample= " << vi.transpose() << std::endl;

 qiP1 = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + q[i];

 std::cout << "q[i+1]= " << qiP1.transpose() << std::endl;

 std::cout << "====================" << std::endl;
 // test
  std::vector<Eigen::Vector3d> last4Cps(4);
   std::copy(q.end() - 3, q.end(), last4Cps.begin());  // copy three elements
   last4Cps[3] = qiP1;
   satisfiesVmaxAmax(last4Cps);


   // more tests
   Eigen::Vector3d vel_test;
   Eigen::Vector3d accel_test;
   computeVeli(vel_test, 2, last4Cps);
   computeAcceli(accel_test, 1, last4Cps);
   std::cout << "vel_test=" << vel_test.transpose() << std::endl;
   std::cout << "accel_test=" << accel_test.transpose() << std::endl;

   std::cout << "====================" << std::endl;
   std::cout << "====================" << std::endl;

 // vel = p_ * (qiP1 - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
 // accel = (p_ - 1) * (viP1 - vi) / (knots_(i + p_ + 1) - knots_(i + 2));
}*/

/*void SolverNlopt::useJPSGuess(vec_Vecf<3> &jps_path)
{
  q_guess_.clear();
  n_guess_.clear();
  d_guess_.clear();
  // std::vector<double> d;

  // Guesses for the control points
  int num_of_intermediate_cps = N_ + 1 - 6;
  vec_Vecf<3> intermediate_cps =
      sampleJPS(jps_path, num_of_intermediate_cps + 1);  //+1 because the first vertex is always returned

  intermediate_cps.erase(intermediate_cps.begin());  // remove the first vertex

  std::cout << "intermediate_cps has size= " << intermediate_cps.size() << std::endl;

  q_guess_.push_back(q0_);  // Not a decision variable
  q_guess_.push_back(q1_);  // Not a decision variable
  q_guess_.push_back(q2_);  // Not a decision variable

  for (auto q_i : intermediate_cps)
  {
    q_guess_.push_back(q_i);
  }

  q_guess_.push_back(final_state_.pos);  // three last cps are the same because of the vel/accel final conditions
  q_guess_.push_back(final_state_.pos);
  q_guess_.push_back(final_state_.pos);

  generateGuessNDFromQ(q_guess_, n_guess_, d_guess_);
  // generateRandomN(n_guess_);
  // Guesses for the planes

  std::cout << "This is the initial guess: " << std::endl;
  std::cout << "q.size()= " << q_guess_.size() << std::endl;
  std::cout << "n.size()= " << n_guess_.size() << std::endl;
  std::cout << "num_of_variables_= " << num_of_variables_ << std::endl;

  printQND(q_guess_, n_guess_, d_guess_);
}*/

/*void SolverNlopt::useRRTGuess()  // vec_E<Polyhedron<3>> &polyhedra
{
  // sleep(1);
  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();
  planes_.clear();

  generateRandomN(n_guess_);
  generateRandomD(d_guess_);
  generateRandomQ(q_guess_);

  int num_of_intermediate_cps = N_ + 1 - 6;

  Eigen::Vector3d qNm2 = final_state_.pos;

  Eigen::Vector3d high_value = 100 * Eigen::Vector3d::Ones();  // to avoid very extreme values

  double best_cost = std::numeric_limits<double>::max();

  signs_.clear();
  for (int i = 0; i < n_guess_.size(); i++)
  {
    signs_.push_back(1);
  }

  planes_.clear();

  MyTimer guess_timer(true);

  for (int trial = 0; trial < 1; trial++)
  {
    std::cout << "trial= " << trial << std::endl;
    std::cout << "num_of_normals_= " << num_of_normals_ << std::endl;
    std::vector<Eigen::Vector3d> q;
    // n-2 because the three last cpoints have the same normals
    std::vector<Eigen::Vector3d> n(std::max(num_of_normals_ - 2, 0),
                                   Eigen::Vector3d::Zero());  // Initialize all elements

    std::vector<double> d(std::max(num_of_normals_ - 2, 0),
                          0.0);  // Initialize all elements

    q.push_back(q0_);
    q.push_back(q1_);
    q.push_back(q2_);

    std::cout << "q0_=" << q0_.transpose() << std::endl;
    std::cout << "q1_=" << q1_.transpose() << std::endl;
    std::cout << "q2_=" << q2_.transpose() << std::endl;

    // sample next cp in a sphere (or spherical surface?) near q2_ (limited by v_max)
    for (int i = 3; i <= (N_ - 2); i++)  // all the intermediate control points, and cp N_-2 of the trajectory
    {
      std::cout << "i= " << i << std::endl;
      Eigen::Vector3d tmp;

      //      Eigen::Vector3d mean = q2_ + (qNm2 - q2_) * (i - 2) / (1.0 * num_of_intermediate_cps);
      //      Eigen::Vector3d max_value = mean + high_value;
      //      std::default_random_engine generator;
      //      generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
      //      std::normal_distribution<double> distribution_x(mean(0), 1.0);
      //      std::normal_distribution<double> distribution_y(mean(1), 1.0);
      //      std::normal_distribution<double> distribution_z(mean(2), 1.0);

    initloop:
      if (guess_timer.ElapsedMs() > max_runtime_ * 1000)
      {
        return;
      }

      // take sample
      std::vector<Eigen::Vector3d> last3Cps;
      last3Cps.push_back(q[q.size() - 3]);
      last3Cps.push_back(q[q.size() - 2]);
      last3Cps.push_back(q[q.size() - 1]);
      sampleFeasible(tmp, last3Cps);

      // tmp = q.back();  // hack;
      // tmp << distribution_x(generator), distribution_y(generator), distribution_z(generator);
      // saturate(tmp, -max_value, max_value);

      std::vector<Eigen::Vector3d> last4Cps;
      last4Cps.push_back(q[q.size() - 3]);
      last4Cps.push_back(q[q.size() - 2]);
      last4Cps.push_back(q[q.size() - 1]);
      last4Cps.push_back(tmp);

      // std::copy(q.end() - 3, q.end(), last4Cps.begin());  // copy three elements
      last4Cps[3] = tmp;

      if ((satisfiesVmaxAmax(last4Cps) == false))
      {
        std::cout << "vmax and amax are not satisfied" << std::endl;
        std::cout << "knots_=" << knots_ << std::endl;

        for (auto x : last4Cps)
        {
          std::cout << x.transpose() << std::endl;
        }

        goto initloop;
      }

      // check that it doesn't collide with the  obstacles at t=i-3
      for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
      {
        Eigen::Vector3d n_i;
        double d_i;
        if (separator_solver->solveModel(n_i, d_i, hulls_[obst_index][i - 3], last4Cps) == false)
        {
          std::cout << "Didn't work, i=" << i << std::endl;
          std::cout << "Obstacle: " << std::endl;
          printStd(hulls_[obst_index][i - 3]);
          std::cout << "Trajectory" << std::endl;
          std::cout << last4Cps[0].transpose() << std::endl;
          std::cout << last4Cps[1].transpose() << std::endl;
          std::cout << last4Cps[2].transpose() << std::endl;
          std::cout << last4Cps[3].transpose() << std::endl;

          goto initloop;
        }  // n_i will point to the points in the obstacle (hulls_)

        std::cout << "Index of n filled= " << obst_index * num_of_segments_ << std::endl;
        n[obst_index * num_of_segments_ + i - 3] = n_i;  // will be overwritten until the solution is found
        d[obst_index * num_of_segments_ + i - 3] = d_i;  // will be overwritten until the solution is found
      }

      std::cout << "Found intermediatee cp " << i << "= " << tmp.transpose() << ", N_-3=" << (N_ - 3) << std::endl;
      q.push_back(tmp);
    }

    std::cout << "Filling last two elements of n" << std::endl;
    std::cout << "n.size() - 3=" << n.size() - 3 << std::endl;

    if (n.size() > 0)
    {  // only do this if there are obstacles
      n.push_back(n.back());
      n.push_back(n.back());

      d.push_back(d.back());
      d.push_back(d.back());
    }

    std::cout << "Filled" << std::endl;

    // sample last cp in a sphere near qNm2_
    q.push_back(q.back());
    q.push_back(q.back());
    // q.push_back(qNm2);

    std::vector<Eigen::Vector3d> n_novale;
    std::vector<double> d_novale;

    double cost = computeObjFuction(num_of_variables_, NULL, q, n_novale, d_novale);
    if (cost < best_cost)
    {
      best_cost = cost;
      q_guess_ = q;
      n_guess_ = n;
      d_guess_ = d;
    }
  }

  fillPlanesFromNDQ(planes_, n_guess_, d_guess_, q_guess_);

  //  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  //  {
  //    for (int i = 0; i < num_of_segments_; i++)
  //    {
  //      Eigen::Vector3d centroid_hull;
  //      findCentroidHull(hulls_[obst_index][i], centroid_hull);
  //
  //      Eigen::Vector3d n_i =
  //          (centroid_hull - q[i]).normalized();  // n_i should point towards the obstacle (i.e. towards the hull)
  //
  //      double alpha = 0.01;  // the smaller, the higher the chances the plane is outside the obstacle. Should be <1
  //
  //      Eigen::Vector3d point_in_middle = q[i] + (centroid_hull - q[i]) * alpha;
  //
  //       double d_i = -n_i.dot(point_in_middle);  // n'x + d = 0
  //
  //       int sign_d_i = (d_i >= 0) ? 1 : -1;
  //
  //       signs_.push_back(sign_d_i);
  //      n.push_back(n_i / d_i);  // n'x + 1 = 0*/
//
//  for (int i = 0; i < n_guess_.size(); i++)
//  {
//     double x = q[i]
//
//         Eigen::Vector3d point_in_plane =
//
//             Hyperplane3D plane(point_in_plane, n_i / d_i);
//     planes_.push_back(plane);
//  }
//
// generateGuessNFromQ(q_best, n);
//  generateRandomN(n_guess_);
//}
//* /

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

/*    double epsilon = 0.01;
    Eigen::Vector3d eps_vector(epsilon, epsilon, epsilon);
#ifdef DEBUG_MODE_NLOPT
    std::cout << "Going to add vf constraints, r= " << r << std::endl;
#endif
    // For vf
    assignEigenToVector(constraints, r, vf - final_state_.vel - eps_vector);  // f<=0
    double tmp5 = p_ / (-tN + tNPp);
    double tmp6 = p_ / (tN - tNPp);

    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(N_), ones * tmp5, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(N_ - 1), ones * tmp6, grad, r, nn);
    }
    r = r + 3;

    assignEigenToVector(constraints, r, final_state_.vel - vf - eps_vector);  // f<=0
    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(N_), -ones * tmp5, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(N_ - 1), -ones * tmp6, grad, r, nn);
    }
    r = r + 3;
#ifdef DEBUG_MODE_NLOPT
    std::cout << "Going to add af constraints, r= " << r << std::endl;
#endif
    // For af
    assignEigenToVector(constraints, r, af - final_state_.accel - eps_vector);  // f<=0

    double tmp1 = (((-1 + p_) * p_) / ((tN - tNm1Pp) * (tN - tNPp)));
    double tmp2 = ((-1 + p_) * p_ * (1 / (tNm1 - tNm1Pp) + 1 / (tN - tNPp))) / (-tN + tNm1Pp);
    double tmp3 = ((-1 + p_) * p_) / ((-tN + tNm1Pp) * (-tNm1 + tNm1Pp));

    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(N_), ones * tmp1, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(N_ - 1), ones * tmp2, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(N_ - 2), ones * tmp3, grad, r, nn);
    }
    r = r + 3;

    assignEigenToVector(constraints, r, final_state_.accel - af - eps_vector);  // f<=0
    if (grad)
    {
      toGradDiffConstraintsDiffVariables(gIndexQ(N_), -ones * tmp1, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(N_ - 1), -ones * tmp2, grad, r, nn);
      toGradDiffConstraintsDiffVariables(gIndexQ(N_ - 2), -ones * tmp3, grad, r, nn);
    }
    r = r + 3;*/

// std::cout << "here2" << std::endl;

// Now add the "equality" constraints (for final velocity and acceleration) using inequalities with an epsilon:

/*  if (force_final_state_ == false)
  {
#ifdef DEBUG_MODE_NLOPT
    if ((N_ + p_) >= knots_.size())
    {
      std::cout << "There is something wrong here, (N_+p)=" << (N_ + p_) << std::endl;
      std::cout << "but knots_.size()=" << knots_.size() << std::endl;
    }
#endif
    double tN = knots_(N_);
    double tNm1 = knots_(N_ - 1);
    double tNPp = knots_(N_ + p_);
    double tNm1Pp = knots_(N_ - 1 + p_);

    // See Mathematica Notebook

    Eigen::Vector3d qNm2 = q[N_ - 2];
    Eigen::Vector3d qNm1 = q[N_ - 1];
    Eigen::Vector3d qN = q[N_];

    Eigen::Vector3d vf = p_ * (qN - qNm1) / (tNPp - tN);
    Eigen::Vector3d vNm2 = (p_ * (qNm1 - qNm2)) / (tNm1Pp - tNm1);
    Eigen::Vector3d af = ((p_ - 1) * (vf - vNm2)) / (tNm1Pp - tN);
  }*/