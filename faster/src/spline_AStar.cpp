#include "spline_AStar.hpp"
#include "bspline_utils.hpp"

#include <vector>

#include "timer.hpp"
#include "termcolor.hpp"
using namespace termcolor;

#define WITHOUT_NUMPY  // for matplotlibcpp   TODO(move from here!)
#include "matplotlibcpp.h"

#include <boost/bind.hpp>

#include <random>

#include "ros/ros.h"  //Just for debugging, to be able to use ROS_INFO...

typedef JPS::Timer MyTimer;

namespace plt = matplotlibcpp;

SplineAStar::SplineAStar(int num_pol, int deg_pol, int num_obst, double t_min, double t_max,
                         const ConvexHullsOfCurves_Std& hulls)
{
  p_ = deg_pol;
  M_ = num_pol + 2 * p_;
  N_ = M_ - p_ - 1;
  num_of_obst_ = num_obst;
  num_of_segments_ = (M_ - 2 * p_);
  num_of_normals_ = num_of_segments_ * num_of_obst_;
  num_pol_ = num_pol;

  hulls_ = hulls;

  double deltaT = (t_max - t_min) / (1.0 * (M_ - 2 * p_ - 1 + 1));

  Eigen::RowVectorXd knots(M_ + 1);
  for (int i = 0; i <= p_; i++)
  {
    knots[i] = t_min;
  }

  for (int i = (p_ + 1); i <= M_ - p_ - 1; i++)
  {
    knots[i] = knots[i - 1] + deltaT;  // Assumming a uniform b-spline (internal knots are equally spaced)
  }

  for (int i = (M_ - p_); i <= M_; i++)
  {
    knots[i] = t_max;
  }

  knots_ = knots;
  // std::cout << "knots_=" << knots_ << std::endl;

  separator_solver_ = new separator::Separator();  // 0.0, 0.0, 0.0

  // Mbs2mv_ << 182, 685, 100, -7,  //////////////////
  //     56, 640, 280, -16,         //////////////////
  //     -16, 280, 640, 56,         //////////////////
  //     -7, 100, 685, 182;
  // Mbs2mv_ = (1.0 / 960.0) * Mbs2mv_;

  // see matlab.
  // This is for the interval [0 1];
  Mbs2mv_ << 0.18372, 0.057009, -0.01545, -0.005338,  ///////////
      0.70176, 0.6665738, 0.29187, 0.119851669,       ////////////
      0.119851669, 0.2918718, 0.66657, 0.7017652,     //////////////////
      -0.00533879, -0.015455, 0.0570095, 0.18372189;  //////////////////

  Mbs2be_ << 1, 0, 0, 0,  //////////
      4, 4, 2, 1,         //////////
      1, 2, 4, 4,         //////////
      0, 0, 0, 1;         //////////

  Mbs2be_ = (1 / 6.0) * Mbs2be_;

  // std::default_random_engine eng{ static_cast<long unsigned int>(time(0)) };
  // double delta = 0.0;
  // std::uniform_real_distribution<> disx(1 - delta, 1 + delta);
  // std::uniform_real_distribution<> disy(1 - delta, 1 + delta);
  // std::uniform_real_distribution<> disz(1 - delta, 1 + delta);
  // epsilons_ << disx(eng), disy(eng), disz(eng);

  // std::cout << bold << red << "EPSILONS= " << epsilons_.transpose() << reset << std::endl;

  // computeInverses();
}

SplineAStar::~SplineAStar()
{
}

int SplineAStar::getNumOfLPsRun()
{
  return num_of_LPs_run_;
}

void SplineAStar::setVisual(bool visual)
{
  visual_ = visual;
}

void SplineAStar::getBestTrajFound(trajectory& best_traj_found)
{
  std::cout << "******************BEST_TRAJ_FOUND**************" << std::endl;
  trajectory traj;
  PieceWisePol pwp;
  CPs2TrajAndPwp(result_, best_traj_found, pwp, N_, p_, num_pol_, knots_, 0.1);  // Last number is the resolution
}

void SplineAStar::getEdgesConvexHulls(faster_types::Edges& edges_convex_hulls)
{
  for (int i = 3; i < result_.size(); i++)
  {
    std::vector<Eigen::Vector3d> last4Cps(4);
    last4Cps[0] = result_[i - 3];
    last4Cps[1] = result_[i - 2];
    last4Cps[2] = result_[i - 1];
    last4Cps[3] = result_[i];
    // std::cout << "[BSpline] Plotting these last4Cps" << std::endl;
    // std::cout << last4Cps[0].transpose() << std::endl;
    // std::cout << last4Cps[1].transpose() << std::endl;
    // std::cout << last4Cps[2].transpose() << std::endl;
    // std::cout << last4Cps[3].transpose() << std::endl;
    if (basis_ == MINVO || basis_ == BEZIER)  // Plot the control points using the MINVO basis
    {
      transformBSpline2otherBasis(last4Cps);
    }
    // std::cout << last4Cps[0].transpose() << std::endl;
    // std::cout << last4Cps[1].transpose() << std::endl;
    // std::cout << last4Cps[2].transpose() << std::endl;
    // std::cout << last4Cps[3].transpose() << std::endl;
    for (int j = 0; j < 4; j++)
    {  // For every point in the convex hull
      faster_types::Edge edge;
      edge.first = last4Cps[j];
      for (int i = 0; i < 4; i++)
      {  // generate an edge from that point j to the other points i!=j
        if (i == j)
        {
          continue;
        }
        else
        {
          edge.second = last4Cps[i];
          edges_convex_hulls.push_back(edge);
        }
      }
    }
  }
}

void SplineAStar::getAllTrajsFound(std::vector<trajectory>& all_trajs_found)
{
  all_trajs_found.clear();

  for (auto node : expanded_nodes_)
  {
    // std::cout << "using expanded_node= " << node.qi.transpose() << std::endl;

    std::vector<Eigen::Vector3d> cps;

    Node* tmp = &node;

    while (tmp != NULL)
    {
      cps.push_back(Eigen::Vector3d(tmp->qi.x(), tmp->qi.y(), tmp->qi.z()));
      tmp = tmp->previous;
    }

    cps.push_back(q1_);
    cps.push_back(q0_);  // cps = [....q4 q3 q2 q1 q0}

    std::reverse(std::begin(cps), std::end(cps));  // cps=[q0 q1 q2 q3 q4 ...]

    trajectory traj;
    PieceWisePol pwp;
    CPs2TrajAndPwp(cps, traj, pwp, N_, p_, num_pol_, knots_, 0.01);  // Last number is the resolution

    all_trajs_found.push_back(traj);
  }
}

void SplineAStar::setBasisUsedForCollision(int basis)
{
  if (basis == MINVO)
  {
    Mbs2basis_ = Mbs2mv_;
  }
  else if (basis == BEZIER)
  {
    Mbs2basis_ = Mbs2be_;
  }
  else if (basis == B_SPLINE)
  {
    Mbs2basis_ = Eigen::Matrix<double, 4, 4>::Identity();
  }
  else
  {
    std::cout << red << "Basis not implemented yet, using the one for B-Spline" << std::endl;
    Mbs2basis_ = Eigen::Matrix<double, 4, 4>::Identity();
  }

  Mbs2basis_inverse_ = Mbs2basis_.inverse();

  basis_ = basis;
}

void SplineAStar::setBBoxSearch(double x, double y, double z)
{
  bbox_x_ = x;
  bbox_y_ = y;
  bbox_z_ = z;
}

void SplineAStar::setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int num_samples_x,
                                         int num_samples_y, int num_samples_z, double fraction_voxel_size)
{
  v_max_ = v_max;
  a_max_ = a_max;

  // ensure they are odd numbers (so that vx=0 is included in the samples)
  num_samples_x_ = (num_samples_x % 2 == 0) ? ceil(num_samples_x) : num_samples_x;
  num_samples_y_ = (num_samples_y % 2 == 0) ? ceil(num_samples_y) : num_samples_y;
  num_samples_z_ = (num_samples_z % 2 == 0) ? ceil(num_samples_z) : num_samples_z;

  ////////////

  for (int i = 0; i < num_samples_x; i++)
  {
    indexes_samples_x_.push_back(i);
  }

  for (int i = 0; i < num_samples_y; i++)
  {
    indexes_samples_y_.push_back(i);
  }

  for (int i = 0; i < num_samples_z; i++)
  {
    indexes_samples_z_.push_back(i);
  }

  // std::srand(unsigned(std::time(0)));
  /*  std::srand(unsigned(std::time(0)));

    if (rand() % 2 == 1)  // o or 1
    {
      std::reverse(indexes_samples_x_.begin(), indexes_samples_x_.end());
    }

    if (rand() % 2 == 1)
    {
      std::reverse(indexes_samples_y_.begin(), indexes_samples_y_.end());
    }

    if (rand() % 2 == 1)
    {
      std::reverse(indexes_samples_z_.begin(), indexes_samples_z_.end());
    }*/

  for (int jx : indexes_samples_x_)
  {
    for (int jy : indexes_samples_y_)
    {
      for (int jz : indexes_samples_z_)
      {
        // std::cout << "Pushing combination " << jx << ", " << jy << ", " << jz << std::endl;
        all_combinations_.push_back(std::tuple<int, int, int>(jx, jy, jz));
      }
    }
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  shuffle(all_combinations_.begin(), all_combinations_.end(), std::default_random_engine(seed));

  //  std::random_shuffle(all_combinations_.begin(), all_combinations_.end());

  /*  std::random_shuffle(indexes_samples_x_.begin(), indexes_samples_x_.end());
    std::random_shuffle(indexes_samples_y_.begin(), indexes_samples_y_.end());
    std::random_shuffle(indexes_samples_z_.begin(), indexes_samples_z_.end());*/
  /*
    std::cout << "indexes_samples_x_" << std::endl;
    for (auto index : indexes_samples_x_)
    {
      std::cout << index << ", " << std::endl;
    }
    std::cout << "indexes_samples_y_" << std::endl;

    for (auto index : indexes_samples_y_)
    {
      std::cout << index << ", " << std::endl;
    }

    std::cout << "indexes_samples_z_" << std::endl;
    for (auto index : indexes_samples_z_)
    {
      std::cout << index << ", " << std::endl;
    }
  */
  // TODO: remove hand-coded stuff
  // int i = 6;
  // voxel_size_ = v_max_(0) * (knots_(i + p_ + 1) - knots_(i + 1)) / (1.0 * p_);
  // voxel_size_ = fabs(vx_[1] - vx_[0]) * (knots_(i + p_ + 1) - knots_(i + 1)) / (1.0 * p_);

  double min_voxel_size;
  double max_voxel_size;
  computeLimitsVoxelSize(min_voxel_size, max_voxel_size);

  // voxel_size_ = std::max(voxel_size_, min_voxel_size);
  // voxel_size_ = std::min(voxel_size, max_voxel_size);

  // Ensure  fraction_voxel_size is in [0,1]
  fraction_voxel_size = (fraction_voxel_size > 1) ? 1 : fraction_voxel_size;
  fraction_voxel_size = (fraction_voxel_size < 0) ? 0 : fraction_voxel_size;

  voxel_size_ = min_voxel_size + fraction_voxel_size * (max_voxel_size - min_voxel_size);

  std::cout << green << "[A*] voxel_size= " << voxel_size_ << ", limits are (" << min_voxel_size << ", "
            << max_voxel_size << ")" << reset << std::endl;

  // Make sure voxel_size_<= (min_voxel_size + max_voxel_size) / 2.0  (if not, very few nodes are expanded)
  // voxel_size_ = (min_voxel_size + max_voxel_size) / 2.0;

  // std::cout << "Using voxel_size_= " << voxel_size_ << std::endl;

  // note that (neighbor.qi - current.qi) is guaranteed to be an integer multiple of voxel_size_

  /*  int length_x = bbox_x_ / voxel_size_;
    int length_y = bbox_y_ / voxel_size_;
    int length_z = bbox_z_ / voxel_size_;*/

  /*  std::cout << "Allocating vector" << std::endl;
    std::vector<std::vector<std::vector<bool>>> novale(
        length_x, std::vector<std::vector<bool>>(length_y, std::vector<bool>(length_z, false)));
    std::cout << "Vector allocated" << std::endl;*/

  orig_ = q2_ - Eigen::Vector3d(bbox_x_ / 2.0, bbox_y_ / 2.0, bbox_z_ / 2.0);

  // matrixExpandedNodes_ = novale;
}

void SplineAStar::setZminZmax(double z_min, double z_max)
{
  z_min_ = z_min;
  z_max_ = z_max;
}

void SplineAStar::setBias(double bias)
{
  bias_ = bias;
}

void SplineAStar::setGoal(Eigen::Vector3d& goal)
{
  goal_ = goal;
}

void SplineAStar::setRunTime(double max_runtime)
{
  max_runtime_ = max_runtime;
}
void SplineAStar::setGoalSize(double goal_size)
{
  goal_size_ = goal_size;
}

// returns the minimum voxel_size needed (if bigger, all the neighbous from q2_ are inside the voxel of q2_)
// min_voxel_size: if voxel_size < min_voxel_size, all the neighbous from q2_ will be expanded correctly
// max_voxel_size: if voxel_size > max_voxel_size, there will be no nodes expanded from q2_ (they are on the same cell
// as q2_)

void SplineAStar::computeLimitsVoxelSize(double& min_voxel_size, double& max_voxel_size)
{
  int i = 2;
  double constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU;

  // std::cout << "Computing upper and lower, qiM1=" << q1_.transpose() << ", and qi=" << q2_.transpose() << std::endl;

  computeUpperAndLowerConstraints(i, q1_, q2_, constraint_xL, constraint_xU, constraint_yL, constraint_yU,
                                  constraint_zL, constraint_zU);

  /*  std::cout << "constraint_xL= " << constraint_xL << std::endl;
    std::cout << "constraint_xU= " << constraint_xU << std::endl;

    std::cout << "constraint_yL= " << constraint_yL << std::endl;
    std::cout << "constraint_yU= " << constraint_yU << std::endl;

    std::cout << "constraint_zL= " << constraint_zL << std::endl;
    std::cout << "constraint_zU= " << constraint_zU << std::endl;*/

  min_voxel_size = std::numeric_limits<double>::max();
  max_voxel_size = std::numeric_limits<double>::min();

  Eigen::Vector3d neighbor_of_q2;

  for (int jx : indexes_samples_x_)
  {
    for (int jy : indexes_samples_y_)
    {
      for (int jz : indexes_samples_z_)
      {
        // sample a velocity
        Eigen::Vector3d vi;
        vi << constraint_xL + jx * ((constraint_xU - constraint_xL) / (num_samples_x_ - 1)),  /////////
            constraint_yL + jy * ((constraint_yU - constraint_yL) / (num_samples_y_ - 1)),    /////////
            constraint_zL + jz * ((constraint_zU - constraint_zL) / (num_samples_z_ - 1));    /////////

        if (vi.norm() < 0.000001)  // remove the cases where vi is very small
        {
          continue;
        }

        //  std::cout << "vx= " << vi.x() << ", vy= " << vi.y() << ", vz= " << vi.z() << std::endl;
        //  std::cout << "(neighbor_of_q2 - q2_).norm()= " << (neighbor_of_q2 - q2_).norm() << std::endl;

        neighbor_of_q2 = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + q2_;

        min_voxel_size = std::min(min_voxel_size, (neighbor_of_q2 - q2_).norm());
        max_voxel_size = std::max(max_voxel_size, (neighbor_of_q2 - q2_).norm());
      }
    }
  }

  /*  Eigen::Vector3d vi;

    vi << constraint_xU, constraint_yU, constraint_zU;
    Eigen::Vector3d neighbor_of_q2_U = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + q2_;

    vi << constraint_xL, constraint_yL, constraint_zL;
    Eigen::Vector3d neighbor_of_q2_L = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + q2_;

    max_voxel_size = std::max((neighbor_of_q2_L - q2_).norm(), (neighbor_of_q2_U - q2_).norm());*/

  /*  return 1.001 * std::max((neighbor_of_q2_L - q2_).norm(), (neighbor_of_q2_U - q2_).norm());

    return min_voxel_size;*/
}

// Compute the lower and upper bounds on the velocity based on the velocity and acceleration constraints
// return false if any of the intervals, the lower bound is > the upper bound
bool SplineAStar::computeUpperAndLowerConstraints(const int i, const Eigen::Vector3d& qiM1, const Eigen::Vector3d& qi,
                                                  double& constraint_xL, double& constraint_xU, double& constraint_yL,
                                                  double& constraint_yU, double& constraint_zL, double& constraint_zU)
{
  Eigen::Vector3d viM1 = p_ * (qi - qiM1) / (knots_(i + p_) - knots_(i));  // velocity_{current.index -1}

  double d = (knots_(i + p_) - knots_(i + 1)) / (1.0 * (p_ - 1));

  // |vi - viM1|<=a_max_*d
  //  <=>
  // (vi - viM1)<=a_max_*d   AND   (vi - viM1)>=-a_max_*d
  //  <=>
  //  vi<=a_max_*d + viM1   AND     vi>=-a_max_*d + viM1

  Eigen::Vector3d max_vel = a_max_ * d + viM1;   // this is to ensure that aiM1 is inside the bounds
  Eigen::Vector3d min_vel = -a_max_ * d + viM1;  // this is to ensure that aiM1 is inside the bounds

  constraint_xL = std::max(-v_max_.x(), min_vel.x());  // lower bound
  constraint_xU = std::min(v_max_.x(), max_vel.x());   // upper bound

  constraint_yL = std::max(-v_max_.y(), min_vel.y());  // lower bound
  constraint_yU = std::min(v_max_.y(), max_vel.y());   // upper bound

  constraint_zL = std::max(-v_max_.z(), min_vel.z());  // lower bound
  constraint_zU = std::min(v_max_.z(), max_vel.z());   // upper bound

  // std::cout << "i " << i << std::endl;
  // std::cout << "Checking for acceleration " << i - 1 << std::endl;

  // Now, if i==(N_-3), I need to impose also the constraint aNm3 \in [-amax,amax]
  if (i == (N_ - 3))
  {
    Eigen::Vector3d vNm2(0.0, 0.0, 0.0);  // Due to the stop condition

    double c = (knots_(N_ - 3 + p_ + 1) - knots_(N_ - 3 + 2)) / (p_ - 1);

    Eigen::Vector3d vNm3_max = vNm2 + c * a_max_;
    Eigen::Vector3d vNm3_min = vNm2 - c * a_max_;

    // std::cout << "Before: " << constraint_xL << " --> " << constraint_xU << std::endl;

    // std::cout << "vNm3_min.x()=: " << vNm3_min.x() << std::endl;
    // std::cout << "constraint_xL=: " << constraint_xL << std::endl;

    constraint_xL = std::max(vNm3_min.x(), constraint_xL);  // lower bound
    constraint_xU = std::min(vNm3_max.x(), constraint_xU);  // upper bound

    constraint_yL = std::max(vNm3_min.y(), constraint_yL);  // lower bound
    constraint_yU = std::min(vNm3_max.y(), constraint_yU);  // upper bound

    constraint_zL = std::max(vNm3_min.z(), constraint_zL);  // lower bound
    constraint_zU = std::min(vNm3_max.z(), constraint_zU);  // upper bound

    // std::cout << "vNm3=v4 in" << constraint_xL << " --> " << constraint_xU << std::endl;
  }

  // if (constraint_xL > constraint_xU)
  // {
  //   std::cout << red << "there is sth wrong, constraint_xL > constraint_xU" << reset << std::endl;
  //   std::cout << "constraint_xL= " << constraint_xL << std::endl;
  //   std::cout << "constraint_xU= " << constraint_xU << std::endl;

  //   abort();
  // }

  if (constraint_xL > constraint_xU || constraint_yL > constraint_yU || constraint_zL > constraint_zU)
  {  // can happen when i==(N_-3), but never happens when i<(N_-3)
    return false;
  }
  else
  {
    return true;  // constraint_xL<=constraint_xU (same with other axes)
  }

  // if (neighbor.index == (N_ - 2))
  // {  // Check that aNm2 is satisfied

  //   Eigen::Vector3d vNm1(0.0, 0.0, 0.0);  // Due to the stop condition
  //   Eigen::Vector3d vNm2 = vi;
  //   Eigen::Vector3d aNm2 = (p_ - 1) * (vNm1 - vNm2) / (knots_(N_ - 2 + p_ + 1) - knots_(N_ - 2 + 2));

  //   if ((aNm2.array() > a_max_.array()).any() || (aNm2.array() < -a_max_.array()).any())
  //   {
  //     continue;
  //   }
  // }

  /*  std::cout << "constraint_xL= " << constraint_xL << std::endl;
    std::cout << "constraint_xU= " << constraint_xU << std::endl;

    std::cout << "constraint_yL= " << constraint_yL << std::endl;
    std::cout << "constraint_yU= " << constraint_yU << std::endl;

    std::cout << "constraint_zL= " << constraint_zL << std::endl;
    std::cout << "constraint_zU= " << constraint_zU << std::endl;*/
}

void SplineAStar::computeInverses()
{
  Ainverses_.clear();
  Ainverses_.push_back(Eigen::MatrixXd::Zero(1, 1));  // dimension 0
  Ainverses_.push_back(Eigen::MatrixXd::Zero(1, 1));  // dimension 1
  Ainverses_.push_back(Eigen::MatrixXd::Zero(2, 2));  // dimension 2
  for (int dim = 3; dim < N_; dim++)
  {
    Eigen::MatrixXd A = 12 * Eigen::MatrixXd::Identity(dim, dim);
    Eigen::Matrix<double, -1, 1> tmp1 = -8 * Eigen::MatrixXd::Ones(dim - 1, 1);
    Eigen::Matrix<double, -1, 1> tmp2 = 2 * Eigen::MatrixXd::Ones(dim - 2, 1);
    A.diagonal(1) = tmp1;
    A.diagonal(-1) = tmp1;
    A.diagonal(2) = tmp2;
    A.diagonal(-2) = tmp2;

    Ainverses_.push_back(A.inverse());
  }
}

void SplineAStar::setq0q1q2(Eigen::Vector3d& q0, Eigen::Vector3d& q1, Eigen::Vector3d& q2)
{
  q0_ = q0;
  q1_ = q1;
  q2_ = q2;
}

double SplineAStar::h(Node& node)
{
  // See Notebook mathematica
  /*  int dim_var = N_ - node.index;

    Eigen::Matrix<double, -1, 1> b = Eigen::MatrixXd::Zero(dim_var, 1);

    double heuristics = 0;

    for (int coord = 0; coord < 3; coord++)  // separation in the three coordinates
    {
      double qi = node.qi(coord);
      double qiM1;

      if (node.index == 2)
      {
        qiM1 = q1_(coord);
      }
      else
      {
        qiM1 = node.previous->qi(coord);
      }

      double goal = goal_(coord);

      b(0, 0) = 2 * (qiM1 - 4 * qi);

      // See mathematica notebook
      if (dim_var == 3)
      {
        b(1, 0) = 2 * qi + 2 * goal;
      }
      else
      {
        b(1, 0) = 2 * qi;
        b(b.size() - 2, 0) = 2 * goal;
      }
      b(b.size() - 1, 0) = -6 * goal;
      // std::cout << "Going to compute coordinates now3" << std::endl;
      double c = qiM1 * qiM1 - 4 * qiM1 * qi + 5 * qi * qi + 2 * goal * goal;

      heuristics = heuristics + (-0.5 * b.transpose() * Ainverses_[dim_var] * b + c);
    }*/

  double heuristics = (node.qi - goal_).norm();  // hack
  // Eigen::Vector3d deltas2 = (node.qi - goal_).array().square();  //[Deltax^2  Deltay^2  Deltaz^2]''
  // double heuristics = sqrt(epsilons_.dot(deltas2));              // hack
  return heuristics;
}

double SplineAStar::g(Node& node)
{
  double cost = 0;
  Node* tmp = &node;
  while (tmp->previous != NULL)
  {
    cost = cost + weightEdge(*tmp->previous, *tmp);
    tmp = tmp->previous;
  }

  return cost;
}

double SplineAStar::weightEdge(Node& node1, Node& node2)  // edge cost when adding node 2
{
  /*  if (node1.index == 2)
    {
      return (node2.qi - 2 * node1.qi + q1_).squaredNorm();
    }
  // return (node2.qi - 2 * node1.qi + node1.previous->qi).squaredNorm();
    */

  double weight_edge = (node2.qi - node1.qi).norm();
  // Eigen::Vector3d deltas2 = (node2.qi - node1.qi).array().square();  //[Deltax^2  Deltay^2  Deltaz^2]''
  // double weight_edge = sqrt(epsilons_.dot(deltas2));                 // hack

  return weight_edge;
}

void SplineAStar::printPath(Node& node1)
{
  Node tmp = node1;
  while (tmp.previous != NULL)
  {
    // std::cout << tmp.index << ", ";  // qi.transpose().x()
    std::cout << tmp.previous->qi.transpose() << std::endl;
    tmp = *tmp.previous;
  }
  std::cout << std::endl;
}

void SplineAStar::recoverPath(Node* result_ptr)
{
  // std::cout << "Recovering path" << std::endl;
  result_.clear();

  if (result_ptr == NULL)
  {
    result_.push_back(q0_);
    result_.push_back(q1_);
    return;
  }

  Node* tmp = result_ptr;

  // std::cout << "Pushing qN= " << tmp->qi.transpose() << std::endl;
  // std::cout << "Pushing qN-1= " << tmp->qi.transpose() << std::endl;

  // std::cout << "qi is" << std::endl;
  std::cout << tmp->qi.transpose() << std::endl;

  //  std::cout << "qi is" << tmp->qi.transpose() << std::endl;
  result_.push_back(tmp->qi);  // qN
  result_.push_back(tmp->qi);  // qN-1

  while (tmp != NULL)
  {
    result_.push_back(tmp->qi);
    // std::cout << "pushing" << tmp->qi.transpose() << std::endl;

    tmp = tmp->previous;
  }
  // std::cout << "going to push q0 and q1" << std::endl;
  result_.push_back(q1_);
  result_.push_back(q0_);

  // std::cout << "reverse" << std::endl;

  std::reverse(std::begin(result_), std::end(result_));  // result_ is [q0 q1 q2 q3 ...]
}

void SplineAStar::plotExpandedNodesAndResult(std::vector<Node>& expanded_nodes, Node* result_ptr)
{
  for (auto node : expanded_nodes)
  {
    // std::cout << "using expanded_node= " << node.qi.transpose() << std::endl;

    Node* tmp = &node;

    std::vector<double> x, y, z;
    while (tmp != NULL)
    {
      x.push_back(tmp->qi.x());
      y.push_back(tmp->qi.y());
      z.push_back(tmp->qi.z());

      tmp = tmp->previous;
    }
    plt::plot(x, y, "ob-");
  }

  std::vector<std::string> colors = { "ok", "og", "oc", "om", "oy", "ok", "og", "or" };
  int counter_color = 0;
  if (result_ptr != NULL)
  {
    std::cout << "calling recoverPath1" << std::endl;
    recoverPath(result_ptr);  // saved in result_
    std::cout << "called recoverPath1" << std::endl;

    std::vector<double> x_result, y_result, z_result;

    for (auto q_i : result_)
    {
      x_result.push_back(q_i.x());
      y_result.push_back(q_i.y());
      z_result.push_back(q_i.z());
    }

    plt::plot(x_result, y_result, "or-");

    std::cout << "Path is:" << std::endl;
    for (auto q_i : result_)
    {
      std::cout << q_i.transpose() << std::endl;
    }

    if (basis_ == MINVO || basis_ == BEZIER)  // Plot the control points using the MINVO basis
    {
      for (int i = 3; i < result_.size(); i++)
      {
        std::vector<Eigen::Vector3d> last4Cps(4);
        last4Cps[0] = result_[i - 3];
        last4Cps[1] = result_[i - 2];
        last4Cps[2] = result_[i - 1];
        last4Cps[3] = result_[i];
        std::cout << "[BSpline] Plotting these last4Cps" << std::endl;
        std::cout << last4Cps[0].transpose() << std::endl;
        std::cout << last4Cps[1].transpose() << std::endl;
        std::cout << last4Cps[2].transpose() << std::endl;
        std::cout << last4Cps[3].transpose() << std::endl;

        std::vector<double> x_result_ov, y_result_ov, z_result_ov;

        transformBSpline2otherBasis(last4Cps);

        std::cout << "[MINVO]  with color=" << colors[counter_color] << std::endl;
        std::cout << last4Cps[0].transpose() << std::endl;
        std::cout << last4Cps[1].transpose() << std::endl;
        std::cout << last4Cps[2].transpose() << std::endl;
        std::cout << last4Cps[3].transpose() << std::endl;
        for (int j = 0; j < 4; j++)
        {
          x_result_ov.push_back(last4Cps[j].x());
          y_result_ov.push_back(last4Cps[j].y());
          z_result_ov.push_back(last4Cps[j].z());
        }
        plt::plot(x_result_ov, y_result_ov, colors[counter_color]);
        counter_color = counter_color + 1;
      }
    }
  }

  plt::show();
}

/*bool SplineAStar::isInExpandedList(Node& tmp)
{

}*/

void SplineAStar::transformBSpline2otherBasis(std::vector<Eigen::Vector3d>& last4Cps)
{
  /////////////////////
  Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
  Eigen::Matrix<double, 3, 4> Qmv;  // minvo
  Qbs.col(0) = last4Cps[0];
  Qbs.col(1) = last4Cps[1];
  Qbs.col(2) = last4Cps[2];
  Qbs.col(3) = last4Cps[3];

  /*  Eigen::Matrix<double, 4, 4> tmp;
    tmp.block(0, 0, 4, 3) = Qbs;
    tmp.block(0, 3, 4, 1) = Eigen::Matrix<double, 4, 1>::Ones();
    std::cout << "tmp BS= " << tmp << std::endl;
    std::cout << "Determinant BS=" << tmp.determinant() << std::endl;*/

  Qmv = Qbs * Mbs2basis_;

  /*  tmp.block(0, 0, 4, 3) = Qmv;
    std::cout << "tmp OV= " << tmp << std::endl;
    std::cout << "Determinant OV=" << tmp.determinant() << std::endl;*/

  last4Cps[0] = Qmv.col(0);
  last4Cps[1] = Qmv.col(1);
  last4Cps[2] = Qmv.col(2);
  last4Cps[3] = Qmv.col(3);

  /////////////////////
}

void SplineAStar::transformOtherBasis2BSpline(std::vector<Eigen::Vector3d>& last4Cps)
{
  /////////////////////
  Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
  Eigen::Matrix<double, 3, 4> Qmv;  // minvo
  Qmv.col(0) = last4Cps[0];
  Qmv.col(1) = last4Cps[1];
  Qmv.col(2) = last4Cps[2];
  Qmv.col(3) = last4Cps[3];

  Qbs = Qmv * Mbs2basis_inverse_;

  last4Cps[0] = Qbs.col(0);
  last4Cps[1] = Qbs.col(1);
  last4Cps[2] = Qbs.col(2);
  last4Cps[3] = Qbs.col(3);

  /////////////////////
}

bool SplineAStar::checkFeasAndFillND(std::vector<Eigen::Vector3d>& q, std::vector<Eigen::Vector3d>& n,
                                     std::vector<double>& d)
{
  n.resize(std::max(num_of_normals_, 0), Eigen::Vector3d::Zero());
  d.resize(std::max(num_of_normals_, 0), 0.0);

  std::vector<Eigen::Vector3d> last4Cps(4);

  bool isFeasible = true;
  /*
   std::cout << "q=" << std::endl;

   for (auto q_i : q)
   {
     std::cout << q_i.transpose() << std::endl;
   }

   std::cout << "num_of_segments_= " << num_of_segments_ << std::endl;
     std::cout << "num_of_normals_= " << num_of_normals_ << std::endl;
     std::cout << "M_= " << M_ << std::endl;
     std::cout << "N_= " << N_ << std::endl;
     std::cout << "p_= " << p_ << std::endl;
   */

  // Check obstacles constraints (and compute n and d)
  for (int index_interv = 0; index_interv < (q.size() - 3); index_interv++)
  {
    last4Cps[0] = q[index_interv];
    last4Cps[1] = q[index_interv + 1];
    last4Cps[2] = q[index_interv + 2];
    last4Cps[3] = q[index_interv + 3];

    if (basis_ == MINVO || basis_ == BEZIER)
    {
      transformBSpline2otherBasis(last4Cps);
    }

    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      Eigen::Vector3d n_i;
      double d_i;

      bool solved = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][index_interv], last4Cps);

      // std::cout << "________________________" << std::endl;
      // std::cout << "Solving LP, interval=" << index_interv << "Obstacle  " << obst_index << std::endl;
      // std::cout << "PointsA=" << std::endl;
      // std::cout << last4Cps[0].transpose() << std::endl;
      // std::cout << last4Cps[1].transpose() << std::endl;
      // std::cout << last4Cps[2].transpose() << std::endl;
      // std::cout << last4Cps[3].transpose() << std::endl;

      // std::cout << "PointsB=" << std::endl;
      // for (auto vertex_i : hulls_[obst_index][index_interv])
      // {
      //   std::cout << vertex_i.transpose() << std::endl;
      // }

      /*      std::cout << "Filling " << obst_index * num_of_segments_ + index_interv << std::endl;
            std::cout << "OBSTACLE= " << obst_index << std::endl;
            std::cout << "INTERVAL= " << index_interv << std::endl;
            std::cout << "last4Cps= " << std::endl;
            for (auto last4Cps_i : last4Cps)
            {
              std::cout << last4Cps_i.transpose() << std::endl;
            }

              */

      // std::cout << "index_interv= " << index_interv << std::endl;
      if (solved == false)
      {
        std::cout << "\nThis does NOT satisfy the LP: (in basis_chosen form) for obstacle= " << obst_index << std::endl;

        std::cout << last4Cps[0].transpose() << std::endl;
        std::cout << last4Cps[1].transpose() << std::endl;
        std::cout << last4Cps[2].transpose() << std::endl;
        std::cout << last4Cps[3].transpose() << std::endl;

        // std::cout << "interval=" << index_interv << std::endl;

        // std::cout << "Obstacle was= " << obst_index << std::endl;
        // for (auto vertex_i : hulls_[obst_index][index_interv])
        // {
        //   std::cout << vertex_i.transpose() << std::endl;
        // }

        /*
                std::cout << "(which, expressed in OV form, it is)" << std::endl;

                transformBSpline2Minvo(last4Cps);
                std::cout << last4Cps[0].transpose() << std::endl;
                std::cout << last4Cps[1].transpose() << std::endl;
                std::cout << last4Cps[2].transpose() << std::endl;
                std::cout << last4Cps[3].transpose() << std::endl;
                transformMinvo2BSpline(last4Cps);*/
        std::cout << bold << red << "[A*] The node provided doesn't satisfy LPs" << reset << std::endl;

        isFeasible = false;
      }
      else
      {
        // std::cout << "\nThis satisfies the LP (in basis_chosen form) for obstacle= " << obst_index << std::endl;

        // std::cout << last4Cps[0].transpose() << std::endl;
        // std::cout << last4Cps[1].transpose() << std::endl;
        // std::cout << last4Cps[2].transpose() << std::endl;
        // std::cout << last4Cps[3].transpose() << std::endl;

        /*        std::cout << "(which, expressed in OV form, it is)" << std::endl;

                transformBSpline2Minvo(last4Cps);
                std::cout << last4Cps[0].transpose() << std::endl;
                std::cout << last4Cps[1].transpose() << std::endl;
                std::cout << last4Cps[2].transpose() << std::endl;
                std::cout << last4Cps[3].transpose() << std::endl;
                transformMinvo2BSpline(last4Cps);*/
      }

      /*      std::cout << "solved with ni=" << n_i.transpose() << std::endl;
            std::cout << "solved with d_i=" << d_i << std::endl;*/

      n[obst_index * num_of_segments_ + index_interv] = n_i;
      d[obst_index * num_of_segments_ + index_interv] = d_i;
    }
  }

  // Check velocity and accel constraints
  Eigen::Vector3d vi;
  Eigen::Vector3d vip1;
  Eigen::Vector3d ai;
  double epsilon = 1.0001;
  for (int i = 0; i <= (N_ - 2); i++)
  {
    vi = p_ * (q[i + 1] - q[i]) / (knots_(i + p_ + 1) - knots_(i + 1));
    vip1 = p_ * (q[i + 1 + 1] - q[i + 1]) / (knots_(i + 1 + p_ + 1) - knots_(i + 1 + 1));
    ai = (p_ - 1) * (vip1 - vi) / (knots_(i + p_ + 1) - knots_(i + 2));

    if ((vi.array() > epsilon * v_max_.array()).any() || (vi.array() < -epsilon * v_max_.array()).any())
    {
      std::cout << red << "velocity constraint for vi is not satisfied, i=" << i << reset << std::endl;
      isFeasible = false;
    }

    if (i == N_ - 2)
    {  // Check also vNm1 (which should be zero)
      if ((vip1.array() > epsilon * v_max_.array()).any() || (vip1.array() < -epsilon * v_max_.array()).any())
      {
        isFeasible = false;
        std::cout << red << "velocity constraint for vNm1 is not satisfied" << reset << std::endl;
      }
    }
    if ((ai.array() > epsilon * a_max_.array()).any() || (ai.array() < -epsilon * a_max_.array()).any())
    {
      std::cout << red << "acceleration constraints are not satisfied" << reset << std::endl;

      isFeasible = false;
    }
  }

  return isFeasible;
}

void SplineAStar::expandAndAddToQueue(Node& current)
{
  // std::cout << bold << "openList_ size " << openList_.size() << reset << std::endl;

  MyTimer timer_expand(true);

  if (current.index == (N_ - 2))
  {
    // std::cout << "can't expand more in this direction" << std::endl;  // can't expand more
    return;  // neighbors = empty vector
  }

  int i = current.index;

  Eigen::Vector3d qiM1;

  if (current.index == 2)
  {
    qiM1 = q1_;
  }
  else
  {
    qiM1 = current.previous->qi;
  }

  double constraint_xL, constraint_xU, constraint_yL, constraint_yU, constraint_zL, constraint_zU;

  bool intervalIsNotZero = computeUpperAndLowerConstraints(i, qiM1, current.qi, constraint_xL, constraint_xU,
                                                           constraint_yL, constraint_yU, constraint_zL, constraint_zU);

  if (intervalIsNotZero == false)  // constraintxL>constraint_xU (or with other axes)
  {
    return;
  }

  //////////////////////////////

  std::vector<Eigen::Vector3d> last4Cps(4);

  if (current.index == 2)
  {
    last4Cps[0] = q0_;
    last4Cps[1] = q1_;
    last4Cps[2] = current.qi;
  }
  else if (current.index == 3)
  {
    last4Cps[0] = q1_;
    last4Cps[1] = current.previous->qi;
    last4Cps[2] = current.qi;
  }
  else
  {
    last4Cps[0] = current.previous->previous->qi;
    last4Cps[1] = current.previous->qi;
    last4Cps[2] = current.qi;
  }

  // stuff used in the loop (here to reduce time)
  Eigen::Vector3d vi;
  Node neighbor;

  neighbor.index = current.index + 1;
  neighbor.previous = &current;
  // Eigen::Vector3d aiM1;
  unsigned int ix, iy, iz;

  int jx, jy, jz;

  double delta_x = ((constraint_xU - constraint_xL) / (num_samples_x_ - 1));
  double delta_y = ((constraint_yU - constraint_yL) / (num_samples_y_ - 1));
  double delta_z = ((constraint_zU - constraint_zL) / (num_samples_z_ - 1));

  MyTimer timer_forLoop(true);

  double time_openList = 0.0;
  for (auto comb : all_combinations_)
  {
    jx = std::get<0>(comb);
    jy = std::get<1>(comb);
    jz = std::get<2>(comb);

    vi << constraint_xL + jx * delta_x, constraint_yL + jy * delta_y, constraint_zL + jz * delta_z;

    // std::cout << "vi" << vi.transpose() << std::endl;

    neighbor.qi = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + current.qi;

    /////
    ix = round((neighbor.qi.x() - orig_.x()) / voxel_size_);
    iy = round((neighbor.qi.y() - orig_.y()) / voxel_size_);
    iz = round((neighbor.qi.z() - orig_.z()) / voxel_size_);
    auto ptr_to_voxel = map_open_list_.find(Eigen::Vector3i(ix, iy, iz));
    bool already_exist = (ptr_to_voxel != map_open_list_.end());
    /////

    // bool already_exists_with_lower_cost = false;
    // already_exist
    if (already_exist ||                                        // Element already exists in the search box
        (vi.x() == 0 && vi.y() == 0 && vi.z() == 0) ||          // Not wanna use v=[0,0,0]
        (neighbor.qi.z() > z_max_ || neighbor.qi.z() < z_min_)  // ||  /// Outside the limits
        // (ix >= bbox_x_ / voxel_size_ ||                            // Out. the search box
        //  iy >= bbox_y_ / voxel_size_ ||                            // Out. the search box
        //  iz >= bbox_z_ / voxel_size_)                              // Out. the search box
    )

    {
      // std::cout << "already_exist" << std::endl;
      continue;
    }

    neighbor.g = current.g + weightEdge(current, neighbor);
    neighbor.h = h(neighbor);

    //////////////////////////////////// // Now let's check if it satisfies the LPs
    last4Cps[3] = neighbor.qi;

    if (collidesWithObstacles(last4Cps, neighbor.index) == true)
    {
      continue;
    }
    else
    {
      if (neighbor.index == (N_ - 2))
      {
        // Check for the convex hull qNm4, qNm3, qNm2, qNm1 (where qNm1==qNm2)
        std::vector<Eigen::Vector3d> last4Cps_tmp;
        last4Cps_tmp.push_back(last4Cps[1]);
        last4Cps_tmp.push_back(last4Cps[2]);
        last4Cps_tmp.push_back(last4Cps[3]);
        last4Cps_tmp.push_back(last4Cps[3]);

        if (collidesWithObstacles(last4Cps_tmp, N_ - 1) == true)
        {
          continue;
        }

        // Check for the convex hull qNm3, qNm2, qNm1, qN (where qN==qNm1==qNm2)
        last4Cps_tmp[0] = last4Cps[2];
        last4Cps_tmp[1] = last4Cps[3];
        last4Cps_tmp[2] = last4Cps[3];
        last4Cps_tmp[3] = last4Cps[3];

        if (collidesWithObstacles(last4Cps_tmp, N_) == true)
        {
          continue;
        }
      }

      MyTimer timer_openList(true);
      openList_.push(neighbor);

      time_openList = time_openList + timer_openList.ElapsedUs() / 1000.0;

      map_open_list_[Eigen::Vector3i(ix, iy, iz)] = true;

      expanded_nodes_.push_back(neighbor);
    }
  }

  std::cout << "pushing to openList  took " << time_openList << std::endl;
  std::cout << "openList size= " << openList_.size() << std::endl;

  std::cout << "for loop took " << timer_forLoop << std::endl;

  std::cout << "time_solving_lps_ " << time_solving_lps_ << std::endl;

  std::cout << bold << "expanding took " << timer_expand << reset << std::endl;

  time_solving_lps_ = 0.0;

  // time_expanding_ += timer_expand.ElapsedMs();
  // std::cout << "End of expand Function" << std::endl;
}

bool SplineAStar::collidesWithObstacles(std::vector<Eigen::Vector3d>& last4Cps, int index_lastCP)
{
  MyTimer timer_function(true);

  // std::cout << "In collidesWithObstacles, index_lastCP= " << index_lastCP << std::endl;

  // std::cout << "last4Cps.size()= " << last4Cps.size() << std::endl;
  // std::cout << "hulls_[obst_index][index_lastCP - 3].size()= " << hulls_[obst_index][index_lastCP - 3].size()
  //           << std::endl;
  // std::cout << " (NO, LP)" << std::endl;

  if (basis_ == MINVO || basis_ == BEZIER)
  {
    transformBSpline2otherBasis(last4Cps);  // now last4Cps are in MINVO BASIS
  }

  ///////////////////////
  bool satisfies_LP = true;

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    Eigen::Vector3d n_i;
    double d_i;
    satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][index_lastCP - 3], last4Cps);
    num_of_LPs_run_++;
    if (satisfies_LP == false)
    {
      goto exit;
    }
  }

  ///////////////////////

  if (basis_ == MINVO || basis_ == BEZIER)
  {
    transformOtherBasis2BSpline(last4Cps);
    // now last4Cps are in BSpline Basis (needed for the next iteration)
  }

exit:

  time_solving_lps_ += timer_function.ElapsedUs() / 1000.0;
  return (!satisfies_LP);
}

bool SplineAStar::run(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d)
{
  // Option 1
  CompareCost comparer;
  comparer.bias = bias_;
  // typedef std::set<Node, CompareCost> my_list;
  // typedef std::set<Node, CompareCost>::iterator my_list_iterator;
  // my_list openList_tmp(comparer);

  // Option 2
  std::priority_queue<Node, std::vector<Node>, CompareCost> openList_tmp(comparer);
  openList_ = openList_tmp;

  // Option 3
  // custom_priority_queue<Node, CompareCost> openList_tmp;
  // openList_ = openList_tmp;

  std::cout << "[A*] Running..." << std::endl;

  expanded_nodes_.clear();

  MyTimer timer_astar(true);
  // std::cout << "this->bias_ =" << this->bias_ << std::endl;

  Node nodeq2;
  nodeq2.index = 0;
  nodeq2.previous = NULL;
  nodeq2.g = 0;
  nodeq2.qi = q2_;
  nodeq2.index = 2;
  nodeq2.h = h(nodeq2);  // f=g+h

  // if (q2_.z() > z_max_ || q2_.z() < z_min_)
  // {  // happens in some corner cases
  //   // I need to detect it here because if not, no other nodes will be expanded, and I'll just fill everything with
  //   q2,
  //   // and nlopt will crash
  //   return false;
  // }

  // Option 1
  // openList_.insert(nodeq2);

  // Option 2
  // openList_.push(nodeq2);

  // Option 3
  openList_.push(nodeq2);

  Node* current_ptr;

  int status;

  int RUNTIME_REACHED = 0;
  int GOAL_REACHED = 1;
  int EMPTY_OPENLIST = 2;

  while (openList_.size() > 0)
  {
    // std::cout << red << "=============================" << reset << std::endl;

    current_ptr = new Node;

    // Option 1
    //*current_ptr = *openList_.begin();

    // Option 2 and 3
    *current_ptr = openList_.top();

    // std::cout << "Path to current_ptr= " << (*current_ptr).qi.transpose() << std::endl;
    // printPath(*current_ptr);

    double dist = ((*current_ptr).qi - goal_).norm();
    // std::cout << "dist2Goal=" << dist << ", index=" << (*current_ptr).index << std::endl;
    // std::cout << "bias_=" << bias_ << std::endl;
    // std::cout << "N_-2=" << N_ - 2 << std::endl;
    // std::cout << "index=" << (*current_ptr).index << std::endl;

    // log the closest solution so far
    // if ((*current_ptr).index == (N_ - 2))
    // {

    if ((*current_ptr).index == (N_ - 2) && dist < complete_closest_dist_so_far_)
    {
      complete_closest_dist_so_far_ = dist;
      complete_closest_result_so_far_ptr_ = current_ptr;
    }

    if (dist < closest_dist_so_far_)
    {
      closest_dist_so_far_ = dist;
      closest_result_so_far_ptr_ = current_ptr;
    }
    // }

    // std::cout << "time_solving_lps_= " << 100 * time_solving_lps_ / (timer_astar.ElapsedMs()) << "%" << std::endl;
    // std::cout << "time_expanding_= " << time_expanding_ << "ms, " << 100 * time_expanding_ /
    // (timer_astar.ElapsedMs())
    //           << "%" << std::endl;

    //////////////////////////////////////////////////////////////////
    //////////////Check if we are in the goal or if the runtime is over

    if (timer_astar.ElapsedMs() > (max_runtime_ * 1000))
    {
      std::cout << "[A*] Max Runtime was reached" << std::endl;
      status = RUNTIME_REACHED;
      goto exitloop;
    }

    // check if we are already in the goal
    if ((dist < goal_size_) && (*current_ptr).index == (N_ - 2))
    {
      std::cout << "[A*] Goal was reached!" << std::endl;
      status = GOAL_REACHED;
      goto exitloop;
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////

    // std::cout << "erasing from openList_" << (*current_ptr).qi.transpose() << std::endl;

    // Option 1
    // openList_.erase(openList_.begin());

    // Option 2
    // openList_.pop();  // remove the element

    // Option 3
    openList_.pop();

    // expanded_nodes_.push_back((*current_ptr));

    //////
    // unsigned int ix, iy, iz;
    // ix = round(((*current_ptr).qi.x() - orig_.x()) / voxel_size_);
    // iy = round(((*current_ptr).qi.y() - orig_.y()) / voxel_size_);
    // iz = round(((*current_ptr).qi.z() - orig_.z()) / voxel_size_);
    // map_open_list_[Eigen::Vector3i(ix, iy, iz)] = (*current_ptr).g + bias_ * (*current_ptr).h;
    ////////
    expandAndAddToQueue(*current_ptr);
  }

  std::cout << "[A*] openList_ is empty" << std::endl;
  status = EMPTY_OPENLIST;
  goto exitloop;

exitloop:

  std::cout << "status= " << status << std::endl;
  std::cout << "expanded_nodes_.size()= " << expanded_nodes_.size() << std::endl;
  std::cout << "complete_closest_dist_so_far_= " << complete_closest_dist_so_far_ << std::endl;

  Node* best_node_ptr = NULL;

  bool have_a_solution = (complete_closest_result_so_far_ptr_ != NULL) || (closest_result_so_far_ptr_ != NULL);

  if (status == GOAL_REACHED)
  {
    std::cout << "[A*] choosing current_ptr as solution" << std::endl;
    best_node_ptr = current_ptr;
  }
  else if ((status == RUNTIME_REACHED || status == EMPTY_OPENLIST) && have_a_solution)
  {
    // if (complete_closest_result_so_far_ptr_ != NULL)
    // {
    //   std::cout << "THERE EXISTS AT LEAST ONE COMPLETE PATH, with distance= " << complete_closest_dist_so_far_
    //             << std::endl;
    // }

    if (complete_closest_result_so_far_ptr_ != NULL)
    {
      std::cout << "[A*] choosing closest complete path as solution" << std::endl;
      best_node_ptr = complete_closest_result_so_far_ptr_;
    }
    else
    {
      std::cout << "[A*] choosing closest path as solution" << std::endl;
      best_node_ptr = closest_result_so_far_ptr_;
    }
  }
  else
  {
    std::cout << red << "This state should never occur" << reset << std::endl;
    abort();
    return false;
  }

  // Fill (until we arrive to N_-2), with the same qi
  // Note that, by doing this, it's not guaranteed feasibility wrt a dynamic obstacle
  // and hence the need of the function checkFeasAndFillND()

  bool path_found_is_not_complete = (best_node_ptr->index < (N_ - 2));

  if (path_found_is_not_complete)
  {
    for (int j = (best_node_ptr->index) + 1; j <= N_ - 2; j++)
    {
      // return false;

      Node* node_ptr = new Node;
      node_ptr->qi = best_node_ptr->qi;
      node_ptr->index = j;
      //   ROS_INFO_STREAM("Filled " << j);
      std::cout << red << "Filled " << j << ", " << reset;

      // << node_ptr->qi.transpose() << std::endl;
      node_ptr->previous = best_node_ptr;
      best_node_ptr = node_ptr;
    }
  }

  recoverPath(best_node_ptr);  // saved in result_

  result = result_;

  // if (visual_)
  // {
  //   plotExpandedNodesAndResult(expanded_nodes_, best_node_ptr);
  // }

  bool isFeasible = checkFeasAndFillND(result_, n, d);

  // std::cout << "____________" << std::endl;
  // for (auto qi : result_)
  // {
  //   std::cout << qi.transpose() << std::endl;
  // }

  if (isFeasible == false && path_found_is_not_complete == false)
  {
    std::cout << red << "This should never happen: All complete paths are guaranteed to be feasible" << reset
              << std::endl;

    // if (accel_constraints_not_satisfied_)
    // {
    abort();
    //}
  }

  std::cout << "returning isFeasible= " << isFeasible << std::endl;

  return isFeasible;

  // if (checkFeasAndFillND(result_, n, d) == false)  // This may be true or false, depending on the case
  // {
  //   return false;
  // }
  // else
  // {
  //   return true;
  // }

  /*
    return false;

    switch (status)
    {
      case GOAL_REACHED:
        recoverPath(current_ptr);                       // saved in result_
        if (checkFeasAndFillND(result, n, d) == false)  // This should always be true
        {
          return false;
        }
        if (visual_)
        {
          // plotExpandedNodesAndResult(expanded_nodes_, current_ptr);
        }
        return true;

      case RUNTIME_REACHED:
    }

    if (closest_result_so_far_ptr_ == NULL || (closest_result_so_far_ptr_->index == 2))
    {
      std::cout << " and couldn't find any solution" << std::endl;
      if (visual_)
      {
        plotExpandedNodesAndResult(expanded_nodes_, closest_result_so_far_ptr_);
      }
      return false;
    }
    else
    {
      // Fill the until we arrive to N_-2, with the same qi
      // Note that, by doing this, it's not guaranteed feasibility wrt a dynamic obstacle
      // and hence the need of the function checkFeasAndFillND()
      for (int j = (closest_result_so_far_ptr_->index) + 1; j <= N_ - 2; j++)
      {
        Node* node_ptr = new Node;
        node_ptr->qi = closest_result_so_far_ptr_->qi;
        node_ptr->index = j;
        std::cout << "Filled " << j << ", ";
        // << node_ptr->qi.transpose() << std::endl;
        node_ptr->previous = closest_result_so_far_ptr_;
        closest_result_so_far_ptr_ = node_ptr;
      }
      std::cout << std::endl;

      // std::cout << " best solution has dist=" << closest_dist_so_far_ << std::endl;

      std::cout << "calling recoverPath3" << std::endl;
      recoverPath(closest_result_so_far_ptr_);  // saved in result_
      std::cout << "called recoverPath3" << std::endl;

      if (visual_)
      {
        plotExpandedNodesAndResult(expanded_nodes_, closest_result_so_far_ptr_);
      }

      if (checkFeasAndFillND(result_, n, d) == false)  // This may be true or false, depending on the case
      {
        return false;
      }

      return true;
    }

    return false;*/
}

//////

// std::cout << " neighbor.qi =" << neighbor.qi.transpose() << std::endl;

// std::cout << "orig_ = " << orig_.transpose() << std::endl;
// std::cout << "voxel_size_ = " << voxel_size_ << std::endl;
/////////////////
// std::cout << "New point at " << ix << ", " << iy << ", " << iz << std::endl;
// std::cout << neighbor.qi.transpose() << std::endl;

////////////////////////////////
// auto ptr_to_node = (*ptr_to_voxel).second;
// auto node_tmp = (*ptr_to_node);
// already_exists_with_lower_cost = (node_tmp.g + bias_ * (node_tmp.h)) < (neighbor.g + bias_ * neighbor.h);
//////////////////////////////////////////////
// std::cout << "constraint_xU= " << constraint_xU << std::endl;
// std::cout << "constraint_xL= " << constraint_xL << std::endl;
// std::cout << "delta_x= " << delta_x << std::endl;
// std::cout << "_____________________________________________" << std::endl;
// std::cout << "_____________________________________________" << std::endl;
// std::cout << "_____________________________________________" << std::endl;
// std::cout << "The neighbors of " << current.qi.transpose() << " are " << std::endl;
//////////////////////////////////////////////
// std::cout << bold << red << "map_open_list_ contains " << reset << std::endl;
// for (auto tmp : map_open_list_)
// {
//   std::cout << red << (*(tmp.second)).qi.transpose() << reset << std::endl;
// }

// std::cout << bold << "openList_ contains " << reset << std::endl;
// for (auto tmp : openList_)
// {
//   std::cout << tmp.qi.transpose() << std::endl;
// }

// std::cout << bold << "openList_ contains " << reset << std::endl;
// for (auto tmp : expanded_nodes_)
// {
//   std::cout << tmp.qi.transpose() << std::endl;
// }

/////////////////////////////////////////////////
// if (already_exists_with_lower_cost == true)
// {
//   std::cout << "already_exists_with_lower_cost" << std::endl;

//   continue;
// }

// if (already_exist == false)
// {
//   std::cout << red << neighbor.qi.transpose() << " does NOT exist" << reset << std::endl;
// }

////////////////////////////////////////////////
// std::cout << "size openList= " << openList_.size() << std::endl;
// if (already_exist)
// {
// deleteFromOpenListtheOldOne
// auto it = std::find_if(openList_.begin(), openList_.end(), boost::bind(&Node::index, _1) == neighbor.index);
//   std::cout << "trying to erase from openList_" << (*((*ptr_to_voxel).second)).qi.transpose() << std::endl;

// Option 1
// openList_.erase((*ptr_to_voxel).second);

// Option 3
// openList_.remove((*ptr_to_voxel).second);

// and now set the direction in the map to Null:
//(*ptr_to_voxel).second = nullptr;
// I should not use more times this pointer. But have to leave it here because it needs to keep "existing" so
// that I don't expand again that same voxel

//   std::cout << "erased" << std::endl;
// }

// typedef std::set<Node, CompareCost> my_list;
// typedef std::set<Node, CompareCost>::iterator my_list_iterator;

// std::cout << "other" << std::endl;

// std::pair<my_list_iterator, bool> ret;
// std::cout << "inserting to openList_" << neighbor.qi.transpose() << std::endl;

// Option 1
// ret = openList_.insert(neighbor);
// if (!ret.second)
// {
//   std::cout << "no insertion!\n";
// }
// map_open_list_[Eigen::Vector3i(ix, iy, iz)] = ret.first;  // Keep an iterator to neighbor

// Option 3
// auto iterator_tmp = openList_.pushAndReturnIterator(neighbor);
// map_open_list_[Eigen::Vector3i(ix, iy, iz)] = iterator_tmp;  // Keep an iterator to neighbor

// std::cout << red << "inserting to map_open_list_" << neighbor.qi.transpose() << reset << std::endl;
// std::cout << "other_done" << std::endl;

/////////////////////////////////////////////////

/*          std::cout << "ix= " << ix << " is outside, max= " << matrixExpandedNodes_.size() << std::endl;
          std::cout << "iy= " << iy << " is outside, max= " << matrixExpandedNodes_[0].size() << std::endl;
          std::cout << "iz= " << iz << " is outside, max= " << matrixExpandedNodes_[0][0].size() << std::endl;
          std::cout << "voxel_size_= " << voxel_size_ << std::endl;
          std::cout << "orig_= " << orig_.transpose() << std::endl;
          std::cout << "neighbor.qi= " << neighbor.qi.transpose() << std::endl;*/

/*  vx_.clear();
  vy_.clear();
  vz_.clear();

  for (int i = 0; i < num_samples_x_; i++)
  {
    vx_.push_back(-v_max_.x() + i * ((2.0 * v_max_.x()) / (num_samples_x_ - 1)));
  }
  for (int i = 0; i < num_samples_y_; i++)
  {
    vy_.push_back(-v_max_.y() + i * ((2.0 * v_max_.y()) / (num_samples_y_ - 1)));
  }
  for (int i = 0; i < num_samples_z_; i++)
  {
    vz_.push_back(-v_max_.z() + i * ((2.0 * v_max_.z()) / (num_samples_z_ - 1)));
  }

  std::cout << "vx is: " << std::endl;
  for (auto vxi : vx_)
  {
    std::cout << "vxi= " << vxi << std::endl;
  }
*/

/*  vx_.clear();
  vy_.clear();
  vz_.clear();

  for (int i = 0; i < num_samples_x_; i++)
  {
    vx_.push_back(constraint_xL + i * ((constraint_xU - constraint_xL) / (num_samples_x_ - 1)));
  }
  for (int i = 0; i < num_samples_y_; i++)
  {
    vy_.push_back(constraint_yL + i * ((constraint_yU - constraint_yL) / (num_samples_y_ - 1)));
  }
  for (int i = 0; i < num_samples_z_; i++)
  {
    vz_.push_back(constraint_zL + i * ((constraint_zU - constraint_zL) / (num_samples_z_ - 1)));
  }*/

/*  std::cout << "vx is: " << std::endl;
  for (auto tmp_i : vx_)
  {
    std::cout << tmp_i << std::endl;
  }
*/
// std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;
//    if (constraint_zL <= vz_i <= constraint_zU)
//    {
// std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;

/*  std::cout << "Current= " << current.qi.transpose() << std::endl;

  std::cout << "Current index= " << current.index << std::endl;
  std::cout << "knots= " << knots_ << std::endl;
  std::cout << "tmp= " << tmp << std::endl;
  std::cout << "a_max_.x() * tmp= " << a_max_.x() * tmp << std::endl;
  std::cout << "v_iM1.x()= " << v_iM1.x() << std::endl;*/

/*  vx_.clear();
  vy_.clear();
  vz_.clear();

  vx_.push_back(v_max_(0));
  vx_.push_back(v_max_(0) / 2.0);
  vx_.push_back(0);
  vx_.push_back(-v_max_(0) / 2.0);
  vx_.push_back(-v_max_(0));

  vy_.push_back(v_max_(1));
  vy_.push_back(v_max_(1) / 2.0);
  vy_.push_back(0);
  vy_.push_back(-v_max_(1) / 2.0);
  vy_.push_back(-v_max_(1));

  vz_.push_back(v_max_(2));
  vz_.push_back(v_max_(2) / 2.0);
  vz_.push_back(0);
  vz_.push_back(-v_max_(2) / 2.0);
  vz_.push_back(-v_max_(2));*/

/*  Eigen::Matrix<double, 4, 1> tmp_x;
  tmp_x << v_max_.x(), -v_max_.x(), a_max_.x() * tmp + viM1.x(), -a_max_.x() * tmp + viM1.x();
  double constraint_xU = tmp_x.maxCoeff();  // upper bound
  double constraint_xL = tmp_x.minCoeff();  // lower bound

  Eigen::Matrix<double, 4, 1> tmp_y;
  tmp_y << v_max_.y(), -v_max_.y(), a_max_.y() * tmp + viM1.y(), -a_max_.y() * tmp + viM1.y();
  double constraint_yU = tmp_y.maxCoeff();  // upper bound
  double constraint_yL = tmp_y.minCoeff();  // lower bound

  Eigen::Matrix<double, 4, 1> tmp_z;
  tmp_z << v_max_.z(), -v_max_.z(), a_max_.z() * tmp + viM1.z(), -a_max_.z() * tmp + viM1.z();
  double constraint_zU = tmp_z.maxCoeff();  // upper bound
  double constraint_zL = tmp_z.minCoeff();  // lower bound*/

/*  std::cout << "constraint_xL= " << constraint_xL << std::endl;
  std::cout << "constraint_xU= " << constraint_xU << std::endl;
  std::cout << "constraint_yL= " << constraint_yL << std::endl;
  std::cout << "constraint_yU= " << constraint_yU << std::endl;
  std::cout << "constraint_zL= " << constraint_zL << std::endl;
  std::cout << "constraint_zU= " << constraint_zU << std::endl;*/

/*    std::cout << "======================" << std::endl;

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> novale = openList;

    while (!novale.empty())
    {
      std::cout << (novale.top().h + novale.top().g) << " ";
      novale.pop();
    }*/

/*      double tentative_g = current.g + weightEdge(current, neighbor);  // Cost to come + new edge
      std::cout << "TEST" << std::endl;
      std::cout << "tentative_g=" << tentative_g << std::endl;
      std::cout << "g(neighbor)=" << g(neighbor) << std::endl;
      std::cout << "neighor.index=" << neighbor.index << std::endl;

      if (tentative_g < g(neighbor))
      {
        neighbor.previous = &current;
        neighbor.g = tentative_g;
        neighbor.f = neighbor.g + h(neighbor);
        // if neighbor not in OPEN SET TODO!!
        openList.push(neighbor);  // the order is automatic (it's a prioriry queue)
      }*/

/*  std::cout << "tmp=" << tmp << std::endl;

  std::cout << "vx_ =" << std::endl;

  for (double vx_i : vx_)
  {
    std::cout << vx_i << ", ";
  }
  std::cout << std::endl;

  for (double vy_i : vy_)
  {
    std::cout << vy_i << ", ";
  }
  std::cout << std::endl;

  for (double vz_i : vz_)
  {
    std::cout << vz_i << ", ";
  }
  std::cout << std::endl;*/

/*  if (current.index == 0)
  {
    Node nodeq1 = current;
    nodeq1.index = 1;
    nodeq1.previous = &current;
    nodeq1.g = 0;
    nodeq1.f = nodeq1.g + h(nodeq1);  // f=g+h

    neighbors.push_back(nodeq1);
    return neighbors;
  }

  if (current.index == 1)
  {
    Node nodeq2 = current;
    nodeq2.index = 1;
    nodeq2.previous = &current;
    nodeq2.g = current.previous->g + weightEdge(current, nodeq2);
    nodeq2.f = nodeq2.g + h(nodeq2);  // f=g+h

    neighbors.push_back(nodeq2);
    return neighbors;
  }*/

/*      std::cout << "Size cvxhull=" << hulls_[obst_index][current.index + 1 - 3].size() << std::endl;

      std::cout << "Vertexes obstacle" << std::endl;
      for (auto vertex_obs : hulls_[obst_index][current.index + 1 - 3])
      {
        std::cout << "vertex_obs= " << vertex_obs.transpose() << std::endl;
      }

      std::cout << "CPs" << std::endl;
      for (auto cp : last4Cps)
      {
        std::cout << "cp= " << cp.transpose() << std::endl;
      }

      std::cout << "n_i= " << n_i.transpose() << ", d_i= " << d_i << std::endl;*/