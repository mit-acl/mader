#include "spline_AStar.hpp"

#include <queue>
#include <vector>

#include "timer.hpp"
#include "termcolor.hpp"
using namespace termcolor;

#define WITHOUT_NUMPY
#include "matplotlibcpp.h"

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

  separator_solver_ = new separator::Separator(1.0, 1.0, 1.0);

  computeInverses();
}

SplineAStar::~SplineAStar()
{
}

void SplineAStar::setVisual(bool visual)
{
  visual_ = visual;
}

void SplineAStar::setBBoxSearch(double x, double y, double z)
{
  bbox_x_ = x;
  bbox_y_ = y;
  bbox_z_ = z;
}

void SplineAStar::setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int samples_x, int samples_y,
                                         int samples_z)
{
  v_max_ = v_max;
  a_max_ = a_max;

  // ensure they are odd numbers (so that vx=0 is included in the samples)
  samples_x_ = (samples_x % 2 == 0) ? ceil(samples_x) : samples_x;
  samples_y_ = (samples_y % 2 == 0) ? ceil(samples_y) : samples_y;
  samples_z_ = (samples_z % 2 == 0) ? ceil(samples_z) : samples_z;

  ////////////
  vx_.clear();
  vy_.clear();
  vz_.clear();

  for (int i = 0; i < samples_x_; i++)
  {
    vx_.push_back(-v_max_.x() + i * ((2.0 * v_max_.x()) / (samples_x_ - 1)));
  }
  for (int i = 0; i < samples_y_; i++)
  {
    vy_.push_back(-v_max_.y() + i * ((2.0 * v_max_.y()) / (samples_y_ - 1)));
  }
  for (int i = 0; i < samples_z_; i++)
  {
    vz_.push_back(-v_max_.z() + i * ((2.0 * v_max_.z()) / (samples_z_ - 1)));
  }

  std::cout << "vx is: " << std::endl;
  for (auto vxi : vx_)
  {
    std::cout << "vxi= " << vxi << std::endl;
  }

  // TODO: remove hand-coded stuff
  int i = 6;
  // increment_ = v_max_(0) * (knots_(i + p_ + 1) - knots_(i + 1)) / (1.0 * p_);
  increment_ = fabs(vx_[1] - vx_[0]) * (knots_(i + p_ + 1) - knots_(i + 1)) / (1.0 * p_);
  // note that (neighbor.qi - current.qi) is guaranteed to be an integer multiple of increment_

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

  int length_x = bbox_x_ / increment_;
  int length_y = bbox_y_ / increment_;
  int length_z = bbox_z_ / increment_;

  std::vector<std::vector<std::vector<bool>>> novale(
      length_x, std::vector<std::vector<bool>>(length_y, std::vector<bool>(length_z, false)));

  orig_ = q2_ - Eigen::Vector3d(bbox_x_ / 2.0, bbox_y_ / 2.0, bbox_z_ / 2.0);

  matrixExpandedNodes_ = novale;
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

std::vector<Node> SplineAStar::expand(Node& current)
{
  MyTimer timer_expand(true);

  // std::cout << "expanding" << std::endl;
  std::vector<Node> neighbors;

  if (current.index == (N_ - 2))
  {
    // std::cout << "can't expand more in this direction" << std::endl;  // can't expand more
    return neighbors;  // empty vector
  }

  // first expansion satisfying vmax and amax
  std::vector<Node> neighbors_va;

  int i = current.index;

  Eigen::Vector3d v_iM1;

  if (current.index == 2)
  {
    v_iM1 = p_ * (current.qi - q1_) / (knots_(i + p_) - knots_(i));  // velocity_{current.index -1}
  }
  else
  {
    v_iM1 = p_ * (current.qi - current.previous->qi) / (knots_(i + p_) - knots_(i));  // velocity_{current.index -1}
  }

  // std::cout << "v_iM1= " << v_iM1.transpose() << std::endl;
  // double tmp = (knots_(i + p_ + 1) - knots_(i + 2)) / (1.0 * (p_ - 1));

  /*  std::cout << "Current= " << current.qi.transpose() << std::endl;

    std::cout << "Current index= " << current.index << std::endl;
    std::cout << "knots= " << knots_ << std::endl;
    std::cout << "tmp= " << tmp << std::endl;
    std::cout << "a_max_.x() * tmp= " << a_max_.x() * tmp << std::endl;
    std::cout << "v_iM1.x()= " << v_iM1.x() << std::endl;*/

  /*  double constraint_xU = std::min(v_max_.x(), a_max_.x() * tmp + v_iM1.x());    // upper bound
    double constraint_xL = std::max(-v_max_.x(), -a_max_.x() * tmp + v_iM1.x());  // lower bound

    double constraint_yU = std::min(v_max_.y(), a_max_.y() * tmp + v_iM1.y());    // upper bound
    double constraint_yL = std::max(-v_max_.y(), -a_max_.y() * tmp + v_iM1.y());  // lower bound

    double constraint_zU = std::min(v_max_.z(), a_max_.z() * tmp + v_iM1.z());    // upper bound
    double constraint_zL = std::max(-v_max_.z(), -a_max_.z() * tmp + v_iM1.z());  // lower bound*/

  /*  std::cout << " constraint_x= " << constraint_x << ", constraint_y= " << constraint_x
              << " constraint_z= " << constraint_x << std::endl;*/

  /////////////

  for (double vx_i : vx_)
  {
    // if (constraint_xL <= vx_i <= constraint_xU)
    // {
    for (double vy_i : vy_)
    {
      // if (constraint_yL <= vy_i <= constraint_yU)
      //{
      for (double vz_i : vz_)
      {
        if (vx_i == 0 && vy_i == 0 && vz_i == 0)
        {
          continue;
        }

        // std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;
        //    if (constraint_zL <= vz_i <= constraint_zU)
        //    {
        // std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;

        Eigen::Vector3d vi(vx_i, vy_i, vz_i);
        Node neighbor;

        neighbor.qi = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + current.qi;
        // note that (neighbor.qi - current.qi) is guaranteed to be an integer multiple of increment_

        unsigned int ix = round((neighbor.qi.x() - orig_.x()) / increment_);
        unsigned int iy = round((neighbor.qi.y() - orig_.y()) / increment_);
        unsigned int iz = round((neighbor.qi.z() - orig_.z()) / increment_);

        if (ix >= matrixExpandedNodes_.size() || iy >= matrixExpandedNodes_[0].size() ||
            iz >= matrixExpandedNodes_[0][0].size())
        {
          // node is outside the search box
          std::cout << "ix= " << ix << " is outside, max= " << matrixExpandedNodes_.size() << std::endl;
          std::cout << "iy= " << iy << " is outside, max= " << matrixExpandedNodes_[0].size() << std::endl;
          std::cout << "iz= " << iz << " is outside, max= " << matrixExpandedNodes_[0][0].size() << std::endl;
          std::cout << "increment_= " << increment_ << std::endl;
          std::cout << "orig_= " << orig_.transpose() << std::endl;
          std::cout << "neighbor.qi= " << neighbor.qi.transpose() << std::endl;
          continue;
        }

        // check here if ix, iy, iz are within the interval of matrixExpandedNodes_

        if (matrixExpandedNodes_[ix][iy][iz] == true)
        {
          // std::cout << "Already in the expanded list" << std::endl;
          continue;  // already in the expanded list
        }
        else
        {
          matrixExpandedNodes_[ix][iy][iz] = true;

          neighbor.index = current.index + 1;
          neighbor.previous = &current;
          neighbor.g = current.g + weightEdge(current, neighbor);
          neighbor.h = h(neighbor);
          neighbors_va.push_back(neighbor);
        }
        // }
        /*        else
                {
                  std::cout << "Doesn't satisfy the constraints" << std::endl;
                }*/
        //  }
      }
      //}
    }
  }
  // std::cout << "neighbors_va.size()= " << neighbors_va.size() << std::endl;

  // if the index is <=6, return what we have //TODO this should be <=2, !!!!

  /*  if (current.index <= 6)
    {*/
  // For now return all the neighbors
  // return neighbors_va;
  //}

  // And now select the ones that satisfy the LP with all the obstacles

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

  for (auto neighbor_va : neighbors_va)
  {
    last4Cps[3] = neighbor_va.qi;
    bool satisfies_LP = true;
    // std::cout << "num_of_obst_=" << num_of_obst_ << std::endl;
    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      Eigen::Vector3d n_i;
      double d_i;

      MyTimer timer_lp(true);
      satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][current.index + 1 - 3], last4Cps);
      time_solving_lps_ += timer_lp.ElapsedMs();

      if (satisfies_LP == false)
      {
        break;
      }
    }
    if (satisfies_LP == true)  // this is true also when num_of_obst_=0;
    {
      neighbors.push_back(neighbor_va);
    }
  }

  time_expanding_ += timer_expand.ElapsedMs();

  // std::cout << "returning neighbors" << std::endl;
  return neighbors;
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

  double heuristics = (node.qi - goal_).squaredNorm();  // hack
  return heuristics;
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
    }*/

  return (node2.qi - node1.qi).norm() / (node2.index);  // hack Comment this and comment out the other stuff
  // return (node2.qi - 2 * node1.qi + node1.previous->qi).squaredNorm();
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

void SplineAStar::recoverPath(Node* node1_ptr, std::vector<Eigen::Vector3d>& result)
{
  result.clear();

  Node* tmp = node1_ptr;

  result.insert(result.begin(), tmp->qi);  // qN
  result.insert(result.begin(), tmp->qi);  // qN-1

  while (tmp != NULL)
  {
    result.insert(result.begin(), tmp->qi);
    tmp = tmp->previous;
  }
  result.insert(result.begin(), q1_);
  result.insert(result.begin(), q0_);
}

void SplineAStar::plotExpandedNodesAndResult(std::vector<Node>& expanded_nodes, Node* result_ptr)
{
  for (auto node : expanded_nodes)
  {
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

  if (result_ptr != NULL)
  {
    std::vector<Eigen::Vector3d> path;
    recoverPath(result_ptr, path);

    std::vector<double> x_result, y_result, z_result;

    for (auto node : path)
    {
      x_result.push_back(node.x());
      y_result.push_back(node.y());
      z_result.push_back(node.z());
    }

    plt::plot(x_result, y_result, "or-");
  }

  plt::show();
}

/*bool SplineAStar::isInExpandedList(Node& tmp)
{

}*/

bool SplineAStar::fillNDFromNode(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n,
                                 std::vector<double>& d)
{
  n.resize(std::max(num_of_normals_, 0), Eigen::Vector3d::Zero());
  d.resize(std::max(num_of_normals_, 0), 0.0);

  std::vector<Eigen::Vector3d> last4Cps(4);
  /*
   std::cout << "result=" << std::endl;

   for (auto result_i : result)
   {
     std::cout << result_i.transpose() << std::endl;
   }

   std::cout << "num_of_segments_= " << num_of_segments_ << std::endl;
     std::cout << "num_of_normals_= " << num_of_normals_ << std::endl;
     std::cout << "M_= " << M_ << std::endl;
     std::cout << "N_= " << N_ << std::endl;
     std::cout << "p_= " << p_ << std::endl;
   */
  for (int index_interv = 0; index_interv < (result.size() - 3); index_interv++)
  {
    last4Cps[0] = result[index_interv];
    last4Cps[1] = result[index_interv + 1];
    last4Cps[2] = result[index_interv + 2];
    last4Cps[3] = result[index_interv + 3];

    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      Eigen::Vector3d n_i;
      double d_i;

      bool solved = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][index_interv], last4Cps);

      /*      std::cout << "Filling " << obst_index * num_of_segments_ + index_interv << std::endl;
            std::cout << "OBSTACLE= " << obst_index << std::endl;
            std::cout << "INTERVAL= " << index_interv << std::endl;
            std::cout << "last4Cps= " << std::endl;
            for (auto last4Cps_i : last4Cps)
            {
              std::cout << last4Cps_i.transpose() << std::endl;
            }

            std::cout << "Obstacle= " << std::endl;
            for (auto vertex_i : hulls_[obst_index][index_interv])
            {
              std::cout << vertex_i.transpose() << std::endl;
            }
      */
      if (solved == false)
      {
        std::cout << bold << red << "The node provided doesn't satisfy LPs" << reset << std::endl;
        return false;
      }

      /*      std::cout << "solved with ni=" << n_i.transpose() << std::endl;
            std::cout << "solved with d_i=" << d_i << std::endl;*/

      n[obst_index * num_of_segments_ + index_interv] = n_i;
      d[obst_index * num_of_segments_ + index_interv] = d_i;
    }
  }
  return true;
}

bool SplineAStar::run(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d)
{
  expanded_nodes_.clear();

  MyTimer timer_astar(true);

  auto cmp = [&](Node& left, Node& right) {
    return (left.g + this->bias_ * left.h) > (right.g + this->bias_ * right.h);
  };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> openList(cmp);  //= OpenSet, = Q

  Node nodeq2;
  nodeq2.index = 0;
  nodeq2.previous = NULL;
  nodeq2.g = 0;
  nodeq2.qi = q2_;
  nodeq2.index = 2;
  nodeq2.h = h(nodeq2);  // f=g+h

  openList.push(nodeq2);

  Node* current_ptr;

  while (openList.size() > 0)
  {
    // std::cout << red << "=============================" << reset << std::endl;

    current_ptr = new Node;
    *current_ptr = openList.top();

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
      goto exitloop;
    }

    // check if we are already in the goal
    if ((dist < goal_size_) && (*current_ptr).index == (N_ - 2))
    {
      std::cout << "[A*] Goal was reached!" << std::endl;
      recoverPath(current_ptr, result);
      if (fillNDFromNode(result, n, d) == false)  // happens sometimes, unsure why yet...
      {
        return false;
      }
      if (visual_)
      {
        plotExpandedNodesAndResult(expanded_nodes_, current_ptr);
      }

      return true;
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////

    openList.pop();  // remove the element

    std::vector<Node> neighbors = expand(*current_ptr);
    expanded_nodes_.push_back((*current_ptr));

    for (auto neighbor : neighbors)
    {
      openList.push(neighbor);
    }
  }

  std::cout << "openList is empty" << std::endl;

exitloop:

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
    for (int j = (closest_result_so_far_ptr_->index) + 1; j <= N_ - 2; j++)
    {
      std::cout << "Filling " << j << std::endl;
      Node* node_ptr = new Node;
      node_ptr->qi = closest_result_so_far_ptr_->qi;
      node_ptr->index = j;
      node_ptr->previous = closest_result_so_far_ptr_;
      closest_result_so_far_ptr_ = node_ptr;
    }

    std::cout << " and the best solution found has dist=" << closest_dist_so_far_ << std::endl;
    recoverPath(closest_result_so_far_ptr_, result);
    if (fillNDFromNode(result, n, d) == false)
    {  // happens sometimes, unsure why yet...
      return false;
    }

    if (visual_)
    {
      plotExpandedNodesAndResult(expanded_nodes_, closest_result_so_far_ptr_);
    }
    return true;
  }

  return false;
}

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