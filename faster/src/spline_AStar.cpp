#include "spline_AStar.hpp"

#include <queue>
#include <vector>

#include "timer.hpp"

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

  separator_solver = new separator::Separator(1.0, 1.0, 1.0);

  computeInverses();
}

SplineAStar::~SplineAStar()
{
}

void SplineAStar::setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int samples_x, int samples_y,
                                         int samples_z)
{
  v_max_ = v_max;
  a_max_ = a_max;

  samples_x_ = samples_x;
  samples_y_ = samples_y;
  samples_z_ = samples_z;
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
  double tmp = (knots_(i + p_ + 1) - knots_(i + 2)) / (1.0 * (p_ - 1));
  double constraint_x = std::min(v_max_.x(), a_max_.x() * tmp + v_iM1.x());
  double constraint_y = std::min(v_max_.y(), a_max_.y() * tmp + v_iM1.y());
  double constraint_z = std::min(v_max_.z(), a_max_.z() * tmp + v_iM1.z());

  ////////////
  vx_.clear();
  vy_.clear();
  vz_.clear();

  for (int i = 0; i < samples_x_; i++)
  {
    vx_.push_back(-constraint_x + i * ((2.0 * constraint_x) / (samples_x_ - 1)));
  }
  for (int i = 0; i < samples_y_; i++)
  {
    vy_.push_back(-constraint_y + i * ((2.0 * constraint_y) / (samples_y_ - 1)));
  }
  for (int i = 0; i < samples_z_; i++)
  {
    vz_.push_back(-constraint_z + i * ((2.0 * constraint_z) / (samples_z_ - 1)));
  }

  /////////////

  for (double vx_i : vx_)
  {
    for (double vy_i : vy_)
    {
      for (double vz_i : vz_)
      {
        if (vx_i == 0 && vy_i == 0 && vz_i == 0)
        {
          continue;
        }

        // std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;
        if (fabs(vx_i) <= constraint_x && fabs(vy_i) <= constraint_y && fabs(vz_i) <= constraint_z)
        {
          // std::cout << "Satisfies constraints" << std::endl;

          Eigen::Vector3d vi(vx_i, vy_i, vz_i);
          Node neighbor;
          neighbor.qi = (knots_(i + p_ + 1) - knots_(i + 1)) * vi / (1.0 * p_) + current.qi;
          neighbor.index = current.index + 1;
          neighbor.previous = &current;
          neighbor.g = current.g + weightEdge(current, neighbor);
          neighbor.h = h(neighbor);
          neighbors_va.push_back(neighbor);
          // std::cout << "*************************" << std::endl;
          // std::cout << "vx_i=" << vx_i << ", vy_i=" << vy_i << ", vz_i=" << vz_i << std::endl;
          // std::cout << "Linking, qi=" << neighbor.qi.transpose() << std::endl;
          // std::cout << "Linking, anterior=" << neighbor.previous->qi.transpose() << std::endl;
        }
        /*        else
                {
                  std::cout << "Doesn't satisfy the constraints" << std::endl;
                }*/
      }
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

      satisfies_LP = separator_solver->solveModel(n_i, d_i, hulls_[obst_index][current.index + 1 - 3], last4Cps);

      if (satisfies_LP == false)
      {
        break;
      }

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
    }
    if (satisfies_LP == true)  // this is true also when num_of_obst_=0;
    {
      neighbors.push_back(neighbor_va);
    }
  }
  // std::cout << "returning neighbors" << std::endl;
  return neighbors;
}

double SplineAStar::h(Node& node)
{
  // See Notebook mathematica
  int dim_var = N_ - node.index;

  Eigen::Matrix<double, -1, 1> b = Eigen::MatrixXd::Zero(dim_var, 1);

  double heuristics = 0;

  // std::cout << "Going to compute coordinates now" << std::endl;

  for (int coord = 0; coord < 3; coord++)  // separation in the three coordinates
  {
    double qi = node.qi(coord);
    double qiM1;

    // std::cout << "Going to compute coordinates now1" << std::endl;

    if (node.index == 2)
    {
      qiM1 = q1_(coord);
    }
    else
    {
      qiM1 = node.previous->qi(coord);
    }
    // std::cout << "Going to compute coordinates now2" << std::endl;

    double goal = goal_(coord);

    b(0, 0) = 2 * (qiM1 - 4 * qi);
    b(1, 0) = 2 * qi;
    b(b.size() - 2, 0) = 2 * goal;
    b(b.size() - 1, 0) = -6 * goal;
    // std::cout << "Going to compute coordinates now3" << std::endl;
    double c = qiM1 * qiM1 - 4 * qiM1 * qi + 5 * qi * qi + 2 * goal * goal;

    heuristics = heuristics + (-0.5 * b.transpose() * Ainverses_[dim_var] * b + c);
  }

  std::cout << "heuristics= " << heuristics << std::endl;

  return heuristics;
  // return (node.qi - goal_).norm();  // hack, should be squaredNorm();
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
  if (node1.index == 2)
  {
    return (node2.qi - 2 * node1.qi + q1_).squaredNorm();
  }

  // return (node2.qi - node1.qi).norm();  // hack Comment this and comment out the other stuff
  return (node2.qi - 2 * node1.qi + node1.previous->qi).squaredNorm();
}

void SplineAStar::printPath(Node& node1)
{
  Node tmp = node1;
  while (tmp.previous != NULL)
  {
    std::cout << tmp.index << ", ";  // qi.transpose().x()
    // std::cout << "El anterior es=" << tmp.previous->qi.transpose() << std::endl;
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
    // std::cout << "Pushing back tmp->qi=" << tmp->qi.transpose() << std::endl;
    result.insert(result.begin(), tmp->qi);
    tmp = tmp->previous;
  }
  result.insert(result.begin(), q1_);
  result.insert(result.begin(), q0_);
}

void SplineAStar::plotExpandedNodesAndResult(std::vector<Eigen::Vector3d>& expanded_nodes,
                                             std::vector<Eigen::Vector3d>& result)
{
  std::vector<double> x, y, z;
  for (auto node : expanded_nodes)
  {
    x.push_back(node.x());
    y.push_back(node.y());
    z.push_back(node.z());
  }

  plt::plot(x, y, "o ");

  std::vector<double> x_result, y_result, z_result;

  for (auto node : result)
  {
    x_result.push_back(node.x());
    y_result.push_back(node.y());
    z_result.push_back(node.z());
  }

  plt::plot(x_result, y_result, "or ");

  plt::show();
}

bool SplineAStar::run(std::vector<Eigen::Vector3d>& result)
{
  std::vector<Eigen::Vector3d> expanded_nodes;

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
    // std::cout << "======================" << std::endl;
    // std::cout << "sizeOpenList=" << openList.size() << std::endl;
    current_ptr = new Node;
    *current_ptr = openList.top();

    expanded_nodes.push_back((*current_ptr).qi);

    // std::cout << "Path to current_ptr= " << (*current_ptr).qi.transpose() << std::endl;
    // printPath(*current_ptr);

    double dist = ((*current_ptr).qi - goal_).norm();
    std::cout << "dist2Goal=" << dist << ", index=" << (*current_ptr).index << std::endl;
    std::cout << "bias_=" << bias_ << std::endl;
    // std::cout << "N_=" << N_ << std::endl;

    // log the closest solution so far
    if ((*current_ptr).index == (N_ - 2))
    {
      if (dist < closest_dist_so_far_)
      {
        closest_dist_so_far_ = dist;
        closest_result_so_far_ptr_ = current_ptr;
      }
    }

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
      plotExpandedNodesAndResult(expanded_nodes, result);

      return true;
    }
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////

    openList.pop();  // remove the element

    std::vector<Node> neighbors = expand(*current_ptr);

    // std::cout << "There are " << neighbors.size() << " neighbors" << std::endl;

    for (auto neighbor : neighbors)
    {
      openList.push(neighbor);
    }

    // closedList.push_back(current_ptr);
  }

  std::cout << "openList is empty" << std::endl;

exitloop:

  if (closest_result_so_far_ptr_ == NULL)
  {
    std::cout << " and couldn't find any solution" << std::endl;
    plotExpandedNodesAndResult(expanded_nodes, result);
    return false;
  }
  else
  {
    std::cout << " and the best solution found has dist=" << closest_dist_so_far_ << std::endl;
    recoverPath(closest_result_so_far_ptr_, result);
    plotExpandedNodesAndResult(expanded_nodes, result);
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