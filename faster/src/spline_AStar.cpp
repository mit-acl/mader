#include "spline_AStar.hpp"

#include <queue>
#include <vector>

SplineAStar::SplineAStar(int p, int N, int num_of_obst, const Eigen::MatrixXd knots,
                         const ConvexHullsOfCurves_Std hulls)
  : p_(p), N_(N), knots_(knots), hulls_(hulls), num_of_obst_(num_of_obst)
{
  separator_solver = new separator::Separator(1.0, 1.0, 1.0);
}

SplineAStar::~SplineAStar()
{
}

void SplineAStar::setSamples(int samples_x, int samples_y, int samples_z)
{
  samples_x_ = samples_x;
  samples_y_ = samples_y;
  samples_z_ = samples_z;
}

void SplineAStar::setMaxValues(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max)
{
  v_max_ = v_max;
  a_max_ = a_max;
}

void SplineAStar::setGoal(Eigen::Vector3d& goal)
{
  goal_ = goal;
}

std::vector<Node> SplineAStar::expand(Node& current)
{
  std::cout << "expanding" << std::endl;
  std::vector<Node> neighbors;

  if (current.index == (N_ - 2))
  {
    std::cout << "can't expand more in this direction" << std::endl;  // can't expand more
    return neighbors;                                                 // empty vector
  }

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

  // first expansion satisfying vmax and amax
  std::vector<Node> neighbors_va;

  int i = current.index;

  double tmp = (knots_(i + p_ + 1) - knots_(i + 2)) / (1.0 * (p_ - 1));
  double constraint_x = std::min(v_max_.x(), a_max_.x() * tmp);
  double constraint_y = std::min(v_max_.y(), a_max_.y() * tmp);
  double constraint_z = std::min(v_max_.z(), a_max_.z() * tmp);

  /*  std::cout << "constraint_x=" << constraint_x << std::endl;
    std::cout << "constraint_y=" << constraint_y << std::endl;
    std::cout << "constraint_z=" << constraint_z << std::endl;*/

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
  ////

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

  /////////////////////////////////
  for (double vx_i : vx_)
  {
    for (double vy_i : vy_)
    {
      for (double vz_i : vz_)
      {
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
        }
        /*        else
                {
                  std::cout << "Doesn't satisfy the constraints" << std::endl;
                }*/
      }
    }
  }
  std::cout << "neighbors_va.size()= " << neighbors_va.size() << std::endl;

  // if the index is <=6, return what we have //TODO this should be <=2, !!!!

  /*  if (current.index <= 6)
    {*/
  // For now return all the neighbors
  return neighbors_va;
  //}

  // And now select the ones that satisfy the LP with all the obstacles

  std::vector<Eigen::Vector3d> last4Cps(4);
  last4Cps[0] = current.previous->previous->qi;
  last4Cps[1] = current.previous->qi;
  last4Cps[2] = current.qi;

  for (auto neighbor_va : neighbors_va)
  {
    last4Cps[3] = neighbor_va.qi;
    bool satisfies_LP = true;
    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      Eigen::Vector3d n_i;
      double d_i;
      if (separator_solver->solveModel(n_i, d_i, hulls_[obst_index][current.index + 1 - 3], last4Cps) == false)
      {
        satisfies_LP = false;
        break;
      }
    }
    if (satisfies_LP == true)  // this is true also when num_of_obst_=0;
    {
      neighbors.push_back(neighbor_va);
    }
  }
  return neighbors;
}

double SplineAStar::h(Node& node)
{
  return (node.qi - goal_).squaredNorm();
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

  return (node2.qi - 2 * node1.qi + node1.previous->qi).squaredNorm();
}

void SplineAStar::run()
{
  auto cmp = [](Node& left, Node& right) { return (left.h + left.g) > (right.h + right.g); };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> openList(cmp);  //= OpenSet, = Q

  Node nodeq2;
  nodeq2.index = 0;
  nodeq2.previous = NULL;
  nodeq2.g = 0;
  nodeq2.qi = q2_;
  nodeq2.h = h(nodeq2);  // f=g+h
  nodeq2.index = 2;

  openList.push(nodeq2);

  while (openList.size() > 0)
  {
    /*    std::cout << "======================" << std::endl;

        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> novale = openList;

        while (!novale.empty())
        {
          std::cout << (novale.top().h + novale.top().g) << " ";
          novale.pop();
        }*/

    std::cout << "======================" << std::endl;
    std::cout << "sizeOpenList=" << openList.size() << std::endl;
    Node current = openList.top();

    std::cout << "dist2Goal=" << (current.qi - goal_).norm() << std::endl;
    if ((current.qi - goal_).norm() < 0.5)
    {
      std::cout << "Success" << std::endl;
      return;
    }

    openList.pop();  // remove the element

    std::vector<Node> neighbors = expand(current);

    std::cout << "There are " << neighbors.size() << " neighbors" << std::endl;

    for (auto neighbor : neighbors)
    {
      openList.push(neighbor);
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
    }

    // closedList.push_back(current);
  }

  std::cout << "Failure!" << std::endl;
}
