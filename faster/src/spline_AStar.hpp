
#include <vector>
#include <Eigen/Dense>
#include "faster_types.hpp"
#include "separator.hpp"

typedef struct Node Node;  // neede to be able to have a pointer inside the struct

struct Node
{
  Eigen::Vector3d qi;
  Node* previous = NULL;
  double g = 0;
  double h = 0;
  int index = 2;  // Start with q2_
};

class SplineAStar
{
public:
  SplineAStar(int p, int N, int num_of_obst, const Eigen::MatrixXd knots, const ConvexHullsOfCurves_Std hulls);
  ~SplineAStar();

  void setMaxValues(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max);
  void setq0q1q2(Eigen::Vector3d& q0, Eigen::Vector3d& q1, Eigen::Vector3d& q2);
  void setGoal(Eigen::Vector3d& goal);
  void setSamples(int samples_x, int samples_y, int samples_z);

  void run();

protected:
private:
  std::vector<Node> expand(Node& current);
  double h(Node& node);
  double g(Node& node);
  double weightEdge(Node& node1, Node& node2);

  Eigen::Vector3d goal_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;
  Eigen::MatrixXd knots_;
  ConvexHullsOfCurves_Std hulls_;
  separator::Separator* separator_solver;

  std::vector<double> vx_;
  std::vector<double> vy_;
  std::vector<double> vz_;

  int samples_x_ = 3;
  int samples_y_ = 3;
  int samples_z_ = 3;

  int p_;
  int N_;
  int num_of_obst_;

  Eigen::Vector3d q0_;
  Eigen::Vector3d q1_;
  Eigen::Vector3d q2_;
};
