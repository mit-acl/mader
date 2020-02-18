
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
  SplineAStar(int num_pol, int deg_pol, int num_obst, double t_min, double t_max, const ConvexHullsOfCurves_Std& hulls);
  ~SplineAStar();

  void setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int samples_x, int samples_y,
                              int samples_z);
  void setq0q1q2(Eigen::Vector3d& q0, Eigen::Vector3d& q1, Eigen::Vector3d& q2);
  void setGoal(Eigen::Vector3d& goal);
  void setSamples(int samples_x, int samples_y, int samples_z);

  void setRunTime(double max_runtime);
  void setGoalSize(double goal_size);

  void setBias(double bias);

  bool run(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d);

  void recoverPath(Node* node1_ptr, std::vector<Eigen::Vector3d>& result);

  void computeInverses();

  void setBBoxSearch(double x, double y, double z);

  void setVisual(bool visual);

protected:
private:
  void plotExpandedNodesAndResult(std::vector<Node>& expanded_nodes, Node* result_ptr);
  std::vector<Node> expand(Node& current);
  void printPath(Node& node1);
  double h(Node& node);
  double g(Node& node);
  double weightEdge(Node& node1, Node& node2);

  void fillNDFromNode(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d);

  Eigen::Vector3d goal_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;

  ConvexHullsOfCurves_Std hulls_;
  separator::Separator* separator_solver_;

  std::vector<double> vx_;
  std::vector<double> vy_;
  std::vector<double> vz_;

  int samples_x_ = 3;
  int samples_y_ = 3;
  int samples_z_ = 3;

  int p_;
  int N_;
  int M_;
  int num_of_obst_;
  int num_of_segments_;
  int num_of_normals_;

  Eigen::Vector3d orig_;  // origin of the search box (left bottom corner)

  Eigen::Vector3d q0_;
  Eigen::Vector3d q1_;
  Eigen::Vector3d q2_;

  Eigen::RowVectorXd knots_;

  double closest_dist_so_far_ = std::numeric_limits<double>::max();
  Node* closest_result_so_far_ptr_ = NULL;

  double goal_size_ = 0.5;    //[m]
  double max_runtime_ = 0.5;  //[s]

  // bias should be >=1.0
  double bias_ = 1.0;  // page 34 of https://www.cs.cmu.edu/~motionplanning/lecture/Asearch_v8.pdf

  std::vector<Eigen::MatrixXd> Ainverses_;

  double time_solving_lps_ = 0.0;
  double time_expanding_ = 0.0;

  double increment_;

  std::vector<Node> expanded_nodes_;

  std::vector<std::vector<std::vector<bool>>> matrixExpandedNodes_;

  double bbox_x_;
  double bbox_y_;
  double bbox_z_;

  bool visual_ = false;

  // bool matrixExpandedNodes_[40][40][40];
};
