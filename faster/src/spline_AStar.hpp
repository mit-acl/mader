
#include <vector>
#include <Eigen/Dense>
#include "faster_types.hpp"

#include "separator.hpp"
#include <unordered_map>

#include "cgal_utils.hpp"

#include <tuple>

typedef struct Node Node;  // needed to be able to have a pointer inside the struct

struct Node
{
  Eigen::Vector3d qi;
  Node* previous = NULL;
  double g = 0;
  double h = 0;
  int index = 2;  // Start with q2_
};

// Taken from https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const& matrix) const
  {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class SplineAStar
{
public:
  SplineAStar(int num_pol, int deg_pol, int num_obst, double t_min, double t_max, const ConvexHullsOfCurves_Std& hulls);
  ~SplineAStar();

  void setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int num_samples_x, int num_samples_y,
                              int num_samples_z, double fraction_voxel_size);
  void setq0q1q2(Eigen::Vector3d& q0, Eigen::Vector3d& q1, Eigen::Vector3d& q2);
  void setGoal(Eigen::Vector3d& goal);
  void setSamples(int num_samples_x, int num_samples_y, int num_samples_z);

  void setRunTime(double max_runtime);
  void setGoalSize(double goal_size);

  void setBasisUsedForCollision(int basis);

  void setBias(double bias);

  bool run(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n, std::vector<double>& d);

  void recoverPath(Node* node1_ptr, std::vector<Eigen::Vector3d>& result);

  void getAllTrajsFound(std::vector<trajectory>& all_trajs_found);

  void computeInverses();

  void setBBoxSearch(double x, double y, double z);

  void setVisual(bool visual);

  void setZminZmax(double z_min, double z_max);

  int getNumOfLPsRun();

  int B_SPLINE = 1;  // B-Spline Basis
  int MINVO = 2;     // Minimum volume basis

protected:
private:
  void transformBSpline2Minvo(std::vector<Eigen::Vector3d>& last4Cps);
  void transformMinvo2BSpline(std::vector<Eigen::Vector3d>& last4Cps);

  void computeLimitsVoxelSize(double& min_voxel_size, double& max_voxel_size);
  void computeUpperAndLowerConstraints(const int i, const Eigen::Vector3d& qiM1, const Eigen::Vector3d& qi,
                                       double& constraint_xL, double& constraint_xU, double& constraint_yL,
                                       double& constraint_yU, double& constraint_zL, double& constraint_zU);

  void plotExpandedNodesAndResult(std::vector<Node>& expanded_nodes, Node* result_ptr);
  void expand(Node& current, std::vector<Node>& neighbors);
  void printPath(Node& node1);
  double h(Node& node);
  double g(Node& node);
  double weightEdge(Node& node1, Node& node2);

  bool checkFeasAndFillND(std::vector<Eigen::Vector3d>& result, std::vector<Eigen::Vector3d>& n,
                          std::vector<double>& d);

  int basis_ = B_SPLINE;

  Eigen::Vector3d goal_;
  Eigen::Vector3d v_max_;
  Eigen::Vector3d a_max_;

  ConvexHullsOfCurves_Std hulls_;
  separator::Separator* separator_solver_;

  std::vector<double> vx_;
  std::vector<double> vy_;
  std::vector<double> vz_;

  int num_samples_x_ = 3;
  int num_samples_y_ = 3;
  int num_samples_z_ = 3;

  std::vector<int> indexes_samples_x_;
  std::vector<int> indexes_samples_y_;
  std::vector<int> indexes_samples_z_;

  std::vector<std::tuple<int, int, int>> all_combinations_;

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

  double voxel_size_;

  std::vector<Node> expanded_nodes_;

  // std::vector<std::vector<std::vector<bool>>> matrixExpandedNodes_;

  std::unordered_map<Eigen::Vector3i, double, matrix_hash<Eigen::Vector3i>> mapExpandedNodes_;

  double bbox_x_;
  double bbox_y_;
  double bbox_z_;

  bool visual_ = false;

  double z_min_ = std::numeric_limits<double>::min();
  double z_max_ = std::numeric_limits<double>::max();

  int num_of_LPs_run_ = 0;

  // transformation between the B-spline control points and the optimal volume control points
  Eigen::Matrix<double, 4, 4> Mbs2ov_;
  Eigen::Matrix<double, 4, 4> Mbs2ov_inverse_;

  Eigen::Vector3d epsilons_;

  int num_pol_;

  // bool matrixExpandedNodes_[40][40][40];
};
