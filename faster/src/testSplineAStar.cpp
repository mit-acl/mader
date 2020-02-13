#include <Eigen/Dense>
#include "spline_AStar.hpp"
#include "faster_types.hpp"

int main()
{
  /*typedef std::vector<Eigen::Vector3d> Polyhedron_Std;
  typedef std::vector<Polyhedron_Std> ConvexHullsOfCurve_Std;
  typedef std::vector<ConvexHullsOfCurve_Std> ConvexHullsOfCurves_Std;
  */

  int p = 3;
  int N = 11;
  int num_of_obst = 1;
  int n_pol = 7;

  int samples_x = 15;
  int samples_y = 15;
  int samples_z = 15;

  Eigen::Vector3d v_max(20.0, 20.0, 20.0);
  Eigen::Vector3d a_max(100.0, 100.0, 100.0);

  Eigen::Vector3d q0(-4.0, 0.0, 0.0);
  Eigen::Vector3d q1 = q0;
  Eigen::Vector3d q2 = q1;
  Eigen::Vector3d goal(4.0, 0.0, 0.0);

  Eigen::Matrix<double, 1, 14> knots;
  knots << 0, 0, 0, 0, 0.143, 0.286, 0.429, 0.571, 0.714, 0.857, 1, 1, 1, 1;

  ConvexHullsOfCurves_Std hulls_curves;
  ConvexHullsOfCurve_Std hulls_curve;
  Polyhedron_Std hull;

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, -0.5));

  hull.push_back(Eigen::Vector3d(-0.5, 0.5, 0.5));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, 0.5));
  hull.push_back(Eigen::Vector3d(0.5, 0.5, -0.5));

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, 0.5));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, -0.5));
  hull.push_back(Eigen::Vector3d(-0.5, 0.5, -0.5));

  hull.push_back(Eigen::Vector3d(0.5, 0.5, 0.5));

  // Assummes static obstacle
  for (int i = 0; i < n_pol; i++)
  {
    hulls_curve.push_back(hull);
  }

  hulls_curves.push_back(hulls_curve);

  SplineAStar myAStarSolver(p, N, num_of_obst, knots, hulls_curves);
  myAStarSolver.setSamples(samples_x, samples_y, samples_z);
  myAStarSolver.setMaxValues(v_max, a_max);  // should go after setSamples
  myAStarSolver.setq0q1q2(q0, q1, q2);
  myAStarSolver.setGoal(goal);
  std::vector<Eigen::Vector3d> result;
  myAStarSolver.run(result);

  std::cout << "This is the result" << std::endl;
  for (auto qi : result)
  {
    std::cout << qi.transpose() << std::endl;
  }

  return 0;
}