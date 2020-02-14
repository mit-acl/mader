#include <Eigen/Dense>
#include "spline_AStar.hpp"
#include "faster_types.hpp"

int main()
{
  int n_pol = 7;
  int deg_pol = 3;

  int samples_x = 5;
  int samples_y = 5;
  int samples_z = 3;

  double runtime = 4;      //[seconds]
  double goal_size = 0.5;  //[meters]

  Eigen::Vector3d v_max(10.0, 10.0, 10.0);
  Eigen::Vector3d a_max(20.0, 20.0, 20.0);

  Eigen::Vector3d q0(-1.0, 0.0, 0.0);
  Eigen::Vector3d q1 = q0;
  Eigen::Vector3d q2 = q1;
  Eigen::Vector3d goal(1.0, 0.0, 0.0);

  double t_min = 0.0;
  double t_max = t_min + (goal - q0).norm() / (0.5 * v_max(0));

  ConvexHullsOfCurves_Std hulls_curves;
  ConvexHullsOfCurve_Std hulls_curve;
  Polyhedron_Std hull;

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, -70.0));

  hull.push_back(Eigen::Vector3d(-0.5, 0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, 0.5, -70.0));

  hull.push_back(Eigen::Vector3d(-0.5, -0.5, 70.0));
  hull.push_back(Eigen::Vector3d(0.5, -0.5, -70.0));
  hull.push_back(Eigen::Vector3d(-0.5, 0.5, -70.0));

  hull.push_back(Eigen::Vector3d(0.5, 0.5, 70.0));

  // Assummes static obstacle
  for (int i = 0; i < n_pol; i++)
  {
    hulls_curve.push_back(hull);
  }

  hulls_curves.push_back(hulls_curve);

  SplineAStar myAStarSolver(n_pol, deg_pol, hulls_curves.size(), t_min, t_max, hulls_curves);

  myAStarSolver.setMaxValuesAndSamples(v_max, a_max, samples_x, samples_y, samples_z);
  myAStarSolver.setq0q1q2(q0, q1, q2);
  myAStarSolver.setGoal(goal);

  myAStarSolver.setRunTime(runtime);
  myAStarSolver.setGoalSize(goal_size);

  myAStarSolver.setBias(1.0);

  std::vector<Eigen::Vector3d> result;
  bool solved = myAStarSolver.run(result);

  if (solved == true)
  {
    std::cout << "This is the result" << std::endl;
    for (auto qi : result)
    {
      std::cout << qi.transpose() << std::endl;
    }
  }
  else
  {
    std::cout << "A* didn't find a solution" << std::endl;
  }

  return 0;
}