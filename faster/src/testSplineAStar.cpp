#include <Eigen/Dense>
#include "spline_AStar.hpp"
#include "faster_types.hpp"
#include "utils.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "decomp_ros_msgs/PolyhedronArray.h"
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros

ConvexHullsOfCurve createStaticObstacle(double x, double y, double z, int num_pol, double bbox_x, double bbox_y,
                                        double bbox_z)
{
  ConvexHullsOfCurve hulls_curve;
  std::vector<Point_3> points;

  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x - bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));

  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y + bbox_y / 2.0, z - bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z + bbox_z / 2.0));
  points.push_back(Point_3(x + bbox_x / 2.0, y - bbox_y / 2.0, z - bbox_z / 2.0));

  CGAL_Polyhedron_3 hull_interval = convexHullOfPoints(points);

  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull_interval);  // static obstacle
  }

  return hulls_curve;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testSplineAStar");
  ros::NodeHandle nh("~");
  ros::Publisher trajectories_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("A_star_trajectories_found", 1000, true);

  ros::Publisher best_trajectory_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("A_star_best_trajectory_found", 1000, true);

  ros::Publisher jps_poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_jps", 1, true);

  ros::Publisher convex_hulls_pub = nh.advertise<visualization_msgs::Marker>("convex_hulls", 1, true);

  std::string basis;

  nh.getParam("basis", basis);

  std::cout << "Basis= " << basis << std::endl;

  int num_pol = 7;
  int deg_pol = 3;

  int samples_x = 3;  // odd number
  int samples_y = 3;  // odd number
  int samples_z = 3;  // odd number

  double fraction_voxel_size = 0.5;  // grid used to prune nodes that are on the same cell

  double runtime = 0.01;   //[seconds]
  double goal_size = 0.1;  //[meters]

  Eigen::Vector3d v_max(7.0, 7.0, 7.0);
  Eigen::Vector3d a_max(400000.0, 4000000.0, 4000000.0);

  Eigen::Vector3d q0(-1.5, 0, 0);
  Eigen::Vector3d q1 = q0;
  Eigen::Vector3d q2 = q1;
  Eigen::Vector3d goal(2.0, 0, 0);

  double t_min = 0.0;
  double t_max = t_min + (goal - q0).norm() / (0.6 * v_max(0));

  ConvexHullsOfCurves hulls_curves;

  double bbox_x = 1.0;
  double bbox_y = 1.0;
  double bbox_z = 6.0;

  int num_of_obs = 1;  // odd number
  double separation = 0.4;

  int num_of_obs_up = (num_of_obs - 1) / 2.0;

  ConvexHullsOfCurve hulls_curve = createStaticObstacle(0.0, 0.0, 0.0, num_pol, bbox_x, bbox_y, bbox_z);
  hulls_curves.push_back(hulls_curve);  // only one obstacle

  for (int i = 1; i <= num_of_obs_up; i++)
  {
    ConvexHullsOfCurve hulls_curve =
        createStaticObstacle(0.0, i * (bbox_y + separation), 0.0, num_pol, bbox_x, bbox_y, bbox_z);
    hulls_curves.push_back(hulls_curve);  // only one obstacle

    hulls_curve = createStaticObstacle(0.0, -i * (bbox_y + separation), 0.0, num_pol, bbox_x, bbox_y, bbox_z);
    hulls_curves.push_back(hulls_curve);  // only one obstacle
  }

  ConvexHullsOfCurves_Std hulls_std = vectorGCALPol2vectorStdEigen(hulls_curves);
  vec_E<Polyhedron<3>> jps_poly = vectorGCALPol2vectorJPSPol(hulls_curves);

  // hull.push_back(Eigen::Vector3d(-1.0, -1.0, -700.0));
  // hull.push_back(Eigen::Vector3d(-1.0, -1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(-1.0, 1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(-1.0, 1.0, -700.0));

  // hull.push_back(Eigen::Vector3d(1.0, 1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(1.0, 1.0, -700.0));
  // hull.push_back(Eigen::Vector3d(1.0, -1.0, 700.0));
  // hull.push_back(Eigen::Vector3d(1.0, -1.0, -700.0));

  // Assummes static obstacle
  /*  for (int i = 0; i < num_pol; i++)
    {
      hulls_curve.push_back(hull);
    }

    hulls_curves.push_back(hulls_curve);*/

  SplineAStar myAStarSolver(num_pol, deg_pol, hulls_std.size(), t_min, t_max, hulls_std);

  myAStarSolver.setq0q1q2(q0, q1, q2);
  myAStarSolver.setGoal(goal);

  myAStarSolver.setZminZmax(-1.0, 10.0);          // z limits for the search, in world frame
  myAStarSolver.setBBoxSearch(30.0, 30.0, 30.0);  // limits for the search, centered on q2
  myAStarSolver.setMaxValuesAndSamples(v_max, a_max, samples_x, samples_y, samples_z, fraction_voxel_size);

  myAStarSolver.setRunTime(runtime);
  myAStarSolver.setGoalSize(goal_size);

  myAStarSolver.setBias(2.0);
  if (basis == "MINVO")
  {
    myAStarSolver.setBasisUsedForCollision(myAStarSolver.MINVO);  // MINVO //B_SPLINE
  }
  else
  {
    myAStarSolver.setBasisUsedForCollision(myAStarSolver.B_SPLINE);  // MINVO //B_SPLINE
  }
  myAStarSolver.setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool solved = myAStarSolver.run(q, n, d);

  // Recover all the trajectories found and the best trajectory
  std::vector<trajectory> all_trajs_found;
  myAStarSolver.getAllTrajsFound(all_trajs_found);

  trajectory best_traj_found;
  myAStarSolver.getBestTrajFound(best_traj_found);

  // Convert to marker arrays
  //---> all the trajectories found
  int increm = 1;
  int type = 6;
  double scale = 0.01;
  int j = 0;

  visualization_msgs::MarkerArray marker_array_all_trajs;
  for (auto traj : all_trajs_found)
  {
    visualization_msgs::MarkerArray marker_array_traj =
        trajectory2ColoredMarkerArray(traj, type, v_max.maxCoeff(), increm, "traj" + std::to_string(j), scale);
    // std::cout << "size of marker_array_traj= " << marker_array_traj.markers.size() << std::endl;
    for (auto marker : marker_array_traj.markers)
    {
      marker_array_all_trajs.markers.push_back(marker);
    }
    j++;
    type++;
  }

  //---> the best trajectory found
  scale = 0.1;
  visualization_msgs::MarkerArray marker_array_best_traj;
  marker_array_best_traj =
      trajectory2ColoredMarkerArray(best_traj_found, type, v_max.maxCoeff(), increm, "traj" + std::to_string(j), scale);

  // Get the edges of the convex hulls and publish them
  faster_types::Edges edges_convex_hulls;
  myAStarSolver.getEdgesConvexHulls(edges_convex_hulls);
  convex_hulls_pub.publish(edges2Marker(edges_convex_hulls, color(RED_NORMAL)));

  // publish the trajectories
  trajectories_found_pub.publish(marker_array_all_trajs);
  best_trajectory_found_pub.publish(marker_array_best_traj);

  // convert the obstacles polyhedron arrays
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(jps_poly);
  poly_msg.header.frame_id = "world";
  jps_poly_pub.publish(poly_msg);

  ros::spinOnce();

  /*
    vectorOfNodes2vectorOfStates()

        traj_committed_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max, increm, name_drone_);
    pub_traj_committed_colored_.publish(traj_committed_colored_);*/

  // if (solved == true)
  // {
  //   std::cout << "This is the result" << std::endl;
  //   for (auto qi : q)
  //   {
  //     std::cout << qi.transpose() << std::endl;
  //   }
  // }
  // else
  // {
  //   std::cout << "A* didn't find a solution" << std::endl;
  // }

  // std::cout << "Normal Vectors: " << std::endl;
  // for (auto ni : n)
  // {
  //   std::cout << ni.transpose() << std::endl;
  // }

  // std::cout << "D coefficients: " << std::endl;
  // for (auto di : d)
  // {
  //   std::cout << di << std::endl;
  // }

  ros::spin();

  return 0;
}