/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <Eigen/Dense>
#include "octopus_search.hpp"
#include "mader_types.hpp"
#include "utils.hpp"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "decomp_ros_msgs/PolyhedronArray.h"
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros

Eigen::Vector3d getPosDynObstacle(double t)
{
  double t_scaled = 4.3 * t;

  Eigen::Vector3d pos;
  pos << sin(t_scaled) + 2 * sin(2 * t_scaled),  //////////////////////
      cos(t_scaled) - 2 * cos(2 * t_scaled),     //////////////////////
      -sin(3 * t_scaled);                        //////////////////////

  pos.x() = pos.x() / 2.5;
  pos.y() = pos.y() / 2.5;
  pos.z() = pos.z() / 1.5;

  pos.z() = pos.z() + 1.0;

  return pos;
}

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

  CGAL_Polyhedron_3 hull_interval = cu::convexHullOfPoints(points);

  for (int i = 0; i < num_pol; i++)
  {
    hulls_curve.push_back(hull_interval);  // static obstacle
  }

  return hulls_curve;
}

visualization_msgs::Marker getMarker(Eigen::Vector3d& center, double bbox_x, double bbox_y, double bbox_z, double t_min,
                                     double t_final, double t, int id)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::CUBE;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "marker_dyn_obs";
  m.action = visualization_msgs::Marker::ADD;
  m.id = id;
  m.color = mu::getColorJet(t, t_min, t_final);
  m.scale.x = bbox_x;
  m.scale.y = bbox_y;  // rviz complains if not
  m.scale.z = bbox_z;  // rviz complains if not
  m.pose.position.x = center.x();
  m.pose.position.y = center.y();
  m.pose.position.z = center.z();
  m.pose.orientation.w = 1.0;
  return m;

  // visualization_msgs::MarkerArray marker_array;

  // if (data.size() == 0)
  // {
  //   return marker_array;
  // }

  // int j = 9000;
  // for (int i = 0; i < data.size(); i = i + 1)
  // {
  //   visualization_msgs::Marker m;
  //   m.type = visualization_msgs::Marker::CUBE;
  //   m.header.frame_id = "world";
  //   m.header.stamp = ros::Time::now();
  //   m.ns = ns;
  //   m.action = visualization_msgs::Marker::ADD;
  //   m.id = j;
  //   m.color = mu::getColorJet(t, t_min, t_final);
  //   m.scale.x = bbox_x;
  //   m.scale.y = bbox_y;  // rviz complains if not
  //   m.scale.z = bbox_z;  // rviz complains if not
  //   m.pose.position.x = data[i].x();
  //   m.pose.position.y = data[i].y();
  //   m.pose.position.z = data[i].z();
  //   m.pose.orientation.w = 1.0;

  //   marker_array.markers.push_back(m);
  //   j = j + 1;
  // }
  // return marker_array;
}

ConvexHullsOfCurve createDynamicObstacle(std::vector<visualization_msgs::MarkerArray>& ma_vector, double x, double y,
                                         double z, int num_pol, double bbox_x, double bbox_y, double bbox_z,
                                         double t_min, double t_max)
{
  double time_per_interval = (t_max - t_min) / num_pol;
  double time_discretization = 0.01;

  ConvexHullsOfCurve hulls_curve;

  ma_vector.clear();
  // ma.markers.clear();

  int j = 0;
  for (int interval_index = 0; interval_index < num_pol; interval_index++)
  {
    std::vector<Point_3> points_interval;

    visualization_msgs::MarkerArray ma;
    std::cout << "--------------Interval " << interval_index << std::endl;
    for (double t = interval_index * time_per_interval; t < (interval_index + 1) * time_per_interval;
         t = t + time_discretization)
    {
      Eigen::Vector3d current_pos = getPosDynObstacle(t);
      x = current_pos.x();
      y = current_pos.y();
      z = current_pos.z();

      double bbox_x_infl = bbox_x + 0.03;
      double bbox_y_infl = bbox_y + 0.03;
      double bbox_z_infl = bbox_z + 0.03;

      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x - bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));

      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y + bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z + bbox_z_infl / 2.0));
      points_interval.push_back(Point_3(x + bbox_x_infl / 2.0, y - bbox_y_infl / 2.0, z - bbox_z_infl / 2.0));

      ma.markers.push_back(getMarker(current_pos, bbox_x, bbox_y, bbox_z, t_min, t_max, t, j));
      j = j + 1;
    }

    ma_vector.push_back(ma);

    CGAL_Polyhedron_3 hull_interval = cu::convexHullOfPoints(points_interval);
    hulls_curve.push_back(hull_interval);
  }
  return hulls_curve;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testOctopusSearch");
  ros::NodeHandle nh("~");
  ros::Publisher trajectories_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/trajectories_found", 1000, true);

  ros::Publisher best_trajectory_found_pub =
      nh.advertise<visualization_msgs::MarkerArray>("/best_trajectory_found", 1000, true);
  ros::Publisher convex_hulls_pub = nh.advertise<visualization_msgs::Marker>("/convex_hulls", 1, true);

  std::vector<ros::Publisher> jps_poly_pubs;  // = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_jps", 1, true);
  std::vector<ros::Publisher> traj_obstacle_colored_pubs;
  std::vector<ros::Publisher> best_trajectory_found_intervals_pubs;

  int num_pol = 7;
  int deg_pol = 3;

  for (int i = 0; i < num_pol; i++)
  {
    ros::Publisher tmp =
        nh.advertise<visualization_msgs::MarkerArray>("/traj_obstacle_colored_int_" + std::to_string(i), 1, true);
    traj_obstacle_colored_pubs.push_back(tmp);

    ros::Publisher tmp2 = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/poly_jps_int_" + std::to_string(i), 1, true);
    jps_poly_pubs.push_back(tmp2);

    ros::Publisher tmp3 =
        nh.advertise<visualization_msgs::MarkerArray>("/best_trajectory_found_int_" + std::to_string(i), 1, true);
    best_trajectory_found_intervals_pubs.push_back(tmp3);
  }

  // ros::Publisher traj_obstacle_colored_int0_pub =
  //     nh.advertise<visualization_msgs::MarkerArray>("traj_obstacle_colored_int0", 1, true);

  std::string basis;

  nh.getParam("basis", basis);

  std::cout << "Basis= " << basis << std::endl;

  int samples_x = 5;  // odd number
  int samples_y = 5;  // odd number
  int samples_z = 5;  // odd number

  double alpha_shrink = 0.9;

  double fraction_voxel_size = 0.0;  // grid used to prune nodes that are on the same cell

  double runtime = 0.1;    //[seconds]
  double goal_size = 0.1;  //[meters]

  Eigen::Vector3d v_max(7.0, 7.0, 7.0);
  Eigen::Vector3d a_max(400000.0, 4000000.0, 4000000.0);

  Eigen::Vector3d q0(-3, 0.5, 1);
  Eigen::Vector3d q1 = q0;
  Eigen::Vector3d q2 = q1;
  Eigen::Vector3d goal(5.0, 0, 1);

  double t_min = 0.0;
  double t_max = t_min + (goal - q0).norm() / (0.8 * v_max(0));

  std::cout << "t_min= " << t_min << std::endl;
  std::cout << "t_max= " << t_max << std::endl;
  std::cout << "=========================" << std::endl;

  ConvexHullsOfCurves hulls_curves;

  double bbox_x = 0.4;
  double bbox_y = 0.4;
  double bbox_z = 0.4;

  double dc = 0.002;  // Simply used for visualization

  int num_of_obs = 1;  // odd number
  double separation = 0.4;

  int num_of_obs_up = (num_of_obs - 1) / 2.0;

  std::vector<visualization_msgs::MarkerArray> ma_vector;

  // ConvexHullsOfCurve hulls_curve = createStaticObstacle(0.0, 0.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z);
  ConvexHullsOfCurve hulls_curve =
      createDynamicObstacle(ma_vector, 0.0, 0.0, bbox_z / 2.0, num_pol, bbox_x, bbox_y, bbox_z, t_min, t_max);
  hulls_curves.push_back(hulls_curve);  // only one obstacle

  for (int i = 1; i <= num_of_obs_up; i++)
  {
    ConvexHullsOfCurve hulls_curve =
        createStaticObstacle(0.0, i * (bbox_y + separation), 0.0, num_pol, bbox_x, bbox_y, bbox_z);
    hulls_curves.push_back(hulls_curve);  // only one obstacle

    hulls_curve = createStaticObstacle(0.0, -i * (bbox_y + separation), 0.0, num_pol, bbox_x, bbox_y, bbox_z);
    hulls_curves.push_back(hulls_curve);  // only one obstacle
  }

  for (int i = 0; i < ma_vector.size(); i++)
  {
    traj_obstacle_colored_pubs[i].publish(ma_vector[i]);
  }

  std::cout << "hulls_curves.size()= " << hulls_curves.size() << std::endl;

  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls_curves);
  // vec_E<Polyhedron<3>> jps_poly = cu::vectorGCALPol2vectorJPSPol(hulls_curves);

  for (int i = 0; i < num_pol; i++)
  {
    ConvexHullsOfCurve tmp2;
    ConvexHullsOfCurves tmp;

    tmp2.push_back(hulls_curve[i]);
    tmp.push_back(tmp2);

    // convert the obstacles polyhedron arrays
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(cu::vectorGCALPol2vectorJPSPol(tmp));
    poly_msg.header.frame_id = "world";
    jps_poly_pubs[i].publish(poly_msg);
  }

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

  OctopusSearch myAStarSolver(basis, num_pol, deg_pol, alpha_shrink);
  myAStarSolver.setUp(t_min, t_max, hulls_std);

  myAStarSolver.setq0q1q2(q0, q1, q2);
  myAStarSolver.setGoal(goal);

  myAStarSolver.setXYZMinMaxAndRa(-1e6, 1e6, -1e6, 1e6, -1.0, 10.0, 1e6);  // limits for the search, in world frame
  myAStarSolver.setBBoxSearch(30.0, 30.0, 30.0);                           // limits for the search, centered on q2
  myAStarSolver.setMaxValuesAndSamples(v_max, a_max, samples_x, samples_y, samples_z, fraction_voxel_size);

  myAStarSolver.setRunTime(runtime);
  myAStarSolver.setGoalSize(goal_size);

  myAStarSolver.setBias(1.0);

  myAStarSolver.setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool is_stuck;
  bool is_q0_fail;

  bool solved = myAStarSolver.run(q, n, d, is_stuck, is_q0_fail);

  if (is_stuck){
    std::cout << "drones are stuck" << std::endl;
  }

  // Recover all the trajectories found and the best trajectory
  std::vector<mt::trajectory> all_trajs_found;
  myAStarSolver.getAllTrajsFound(all_trajs_found);

  mt::trajectory best_traj_found;
  mt::PieceWisePol pwp_best_traj_found;
  myAStarSolver.getBestTrajFound(best_traj_found, pwp_best_traj_found, dc);

  // for (double t = t_min; t <= t_max; t = t + dc)
  // {
  //   Eigen::Vector3d pos = pwp_best_traj_found.eval(t);
  //   std::cout << "At t= " << t << " pos=" << pos.transpose() << " pos_obstacle=" << getPosDynObstacle(t).transpose()
  //             << std::endl;
  // }

  // Convert to marker arrays
  //---> all the trajectories found
  int increm = 2;
  int increm_best = 1;
  double scale = 0.02;
  int j = 0;

  visualization_msgs::MarkerArray marker_array_all_trajs;
  for (auto traj : all_trajs_found)
  {
    visualization_msgs::MarkerArray marker_array_traj = mu::trajectory2ColoredMarkerArray(
        traj, v_max.maxCoeff(), increm, "traj" + std::to_string(j), scale, "time", 0, 1);
    // std::cout << "size of marker_array_traj= " << marker_array_traj.markers.size() << std::endl;
    for (auto marker : marker_array_traj.markers)
    {
      marker_array_all_trajs.markers.push_back(marker);
    }
    j++;
  }

  //---> the best trajectory found
  scale = 0.1;
  visualization_msgs::MarkerArray marker_array_best_traj;
  marker_array_best_traj = mu::trajectory2ColoredMarkerArray(best_traj_found, v_max.maxCoeff(), increm_best,
                                                             "traj" + std::to_string(j), scale, "time", 0, 1);

  double entries_per_interval = marker_array_best_traj.markers.size() / num_pol;
  for (int i = 0; i < num_pol; i++)
  {
    std::vector<visualization_msgs::Marker> tmp(                                 /////////
        marker_array_best_traj.markers.begin() + i * entries_per_interval,       /////////
        marker_array_best_traj.markers.begin() + (i + 1) * entries_per_interval  /////////
    );

    visualization_msgs::MarkerArray ma;
    ma.markers = tmp;
    best_trajectory_found_intervals_pubs[i].publish(ma);
  }

  best_trajectory_found_pub.publish(marker_array_best_traj);

  // Get the edges of the convex hulls and publish them
  mt::Edges edges_convex_hulls;
  myAStarSolver.getEdgesConvexHulls(edges_convex_hulls);
  convex_hulls_pub.publish(mu::edges2Marker(edges_convex_hulls, mu::color(mu::red_normal)));

  // publish the trajectories
  trajectories_found_pub.publish(marker_array_all_trajs);

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