/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <vector>

#include <ros/ros.h>

#include <mader_msgs/DynTraj.h>
#include <Eigen/Dense>
#include "mader_types.hpp"

#include <snapstack_msgs/State.h>
#include <mader_msgs/DynTraj.h>
#include <mader_msgs/mt::PieceWisePolTraj.h>
#include <mader_msgs/CoeffPoly3.h>

using namespace termcolor;

class Predictor
{
public:
  Predictor(ros::NodeHandle nh);

  mt::PieceWisePol predictPwp(std::vector<double>& times, std::vector<Eigen::Vector3d>& last_positions);

private:
  void trajCB(const mader_msgs::DynTraj& msg);
  void stateCB(const snapstack_msgs::State& msg);

  void sample(const mt::dynTrajCompiled& traj_compiled, std::vector<double>& times,
              std::vector<Eigen::Vector3d>& last_positions);

  ros::Publisher pub_predicted_traj_;
  ros::Publisher pub_marker_predicted_traj_;
  state state_;

  ros::NodeHandle nh_;

  ros::Subscriber sub_state_;
  ros::Subscriber sub_traj_;
  double t_;
  double R_local_map_;
  std::string name_drone_;
  int id_;
};

#endif