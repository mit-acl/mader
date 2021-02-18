/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "mader_ros.hpp"
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mader");

  ros::NodeHandle nh1("~");
  ros::NodeHandle nh2("~");
  ros::NodeHandle nh3("~");

  // Concurrency and parallelism in ROS:
  // https://nicolovaligi.com/concurrency-and-parallelism-in-ros1-and-ros2-application-apis.html

  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  ros::CallbackQueue custom_queue3;

  nh1.setCallbackQueue(&custom_queue1);
  nh2.setCallbackQueue(&custom_queue2);
  nh3.setCallbackQueue(&custom_queue3);

  MaderRos MaderRos(nh1, nh2, nh3);

  ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner3(1, &custom_queue3);  // 1 thread for the custom_queue3 // 0 means threads= # of CPU cores

  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2
  spinner3.start();  // start spinner of the custom queue 3

  ros::waitForShutdown();
  return 0;
}