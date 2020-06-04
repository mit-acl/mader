#include "predictor.hpp"

int main(int argc, char **argv)
{
  std::cout << "Going to initialize predictor node" << std::endl;
  ros::init(argc, argv, "predictor");
  ros::NodeHandle nh("~");

  std::cout << "Creating Predictor object" << std::endl;
  Predictor Predictor(nh);

  while (ros::ok())
  {
    ros::spinOnce();  // spin the normal queue
  }

  ros::waitForShutdown();
  return 0;
}