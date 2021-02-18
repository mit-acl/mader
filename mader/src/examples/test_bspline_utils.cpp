/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "bspline_utils.hpp"

int main()
{
  std::vector<double> times;
  // times.push_back(0.0);
  // times.push_back(0.5);
  // times.push_back(0.7);
  // times.push_back(1.0);

  times.push_back(1591133329.99584);
  times.push_back(1591133330.29584);
  times.push_back(1591133330.59584);
  times.push_back(1591133330.89584);
  times.push_back(1591133331.19584);
  times.push_back(1591133331.49585);
  times.push_back(1591133331.79585);

  std::vector<Eigen::Vector3d> positions;
  // positions.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  // positions.push_back(Eigen::Vector3d(5.0, 0.0, 0.0));
  // positions.push_back(Eigen::Vector3d(7.0, 0.0, 0.0));
  // positions.push_back(Eigen::Vector3d(7.5, 0.0, 0.0));

  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));
  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));
  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));
  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));
  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));
  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));
  positions.push_back(Eigen::Vector3d(1.42555006072, -0.535729530809, 1));

  Eigen::Spline3d spline = findInterpolatingBsplineNormalized(times, positions);
  std::cout << "Control points are " << spline.ctrls() << std::endl;
  std::cout << "Knots are " << spline.knots() << std::endl;
}