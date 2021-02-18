/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "predictor.hpp"
#include "termcolor.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "predictor");
  ros::NodeHandle nh("~");

  Predictor pred(nh);

  std::vector<double> times;

  times.push_back(0.0);
  times.push_back(1.0);
  times.push_back(2.0);
  times.push_back(3.0);
  times.push_back(4.0);

  // times.push_back(1591218822.82879);
  // times.push_back(1591218823.12879);
  // times.push_back(1591218823.42879);
  // times.push_back(1591218823.72879);
  // times.push_back(1591218824.02879);
  // times.push_back(1591218824.32879);
  // times.push_back(1591218824.62879);

  // times.push_back(1591218822.82879 - 1591218822.82879);
  // times.push_back(1591218823.12879 - 1591218822.82879);
  // times.push_back(1591218823.42879 - 1591218822.82879);
  // times.push_back(1591218823.72879 - 1591218822.82879);
  // times.push_back(1591218824.02879 - 1591218822.82879);
  // times.push_back(1591218824.32879 - 1591218822.82879);
  // times.push_back(1591218824.62879 - 1591218822.82879);

  std::vector<Eigen::Vector3d> positions;

  positions.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
  positions.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  positions.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
  positions.push_back(Eigen::Vector3d(3.0, 0.0, 0.0));
  positions.push_back(Eigen::Vector3d(4.0, 0.0, 0.0));

  // positions.push_back(Eigen::Vector3d(-1.07839598244821, 1.84147040351636, 0.592203980323135));
  // positions.push_back(Eigen::Vector3d(-2.20821019206822, 1.58267765401245, 1.38770641869342));
  // positions.push_back(Eigen::Vector3d(-2.88615123725143, 0.927505883856573, 1.93782979917839));
  // positions.push_back(Eigen::Vector3d(-2.93682321060551, 0.120889127242631, 1.89439940257442));
  // positions.push_back(Eigen::Vector3d(-2.35132341916533, -0.544558509987802, 1.28490238696429));
  // positions.push_back(Eigen::Vector3d(-1.28940191662426, -0.817790982093464, 0.495090256572718));
  // positions.push_back(Eigen::Vector3d(-0.0338476351997402, -0.566472456216861, 0.0248364124030608));

  mt::PieceWisePol pwp = pred.predictPwp(times, positions);

  std::cout << green << "pwp obtained is " << reset << std::endl;
  pwp.print();

  std::cout << "__________________" << std::endl;
  for (double t = times[times.size() - 1]; t < (times[times.size() - 1] + 10); t = t + 0.5)
  {
    std::cout << "At t= " << std::setprecision(15) << t << reset << ", it gives " << pwp.eval(t).transpose()
              << std::endl;
  }
}