/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "bspline_utils.hpp"

// Given the control points, this function returns the associated traj and mt::PieceWisePol
// Note that if q.size()!=(N+1), then only some of the knots are used
void CPs2TrajAndPwp(std::vector<Eigen::Vector3d> &q, std::vector<mt::state> &traj, mt::PieceWisePol &solution, int N,
                    int p, int num_pol, Eigen::RowVectorXd &knots, double dc)
{
  // std::cout << "q.size()= " << q.size() << std::endl;
  // std::cout << "N= " << N << std::endl;
  // std::cout << "p= " << p << std::endl;
  // std::cout << "knots.size()= " << knots.size() << std::endl;

  // std::cout << "knots= " << knots << std::endl;

  // std::cout << "q= " << std::endl;

  // for (auto q_i : q)
  // {
  //   std::cout << q_i.transpose() << std::endl;
  // }

  int N_effective = q.size() - 1;
  Eigen::RowVectorXd knots_effective = knots.block(0, 0, 1, N_effective + p + 2);
  int num_effective_pol = (N_effective + 1 - p);  // is not num_pol when q.size()!=N+1

  Eigen::MatrixXd control_points(3, N_effective + 1);

  for (int i = 0; i < (N_effective + 1); i++)
  {
    control_points.col(i) = q[i];
  }

  // std::cout << "Matrix control_points is= " << std::endl;
  // std::cout << control_points << std::endl;

  /*  std::cout << "knots_effective is" << std::endl;
    std::cout << knots_effective << std::endl;*/

  // std::cout << "N_effective= " << N_effective << std::endl;
  // std::cout << "M_effective= " << knots_effective.size() - 1 << std::endl;
  // std::cout << "p= " << p << std::endl;

  Eigen::Matrix<double, 4, 4> M;
  M << 1, 4, 1, 0,   //////
      -3, 0, 3, 0,   //////
      3, -6, 3, 0,   //////
      -1, 3, -3, 1;  //////
  M = M / 6.0;       // *1/3!

  // std::cout << "Control Points used are\n" << control_points << std::endl;
  // std::cout << "====================" << std::endl;

  solution.clear();

  for (int i = p; i < (p + num_effective_pol + 1); i++)  // i < knots.size() - p
  {
    solution.times.push_back(knots(i));
  }

  for (int j = 0; j < num_effective_pol; j++)
  {
    Eigen::Matrix<double, 4, 1> cps_x = (control_points.block(0, j, 1, 4).transpose());
    Eigen::Matrix<double, 4, 1> cps_y = (control_points.block(1, j, 1, 4).transpose());
    Eigen::Matrix<double, 4, 1> cps_z = (control_points.block(2, j, 1, 4).transpose());

    solution.coeff_x.push_back((M * cps_x).reverse());  // at^3 + bt^2 + ct + d --> [a b c d]'
    solution.coeff_y.push_back((M * cps_y).reverse());  // at^3 + bt^2 + ct + d --> [a b c d]'
    solution.coeff_z.push_back((M * cps_z).reverse());  // at^3 + bt^2 + ct + d --> [a b c d]'
  }

  // std::cout << "polynomials in solution=" << solution.coeff_x.size() << std::endl;
  // std::cout << "num_effective_pol=" << num_effective_pol << std::endl;
  // std::cout << "times =" << solution.times.size() << std::endl;

  // solution.print();

  /*

  Eigen::Matrix<double, 1, 4> tmp;
  double u = 0.5;
  tmp << 1.0, u, u * u, u * u * u;

  double evaluation = tmp * M * cps;


    // std::cout << "Knots= " << knots_ << std::endl;
    // std::cout << "t_min_= " << t_min_ << std::endl;

    std::cout << "evaluation by my method: " << evaluation << std::endl;
    Eigen::MatrixXd novale = spline.derivatives(t_min_ + deltaT_ / 2.0, 4);
    std::cout << "evaluation by Eigen: " << novale.col(0).x() << std::endl;*/

  // Construct now the B-Spline
  // See example at https://github.com/libigl/eigen/blob/master/unsupported/test/splines.cpp#L37

  Eigen::Spline<double, 3, Eigen::Dynamic> spline(knots_effective, control_points);
  traj.clear();

  double t_min = knots_effective(0);
  double t_max = knots_effective(knots_effective.size() - 4);  // t_max is this size()-4, see demo in
                                                               // http://nurbscalculator.in/  (slider)

  /*  std::cout << "t_min= " << t_min << std::endl;
    std::cout << "t_max= " << t_max << std::endl;*/

  for (double t = t_min; t <= t_max; t = t + dc)
  {
    // std::cout << "t= " << t << std::endl;
    Eigen::MatrixXd derivatives = spline.derivatives(t, 4);  // Compute all the derivatives up to order 4

    mt::state state_i;

    state_i.setPos(derivatives.col(0));  // First column
    state_i.setVel(derivatives.col(1));
    state_i.setAccel(derivatives.col(2));
    state_i.setJerk(derivatives.col(3));
    traj.push_back(state_i);

    // std::cout << "Creating markers best traj, t= " << t << " pos=" << state_i.pos.transpose() << std::endl;
    // std::cout << "Aceleration= " << derivatives.col(2).transpose() << std::endl;
    // state_i.printHorizontal();
  }
}

Eigen::Spline3d findInterpolatingBsplineNormalized(const std::vector<double> &times,
                                                   const std::vector<Eigen::Vector3d> &positions)
{
  if (times.size() != positions.size())
  {
    std::cout << "times.size() should be == positions.size()" << std::endl;
    abort();
  }

  // check that times is  increasing
  for (int i = 0; i < (times.size() - 1); i++)
  {
    if (times[i + 1] < times[i])
    {
      std::cout << "times should be increasing" << std::endl;
      abort();
    }
  }

  if ((times.back() - times.front()) < 1e-7)
  {
    std::cout << "there is no time span in the vector times" << std::endl;
    abort();
  }

  // See example here: https://github.com/libigl/eigen/blob/master/unsupported/test/splines.cpp

  Eigen::MatrixXd points(3, positions.size());
  for (int i = 0; i < positions.size(); i++)
  {
    points.col(i) = positions[i];
  }

  Eigen::RowVectorXd knots_normalized(times.size());
  Eigen::RowVectorXd knots(times.size());

  for (int i = 0; i < times.size(); i++)
  {
    knots(i) = times[i];
    knots_normalized(i) = (times[i] - times[0]) / (times.back() - times.front());
  }

  Eigen::Spline3d spline_normalized = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(points, 3, knots_normalized);

  // Eigen::Spline3d spline(knots, spline_normalized.ctrls());

  // for (int i = 0; i < points.cols(); ++i)
  // {
  //   std::cout << "findInterpolatingBspline 6" << std::endl;

  //   std::cout << "knots(i)= " << knots(i) << std::endl;
  //   std::cout << "knots= " << knots << std::endl;
  //   std::cout << "spline.ctrls()= " << spline.ctrls() << std::endl;

  //   Eigen::Vector3d pt1 = spline_normalized(knots_normalized(i));
  //   std::cout << "pt1= " << pt1.transpose() << std::endl;

  //   // Eigen::Vector3d pt2 = spline(knots(i)); //note that spline(x) requires x to be in [0,1]

  //   // std::cout << "pt2= " << pt2.transpose() << std::endl;

  //   Eigen::Vector3d ref = points.col(i);
  //   std::cout << "norm= " << (pt1 - ref).norm() << std::endl;  // should be ~zero
  //   // std::cout << "norm= " << (pt2 - ref).norm() << std::endl;  // should be ~zero
  // }

  return spline_normalized;
}