/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "predictor.hpp"
#include "utils.hpp"
#include "bspline_utils.hpp"
#include <unsupported/Eigen/Splines>

Predictor::Predictor(ros::NodeHandle nh) : nh_(nh)
{
  sub_state_ = nh_.subscribe("state", 1, &Predictor::stateCB, this);
  sub_traj_ = nh_.subscribe("/trajs_obstacles", 20, &Predictor::trajCB, this);  // The number is the queue size

  pub_predicted_traj_ = nh_.advertise<mader_msgs::PieceWisePolTraj>("/trajs", 1);
  pub_marker_predicted_traj_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_predicted_traj", 1);

  mu::safeGetParam(nh_, "mader/Ra", R_local_map_);

  std::cout << bold << "R_local_map_ = " << R_local_map_ << reset << std::endl;

  name_drone_ = ros::this_node::getNamespace();
  name_drone_.erase(std::remove(name_drone_.begin(), name_drone_.end(), '/'), name_drone_.end());  // Remove the slashes
  std::string id = name_drone_;
  id.erase(0, 2);  // Erase SQ or HX i.e. SQ12 --> 12  HX8621 --> 8621
  id_ = std::stoi(id);

  std::cout << bold << "Predictor object created " << reset << std::endl;
}
void Predictor::trajCB(const mader_msgs::DynTraj& msg)
{
  if (msg.id == id_)
  {  // Ignore my own trajectory
    return;
  }

  Eigen::Vector3d W_pos(msg.pos.x, msg.pos.y, msg.pos.z);  // position in world frame
  double dist = (state_.pos - W_pos).norm();

  if (dist > R_local_map_)
  {
    return;
  }

  mt::dynTraj traj;
  traj.function.push_back(msg.function[0]);
  traj.function.push_back(msg.function[1]);
  traj.function.push_back(msg.function[2]);
  traj.bbox << msg.bbox[0], msg.bbox[1], msg.bbox[2];
  traj.id = msg.id;
  traj.is_agent = msg.is_agent;
  traj.time_received = ros::Time::now().toSec();

  mt::dynTrajCompiled traj_compiled;

  for (auto function_i : traj.function)
  {
    typedef exprtk::symbol_table<double> symbol_table_t;
    typedef exprtk::expression<double> expression_t;
    typedef exprtk::parser<double> parser_t;

    symbol_table_t symbol_table;
    symbol_table.add_variable("t", t_);
    symbol_table.add_constants();
    expression_t expression;
    expression.register_symbol_table(symbol_table);

    parser_t parser;
    parser.compile(function_i, expression);

    traj_compiled.function.push_back(expression);
  }
  traj_compiled.bbox = traj.bbox;
  traj_compiled.id = traj.id;
  traj_compiled.time_received = traj.time_received;  // ros::Time::now().toSec();

  std::vector<double> times;
  std::vector<Eigen::Vector3d> last_positions;

  sample(traj_compiled, times, last_positions);

  // std::cout << "times= " << times << std::endl;
  // std::cout << "last_positions= " << last_positions << std::endl;

  std::cout << "Last positions: " << std::endl;
  for (auto pos_i : last_positions)
  {
    std::cout << pos_i.transpose() << std::endl;
  }

  std::cout << "Last times: " << std::endl;
  for (auto times_i : times)
  {
    std::cout << std::setprecision(15) << times_i << reset << std::endl;
  }

  mt::PieceWisePol pwp_predicted_traj = predictPwp(times, last_positions);

  std::cout << "At the end of the past traj = " << pwp_predicted_traj.eval(times.back());

  // mu::pwp2PwpMsg(mt::PieceWisePol pwp, const std::vector<double>& bbox, const int& id,
  // const bool& is_agent)
  bool is_agent = true;
  mader_msgs::PieceWisePolTraj pwp_msg =
      mu::pwp2PwpMsg(pwp_predicted_traj, traj_compiled.bbox, traj_compiled.id, is_agent);

  std::cout << green << "going to publish\n" << reset;
  pwp_predicted_traj.print();

  pub_predicted_traj_.publish(pwp_msg);

  int samples = 20;

  std::string ns = "predicted_traj_" + std::to_string(msg.id);

  pub_marker_predicted_traj_.publish(mu::pwp2ColoredMarkerArray(pwp_predicted_traj, ros::Time::now().toSec(),
                                                                ros::Time::now().toSec() + 1, samples, ns));
}

void Predictor::stateCB(const snapstack_msgs::State& msg)
{
  state_.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_.setAccel(0.0, 0.0, 0.0);
}

void Predictor::sample(const mt::dynTrajCompiled& traj_compiled, std::vector<double>& times,
                       std::vector<Eigen::Vector3d>& last_positions)
{
  // sample last N positions

  times.clear();
  last_positions.clear();

  int n_samples = 7;
  double deltaT = 0.3;

  double t_now = ros::Time::now().toSec();

  for (int i = 1; i <= n_samples; i++)
  {
    t_ = t_now - (n_samples - i) * deltaT;  // note that t_now is included in the last sample

    double x = traj_compiled.function[0].value();
    double y = traj_compiled.function[1].value();
    double z = traj_compiled.function[2].value();

    last_positions.push_back(Eigen::Vector3d(x, y, z));

    times.push_back(t_);
  }
}
mt::PieceWisePol Predictor::predictPwp(std::vector<double>& times, std::vector<Eigen::Vector3d>& last_positions)
{
  Eigen::Spline3d spline = findInterpolatingBsplineNormalized(times, last_positions);

  // And now take the last polynomial of the spline and create a mt::PieceWisePol with only it

  Eigen::RowVectorXd knots = spline.knots();
  Eigen::MatrixXd control_points = spline.ctrls();

  // std::cout << bold << "===========================" << reset << std::endl;
  // std::cout << "Control points are \n" << control_points << std::endl;
  // std::cout << "Knots are \n" << knots << std::endl;
  // std::cout << bold << "===========================" << reset << std::endl;

  // std::cout << "spline(0.5)= " << spline(0.5).transpose() << std::endl;

  Eigen::Matrix<double, 3, 4> V =
      control_points.block(0, control_points.cols() - 4, 3,
                           4);  // The columns of V contain the control points of the last interval

  // now create the pwp that contains only the last interval of the trajectory
  mt::PieceWisePol pwp_predicted_traj;

  double t0 = times.back();

  double t1 = t0 + 27.0;  // t -> Infinity TODO

  pwp_predicted_traj.times.push_back(t0);  // From the end of the fitted trajectory
  pwp_predicted_traj.times.push_back(t1);  // to t -> Infinity

  // Eigen::Matrix<double, 4, 4> A_bs;
  // A_bs << -1, 3, -3, 1,  /////////////////
  //     3, -6, 0, 4,       /////////////////
  //     -3, 3, 3, 1,       /////////////////
  //     1, 0, 0, 0;        /////////////////
  // A_bs = A_bs * (1 / 6.0);

  Eigen::Matrix<double, 4, 4> A_bs_last;
  A_bs_last << -2, 6, -6, 2,  /////////////
      11, -15, -3, 7,         /////////////
      -21, 9, 9, 3,           /////////////
      12, 0, 0, 0;            /////////////

  A_bs_last = A_bs_last * (1.0 / 12.0);

  Eigen::Matrix<double, 3, 4> P = V * A_bs_last;
  // Each row j of P contains [a b c d] of the coordinate j (j={x,y,z})

  // std::cout << "A_bs_last is \n" << A_bs_last << std::endl;
  // std::cout << "=======" << std::endl;
  // std::cout << "V is \n" << V << std::endl;
  // std::cout << "=======" << std::endl;
  // std::cout << "P is \n" << P << std::endl;
  // std::cout << "=======" << std::endl;

  std::cout << "All the intervals of the fitted B-Sline" << std::endl;
  V = control_points.block(0, control_points.cols() - 4, 3, 4);
  // std::cout << "At t=1, last interval=\n" << P * Eigen::Matrix<double, 4, 1>::Ones() << std::endl;

  Eigen::Matrix<double, 4, 1> coeff_old_x = P.row(0).transpose();
  Eigen::Matrix<double, 4, 1> coeff_old_y = P.row(1).transpose();
  Eigen::Matrix<double, 4, 1> coeff_old_z = P.row(2).transpose();

  Eigen::Matrix<double, 4, 1> coeff_new_x, coeff_new_y, coeff_new_z;

  // double t_end=

  mu::rescaleCoeffPol(coeff_old_x, coeff_new_x, 1.0,
                      (t1 - times[times.size() - 2]) / (times[times.size() - 1] - times[times.size() - 2]));
  mu::rescaleCoeffPol(coeff_old_y, coeff_new_y, 1.0,
                      (t1 - times[times.size() - 2]) / (times[times.size() - 1] - times[times.size() - 2]));
  mu::rescaleCoeffPol(coeff_old_z, coeff_new_z, 1.0,
                      (t1 - times[times.size() - 2]) / (times[times.size() - 1] - times[times.size() - 2]));

  // std::cout << "coeff_new_x= " << coeff_new_x.transpose() << std::endl;
  // std::cout << "coeff_new_y= " << coeff_new_y.transpose() << std::endl;
  // std::cout << "coeff_new_z= " << coeff_new_z.transpose() << std::endl;

  pwp_predicted_traj.coeff_x.push_back(coeff_new_x);  // the predicted traj is a polynomial = to the last
                                                      // interval
  pwp_predicted_traj.coeff_y.push_back(coeff_new_y);  // the predicted traj is a polynomial = to the last
                                                      // interval
  pwp_predicted_traj.coeff_z.push_back(coeff_new_z);  // the predicted traj is a polynomial = to the last
                                                      // interval

  return pwp_predicted_traj;
}

// std::cout << "Last positions: " << std::endl;
// for (auto pos_i : last_positions)
// {
//   std::cout << pos_i.transpose() << std::endl;
// }

// std::cout << "Last times: " << std::endl;
// for (auto times_i : times)
// {
//   std::cout << std::setprecision(15) << times_i << reset << std::endl;
// }

// if ((last_positions[-2] - last_positions.back()).norm() < 1e-6)
// {
//   // this is a static obstacle
//   mt::PieceWisePol pwp_predicted_traj;
//   pwp_predicted_traj.times.push_back(pwp.times.back());                    // From the end of the fitted
//   trajectory pwp_predicted_traj.times.push_back(std::numeric_limits<double>::max());  // to t -> Infinity

//   std::vector<Eigen::Matrix<double, 4, 1>> coeff_x;  // [a b c d]'
//   std::vector<Eigen::Matrix<double, 4, 1>> coeff_y;  // [a b c d]'
//   std::vector<Eigen::Matrix<double, 4, 1>> coeff_z;

//   pwp_predicted_traj.coeff_x.push_back();
// }