/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */
#include "utils.hpp"

#include "termcolor.hpp"

visualization_msgs::MarkerArray mu::pwp2ColoredMarkerArray(mt::PieceWisePol& pwp, double t_init, double t_final,
                                                           int samples, std::string ns)
{
  visualization_msgs::MarkerArray marker_array;

  if (t_final < t_init)
  {
    // std::cout << "t_final<t_init" << std::endl;
    abort();
    return marker_array;
  }

  // std::cout << "t_init= " << std::setprecision(15) << t_init << std::endl;
  // std::cout << "t_final= " << std::setprecision(15) << t_final << std::endl;

  double deltaT = (t_final - t_init) / (1.0 * samples);

  geometry_msgs::Point p_last = mu::eigen2point(pwp.eval(t_init));

  int j = 7 * 9000;  // TODO

  for (double t = t_init; t <= t_final; t = t + deltaT)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.action = visualization_msgs::Marker::ADD;
    m.id = j;
    m.color = mu::color(mu::red_normal);
    m.scale.x = 0.1;
    m.scale.y = 0.0000001;  // rviz complains if not
    m.scale.z = 0.0000001;  // rviz complains if not

    m.pose.orientation.w = 1.0;

    // std::cout << "t= " << std::setprecision(15) << t << std::endl;
    // std::cout << "is " << pwp.eval(t).transpose() << std::endl;

    geometry_msgs::Point p = mu::eigen2point(pwp.eval(t));

    m.points.push_back(p_last);
    m.points.push_back(p);

    p_last = p;
    marker_array.markers.push_back(m);
    j = j + 1;
  }

  return marker_array;
}

double mu::getMinTimeDoubleIntegrator1D(const double& p0, const double& v0, const double& pf, const double& vf,
                                        const double& v_max, const double& a_max)
{
  // See also minimum_time.m
  // %The notation of this function is based on the paper "Constrained time-optimal
  // %control of double integrator system and its application in MPC"
  // % https://iopscience.iop.org/article/10.1088/1742-6596/783/1/012024

  double x1 = v0;
  double x2 = p0;
  double x1r = vf;
  double x2r = pf;

  double k1 = a_max;  // Note that the paper uses u\in[-1, 1].But setting k1 to a_max has the same effect k2 = 1.0;
  double k2 = 1.0;

  double x1_bar = v_max;

  double B = (k2 / (2 * k1)) * sgn(-x1 + x1r) * (pow(x1, 2) - pow(x1r, 2)) + x2r;
  double C = (k2 / (2 * k1)) * (pow(x1, 2) + pow(x1r, 2)) - (k2 / k1) * pow(x1_bar, 2) + x2r;
  double D = (-k2 / (2 * k1)) * (pow(x1, 2) + pow(x1r, 2)) + (k2 / k1) * pow(x1_bar, 2) + x2r;

  double time;

  if ((x2 <= B) && (x2 >= C))
  {
    time = (-k2 * (x1 + x1r) +
            2 * sqrt(pow(k2, 2) * pow(x1, 2) - k1 * k2 * ((k2 / (2 * k1)) * (pow(x1, 2) - pow(x1r, 2)) + x2 - x2r))) /
           (k1 * k2);
  }

  else if ((x2 <= B) && (x2 < C))
  {
    time = (x1_bar - x1 - x1r) / k1 + (pow(x1, 2) + pow(x1r, 2)) / (2 * k1 * x1_bar) + (x2r - x2) / (k2 * x1_bar);
  }

  else if ((x2 > B) && (x2 <= D))
  {
    time = (k2 * (x1 + x1r) +
            2 * sqrt(pow(k2, 2) * pow(x1, 2) + k1 * k2 * ((k2 / (2 * k1)) * (-pow(x1, 2) + pow(x1r, 2)) + x2 - x2r))) /
           (k1 * k2);
  }

  else
  {  // (x2 > B) && (x2 > D)

    time = (x1_bar + x1 + x1r) / k1 + (pow(x1, 2) + pow(x1r, 2)) / (2 * k1 * x1_bar) + (-x2r + x2) / (k2 * x1_bar);
  }

  return time;
}

double mu::getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pf,
                                        const Eigen::Vector3d& vf, const Eigen::Vector3d& v_max,
                                        const Eigen::Vector3d& a_max)
{
  double min_x = getMinTimeDoubleIntegrator1D(p0.x(), v0.x(), pf.x(), vf.x(), v_max.x(), a_max.x());
  double min_y = getMinTimeDoubleIntegrator1D(p0.y(), v0.y(), pf.y(), vf.y(), v_max.y(), a_max.y());
  double min_z = getMinTimeDoubleIntegrator1D(p0.z(), v0.z(), pf.z(), vf.z(), v_max.z(), a_max.z());

  double min_time = std::max({ min_x, min_y, min_z });  // Note that it's the maximum of all the axes

  return min_time;
}

// coeff_old are the coeff [a b c d]' of a polynomial p(t) defined in [0,1]
// coeff_new are the coeff [a b c d]' of a polynomial q(t) defined in [0,1] such that:
//     q((t-t0)/(tf-t0))=p(t) \forall t \in [t0,tf]
// t0 and tf do not have to be \in [0,1]. If t0<0, we'll be doing extrapolation in the past, and if
// tf>1, we will be doing extrapolation in the future
void mu::rescaleCoeffPol(const Eigen::Matrix<double, 4, 1>& coeff_old, Eigen::Matrix<double, 4, 1>& coeff_new,
                         double t0, double tf)
{
  // if (t0 < 0 || t0 > 1 || tf < 0 || tf > 1)
  // {
  //   std::cout << "tf and t0 should be in [0,1]" << std::endl;
  //   std::cout << "t0= " << t0 << std::endl;
  //   std::cout << "tf= " << tf << std::endl;
  //   std::cout << "================================" << std::endl;
  //   abort();
  // }

  double a = coeff_old(0);
  double b = coeff_old(1);
  double c = coeff_old(2);
  double d = coeff_old(3);

  double delta = tf - t0;
  double delta_2 = delta * delta;
  double delta_3 = delta * delta * delta;

  std::cout << "delta= " << delta << std::endl;

  // if (isnan(delta))
  // {
  //   std::cout << "tf= " << tf << std::endl;
  //   std::cout << "t0= " << tf << std::endl;
  //   std::cout << "delta is NAN" << std::endl;
  //   abort();
  // }

  std::cout << "a= " << a << std::endl;
  std::cout << "b= " << b << std::endl;
  std::cout << "c= " << c << std::endl;
  std::cout << "d= " << d << std::endl;

  double t0_2 = t0 * t0;
  double t0_3 = t0 * t0 * t0;

  coeff_new(0) = a * delta_3;
  coeff_new(1) = b * delta_2 + 3 * a * delta_2 * t0;
  coeff_new(2) = c * delta + 2 * b * delta * t0 + 3 * a * delta * t0_2;
  coeff_new(3) = d + c * t0 + b * t0_2 + a * t0_3;
}

mader_msgs::PieceWisePolTraj mu::pwp2PwpMsg(const mt::PieceWisePol& pwp)
{
  mader_msgs::PieceWisePolTraj pwp_msg;

  for (int i = 0; i < pwp.times.size(); i++)
  {
    // std::cout << termcolor::red << "in mu::pwp2PwpMsg, pushing back" << std::setprecision(20) << pwp.times[i]
    //           << termcolor::reset << std::endl;
    pwp_msg.times.push_back(pwp.times[i]);
  }

  // push x
  for (auto coeff_x_i : pwp.coeff_x)
  {
    mader_msgs::CoeffPoly3 coeff_poly3;
    coeff_poly3.a = coeff_x_i(0);
    coeff_poly3.b = coeff_x_i(1);
    coeff_poly3.c = coeff_x_i(2);
    coeff_poly3.d = coeff_x_i(3);
    pwp_msg.coeff_x.push_back(coeff_poly3);
  }

  // push y
  for (auto coeff_y_i : pwp.coeff_y)
  {
    mader_msgs::CoeffPoly3 coeff_poly3;
    coeff_poly3.a = coeff_y_i(0);
    coeff_poly3.b = coeff_y_i(1);
    coeff_poly3.c = coeff_y_i(2);
    coeff_poly3.d = coeff_y_i(3);
    pwp_msg.coeff_y.push_back(coeff_poly3);
  }

  // push z
  for (auto coeff_z_i : pwp.coeff_z)
  {
    mader_msgs::CoeffPoly3 coeff_poly3;
    coeff_poly3.a = coeff_z_i(0);
    coeff_poly3.b = coeff_z_i(1);
    coeff_poly3.c = coeff_z_i(2);
    coeff_poly3.d = coeff_z_i(3);
    pwp_msg.coeff_z.push_back(coeff_poly3);
  }

  return pwp_msg;
}

mt::PieceWisePol mu::pwpMsg2Pwp(const mader_msgs::PieceWisePolTraj& pwp_msg)
{
  mt::PieceWisePol pwp;

  if (pwp_msg.coeff_x.size() != pwp_msg.coeff_y.size() || pwp_msg.coeff_x.size() != pwp_msg.coeff_z.size())
  {
    std::cout << " coeff_x,coeff_y,coeff_z of pwp_msg should have the same elements" << std::endl;
    std::cout << " ================================" << std::endl;
    abort();
  }

  for (int i = 0; i < pwp_msg.times.size(); i++)
  {
    pwp.times.push_back(pwp_msg.times[i]);
  }

  for (int i = 0; i < pwp_msg.coeff_x.size(); i++)
  {
    Eigen::Matrix<double, 4, 1> tmp_x, tmp_y, tmp_z;

    // std::cout << termcolor::on_blue << "pwpMsg2Pwp: " << pwp_msg.coeff_z[i].a << ", " << pwp_msg.coeff_z[i].b << ", "
    //           << pwp_msg.coeff_z[i].c << ", " << pwp_msg.coeff_z[i].d << termcolor::reset << std::endl;

    tmp_x << pwp_msg.coeff_x[i].a, pwp_msg.coeff_x[i].b, pwp_msg.coeff_x[i].c, pwp_msg.coeff_x[i].d;
    pwp.coeff_x.push_back(tmp_x);

    tmp_y << pwp_msg.coeff_y[i].a, pwp_msg.coeff_y[i].b, pwp_msg.coeff_y[i].c, pwp_msg.coeff_y[i].d;
    pwp.coeff_y.push_back(tmp_y);

    tmp_z << pwp_msg.coeff_z[i].a, pwp_msg.coeff_z[i].b, pwp_msg.coeff_z[i].c, pwp_msg.coeff_z[i].d;
    pwp.coeff_z.push_back(tmp_z);
  }

  return pwp;
}

visualization_msgs::Marker mu::edges2Marker(const mt::Edges& edges, std_msgs::ColorRGBA color_marker)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "markertacles";

  if (edges.size() == 0)  // there are no edges
  {
    // std::cout << "there are no edges" << std::endl;
    return marker;
  }

  marker.id = 0;
  marker.type = marker.LINE_LIST;
  marker.action = marker.ADD;
  marker.pose = mu::identityGeometryMsgsPose();

  marker.points.clear();

  for (auto edge : edges)
  {
    marker.points.push_back(mu::eigen2point(edge.first));
    marker.points.push_back(mu::eigen2point(edge.second));
  }

  marker.scale.x = 0.03;
  // marker.scale.y = 0.00001;
  // marker.scale.z = 0.00001;
  marker.color = color_marker;

  return marker;
}

mt::PieceWisePol mu::createPwpFromStaticPosition(const mt::state& current_state)
{
  mt::PieceWisePol pwp;
  pwp.times = { ros::Time::now().toSec(), ros::Time::now().toSec() + 1e10 };

  Eigen::Matrix<double, 4, 1> coeff_x_interv0;  // [a b c d]' of the interval 0
  Eigen::Matrix<double, 4, 1> coeff_y_interv0;  // [a b c d]' of the interval 0
  Eigen::Matrix<double, 4, 1> coeff_z_interv0;  // [a b c d]' of the interval 0

  coeff_x_interv0 << 0.0, 0.0, 0.0, current_state.pos.x();
  coeff_y_interv0 << 0.0, 0.0, 0.0, current_state.pos.y();
  coeff_z_interv0 << 0.0, 0.0, 0.0, current_state.pos.z();

  pwp.coeff_x.push_back(coeff_x_interv0);
  pwp.coeff_y.push_back(coeff_y_interv0);
  pwp.coeff_z.push_back(coeff_z_interv0);

  return pwp;
}

// returns a mt::PieceWisePol, taking the polynomials of p1 and p2 that should satisfy p1>t, p2>t
mt::PieceWisePol mu::composePieceWisePol(const double t, const double dc, mt::PieceWisePol& p1, mt::PieceWisePol& p2)
{
  // if t is in between p1 and p2, force p2[0] to be t
  if (t > p1.times.back() && t < p2.times.front())  // && fabs(t - p2.times.front()) <= dc) TODO Sometimes fabs(t -
                                                    // p2.times.front()) is 0.18>c
  {
    p2.times.front() = t;
  }

  if (p1.times.back() < p2.times.front())  // && fabs(p1.times.back() - p2.times.front()) <= dc) TODO
  {
    p2.times.front() = p1.times.back();
  }

  if (t < p1.times.front())  // TODO
  {
    p1.times.front() = t;
  }

  if (fabs(t - p2.times.front()) < 1e-5)
  {
    return p2;
  }

  if (p1.times.back() < p2.times.front() || t > p2.times.back() || t < p1.times.front())
  {
    // TODO: does this happen?

    // std::cout << "Error composing the piecewisePol" << std::endl;
    // std::cout << std::setprecision(30) << "t= " << t << std::endl;
    // std::cout << std::setprecision(30) << "p1.times.front()= " << p1.times.front() << std::endl;
    // std::cout << std::setprecision(30) << "p1.times.back()= " << p1.times.back() << std::endl;
    // std::cout << std::setprecision(30) << "p2.times.front() = " << p2.times.front() << std::endl;
    // std::cout << std::setprecision(30) << "p2.times.back() = " << p2.times.back() << std::endl;
    mt::PieceWisePol dummy;
    return dummy;
  }

  std::vector<int> indexes1, indexes2;

  for (int i = 0; i < p1.times.size(); i++)
  {
    if (p1.times[i] > t && p1.times[i] < p2.times[0])
    {
      indexes1.push_back(i);
    }
  }

  for (int i = 0; i < p2.times.size(); i++)
  {
    if (p2.times[i] > t)
    {
      indexes2.push_back(i);
    }
  }

  mt::PieceWisePol p;
  p.times.push_back(t);

  for (auto index_1_i : indexes1)
  {
    p.times.push_back(p1.times[index_1_i]);
    p.coeff_x.push_back(p1.coeff_x[index_1_i - 1]);
    p.coeff_y.push_back(p1.coeff_y[index_1_i - 1]);
    p.coeff_z.push_back(p1.coeff_z[index_1_i - 1]);
  }

  for (auto index_2_i : indexes2)
  {
    if (index_2_i == 0)
    {
      p.coeff_x.push_back(p1.coeff_x.back());
      p.coeff_y.push_back(p1.coeff_y.back());
      p.coeff_z.push_back(p1.coeff_z.back());
      p.times.push_back(p2.times[index_2_i]);
      continue;
    }
    p.times.push_back(p2.times[index_2_i]);
    p.coeff_x.push_back(p2.coeff_x[index_2_i - 1]);
    p.coeff_y.push_back(p2.coeff_y[index_2_i - 1]);
    p.coeff_z.push_back(p2.coeff_z[index_2_i - 1]);
  }

  return p;
}

std::vector<std::string> mu::pieceWisePol2String(const mt::PieceWisePol& piecewisepol)
{
  // Define strings
  std::string s_x = "0.0";
  std::string s_y = "0.0";
  std::string s_z = "0.0";

  //(piecewisepol.times - 1) is the number of intervals
  for (int i = 0; i < (piecewisepol.times.size() - 1); i++)  // i is the index of the interval
  {
    std::string div_by_delta = "/ (" + std::to_string(piecewisepol.times[i + 1] - piecewisepol.times[i]) + ")";

    std::string t = "(min(t," + std::to_string(piecewisepol.times.back()) + "))";

    std::string u = "(" + t + "-" + std::to_string(piecewisepol.times[i]) + ")" + div_by_delta;
    u = "(" + u + ")";
    std::string uu = u + "*" + u;
    std::string uuu = u + "*" + u + "*" + u;
    /*    std::cout << "piecewisepol.times[i]= " << piecewisepol.times[i] << std::endl;
        std::cout << "piecewisepol.times[i+1]= " << piecewisepol.times[i + 1] << std::endl;*/

    std::string cond;
    if (i == (piecewisepol.times.size() - 2))  // if the last interval
    {
      cond = "(t>=" + std::to_string(piecewisepol.times[i]) + ")";
    }
    else
    {
      cond = "(t>=" + std::to_string(piecewisepol.times[i]) + " and " + "t<" +
             std::to_string(piecewisepol.times[i + 1]) + ")";
    }

    std::string s_x_i = std::to_string((double)piecewisepol.coeff_x[i](0)) + "*" + uuu;   //////////////////
    s_x_i = s_x_i + "+" + std::to_string((double)piecewisepol.coeff_x[i](1)) + "*" + uu;  //////////////////
    s_x_i = s_x_i + "+" + std::to_string((double)piecewisepol.coeff_x[i](2)) + "*" + u;   //////////////////
    s_x_i = s_x_i + "+" + std::to_string((double)piecewisepol.coeff_x[i](3));             //////////////////
    s_x_i = cond + "*(" + s_x_i + ")";

    std::string s_y_i = std::to_string((double)piecewisepol.coeff_y[i](0)) + "*" + uuu;   //////////////////
    s_y_i = s_y_i + "+" + std::to_string((double)piecewisepol.coeff_y[i](1)) + "*" + uu;  //////////////////
    s_y_i = s_y_i + "+" + std::to_string((double)piecewisepol.coeff_y[i](2)) + "*" + u;   //////////////////
    s_y_i = s_y_i + "+" + std::to_string((double)piecewisepol.coeff_y[i](3));             //////////////////
    s_y_i = cond + "*(" + s_y_i + ")";

    std::string s_z_i = std::to_string((double)piecewisepol.coeff_z[i](0)) + "*" + uuu;   //////////////////
    s_z_i = s_z_i + "+" + std::to_string((double)piecewisepol.coeff_z[i](1)) + "*" + uu;  //////////////////
    s_z_i = s_z_i + "+" + std::to_string((double)piecewisepol.coeff_z[i](2)) + "*" + u;   //////////////////
    s_z_i = s_z_i + "+" + std::to_string((double)piecewisepol.coeff_z[i](3));             //////////////////
    s_z_i = cond + "*(" + s_z_i + ")";

    s_x = s_x + " + " + s_x_i;
    s_y = s_y + " + " + s_y_i;
    s_z = s_z + " + " + s_z_i;
  }

  std::vector<std::string> s;
  s.push_back(s_x);
  s.push_back(s_y);
  s.push_back(s_z);

  return s;
}

geometry_msgs::Pose mu::identityGeometryMsgsPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

std_msgs::ColorRGBA mu::getColorJet(double v, double vmin, double vmax)
{
  std_msgs::ColorRGBA c;
  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;
  // white
  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv))
  {
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  }
  else if (v < (vmin + 0.5 * dv))
  {
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  }
  else if (v < (vmin + 0.75 * dv))
  {
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  }
  else
  {
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;
  }

  return (c);
}

std_msgs::ColorRGBA mu::color(int id)
{
  std_msgs::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;
  std_msgs::ColorRGBA red_trans;
  red_trans.r = 1;
  red_trans.g = 0;
  red_trans.b = 0;
  red_trans.a = 0.7;
  std_msgs::ColorRGBA red_trans_trans;
  red_trans_trans.r = 1;
  red_trans_trans.g = 0;
  red_trans_trans.b = 0;
  red_trans_trans.a = 0.4;
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1;
  blue.a = 1;
  std_msgs::ColorRGBA blue_trans;
  blue_trans.r = 0;
  blue_trans.g = 0;
  blue_trans.b = 1;
  blue_trans.a = 0.7;
  std_msgs::ColorRGBA blue_trans_trans;
  blue_trans_trans.r = 0;
  blue_trans_trans.g = 0;
  blue_trans_trans.b = 1;
  blue_trans_trans.a = 0.4;
  std_msgs::ColorRGBA blue_light;
  blue_light.r = 0.5;
  blue_light.g = 0.7;
  blue_light.b = 1;
  blue_light.a = 1;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1;
  green.b = 0;
  green.a = 1;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1;
  yellow.g = 1;
  yellow.b = 0;
  yellow.a = 1;
  std_msgs::ColorRGBA orange_trans;  // orange transparent
  orange_trans.r = 1;
  orange_trans.g = 0.5;
  orange_trans.b = 0;
  orange_trans.a = 0.7;
  std_msgs::ColorRGBA teal_normal;  // orange transparent
  teal_normal.r = 25 / 255.0;
  teal_normal.g = 1.0;
  teal_normal.b = 240.0 / 255.0;
  teal_normal.a = 1.0;
  std_msgs::ColorRGBA black_trans;  // orange transparent
  black_trans.r = 0.0;
  black_trans.g = 0.0;
  black_trans.b = 0.0;
  black_trans.a = 0.2;

  switch (id)
  {
    case mu::red_normal:
      return red;
      break;
    case mu::red_trans:
      return red_trans;
      break;
    case mu::red_trans_trans:
      return red_trans_trans;
      break;
    case mu::blue_normal:
      return blue;
      break;
    case mu::blue_trans:
      return blue_trans;
      break;
    case mu::blue_trans_trans:
      return blue_trans_trans;
      break;
    case mu::blue_light:
      return blue_light;
      break;
    case mu::green_normal:
      return green;
      break;
    case mu::yellow_normal:
      return yellow;
      break;
    case mu::orange_trans:
      return orange_trans;
      break;
    case mu::black_trans:
      return black_trans;
      break;
    case mu::teal_normal:
      return teal_normal;
      break;
    default:
      std::cout << "COLOR NOT DEFINED, returning RED" << std::endl;
      return red;
  }
}

// It assummes that the box is aligned with x, y, z
// C1 is the corner with lowest x,y,z
// C2 is the corner with highest x,y,z
// center is the center of the sphere
// r is the radius of the sphere
bool mu::boxIntersectsSphere(Eigen::Vector3d center, double r, Eigen::Vector3d c1, Eigen::Vector3d c2)
{
  // https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection
  //(Section Sphere vs AABB)

  Eigen::Vector3d closest_point;  // closest point from the center of the sphere to the box

  closest_point(0) = std::max(c1.x(), std::min(center.x(), c2.x()));
  closest_point(1) = std::max(c1.y(), std::min(center.y(), c2.y()));
  closest_point(2) = std::max(c1.z(), std::min(center.z(), c2.z()));

  // this is the same as isPointInsideSphere
  double distance_to_closest_point = (center - closest_point).norm();

  return (distance_to_closest_point < r);  // true if the box intersects the sphere
}

//## From Wikipedia - http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void mu::quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void mu::quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  mu::quaternion2Euler(tf_q, roll, pitch, yaw);
}

void mu::quaternion2Euler(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  mu::quaternion2Euler(tf_q, roll, pitch, yaw);
}

void mu::saturate(Eigen::Vector3d& tmp, const Eigen::Vector3d& min, const Eigen::Vector3d& max)
{
  mu::saturate(tmp(0), min(0), max(0));
  mu::saturate(tmp(1), min(1), max(1));
  mu::saturate(tmp(2), min(2), max(2));
}

void mu::saturate(int& var, const int min, const int max)
{
  if (var < min)
  {
    var = min;
  }
  else if (var > max)
  {
    var = max;
  }
}

// TODO: Make a template here
void mu::saturate(double& var, const double min, const double max)
{
  if (var < min)
  {
    var = min;
  }
  else if (var > max)
  {
    var = max;
  }
}

visualization_msgs::Marker mu::getMarkerSphere(double scale, int my_color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color = mu::color(my_color);

  return marker;
}

void mu::angle_wrap(double& diff)
{
  diff = fmod(diff + M_PI, 2 * M_PI);
  if (diff < 0)
    diff += 2 * M_PI;
  diff -= M_PI;
}

geometry_msgs::Point mu::pointOrigin()
{
  geometry_msgs::Point tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

Eigen::Vector3d mu::vec2eigen(geometry_msgs::Vector3 vector)
{
  Eigen::Vector3d tmp;
  tmp << vector.x, vector.y, vector.z;
  return tmp;
}

geometry_msgs::Vector3 mu::eigen2rosvector(Eigen::Vector3d vector)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = vector(0, 0);
  tmp.y = vector(1, 0);
  tmp.z = vector(2, 0);
  return tmp;
}

geometry_msgs::Point mu::eigen2point(Eigen::Vector3d vector)
{
  geometry_msgs::Point tmp;
  tmp.x = vector[0];
  tmp.y = vector[1];
  tmp.z = vector[2];
  return tmp;
}

geometry_msgs::Vector3 mu::vectorNull()
{
  geometry_msgs::Vector3 tmp;
  tmp.x = 0;
  tmp.y = 0;
  tmp.z = 0;
  return tmp;
}

geometry_msgs::Vector3 mu::vectorUniform(double a)
{
  geometry_msgs::Vector3 tmp;
  tmp.x = a;
  tmp.y = a;
  tmp.z = a;
  return tmp;
}

// given 2 points (A inside and B outside the sphere) it computes the intersection of the lines between
// that 2 points and the sphere
Eigen::Vector3d mu::getIntersectionWithSphere(Eigen::Vector3d& A, Eigen::Vector3d& B, double r, Eigen::Vector3d& center)
{
  // http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm

  float x1 = A[0];
  float y1 = A[1];
  float z1 = A[2];

  float x2 = B[0];
  float y2 = B[1];
  float z2 = B[2];

  float x3 = center[0];
  float y3 = center[1];
  float z3 = center[2];

  float a = pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2);
  float b = 2 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3) + (z2 - z1) * (z1 - z3));
  float c = x3 * x3 + y3 * y3 + z3 * z3 + x1 * x1 + y1 * y1 + z1 * z1 - 2 * (x3 * x1 + y3 * y1 + z3 * z1) - r * r;

  float discrim = b * b - 4 * a * c;
  if (discrim <= 0)
  {
    printf("The line is tangent or doesn't intersect, returning the intersection with the center and the first "
           "point\n");

    float x1 = center[0];
    float y1 = center[1];
    float z1 = center[2];

    float x2 = A[0];
    float y2 = A[1];
    float z2 = A[2];

    float x3 = center[0];
    float y3 = center[1];
    float z3 = center[2];

    float a = pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2);
    float b = 2 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3) + (z2 - z1) * (z1 - z3));
    float c = x3 * x3 + y3 * y3 + z3 * z3 + x1 * x1 + y1 * y1 + z1 * z1 - 2 * (x3 * x1 + y3 * y1 + z3 * z1) - r * r;

    float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    float x_int = x1 + (x2 - x1) * t;
    float y_int = y1 + (y2 - y1) * t;
    float z_int = z1 + (z2 - z1) * t;
    Eigen::Vector3d intersection(x_int, y_int, z_int);

    return intersection;
  }
  else
  {
    float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    float x_int = x1 + (x2 - x1) * t;
    float y_int = y1 + (y2 - y1) * t;
    float z_int = z1 + (z2 - z1) * t;
    Eigen::Vector3d intersection(x_int, y_int, z_int);
    // std::cout << "Intersection=" << std::endl << intersection << std::endl;
    return intersection;
  }
}

// Given a path (starting inside the sphere and finishing outside of it) expressed by a vector of 3D-vectors (points),
// it returns its first intersection with a sphere of radius=r and center=center
// the center is added as the first point of the path to ensure that the first element of the path is inside the sphere
// (to avoid issues with the first point of JPS2)
Eigen::Vector3d mu::getFirstIntersectionWithSphere(std::vector<Eigen::Vector3d>& path, double r,
                                                   Eigen::Vector3d& center, int* last_index_inside_sphere,
                                                   bool* noPointsOutsideSphere)
{
  if (noPointsOutsideSphere != NULL)
  {  // this argument has been provided
    *noPointsOutsideSphere = false;
  }

  int index = -1;
  for (int i = 0; i < path.size(); i++)
  {
    double dist = (path[i] - center).norm();

    if (dist > r)
    {
      index = i;  // This is the first point outside the sphere
      break;
    }
  }

  Eigen::Vector3d A;
  Eigen::Vector3d B;

  Eigen::Vector3d intersection;
  switch (index)
  {
    case -1:  // no points are outside the sphere --> return last element
      // std::cout << "Utils: no points are outside the sphere!!!" << std::endl;
      A = center;
      B = path[path.size() - 1];
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = path.size() - 1;
      }
      if (noPointsOutsideSphere != NULL)
      {  // this argument has been provided
        *noPointsOutsideSphere = true;
      }

      intersection = mu::getIntersectionWithSphere(A, B, r, center);

      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = path.size() - 1;
      }
      break;
    case 0:  // First element is outside the sphere
      printf("First element is still oustide the sphere, there is sth wrong, returning the first element\n");
      intersection = path[0];

      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = 1;
      }
      break;
    default:
      A = path[index - 1];
      B = path[index];

      intersection = mu::getIntersectionWithSphere(A, B, r, center);
      // printf("index-1=%d\n", index - 1);
      if (last_index_inside_sphere != NULL)
      {
        *last_index_inside_sphere = index - 1;
      }
  }

  return intersection;
}

visualization_msgs::MarkerArray mu::trajectory2ColoredMarkerArray(const mt::trajectory& data, double max_value,
                                                                  int increm, std::string ns, double scale,
                                                                  std::string color_type, int id_agent, int n_agents)
{
  visualization_msgs::MarkerArray marker_array;

  if (data.size() == 0)
  {
    return marker_array;
  }
  geometry_msgs::Point p_last;
  p_last.x = data[0].pos(0);
  p_last.y = data[0].pos(1);
  p_last.z = data[0].pos(2);

  increm = (increm < 1.0) ? 1 : increm;

  int j = 9000;
  for (int i = 0; i < data.size(); i = i + increm)
  {
    double vel = data[i].vel.norm();
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.action = visualization_msgs::Marker::ADD;
    m.id = j;
    if (color_type == "vel")  // TODO: "vel" is hand-coded
    {
      m.color = mu::getColorJet(vel, 0, max_value);  // note that par_.v_max is per axis!
    }
    else if (color_type == "time")  // TODO: "time" is hand-coded
    {
      m.color = mu::getColorJet(i, 0, data.size());  // note that par_.v_max is per axis!
    }
    else
    {
      m.color = mu::getColorJet(id_agent, 0, n_agents);  // note that par_.v_max is per axis!
    }
    m.scale.x = scale;
    m.scale.y = 0.0000001;  // rviz complains if not
    m.scale.z = 0.0000001;  // rviz complains if not

    m.pose.orientation.w = 1.0;
    // std::cout << "Mandando bloque" << X.block(i, 0, 1, 3) << std::endl;
    geometry_msgs::Point p;
    p.x = data[i].pos(0);
    p.y = data[i].pos(1);
    p.z = data[i].pos(2);
    m.points.push_back(p_last);
    m.points.push_back(p);
    // std::cout << "pushing marker\n" << m << std::endl;
    p_last = p;
    marker_array.markers.push_back(m);
    j = j + 1;
  }
  return marker_array;
}
