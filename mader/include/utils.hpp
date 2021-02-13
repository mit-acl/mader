/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "mader_types.hpp"
#include <deque>

#include <mader_msgs/PieceWisePolTraj.h>
#include <mader_msgs/CoeffPoly3.h>

#include "ros/ros.h"

namespace mu
{
static constexpr int red_normal = 1;  // mc \equiv mader colors
static constexpr int red_trans = 2;
static constexpr int red_trans_trans = 3;
static constexpr int green_normal = 4;
static constexpr int blue_normal = 5;
static constexpr int blue_trans = 6;
static constexpr int blue_trans_trans = 7;
static constexpr int blue_light = 8;
static constexpr int yellow_normal = 9;
static constexpr int orange_trans = 10;
static constexpr int black_trans = 11;
static constexpr int teal_normal = 12;

template <typename T>
bool safeGetParam(ros::NodeHandle& nh, std::string const& param_name, T& param_value)
{
  if (!nh.getParam(param_name, param_value))
  {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(param_name, true).c_str());
    exit(1);
  }
  return true;
}

double getMinTimeDoubleIntegrator1D(const double& p0, const double& v0, const double& pf, const double& vf,
                                    const double& v_max, const double& a_max);

double getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pf,
                                    const Eigen::Vector3d& vf, const Eigen::Vector3d& v_max,
                                    const Eigen::Vector3d& a_max);

visualization_msgs::MarkerArray pwp2ColoredMarkerArray(mt::PieceWisePol& pwp, double t_init, double t_final,
                                                       int samples, std::string ns);

void rescaleCoeffPol(const Eigen::Matrix<double, 4, 1>& coeff_old, Eigen::Matrix<double, 4, 1>& coeff_new, double t0,
                     double tf);

mt::PieceWisePol createPwpFromStaticPosition(const mt::state& current_state);

mt::PieceWisePol pwpMsg2Pwp(const mader_msgs::PieceWisePolTraj& pwp_msg);
mader_msgs::PieceWisePolTraj pwp2PwpMsg(const mt::PieceWisePol& pwp);

visualization_msgs::Marker edges2Marker(const mt::Edges& edges, std_msgs::ColorRGBA color_marker);

geometry_msgs::Pose identityGeometryMsgsPose();

mt::PieceWisePol composePieceWisePol(const double t, const double dc, mt::PieceWisePol& p1, mt::PieceWisePol& p2);

bool boxIntersectsSphere(Eigen::Vector3d center, double r, Eigen::Vector3d c1, Eigen::Vector3d c2);

std::vector<std::string> pieceWisePol2String(const mt::PieceWisePol& piecewisepol);

std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax);

std_msgs::ColorRGBA color(int id);

void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw);

void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

void quaternion2Euler(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw);

void saturate(int& var, const int min, const int max);

void saturate(double& var, const double min, const double max);

void saturate(Eigen::Vector3d& tmp, const Eigen::Vector3d& min, const Eigen::Vector3d& max);

visualization_msgs::Marker getMarkerSphere(double scale, int my_color);

void angle_wrap(double& diff);

geometry_msgs::Point pointOrigin();

Eigen::Vector3d vec2eigen(geometry_msgs::Vector3 vector);

geometry_msgs::Vector3 eigen2rosvector(Eigen::Vector3d vector);

geometry_msgs::Point eigen2point(Eigen::Vector3d vector);

geometry_msgs::Vector3 vectorNull();

geometry_msgs::Vector3 vectorUniform(double a);

// sign function
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

// given 2 points (A inside and B outside the sphere) it computes the intersection of the lines between
// that 2 points and the sphere
Eigen::Vector3d getIntersectionWithSphere(Eigen::Vector3d& A, Eigen::Vector3d& B, double r, Eigen::Vector3d& center);

// Given a path (starting inside the sphere and finishing outside of it) expressed by a vector of 3D-vectors (points),
// it returns its first intersection with a sphere of radius=r and center=center
// the center is added as the first point of the path to ensure that the first element of the path is inside the sphere
// (to avoid issues with the first point of JPS2)
Eigen::Vector3d getFirstIntersectionWithSphere(std::vector<Eigen::Vector3d>& path, double r, Eigen::Vector3d& center,
                                               int* last_index_inside_sphere = NULL,
                                               bool* noPointsOutsideSphere = NULL);

visualization_msgs::MarkerArray trajectory2ColoredMarkerArray(const mt::trajectory& data, double max_value, int increm,
                                                              std::string ns, double scale, std::string color_type,
                                                              int id_agent, int n_agents);

}  // namespace mu

// Overload to be able to print a std::vector
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
{
  if (!v.empty())
  {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

#endif
