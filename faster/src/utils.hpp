#ifndef UTILS_HPP
#define UTILS_HPP
//#include <iostream>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <jps_basis/data_utils.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "faster_types.hpp"
#include <deque>

#include <faster_msgs/PieceWisePolTraj.h>
#include <faster_msgs/CoeffPoly3.h>

#include "ros/ros.h"  //TODO this shouldn't be here (separate in utils_ros and utils)

#define RED_NORMAL 1
#define RED_TRANS 2
#define RED_TRANS_TRANS 3
#define GREEN_NORMAL 4
#define BLUE_NORMAL 5
#define BLUE_TRANS 6
#define BLUE_TRANS_TRANS 7
#define BLUE_LIGHT 8
#define YELLOW_NORMAL 9
#define ORANGE_TRANS 10
#define BLACK_TRANS 11
#define TEAL_NORMAL 12

#define STATE 0
#define INPUT 1

#define POS 0
#define VEL 1
#define ACCEL 2
#define JERK 3

#define WHOLE_TRAJ 0
#define RESCUE_PATH 1

#define OCCUPIED_SPACE 1
#define UNKOWN_AND_OCCUPIED_SPACE 2

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

visualization_msgs::MarkerArray pwp2ColoredMarkerArray(PieceWisePol& pwp, double t_init, double t_final, int samples,
                                                       std::string ns);

void rescaleCoeffPol(const Eigen::Matrix<double, 4, 1>& coeff_old, Eigen::Matrix<double, 4, 1>& coeff_new, double t0,
                     double tf);

faster_msgs::PieceWisePolTraj pwp2PwpMsg(PieceWisePol pwp, const Eigen::Vector3d& bbox, const int& id,
                                         const bool& is_agent);

PieceWisePolWithInfo pwpMsg2PwpWithInfo(const faster_msgs::PieceWisePolTraj& pwp_msg);

visualization_msgs::Marker edges2Marker(const faster_types::Edges& edges, std_msgs::ColorRGBA color_marker);

geometry_msgs::Pose identityGeometryMsgsPose();

PieceWisePol composePieceWisePol(const double t, const double dc, PieceWisePol& p1, PieceWisePol& p2);

bool boxIntersectsSphere(Eigen::Vector3d center, double r, Eigen::Vector3d c1, Eigen::Vector3d c2);

void printStateDeque(std::deque<state>& data);

std::vector<std::string> pieceWisePol2String(const PieceWisePol& piecewisepol);

void printStateVector(std::vector<state>& data);

std_msgs::ColorRGBA getColorJet(double v, double vmin, double vmax);

std_msgs::ColorRGBA color(int id);

//## From Wikipedia - http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw);

void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

void quaternion2Euler(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw);

void saturate(double& var, const double min, const double max);

void saturate(Eigen::Vector3d& tmp, const Eigen::Vector3d& min, const Eigen::Vector3d& max);

visualization_msgs::Marker getMarkerSphere(double scale, int my_color);

double angleBetVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

void angle_wrap(double& diff);

// coeff is from highest degree to lowest degree. Returns the smallest positive real solution. Returns -1 if a
// root is imaginary or if it's negative

geometry_msgs::Point pointOrigin();

Eigen::Vector3d vec2eigen(geometry_msgs::Vector3 vector);

geometry_msgs::Vector3 eigen2rosvector(Eigen::Vector3d vector);

geometry_msgs::Point eigen2point(Eigen::Vector3d vector);

geometry_msgs::Vector3 vectorNull();

geometry_msgs::Vector3 vectorUniform(double a);

template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;  // Be CAREFUL, because this is with doubles!

template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

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

visualization_msgs::MarkerArray trajectory2ColoredMarkerArray(const trajectory& data, int type, double max_value,
                                                              int increm, std::string ns, double scale);

#endif
