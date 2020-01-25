// Jesus Tordesillas Torres, jtorde@mit.edu, January 2020

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>
#include <random>
#include "timer.hpp"
#include "solverNlopt.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
namespace bg = boost::geometry;

#define DEG_POL 3
#define NUM_POL 5

typedef Timer MyTimer;

std::vector<Eigen::Vector3d> convexHullOfInterval(double t_start, double t_end, double inc)
{
  typedef boost::tuple<double, double, double> point;
  typedef boost::geometry::model::polygon<point> polygon;
  typedef boost::geometry::model::multi_point<point> mpoint;  // multiple points
  mpoint mpt;

  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> distribution(-1, 1);  // doubles from -1 to 1

  for (double t = t_start; t <= t_end; t = t + inc)
  {
    double int_random = 1;

    double r = int_random * distribution(generator);

    std::cout << "r= " << r << std::endl;

    /*    boost::geometry::append(mpt, point(sin(t), cos(t), sin(t)));*/
    boost::geometry::append(mpt, point(5 + r, r, r));
  }

  polygon hull;
  MyTimer timer(true);
  boost::geometry::convex_hull(mpt, hull);
  std::cout << "ConvexHull time = " << timer << std::endl;

  using boost::geometry::dsv;
  std::cout << "hull: " << dsv(hull) << std::endl;

  // convert to std::vector
  std::vector<Eigen::Vector3d> result;
  for (auto it = boost::begin(bg::exterior_ring(hull)); it != boost::end(bg::exterior_ring(hull)); ++it)
  {
    result.push_back(Eigen::Vector3d(bg::get<0>(*it), bg::get<1>(*it), bg::get<2>(*it)));
  }

  std::cout << "result.size()= " << result.size() << std::endl;

  return result;
}

std::vector<std::vector<Eigen::Vector3d>> convexHullsOfCurve(double t_start, double t_end, int intervals, double inc)
{
  std::vector<std::vector<Eigen::Vector3d>> convexHulls;

  double deltaT = (t_end - t_start) / (1.0 * intervals);
  std::cout << "deltaT= " << deltaT << std::endl;
  for (int i = 0; i < intervals; i++)
  {
    convexHulls.push_back(convexHullOfInterval(t_start + i * deltaT, t_start + (i + 1) * deltaT, inc));
  }

  return convexHulls;
}

int main()
{
  int n_pol = 5;
  int deg = 3;

  std::vector<std::vector<Eigen::Vector3d>> hulls = convexHullsOfCurve(0, 10, n_pol, 0.1);
  SolverNlopt snlopt(n_pol, deg);  // snlopt(a,g) a polynomials of degree 3
  snlopt.setTminAndTmax(0, 10);
  snlopt.setMaxValues(10, 10);  // v_max and a_max

  snlopt.setHulls(hulls);
  Eigen::Vector3d initial_point, final_point;
  initial_point << 0, 0, 0;
  final_point << 10, 0, 0;

  std::cout << "Setting Points\n";
  snlopt.setInitAndFinalPoints(initial_point, final_point);

  std::cout << "Optimizing\n";
  snlopt.optimize();

  /*  polygon poly;
    boost::geometry::read_wkt("polygon((    4.1 3.0"
                              ", 5.3 2.6, 5.4 1.2, 4.9 0.8, 2.9 0.7,2.0 1.3))",
                              poly);*/
}
