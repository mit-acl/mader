// Jesus Tordesillas Torres, jtorde@mit.edu, January 2020

#include <iostream>
#include <vector>
#include <iomanip>
#include <nlopt.hpp>

#include <Eigen/Dense>
#include <random>
#include "timer.hpp"
#include "solverNlopt.hpp"

/*#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polyhedron_traits_with_normals_3.h>*/

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Convex_hull_traits_3.h>
#include <CGAL/convex_hull_3.h>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Convex_hull_traits_3<K> Traits;
typedef Traits::Polyhedron_3 CGAL_Polyhedron_3;
typedef K::Segment_3 Segment_3;
typedef K::Plane_3 Plane_3;
// define point creator
typedef K::Point_3 Point_3;
typedef CGAL::Creator_uniform_3<double, Point_3> PointCreator;

#define DEG_POL 3
#define NUM_POL 5

typedef Timer MyTimer;

struct Plane_equation
{
  template <class Facet>
  typename Facet::Plane_3 operator()(Facet& f)
  {
    typename Facet::Halfedge_handle h = f.halfedge();
    typedef typename Facet::Plane_3 Plane;
    return Plane(h->vertex()->point(), h->next()->vertex()->point(), h->next()->next()->vertex()->point());
  }
};

// See https://doc.cgal.org/Manual/3.7/examples/Convex_hull_3/quickhull_3.cpp
CGAL_Polyhedron_3 convexHullOfInterval(double t_start, double t_end, double inc)
{
  /*  std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(-1, 1);  // doubles from -1 to 1
        double int_random = 1;
      double r = int_random * distribution(generator);
      double r2 = int_random * distribution(generator);
      double r3 = int_random * distribution(generator);*/

  std::vector<Point_3> points;

  for (double t = t_start; t <= t_end; t = t + inc)
  {
    // Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    double x = sin(t) + 2 * sin(2 * t);
    double y = cos(t) - 2 * cos(2 * t);
    double z = -sin(3 * t);

    Point_3 p(x, y, z);
    points.push_back(p);
  }

  // generate 3 points randomly on a sphere of radius 1.0
  // and copy them to a vector
  /*  CGAL::Random_points_in_sphere_3<Point_3, PointCreator> gen(2.0);
    std::vector<Point_3> points;
    CGAL::copy_n(gen, 6, std::back_inserter(points));*/

  // define object to hold convex hull
  CGAL::Object ch_object;

  // compute convex hull
  CGAL::convex_hull_3(points.begin(), points.end(), ch_object);

  // determine what kind of object it is
  if (CGAL::object_cast<Segment_3>(&ch_object))
    std::cout << "convex hull is a segment " << std::endl;
  else if (CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object))
    std::cout << "convex hull is a polyhedron " << std::endl;
  else
    std::cout << "convex hull error!" << std::endl;

  CGAL_Polyhedron_3 poly = *CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object);

  std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_equation());  // Compute the planes

  /*
CGAL::set_pretty_mode(std::cout);
// std::copy(poly.planes_begin(), poly.planes_end(), std::ostream_iterator<Plane_3>(std::cout, "\n"));
*/
  return poly;
}

std::vector<CGAL_Polyhedron_3> convexHullsOfCurve(double t_start, double t_end, int intervals, double inc)
{
  std::vector<CGAL_Polyhedron_3> convexHulls;

  double deltaT = (t_end - t_start) / (1.0 * intervals);
  std::cout << "deltaT= " << deltaT << std::endl;
  for (int i = 0; i < intervals; i++)
  {
    convexHulls.push_back(convexHullOfInterval(t_start + i * deltaT, t_start + (i + 1) * deltaT, inc));
  }

  return convexHulls;
}

std::vector<std::vector<Eigen::Vector3d>> vectorGCALPol2vectorStdEigen(std::vector<CGAL_Polyhedron_3> convexHulls)
{
  std::vector<std::vector<Eigen::Vector3d>> convexHulls_std;

  for (int i = 0; i < convexHulls.size(); i++)
  {
    CGAL_Polyhedron_3 poly = convexHulls[i];
    std::vector<Eigen::Vector3d> convexHull_std;
    for (CGAL_Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
    {
      Eigen::Vector3d vertex(v->point().x(), v->point().y(), v->point().z());
      convexHull_std.push_back(vertex);
      // std::cout << v->point() << std::endl;
    }
    convexHulls_std.push_back(convexHull_std);
  }

  return convexHulls_std;
}

int main()
{
  int n_pol = 7;
  int deg = 3;

  std::vector<CGAL_Polyhedron_3> hulls = convexHullsOfCurve(0, 10, n_pol, 0.1);
  std::cout << "hulls size=" << hulls.size() << std::endl;
  std::vector<std::vector<Eigen::Vector3d>> hulls_std = vectorGCALPol2vectorStdEigen(hulls);

  std::cout << "hulls_std size=" << hulls_std.size() << std::endl;

  SolverNlopt snlopt(n_pol, deg, true);  // snlopt(a,g) a polynomials of degree 3

  snlopt.setHulls(hulls_std);
  /*
    for (int i = 0; i < 100; i++)
    {*/
  // std::cout << "Optimizing, i=" << i << std::endl;

  double v_max = 10;
  double a_max = 10;

  snlopt.setMaxValues(300, 200);  // v_max and a_max
  snlopt.setDC(0.1);              // dc

  snlopt.setTminAndTmax(0, 30);

  state initial_state, final_state;
  initial_state.setPos(-10, -10, -10);
  initial_state.setVel(1, 0, 0);
  initial_state.setAccel(0, 0, 0);
  final_state.setPos(10, 0, 0);
  final_state.setVel(0, 0, 0);
  final_state.setAccel(0, 0, 0);
  snlopt.setInitAndFinalStates(initial_state, final_state);

  std::cout << "Calling optimize" << std::endl;
  snlopt.optimize();
  std::cout << "Below of loop\n";

  //}
  // X_whole_out = snlopt.X_temp_;

  /*  polygon poly;
    boost::geometry::read_wkt("polygon((    4.1 3.0"
                              ", 5.3 2.6, 5.4 1.2, 4.9 0.8, 2.9 0.7,2.0 1.3))",
                              poly);*/
}
