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
CGAL_Polyhedron_3 convexHullOfInterval(double t_start, double t_end, int samples_per_interval)
{
  /*  std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(-1, 1);  // doubles from -1 to 1
        double int_random = 1;
      double r = int_random * distribution(generator);
      double r2 = int_random * distribution(generator);
      double r3 = int_random * distribution(generator);*/

  samples_per_interval = std::max(samples_per_interval, 4);  // at least 4 samples per interval
  double inc = (t_end - t_start) / (1.0 * samples_per_interval);

  std::vector<Point_3> points;

  for (int i = 0; i < samples_per_interval; i++)
  {
    // Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    double t = t_start + i * inc;
    double x = sin(t) + 2 * sin(2 * t) - 5;  // traj_.function[0].value();  //    sin(t) + 2 * sin(2 * t);
    double y = cos(t) - 2 * cos(2 * t);      // traj_.function[1].value();  // cos(t) - 2 * cos(2 * t);
    double z = -sin(3 * t) + 1;              // traj_.function[2].value();  //-sin(3 * t);

    double half_side = 0.2 / 2.0;

    //"Minkowski sum along the trajectory: box centered on the trajectory"

    Point_3 p0(x + half_side, y + half_side, z + half_side);
    Point_3 p1(x + half_side, y - half_side, z - half_side);
    Point_3 p2(x + half_side, y + half_side, z - half_side);
    Point_3 p3(x + half_side, y - half_side, z + half_side);

    Point_3 p4(x - half_side, y - half_side, z - half_side);
    Point_3 p5(x - half_side, y + half_side, z + half_side);
    Point_3 p6(x - half_side, y + half_side, z - half_side);
    Point_3 p7(x - half_side, y - half_side, z + half_side);

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
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
  /*  if (CGAL::object_cast<Segment_3>(&ch_object))
      std::cout << "convex hull is a segment " << std::endl;
    else if (CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object))
      std::cout << "convex hull is a polyhedron " << std::endl;
    else
      std::cout << "convex hull error!" << std::endl;*/

  CGAL_Polyhedron_3 poly = *CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object);

  std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(), Plane_equation());  // Compute the planes

  /*
CGAL::set_pretty_mode(std::cout);
// std::copy(poly.planes_begin(), poly.planes_end(), std::ostream_iterator<Plane_3>(std::cout, "\n"));
*/
  return poly;
}

std::vector<CGAL_Polyhedron_3> convexHullsOfCurve(double t_start, double t_end, int intervals, int samples_per_interval)
{
  std::vector<CGAL_Polyhedron_3> convexHulls;

  double deltaT = (t_end - t_start) / (1.0 * intervals);
  std::cout << "deltaT= " << deltaT << std::endl;
  for (int i = 0; i < intervals; i++)
  {
    std::cout << "i= " << i << std::endl;
    convexHulls.push_back(convexHullOfInterval(t_start + i * deltaT, t_start + (i + 1) * deltaT, samples_per_interval));
  }

  std::cout << "Done with convexHullsOfCurve" << std::endl;

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
  /////////////////////////////////////

  double v_max = 3000;
  double a_max = 1000;
  state A;
  state G_term;
  /*  A.pos = Eigen::Vector3d(2.75, -0.222, 0.866);
    A.vel = Eigen::Vector3d(1.41, 0.422, -0.0466);
    A.accel = Eigen::Vector3d(0.305, 0.702, 0.243);
    G_term.pos = Eigen::Vector3d(4, 0.319, 1);
  */
  A.pos = Eigen::Vector3d(0, 0, 1);
  A.vel = Eigen::Vector3d(3, 3, 3);
  A.accel = Eigen::Vector3d(4, 5, 6);
  G_term.pos = Eigen::Vector3d(10, 0, 0);

  G_term.vel = Eigen::Vector3d(0, 0, 0);
  G_term.accel = Eigen::Vector3d(0, 0, 0);
  /////////////////////////////////////

  int n_pol = 7;  // was 7
  int deg = 3;
  int samples_per_interval = 5;

  double t_min = 8;   // 0;  // TODO this ros dependency shouldn't be here
  double t_max = 10;  // t_min + (G_term.pos - A.pos).norm() /
                      //    (0.4 * v_max);  // 72.8861;  // t_min + (A.pos - G_term.pos).norm() / (v_max);

  std::vector<CGAL_Polyhedron_3> hulls = convexHullsOfCurve(t_min, t_max, n_pol, samples_per_interval);
  std::cout << "hulls size=" << hulls.size() << std::endl;
  std::vector<std::vector<Eigen::Vector3d>> hulls_std = vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = vectorGCALPol2vectorJPSPol(hulls);
  std::cout << "hulls_std size=" << hulls_std.size() << std::endl;

  SolverNlopt snlopt(n_pol, deg, false);  // snlopt(a,g) a polynomials of degree 3

  snlopt.setHulls(hulls_std);

  // std::cout << "Optimizing, i=" << i << std::endl;

  snlopt.setMaxValues(v_max, a_max);  // v_max and a_max
  snlopt.setDC(0.1);                  // dc
  snlopt.setTminAndTmax(t_min, t_max);
  snlopt.setInitAndFinalStates(A, G_term);

  std::cout << "Calling optimize" << std::endl;
  bool result = snlopt.optimize();
  //}
  // X_whole_out = snlopt.X_temp_;

  /*  polygon poly;
    boost::geometry::read_wkt("polygon((    4.1 3.0"
                              ", 5.3 2.6, 5.4 1.2, 4.9 0.8, 2.9 0.7,2.0 1.3))",
                              poly);*/
}
