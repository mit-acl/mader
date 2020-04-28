#include "cgal_utils.hpp"
#include <CGAL/convex_hull_3.h>

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

ConvexHullsOfCurves_Std vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls)
{
  ConvexHullsOfCurves_Std convexHulls_of_curves_std;

  // std::cout << "convexHulls.size()= " << convexHulls.size() << std::endl;

  for (int index_curve = 0; index_curve < convexHulls.size(); index_curve++)
  {
    ConvexHullsOfCurve_Std convexHulls_of_curve_std;

    for (int i = 0; i < convexHulls[index_curve].size(); i++)  // for each interval along the curve
    {
      CGAL_Polyhedron_3 poly = convexHulls[index_curve][i];
      std::vector<Eigen::Vector3d> convexHull_std;
      for (CGAL_Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
      {
        Eigen::Vector3d vertex(v->point().x(), v->point().y(), v->point().z());
        convexHull_std.push_back(vertex);
        // std::cout << v->point() << std::endl;
      }

      convexHulls_of_curve_std.push_back(convexHull_std);
    }

    convexHulls_of_curves_std.push_back(convexHulls_of_curve_std);
  }
  // std::cout << "convexHulls_of_curves_std.size()= " << convexHulls_of_curves_std.size() << std::endl;

  return convexHulls_of_curves_std;
}

vec_E<Polyhedron<3>> vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves)
{
  vec_E<Polyhedron<3>> vector_of_polyhedron_jps;

  for (auto convex_hulls_of_curve : convex_hulls_of_curves)
  {
    for (auto polyhedron_i : convex_hulls_of_curve)
    {
      vec_E<Hyperplane<3>> hyperplanes;
      for (auto it = polyhedron_i.planes_begin(); it != polyhedron_i.planes_end(); it++)
      {
        Vector_3 n = it->orthogonal_vector();
        Point_3 p = it->point();
        hyperplanes.push_back(
            Hyperplane<3>(Eigen::Vector3d(p.x(), p.y(), p.z()), Eigen::Vector3d(n.x(), n.y(), n.z())));
      }
      Polyhedron<3> polyhedron_jps(hyperplanes);
      vector_of_polyhedron_jps.push_back(polyhedron_jps);
      // std::cout << red << bold << "Size=" << vector_of_polyhedron_jps.size() << reset << std::endl;
    }
  }
  return vector_of_polyhedron_jps;
}

CGAL_Polyhedron_3 convexHullOfPoints(const std::vector<Point_3>& points)
{
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

  return poly;

  /*
CGAL::set_pretty_mode(std::cout);
// std::copy(poly.planes_begin(), poly.planes_end(), std::ostream_iterator<Plane_3>(std::cout, "\n"));
*/
}