/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "cgal_utils.hpp"
#include <CGAL/convex_hull_3.h>
#include <CGAL/Triangulation_3.h>

// Convert several polyhedra to a vector that contains all the edges of all these polyhedra
mt::Edges cu::vectorGCALPol2edges(const ConvexHullsOfCurves& convexHulls)
{
  // See example here:
  // http://cgal-discuss.949826.n4.nabble.com/Take-the-triangles-of-a-polyhedron-td4460275.html

  // Other related questions:
  // https://doc.cgal.org/5.0/Triangulation_3/Triangulation_3_2for_loop_8cpp-example.html
  // https://stackoverflow.com/questions/4837179/getting-a-vertex-handle-from-an-edge-iterator

  mt::Edges all_edges;

  for (int index_curve = 0; index_curve < convexHulls.size(); index_curve++)  // for each curve
  {
    for (int i = 0; i < convexHulls[index_curve].size(); i++)  // for each interval along the curve
    {
      CGAL_Polyhedron_3 poly = convexHulls[index_curve][i];

      for (CGAL_Polyhedron_3::Edge_iterator w = poly.edges_begin(); w != poly.edges_end();
           ++w)  // for all the edges of that polyhedron
      {
        // std::cout << "First Vertex of the edge" << w->opposite()->vertex()->point() << std::endl;
        // std::cout << "Second Vertex of the edge" << w->vertex()->point() << std::endl;

        Eigen::Vector3d vertex1(w->opposite()->vertex()->point().x(), w->opposite()->vertex()->point().y(),
                                w->opposite()->vertex()->point().z());

        Eigen::Vector3d vertex2(w->vertex()->point().x(), w->vertex()->point().y(), w->vertex()->point().z());

        std::pair<Eigen::Vector3d, Eigen::Vector3d> edge;
        edge.first = vertex1;
        edge.second = vertex2;

        all_edges.push_back(edge);
      }
    }
  }

  return all_edges;
}

mt::ConvexHullsOfCurves_Std cu::vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls)
{
  mt::ConvexHullsOfCurves_Std convexHulls_of_curves_std;

  // std::cout << "convexHulls.size()= " << convexHulls.size() << std::endl;

  for (int index_curve = 0; index_curve < convexHulls.size(); index_curve++)  // for each curve
  {
    mt::ConvexHullsOfCurve_Std convexHulls_of_curve_std;

    for (int i = 0; i < convexHulls[index_curve].size(); i++)  // for each interval along the curve
    {
      CGAL_Polyhedron_3 poly = convexHulls[index_curve][i];

      mt::Polyhedron_Std convexHull_std(3,
                                        poly.size_of_vertices());  // poly.size_of_vertices() is the number of vertexes
      // std::vector<Eigen::Vector3d> convexHull_std;
      int j = 0;
      for (CGAL_Polyhedron_3::Vertex_iterator v = poly.vertices_begin(); v != poly.vertices_end(); ++v)
      {
        Eigen::Vector3d vertex(v->point().x(), v->point().y(), v->point().z());
        convexHull_std.col(j) = vertex;
        // convexHull_std.push_back(vertex);
        j = j + 1;
        // std::cout << v->point() << std::endl;
      }

      convexHulls_of_curve_std.push_back(convexHull_std);
    }

    convexHulls_of_curves_std.push_back(convexHulls_of_curve_std);
  }
  // std::cout << "convexHulls_of_curves_std.size()= " << convexHulls_of_curves_std.size() << std::endl;

  return convexHulls_of_curves_std;
}

vec_E<Polyhedron<3>> cu::vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves)
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

CGAL_Polyhedron_3 cu::convexHullOfPoints(const std::vector<Point_3>& points)
{
  // generate 3 points randomly on a sphere of radius 1.0
  // and copy them to a vector
  /*  CGAL::Random_points_in_sphere_3<Point_3, PointCreator> gen(2.0);
    std::vector<Point_3> points;
    CGAL::copy_n(gen, 6, std::back_inserter(points));*/

  // define object to hold convex hull
  CGAL::Object ch_object;

  // compute convex hull
  // std::cout << "Computing the convex hull CGAL for these points:" << std::endl;
  // for (auto points_i : points)
  // {
  //   std::cout << points_i.x() << ", " << points_i.y() << ", " << points_i.z() << std::endl;
  // }
  CGAL::convex_hull_3(points.begin(), points.end(), ch_object);
  // std::cout << "convexHullCgal Computed!" << std::endl;

  // determine what kind of object it is
  // if (CGAL::object_cast<Segment_3>(&ch_object))
  //   std::cout << "convex hull is a segment " << std::endl;
  // else if (CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object))
  //   std::cout << "convex hull is a polyhedron " << std::endl;
  // else
  //   std::cout << "convex hull error!" << std::endl;

  CGAL_Polyhedron_3 poly = *CGAL::object_cast<CGAL_Polyhedron_3>(&ch_object);

  std::transform(poly.facets_begin(), poly.facets_end(), poly.planes_begin(),
                 cu::Plane_equation());  // Compute the planes

  return poly;

  /*
CGAL::set_pretty_mode(std::cout);
// std::copy(poly.planes_begin(), poly.planes_end(), std::ostream_iterator<Plane_3>(std::cout, "\n"));
*/
}