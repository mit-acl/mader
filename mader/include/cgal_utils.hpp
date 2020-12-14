/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include "mader_types.hpp"
#include <Eigen/Dense>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Convex_hull_traits_3.h>
#include <decomp_geometry/polyhedron.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Convex_hull_traits_3<K> Traits;
typedef Traits::Polyhedron_3 CGAL_Polyhedron_3;
typedef K::Segment_3 Segment_3;
typedef K::Plane_3 Plane_3;
// define point creator
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef CGAL::Creator_uniform_3<double, Point_3> PointCreator;

typedef std::vector<CGAL_Polyhedron_3> ConvexHullsOfCurve;
typedef std::vector<ConvexHullsOfCurve> ConvexHullsOfCurves;

namespace cu  // cgal utils
{
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

mt::ConvexHullsOfCurves_Std vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls);

vec_E<Polyhedron<3>> vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves);

CGAL_Polyhedron_3 convexHullOfPoints(const std::vector<Point_3>& points);

mt::Edges vectorGCALPol2edges(const ConvexHullsOfCurves& convexHulls);
}  // namespace cu