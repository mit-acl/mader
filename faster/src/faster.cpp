#include "faster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>

/*#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polyhedron_traits_with_normals_3.h>*/

using namespace JPS;
using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
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

Faster::Faster(parameters par) : par_(par)
{
  drone_status_ == DroneStatus::YAWING;
  // mtx_G.lock();
  G_.pos << 0, 0, 0;
  // mtx_G.unlock();
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  // Setup of jps_manager
  std::cout << "par_.wdx / par_.res =" << par_.wdx / par_.res << std::endl;
  jps_manager_.setNumCells((int)par_.wdx / par_.res, (int)par_.wdy / par_.res, (int)par_.wdz / par_.res);
  jps_manager_.setFactorJPS(par_.factor_jps);
  jps_manager_.setResolution(par_.res);
  jps_manager_.setInflationJPS(par_.inflation_jps);
  jps_manager_.setZGroundAndZMax(par_.z_ground, par_.z_max);
  // jps_manager_.setVisual(par_.visual);
  jps_manager_.setDroneRadius(par_.drone_radius);

  // Setup of jps_manager_dyn_
  jps_manager_dyn_.setNumCells((int)par_.wdx / par_.res, (int)par_.wdy / par_.res, (int)par_.wdz / par_.res);
  jps_manager_dyn_.setFactorJPS(par_.factor_jps);
  jps_manager_dyn_.setResolution(par_.res);
  jps_manager_dyn_.setInflationJPS(par_.inflation_jps);
  jps_manager_dyn_.setZGroundAndZMax(par_.z_ground, par_.z_max);
  // jps_manager_dyn_.setVisual(par_.visual);
  jps_manager_dyn_.setDroneRadius(par_.drone_radius);

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };

  // Setup of sg_whole_
  sg_whole_.setN(par_.N_whole);
  sg_whole_.createVars();
  sg_whole_.setDC(par_.dc);
  sg_whole_.setBounds(max_values);
  sg_whole_.setForceFinalConstraint(true);
  sg_whole_.setFactorInitialAndFinalAndIncrement(1, 10, par_.increment_whole);
  sg_whole_.setVerbose(par_.gurobi_verbose);
  sg_whole_.setThreads(par_.gurobi_threads);
  sg_whole_.setWMax(par_.w_max);

  // Setup of sg_safe_
  sg_safe_.setN(par_.N_safe);
  sg_safe_.createVars();
  sg_safe_.setDC(par_.dc);
  sg_safe_.setBounds(max_values);
  sg_safe_.setForceFinalConstraint(false);
  sg_safe_.setFactorInitialAndFinalAndIncrement(1, 10, par_.increment_safe);
  sg_safe_.setVerbose(par_.gurobi_verbose);
  sg_safe_.setThreads(par_.gurobi_threads);
  sg_safe_.setWMax(par_.w_max);

  pclptr_unk_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pclptr_map_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  changeDroneStatus(DroneStatus::GOAL_REACHED);
  resetInitialization();

  n_pol_ = 7;
  deg_ = 3;
}

void Faster::updateTrajObstacles(std::vector<dynTraj> trajs)
{
  mtx_trajs_.lock();
  trajs_.clear();

  // std::cout << "init of updateTrajObstacles" << std::endl;

  for (auto traj : trajs)
  {
    dynTrajCompiled traj_compiled;
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
      // std::cout << "function_i=" << function_i << std::endl;

      parser_t parser;
      parser.compile(function_i, expression);
      traj_compiled.function.push_back(expression);
    }

    // std::cout << "traj_compiled.function.size()= " << traj_compiled.function.size() << std::endl;

    traj_compiled.bbox = traj.bbox;
    trajs_.push_back(traj_compiled);
  }

  // std::cout << "end of updateTrajObstacles" << std::endl;

  mtx_trajs_.unlock();

  // std::cout << "end of updateTrajObstacles" << std::endl;
}

vec_E<Polyhedron<3>> Faster::vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves)
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

// See https://doc.cgal.org/Manual/3.7/examples/Convex_hull_3/quickhull_3.cpp
CGAL_Polyhedron_3 Faster::convexHullOfInterval(dynTrajCompiled& traj, double t_start, double t_end)
{
  /*  std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(-1, 1);  // doubles from -1 to 1
        double int_random = 1;
      double r = int_random * distribution(generator);
      double r2 = int_random * distribution(generator);
      double r3 = int_random * distribution(generator);*/

  int samples_per_interval = std::max(par_.samples_per_interval, 4);  // at least 4 samples per interval
  double inc = (t_end - t_start) / (1.0 * samples_per_interval);

  std::vector<Point_3> points;

  // mtx_trajs_.lock();

  for (int i = 0; i < samples_per_interval; i++)
  {
    // Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    t_ = t_start + i * inc;
    // std::cout << "calling value(), traj_compiled.function.size()= " << traj.function.size() << std::endl;

    double x = traj.function[0].value();  //    sin(t) + 2 * sin(2 * t);
    double y = traj.function[1].value();  // cos(t) - 2 * cos(2 * t);
    double z = traj.function[2].value();  //-sin(3 * t);

    double half_side = par_.drone_radius / 2.0;

    //"Minkowski sum along the trajectory: box centered on the trajectory"

    Point_3 p0(x + traj.bbox[0] / 2.0, y + traj.bbox[1] / 2.0, z + traj.bbox[2] / 2.0);
    Point_3 p1(x + traj.bbox[0] / 2.0, y - traj.bbox[1] / 2.0, z - traj.bbox[2] / 2.0);
    Point_3 p2(x + traj.bbox[0] / 2.0, y + traj.bbox[1] / 2.0, z - traj.bbox[2] / 2.0);
    Point_3 p3(x + traj.bbox[0] / 2.0, y - traj.bbox[1] / 2.0, z + traj.bbox[2] / 2.0);

    Point_3 p4(x - traj.bbox[0] / 2.0, y - traj.bbox[1] / 2.0, z - traj.bbox[2] / 2.0);
    Point_3 p5(x - traj.bbox[0] / 2.0, y + traj.bbox[1] / 2.0, z + traj.bbox[2] / 2.0);
    Point_3 p6(x - traj.bbox[0] / 2.0, y + traj.bbox[1] / 2.0, z - traj.bbox[2] / 2.0);
    Point_3 p7(x - traj.bbox[0] / 2.0, y - traj.bbox[1] / 2.0, z + traj.bbox[2] / 2.0);

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
  }
  // mtx_trajs_.unlock();

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

ConvexHullsOfCurve Faster::convexHullsOfCurve(dynTrajCompiled& traj, double t_start, double t_end)
{
  ConvexHullsOfCurve convexHulls;

  double deltaT = (t_end - t_start) / (1.0 * par_.n_pol);  // n_pol is the number of intervals
  // std::cout << "deltaT= " << deltaT << std::endl;
  for (int i = 0; i < par_.n_pol; i++)
  {
    // std::cout << "i= " << i << std::endl;
    convexHulls.push_back(convexHullOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT));
  }

  // std::cout << "Done with convexHullsOfCurve" << std::endl;

  return convexHulls;
}

ConvexHullsOfCurves Faster::convexHullsOfCurves(double t_start, double t_end)
{
  ConvexHullsOfCurves result;
  mtx_trajs_.lock();
  for (auto traj : trajs_)
  {
    // std::cout << "above, traj.function.size()= " << traj.function.size() << std::endl;
    // std::cout << "going to call convexHullsOfCurve" << std::endl;
    result.push_back(convexHullsOfCurve(traj, t_start, t_end));
    // std::cout << "called convexHullsOfCurve" << std::endl;
  }
  mtx_trajs_.unlock();

  // std::cout << "end of convexHullsOfCurves" << std::endl;

  return result;
}

ConvexHullsOfCurves_Std Faster::vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls)
{
  ConvexHullsOfCurves_Std convexHulls_of_curves_std;

  std::cout << "convexHulls.size()= " << convexHulls.size() << std::endl;

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
  std::cout << "convexHulls_of_curves_std.size()= " << convexHulls_of_curves_std.size() << std::endl;

  return convexHulls_of_curves_std;
}

void Faster::createMoreVertexes(vec_Vecf<3>& path, double d)
{
  for (int j = 0; j < path.size() - 1; j++)
  {
    double dist = (path[j + 1] - path[j]).norm();
    int vertexes_to_add = floor(dist / d);
    Eigen::Vector3d v = (path[j + 1] - path[j]).normalized();
    // std::cout << "Vertexes to add=" << vertexes_to_add << std::endl;
    if (dist > d)
    {
      for (int i = 0; i < vertexes_to_add; i++)
      {
        path.insert(path.begin() + j + 1, path[j] + v * d);
        j = j + 1;
      }
    }
  }
}

void Faster::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map, pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk)
{
  mtx_map.lock();
  mtx_unk.lock();

  // TODO
  pclptr_map_ = pclptr_map;
  pclptr_unk_ = pclptr_unk;

  // Update even when there are no points
  jps_manager_.createNewMap(state_.pos);
  jps_manager_.addToMap(pclptr_map_, par_.inflation_jps, par_.inflation_jps,
                        par_.inflation_jps);  // same inflation in all directions

  if (pclptr_map_->width != 0 && pclptr_map_->height != 0)  // Point Cloud is not empty
  {
    kdtree_map_.setInputCloud(pclptr_map_);
    kdtree_map_initialized_ = 1;
    jps_manager_.vec_o_ = pclptr_to_vec(pclptr_map_);  // Needed for the ellipsoid decomp
  }
  else
  {
    std::cout << "Occupancy Grid received is empty, maybe map is too small?" << std::endl;
  }

  if (pclptr_unk_->points.size() == 0)
  {
    std::cout << "Unkown cloud has 0 points" << std::endl;
    return;
  }
  else
  {
    kdtree_unk_.setInputCloud(pclptr_unk_);
    kdtree_unk_initialized_ = 1;
    jps_manager_.vec_uo_ = pclptr_to_vec(pclptr_unk_);  // insert unknown space
    jps_manager_.vec_uo_.insert(jps_manager_.vec_uo_.end(), jps_manager_.vec_o_.begin(),
                                jps_manager_.vec_o_.end());  // append known space
  }

  mtx_map.unlock();
  mtx_unk.unlock();
}

void Faster::setTerminalGoal(state& term_goal)
{
  mtx_G_term.lock();
  mtx_G.lock();
  mtx_state.lock();
  mtx_planner_status_.lock();

  G_term_.pos = term_goal.pos;
  Eigen::Vector3d temp = state_.pos;
  G_.pos = projectPointToBox(temp, G_term_.pos, par_.wdx, par_.wdy, par_.wdz);
  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    changeDroneStatus(DroneStatus::YAWING);  // not done when drone_status==traveling
  }
  terminal_goal_initialized_ = true;

  mtx_state.unlock();
  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_planner_status_.unlock();
}

void Faster::getG(state& G)
{
  G = G_;
}

void Faster::getState(state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

int Faster::findIndexR(int indexH)
{
  // Ignore z to obtain this heuristics (if not it can become very conservative)
  // mtx_X_U_temp.lock();
  Eigen::Vector2d posHk;
  posHk << sg_whole_.X_temp_[indexH].pos(0), sg_whole_.X_temp_[indexH].pos(1);
  int indexR = indexH;

  for (int i = 0; i <= indexH; i = i + 1)  // Loop from A to H
  {
    Eigen::Vector2d vel;
    vel << sg_whole_.X_temp_[i].vel(0), sg_whole_.X_temp_[i].vel(1);  //(i, 3), sg_whole_.X_temp_(i, 4);

    Eigen::Vector2d pos;
    pos << sg_whole_.X_temp_[i].pos(0), sg_whole_.X_temp_[i].pos(1);

    Eigen::Vector2d braking_distance =
        (vel.array() * (posHk - pos).array()).sign() * vel.array().square() / (2 * par_.delta_a * par_.a_max);

    // std::cout << "braking_distance=" << braking_distance.transpose() << std::endl;
    // std::cout << "(posHk - pos).cwiseAbs().array())=" << (posHk - pos).cwiseAbs().array().transpose() << std::endl;

    bool thereWillBeCollision =
        (braking_distance.array() > (posHk - pos).cwiseAbs().array()).any();  // Any of the braking distances (in x, y,
                                                                              // z) is bigger than the distance to the
                                                                              // obstacle in that direction
    if (thereWillBeCollision)
    {
      indexR = i;

      if (indexR == 0)
      {
        std::cout << bold << red << "R was taken in A" << reset << std::endl;
      }

      break;
    }
  }
  std::cout << red << bold << "indexR=" << indexR << " /" << sg_whole_.X_temp_.size() - 1 << reset << std::endl;
  // std::cout << red << bold << "indexH=" << indexH << " /" << sg_whole_.X_temp_.rows() - 1 << reset << std::endl;
  // mtx_X_U_temp.unlock();

  return indexR;
}

int Faster::findIndexH(bool& needToComputeSafePath)
{
  int n = 1;  // find one neighbour
  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  needToComputeSafePath = false;

  mtx_unk.lock();
  mtx_X_U_temp.lock();
  int indexH = sg_whole_.X_temp_.size() - 1;

  for (int i = 0; i < sg_whole_.X_temp_.size(); i = i + 10)
  {  // Sample points along the trajectory

    Eigen::Vector3d tmp = sg_whole_.X_temp_[i].pos;
    pcl::PointXYZ searchPoint(tmp(0), tmp(1), tmp(2));

    if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (sqrt(pointNKNSquaredDistance[0]) < par_.drone_radius)
      {
        needToComputeSafePath = true;  // There is intersection, so there is need to compute safe path
        indexH = (int)(par_.delta_H * i);
        break;
      }
    }
  }
  std::cout << red << bold << "indexH=" << indexH << " /" << sg_whole_.X_temp_.size() - 1 << reset << std::endl;
  mtx_unk.unlock();
  mtx_X_U_temp.unlock();

  return indexH;
}

bool Faster::ARisInFreeSpace(int index)
{  // We have to check only against the unkown space (A-R won't intersect the obstacles for sure)

  // std::cout << "In ARisInFreeSpace, radius_drone= " << par_.drone_radius << std::endl;
  int n = 1;  // find one neighbour

  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  bool isFree = true;

  // std::cout << "Before mtx_unk" << std::endl;
  mtx_unk.lock();
  mtx_X_U_temp.lock();
  // std::cout << "After mtx_unk. index=" << index << std::endl;
  for (int i = 0; i < index; i = i + 10)
  {  // Sample points along the trajectory
     // std::cout << "i=" << i << std::endl;
    Eigen::Vector3d tmp = sg_whole_.X_temp_[i].pos;
    pcl::PointXYZ searchPoint(tmp(0), tmp(1), tmp(2));

    if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (sqrt(pointNKNSquaredDistance[0]) < 0.2)
      {  // TODO: 0.2 is the radius of the drone.
        std::cout << "A->R collides, with d=" << sqrt(pointNKNSquaredDistance[0])
                  << ", radius_drone=" << par_.drone_radius << std::endl;
        isFree = false;
        break;
      }
    }
  }

  mtx_unk.unlock();
  mtx_X_U_temp.unlock();

  return isFree;
}

// Returns the first collision of JPS with the map (i.e. with the known obstacles). Note that JPS will collide with a
// map B if JPS was computed using an older map A
// If type_return==Intersection, it returns the last point in the JPS path that is at least par_.inflation_jps from map
Eigen::Vector3d Faster::getFirstCollisionJPS(vec_Vecf<3>& path, bool* thereIsIntersection, int map, int type_return)
{
  vec_Vecf<3> original = path;

  Eigen::Vector3d first_element = path[0];
  Eigen::Vector3d last_search_point = path[0];
  Eigen::Vector3d inters = path[0];
  pcl::PointXYZ pcl_search_point = eigenPoint2pclPoint(path[0]);

  Eigen::Vector3d result;

  // occupied (map)
  int n = 1;
  std::vector<int> id_map(n);
  std::vector<float> dist2_map(n);  // squared distance
  double r = 1000000;
  // printElementsOfJPS(path);
  // printf("In 2\n");

  mtx_map.lock();
  mtx_unk.lock();

  // Find the next eig_search_point
  int last_id = -1;  // this is the last index inside the sphere
  int iteration = 0;
  while (path.size() > 0)
  {
    // std::cout<<red<<"New Iteration, iteration="<<iteration<<reset<<std::endl;
    // std::cout << red << "Searching from point=" << path[0].transpose() << reset << std::endl;
    pcl_search_point = eigenPoint2pclPoint(path[0]);

    int number_of_neigh;

    if (map == MAP)
    {
      number_of_neigh = kdtree_map_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
    }
    else  // map == UNKNOWN_MAP
    {
      number_of_neigh = kdtree_unk_.nearestKSearch(pcl_search_point, n, id_map, dist2_map);
      // std::cout << "In unknown_map, number of neig=" << number_of_neigh << std::endl;
    }
    // printf("************NearestSearch: TotalTime= %0.2f ms\n", 1000 * (ros::Time::now().toSec() - before));

    if (number_of_neigh > 0)
    {
      r = sqrt(dist2_map[0]);

      // std::cout << "r=" << r << std::endl;
      // std::cout << "Point=" << r << std::endl;

      if (r < par_.drone_radius)  // collision of the JPS path and an inflated obstacle --> take last search point
      {
        // std::cout << "Collision detected" << std::endl;  // We will return the search_point
        // pubJPSIntersection(inters);
        // inters = path[0];  // path[0] is the search_point I'm using.
        if (iteration == 0)
        {
          std::cout << red << bold << "The first point is in collision --> Hacking" << reset << std::endl;
        }
        switch (type_return)
        {
          case RETURN_LAST_VERTEX:
            result = last_search_point;
            break;
          case RETURN_INTERSECTION:
            if (iteration == 0)
            {  // Hacking (TODO)
              Eigen::Vector3d tmp;
              tmp << original[0](0) + 0.01, original[0](1), original[0](2);
              path.clear();
              path.push_back(original[0]);
              path.push_back(tmp);
              result = path[path.size() - 1];
              // result=original[original.size() - 1];
            }
            else
            {
              // std::cout << "In Return Intersection, last_id=" << last_id<<el_eliminated<< std::endl;
              int vertexes_eliminated_tmp = original.size() - path.size() + 1;
              // std::cout << "In Return Intersection, vertexes_eliminated_tmp=" << vertexes_eliminated_tmp <<
              // std::endl;
              original.erase(original.begin() + vertexes_eliminated_tmp,
                             original.end());  // Now original contains all the elements eliminated
              original.push_back(path[0]);

              /*              std::cout << "Result before reduceJPSbyDistance" << original[original.size() -
                 1].transpose()
                                      << std::endl;*/

              // This is to force the intersection point to be at least par_.drone_radius away from the obstacles
              reduceJPSbyDistance(original, par_.drone_radius);

              result = original[original.size() - 1];

              // std::cout<<"Result here is"<<result.transpose()<<std::endl;

              path = original;
            }
            // Copy the resulting path to the reference
            /*     std::reverse(original.begin(), original.end());  // flip all the vector
               result = getFirstIntersectionWithSphere(original, par_.inflation_jps, original[0]);*/
            break;
        }

        *thereIsIntersection = true;

        break;  // Leave the while loop
      }

      bool no_points_outside_sphere = false;

      inters = getFirstIntersectionWithSphere(path, r, path[0], &last_id, &no_points_outside_sphere);
      // printf("**********Found it*****************\n");
      if (no_points_outside_sphere == true)
      {  // JPS doesn't intersect with any obstacle
        *thereIsIntersection = false;
        /*        std::cout << "JPS provided doesn't intersect any obstacles, returning the first element of the path
           you gave " "me\n"
                          << std::endl;*/
        result = first_element;

        if (type_return == RETURN_INTERSECTION)
        {
          result = original[original.size() - 1];
          path = original;
        }

        break;  // Leave the while loop
      }
      // printf("In 4\n");

      last_search_point = path[0];
      // Remove all the points of the path whose id is <= to last_id:
      path.erase(path.begin(), path.begin() + last_id + 1);

      // and add the intersection as the first point of the path
      path.insert(path.begin(), inters);
    }
    else
    {  // There is no neighbours
      *thereIsIntersection = false;
      ROS_INFO("JPS provided doesn't intersect any obstacles, returning the first element of the path you gave me\n");
      result = first_element;

      if (type_return == RETURN_INTERSECTION)
      {
        result = original[original.size() - 1];
        path = original;
      }

      break;
    }
    iteration = iteration + 1;
  }
  mtx_map.unlock();
  mtx_unk.unlock();

  return result;
}

void Faster::updateState(state data)
{
  state_ = data;

  if (state_initialized_ == false)
  {
    state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    plan_.push_back(tmp);
  }

  state_initialized_ = true;
}

bool Faster::initializedAllExceptPlanner()
{
  if (!state_initialized_ || !kdtree_map_initialized_ || !kdtree_unk_initialized_ || !terminal_goal_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "kdtree_map_initialized_= " << kdtree_map_initialized_ << std::endl;
        std::cout << "kdtree_unk_initialized_= " << kdtree_unk_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

bool Faster::initialized()
{
  if (!state_initialized_ || !kdtree_map_initialized_ || !kdtree_unk_initialized_ || !terminal_goal_initialized_ ||
      !planner_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "kdtree_map_initialized_= " << kdtree_map_initialized_ << std::endl;
        std::cout << "kdtree_unk_initialized_= " << kdtree_unk_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;
        std::cout << "planner_initialized_= " << planner_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

void Faster::replan(vec_Vecf<3>& JPS_safe_out, vec_Vecf<3>& JPS_whole_out, vec_E<Polyhedron<3>>& poly_safe_out,
                    vec_E<Polyhedron<3>>& poly_whole_out, std::vector<state>& X_safe_out,
                    std::vector<state>& X_whole_out, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcloud_jps)
{
  // std::cout << "here1" << std::endl;

  MyTimer replanCB_t(true);

  // std::cout << "here2" << std::endl;

  if (initializedAllExceptPlanner() == false)
  {
    return;
  }
  // std::cout << "here3" << std::endl;

  sg_whole_.ResetToNormalState();
  sg_safe_.ResetToNormalState();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// G <-- Project GTerm ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_state.lock();
  mtx_G.lock();
  mtx_G_term.lock();

  state state_local = state_;
  state G;
  G.pos = projectPointToBox(state_local.pos, G_term_.pos, par_.wdx, par_.wdy, par_.wdz);
  state G_term = G_term_;  // Local copy of the terminal terminal goal

  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_state.unlock();

  // Check if we have reached the goal
  double dist_to_goal = (G_term.pos - state_local.pos).norm();
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
  }
  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING))
  {
    // std::cout << "No replanning needed because" << std::endl;
    // print_status();
    return;
  }

  std::cout << bold << "************IN REPLAN CB*********" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  state A;
  int k_safe, k_end_whole;

  // If k_end_whole=0, then A = plan_.back() = plan_[plan_.size() - 1]
  k_end_whole = std::max((int)(plan_.size() - deltaT_), 0);
  A = plan_.get(plan_.size() - 1 - k_end_whole);

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve JPS //////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  bool solvedjps = false;
  MyTimer timer_jps(true);

  vec_Vecf<3> JPSk;
  // JPSk = jps_manager_.solveJPS3D(A.pos, G.pos, &solvedjps, 1);
  JPSk.push_back(A.pos);  // hack to ignore the static obstacles for now
  JPSk.push_back(G.pos);  // hack to ignore the static obstacles for now
  solvedjps = true;       // hack to ignore the static obstacles for now

  if (solvedjps == false)
  {
    std::cout << bold << red << "JPS didn't find a solution" << reset << std::endl;
    return;
  }

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Find JPS_in ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double ra = std::min((dist_to_goal - 0.001), par_.Ra);  // radius of the sphere S
  bool noPointsOutsideS;
  int li1;  // last index inside the sphere of JPSk
  state E;
  E.pos = getFirstIntersectionWithSphere(JPSk, ra, JPSk[0], &li1, &noPointsOutsideS);
  vec_Vecf<3> JPS_in(JPSk.begin(), JPSk.begin() + li1 + 1);
  if (noPointsOutsideS == false)
  {
    JPS_in.push_back(E.pos);
  }
  // createMoreVertexes in case dist between vertexes is too big
  createMoreVertexes(JPS_in, par_.dist_max_vertexes);
  /*
    //////////////////////////////////////////////////////////////////////////
    ///////////////// Solve with GUROBI Whole trajectory /////////////////////
    //////////////////////////////////////////////////////////////////////////

    if (par_.use_faster == true)
    {
      vec_Vecf<3> JPS_whole = JPS_in;
      deleteVertexes(JPS_whole, par_.max_poly_whole);
      E.pos = JPS_whole[JPS_whole.size() - 1];

      // Convex Decomp around JPS_whole
      MyTimer cvx_ellip_decomp_t(true);
      jps_manager_.cvxEllipsoidDecomp(JPS_whole, OCCUPIED_SPACE, l_constraints_whole_, poly_whole_out);
      std::cout << "poly_whole_out= " << poly_whole_out.size() << std::endl;

      // Check if G is inside poly_whole
      bool isGinside_whole = l_constraints_whole_[l_constraints_whole_.size() - 1].inside(G.pos);
      E.pos = (isGinside_whole == true) ? G.pos : E.pos;

      // Set Initial cond, Final cond, and polytopes for the whole traj
      sg_whole_.setX0(A);
      sg_whole_.setXf(E);
      sg_whole_.setPolytopes(l_constraints_whole_);

      std::cout << "Initial Position is inside= " << l_constraints_whole_[l_constraints_whole_.size() - 1].inside(A.pos)
                << std::endl;
      std::cout << "Final Position is inside= " << l_constraints_whole_[l_constraints_whole_.size() - 1].inside(E.pos)
                << std::endl;

      // Solve with Gurobi
      MyTimer whole_gurobi_t(true);
      bool solved_whole = sg_whole_.genNewTraj();

      if (solved_whole == false)
      {
        std::cout << bold << red << "No solution found for the whole trajectory" << reset << std::endl;
        return;
      }

      // Get Results
      sg_whole_.fillX();

      // Copy for visualization
      X_whole_out = sg_whole_.X_temp_;
      JPS_whole_out = JPS_whole;
    }
    else
    {  // Dummy whole trajectory
      state dummy;
      std::vector<state> dummy_vector;
      dummy_vector.push_back(dummy);
      sg_whole_.X_temp_ = dummy_vector;
    }

    // std::cout << "This is the WHOLE TRAJECTORY" << std::endl;
    // printStateVector(sg_whole_.X_temp_);
    // std::cout << "===========================" << std::endl;

    //////////////////////////////////////////////////////////////////////////
    ///////////////// Solve with GUROBI Safe trajectory /////////////////////
    //////////////////////////////////////////////////////////////////////////

    vec_Vecf<3> JPSk_inside_sphere_tmp = JPS_in;
    bool thereIsIntersection2;
    // state M;
    M_.pos = getFirstCollisionJPS(JPSk_inside_sphere_tmp, &thereIsIntersection2, UNKNOWN_MAP,
                                  RETURN_INTERSECTION);  // results saved in JPSk_inside_sphere_tmp

    bool needToComputeSafePath;
    int indexH = findIndexH(needToComputeSafePath);

    std::cout << "NeedToComputeSafePath=" << needToComputeSafePath << std::endl;

    if (par_.use_faster == false)
    {
      needToComputeSafePath = true;
    }

    if (needToComputeSafePath == false)
    {
      k_safe = indexH;
      sg_safe_.X_temp_ = std::vector<state>();  // 0 elements
    }
    else
    {
      mtx_X_U_temp.lock();

      k_safe = findIndexR(indexH);
      state R = sg_whole_.X_temp_[k_safe];

      mtx_X_U_temp.unlock();

      // if (ARisInFreeSpace(indexR_) == false and takeoff_done_ == true)
      // {
      //  std::cout << red << bold << "The piece A-->R is not in Free Space" << std::endl;
      //  return;
      //}

      JPSk_inside_sphere_tmp[0] = R.pos;

      if (par_.use_faster == false)
      {
        JPSk_inside_sphere_tmp[0] = A.pos;
      }

      vec_Vecf<3> JPS_safe = JPSk_inside_sphere_tmp;

      // delete extra vertexes
      deleteVertexes(JPS_safe, par_.max_poly_safe);
      M_.pos = JPS_safe[JPS_safe.size() - 1];

      // compute convex decomposition of JPS_safe
      jps_manager_.cvxEllipsoidDecomp(JPS_safe, UNKOWN_AND_OCCUPIED_SPACE, l_constraints_safe_, poly_safe_out);

      JPS_safe_out = JPS_safe;

      bool isGinside = l_constraints_safe_[l_constraints_safe_.size() - 1].inside(G.pos);
      M_.pos = (isGinside == true) ? G.pos : M_.pos;

      state x0_safe;
      x0_safe = R;

      if (par_.use_faster == false)
      {
        x0_safe = stateA_;
      }

      bool shouldForceFinalConstraint_for_Safe = (par_.use_faster == false) ? true : false;

      if (l_constraints_safe_[0].inside(x0_safe.pos) == false)
      {
        std::cout << red << "First point of safe traj is outside" << reset << std::endl;
      }

      sg_safe_.setX0(x0_safe);
      sg_safe_.setXf(M_);  // only used to compute dt
      sg_safe_.setPolytopes(l_constraints_safe_);
      sg_safe_.setForceFinalConstraint(shouldForceFinalConstraint_for_Safe);
      MyTimer safe_gurobi_t(true);
      std::cout << "Calling to Gurobi" << std::endl;
      bool solved_safe = sg_safe_.genNewTraj();

      if (solved_safe == false)
      {
        std::cout << red << "No solution found for the safe path" << reset << std::endl;
        return;
      }

      // Get the solution
      sg_safe_.fillX();
      X_safe_out = sg_safe_.X_temp_;
    }

     // std::cout << "This is the SAFE TRAJECTORY" << std::endl;
      //printStateVector(sg_safe_.X_temp_);
     // std::cout << "===========================" << std::endl;

    ///////////////////////////////////////////////////////////
    ///////////////       Append RESULTS    ////////////////////
    ///////////////////////////////////////////////////////////
    std::cout << "Going to append" << std::endl;

      if (appendToPlan(k_end_whole, sg_whole_.X_temp_, k_safe, sg_safe_.X_temp_) != true)
      {
        return;
      }
    */

  //######################### Solve with the NLOPT solver: //#########################

  // Create a map with the moving obstacles as occupied space

  // DEBUGGING

  int n_pol = par_.n_pol;
  int deg = par_.deg;
  int samples_per_interval = par_.samples_per_interval;

  state initial = A;
  state final = E;

  double t_min = deltaT_ * par_.dc + ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here
  double t_max = t_min + (initial.pos - final.pos).norm() / (0.6 * par_.v_max);  // time to execute the optimized path

  std::cout << "Going to compute the convex hulls, t_min= " << t_min << std::endl;
  std::cout << "Going to compute the convex hulls, t_max= " << t_max << std::endl;
  std::cout << "ros::Time::now().toSec()= " << ros::Time::now().toSec() << std::endl;
  std::cout << "deltaT_= " << deltaT_ << std::endl;
  std::cout << "par_.dc= " << par_.dc << std::endl;

  MyTimer convex_hulls_timer(true);

  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_min, t_max);
  std::cout << "hulls.size()=" << hulls.size() << std::endl;
  ConvexHullsOfCurves_Std hulls_std = vectorGCALPol2vectorStdEigen(hulls);
  poly_safe_out = vectorGCALPol2vectorJPSPol(hulls);

  std::cout << cyan << "Convex Hull time = " << convex_hulls_timer << reset << std::endl;

  std::cout << bold << "hulls has size=" << hulls.size() << reset << std::endl;

  int num_obst = hulls.size();

  SolverNlopt snlopt(n_pol, deg, num_obst, par_.weight, par_.epsilon_tol_constraints, false,
                     par_.solver);  // snlopt(a,g) a polynomials of degree 3
  snlopt.setHulls(hulls_std);

  snlopt.setMaxValues(par_.v_max, par_.a_max);  // v_max and a_max
  snlopt.setDC(par_.dc);                        // dc
  snlopt.setTminAndTmax(t_min, t_max);
  snlopt.setMaxRuntime(0.8 * deltaT_ * par_.dc);  // 0.8 to take into account other computations
  snlopt.setInitAndFinalStates(initial, final);

  snlopt.getGuessForCPs(poly_safe_out);  // in testing phase
  X_safe_out = snlopt.X_temp_;           // in testing phase
  return;                                // // in testing phase

  // Initial GUESS: run JPS with the dynamic obstacles as static obstacles
  mtx_map.lock();
  mtx_unk.lock();

  createObstacleMapFromTrajs(t_min, t_max);
  bool solvedjps_dyn = false;

  vec_Vecf<3> JPSk_dyn = jps_manager_dyn_.solveJPS3D(initial.pos, final.pos, &solvedjps_dyn, 1);

  if (solvedjps_dyn == false)
  {
    std::cout << bold << red << "JPS didn't find a solution, using straight line" << reset << std::endl;  // but
  }

  JPS_safe_out = JPSk_dyn;              // for visualization
  jps_manager_dyn_.getMap(pcloud_jps);  // for visualization

  mtx_map.unlock();
  mtx_unk.unlock();
  // end of Initial GUESSS
  snlopt.setInitialGuess(JPSk_dyn);

  std::cout << "Calling optimize" << std::endl;
  bool result = snlopt.optimize();
  std::cout << on_cyan << bold << "Solved " << solutions_found_ << "/" << total_replannings_ << reset << std::endl;

  total_replannings_++;
  if (result == false)
  {
    deltaT_ = std::min(par_.alpha * deltaT_,
                       2 / par_.dc);  // Increases deltaT_ (to increase the allowed runtime in the next iteration)
    return;
  }

  solutions_found_++;

  std::cout << "Below of loop\n";

  /////// END OF DEBUGGING
  /*
    SolverNlopt snlopt(n_pol_, deg_, true);  // snlopt(a,g) a polynomials of degree 3

    // SolverNlopt snlopt(n_pol_, deg_);  // snlopt(a,g) a polynomials of degree 3

    snlopt.setMaxValues(300, 200);  // v_max and a_max
    snlopt.setDC(par_.dc);          // dc

    double secs = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here
    std::vector<CGAL_Polyhedron_3> hulls = convexHullsOfCurve(secs, secs + 10, n_pol_, 0.1);  // TODO: Change this time
    poly_safe_out = vectorGCALPol2vectorJPSPol(hulls);
    std::vector<std::vector<Eigen::Vector3d>> hulls_std = vectorGCALPol2vectorStdEigen(hulls);
    snlopt.setHulls(hulls_std);

    // snlopt.setTminAndTmax(0, (A.pos - G_term.pos).norm() / (0.4 * par_.v_max));
    snlopt.setTminAndTmax(0, (A.pos - G_term.pos).norm() / (0.4 * par_.v_max));
    snlopt.setInitAndFinalStates(A, G_term);
    bool result = snlopt.optimize();
    if (result == false)
    {
      return;
    }
    // X_safe_out = snlopt.X_temp_;
*/
  std::vector<state> dummy_vector;
  dummy_vector.push_back(A);
  sg_whole_.X_temp_ = dummy_vector;

  M_ = G_term;

  k_safe = 0;

  if (appendToPlan(k_end_whole, sg_whole_.X_temp_, k_safe, snlopt.X_temp_) != true)
  {
    return;
  }

  X_safe_out = plan_.toStdVector();
  // novale_already_done_ = true;
  //######################### End of solve with the NLOPT solver: //#########################
  /*
    std::cout << "This is the Whole TRAJECTORY" << std::endl;
    printStateVector(sg_whole_.X_temp_);

    std::cout << "This is the Safe TRAJECTORY" << std::endl;
    printStateVector(snlopt.X_temp_);

    mtx_plan_.lock();
    std::cout << "This is the COMMITED TRAJECTORY" << std::endl;
    plan_.print();
    // printStateDeque(plan_);
    std::cout << "===========================" << std::endl;
    mtx_plan_.unlock();*/

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  // Check if we have planned until G_term
  state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
  std::cout << "F is " << std::endl;
  F.print();
  double dist = (G_term_.pos - F.pos).norm();
  std::cout << "Computed norm" << std::endl;
  if (dist < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
  }

  mtx_offsets.lock();

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                             // would have been needed for
                                                                             // the last replan

  deltaT_ = std::max(par_.alpha * states_last_replan, 1.0);
  // std::max(par_.alpha * states_last_replan,(double)par_.min_states_deltaT);  // Delta_t
  mtx_offsets.unlock();

  // Time allocation
  double new_init_whole = std::max(sg_whole_.factor_that_worked_ - par_.gamma_whole, 1.0);
  double new_final_whole = sg_whole_.factor_that_worked_ + par_.gammap_whole;
  sg_whole_.setFactorInitialAndFinalAndIncrement(new_init_whole, new_final_whole, par_.increment_whole);

  double new_init_safe = std::max(sg_safe_.factor_that_worked_ - par_.gamma_safe, 1.0);
  double new_final_safe = sg_safe_.factor_that_worked_ + par_.gammap_safe;
  sg_safe_.setFactorInitialAndFinalAndIncrement(new_init_safe, new_final_safe, par_.increment_safe);

  planner_initialized_ = true;

  return;
}

void Faster::createObstacleMapFromTrajs(double t_min, double t_max)
{
  mtx_trajs_.lock();

  jps_manager_dyn_.createNewMap(state_.pos);

  int number_of_samples = 10;

  for (auto traj : trajs_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    tmp->width = number_of_samples;  // number of samples
    tmp->height = 1;
    tmp->points.resize(tmp->width * tmp->height);

    for (int j = 0; j < number_of_samples; j++)
    {
      t_ = t_min + j * (t_max - t_min) / (1.0 * number_of_samples);

      tmp->points[j].x = traj.function[0].value();
      tmp->points[j].y = traj.function[1].value();
      tmp->points[j].z = traj.function[2].value();
    }

    jps_manager_dyn_.addToMap(tmp, traj.bbox[0] / 2.0 + 0, traj.bbox[1] / 2.0 + 0, traj.bbox[2] / 2.0 + 0);
  }

  mtx_trajs_.unlock();
}

void Faster::resetInitialization()
{
  planner_initialized_ = false;
  state_initialized_ = false;
  kdtree_map_initialized_ = false;
  kdtree_unk_initialized_ = false;
  terminal_goal_initialized_ = false;
}

bool Faster::appendToPlan(int k_end_whole, const std::vector<state>& whole, int k_safe, const std::vector<state>& safe)
{
  mtx_plan_.lock();

  std::cout << "Erasing" << std::endl;
  bool output;
  int plan_size = plan_.size();
  std::cout << "plan_.size()= " << plan_.size() << std::endl;
  std::cout << "plan_size - k_end_whole = " << plan_size - k_end_whole << std::endl;
  if ((plan_size - 1 - k_end_whole) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    output = false;
  }
  else
  {
    std::cout << "k_end_whole= " << k_end_whole << std::endl;
    std::cout << "k_safe = " << k_safe << std::endl;

    plan_.erase(plan_.end() - k_end_whole - 1, plan_.end());

    std::cout << "Erased" << std::endl;

    std::cout << "whole.size() = " << whole.size() << std::endl;
    std::cout << "safe.size() = " << safe.size() << std::endl;
    for (int i = 0; i <= k_safe; i++)
    {
      plan_.push_back(whole[i]);
    }

    for (int i = 1; i < safe.size(); i++)
    {
      plan_.push_back(safe[i]);
    }
    std::cout << "Pushed everything back" << std::endl;

    output = true;
  }

  mtx_plan_.unlock();
  return output;
}

void Faster::yaw(double diff, state& next_goal)
{
  saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  double dyaw_not_filtered;

  dyaw_not_filtered = copysign(1, diff) * par_.w_max;

  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered + par_.alpha_filter_dyaw * dyaw_filtered_;
  next_goal.dyaw = dyaw_filtered_;

  // std::cout << "Before next_goal.yaw=" << next_goal.yaw << std::endl;

  next_goal.yaw = previous_yaw_ + dyaw_filtered_ * par_.dc;
  // std::cout << "After next_goal.yaw=" << next_goal.yaw << std::endl;
}

void Faster::getDesiredYaw(state& next_goal)
{
  double diff = 0.0;
  double desired_yaw = 0.0;

  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      desired_yaw = atan2(G_term_.pos[1] - next_goal.pos[1], G_term_.pos[0] - next_goal.pos[0]);
      diff = desired_yaw - state_.yaw;
      // std::cout << "diff1= " << diff << std::endl;
      break;
    case DroneStatus::TRAVELING:
    case DroneStatus::GOAL_SEEN:
      desired_yaw = atan2(M_.pos[1] - next_goal.pos[1], M_.pos[0] - next_goal.pos[0]);
      diff = desired_yaw - state_.yaw;
      break;
    case DroneStatus::GOAL_REACHED:
      next_goal.dyaw = 0.0;
      next_goal.yaw = previous_yaw_;
      return;
  }

  angle_wrap(diff);
  if (fabs(diff) < 0.04 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }
  // std::cout << "diff2= " << diff << std::endl;
  yaw(diff, next_goal);
  // std::cout << "yaw3= " << next_goal.yaw << std::endl;
}

bool Faster::getNextGoal(state& next_goal)
{
  if (initializedAllExceptPlanner() == false)
  {
    std::cout << "Not publishing new goal!!" << std::endl;
    return false;
  }

  mtx_goals.lock();
  mtx_plan_.lock();

  next_goal.setZero();
  next_goal = plan_.front();
  if (plan_.size() > 1)
  {
    plan_.pop_front();
  }
  getDesiredYaw(next_goal);

  previous_yaw_ = next_goal.yaw;

  mtx_goals.unlock();
  mtx_plan_.unlock();
  return true;
}

// Debugging functions
void Faster::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
  {
    return;
  }

  std::cout << "Changing DroneStatus from ";
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
  }
  std::cout << " to ";

  switch (new_status)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

void Faster::print_status()
{
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "status_=YAWING" << reset << std::endl;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "status_=TRAVELING" << reset << std::endl;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "status_=GOAL_SEEN" << reset << std::endl;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "status_=GOAL_REACHED" << reset << std::endl;
      break;
  }
}