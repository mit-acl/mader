// Jesus Tordesillas Torres, jtorde@mit.edu
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>

#include "mader.hpp"
#include "timer.hpp"
#include "termcolor.hpp"

#include "nlopt_utils.hpp"

using namespace JPS;
using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
typedef Timer MyTimer;

Mader::Mader(parameters par) : par_(par)
{
  drone_status_ == DroneStatus::YAWING;
  // mtx_G.lock();
  G_.pos << 0, 0, 0;
  // mtx_G.unlock();
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

  // Check that the gradients are right
  if (nlopt_utils::checkGradientsNlopt(par_.basis) == false)
  {
    std::cout << "==============================================" << std::endl;
    std::cout << bold << "Gradient check was " << red << "NOT OK " << reset << std::endl;
    std::cout << "==============================================" << std::endl;

    abort();
  }
  else
  {
    std::cout << "==============================================" << std::endl;
    std::cout << bold << "Gradient check was " << green << " OK " << reset << std::endl;
    std::cout << "==============================================" << std::endl;
  }

  changeDroneStatus(DroneStatus::GOAL_REACHED);
  resetInitialization();

  par_snlopt par_for_solver;

  par_for_solver.x_min = par_.x_min;
  par_for_solver.x_max = par_.x_max;

  par_for_solver.y_min = par_.y_min;
  par_for_solver.y_max = par_.y_max;

  par_for_solver.z_min = par_.z_ground;
  par_for_solver.z_max = par_.z_max;

  par_for_solver.Ra = par_.Ra;
  par_for_solver.v_max = par_.v_max;
  par_for_solver.a_max = par_.a_max;
  par_for_solver.dc = par_.dc;
  par_for_solver.dist_to_use_straight_guess = par_.goal_radius;
  par_for_solver.a_star_samp_x = par_.a_star_samp_x;
  par_for_solver.a_star_samp_y = par_.a_star_samp_y;
  par_for_solver.a_star_samp_z = par_.a_star_samp_z;
  par_for_solver.a_star_fraction_voxel_size = par_.a_star_fraction_voxel_size;
  par_for_solver.num_pol = par_.num_pol;
  par_for_solver.deg_pol = par_.deg_pol;
  par_for_solver.weight = par_.weight;
  par_for_solver.epsilon_tol_constraints = par_.epsilon_tol_constraints;
  par_for_solver.xtol_rel = par_.xtol_rel;
  par_for_solver.ftol_rel = par_.ftol_rel;
  par_for_solver.solver = par_.solver;
  par_for_solver.basis = par_.basis;
  par_for_solver.a_star_bias = par_.a_star_bias;
  par_for_solver.allow_infeasible_guess = par_.allow_infeasible_guess;
  par_for_solver.alpha_shrink = par_.alpha_shrink;

  basisConverter basis_converter;

  if (par.basis == "MINVO")
  {
    A_rest_pos_basis_ = basis_converter.getArestMinvo();
  }
  else if (par.basis == "BEZIER")
  {
    A_rest_pos_basis_ = basis_converter.getArestBezier();
  }
  else if (par.basis == "B_SPLINE")
  {
    A_rest_pos_basis_ = basis_converter.getArestBSpline();
  }
  else
  {
    std::cout << red << "Basis " << par.basis << " not implemented yet" << reset << std::endl;
    std::cout << red << "============================================" << reset << std::endl;
    abort();
  }

  A_rest_pos_basis_inverse_ = A_rest_pos_basis_.inverse();

  snlopt_ = new SolverNlopt(par_for_solver);
  separator_solver_ = new separator::Separator();
}

void Mader::dynTraj2dynTrajCompiled(const dynTraj& traj, dynTrajCompiled& traj_compiled)
{
  mtx_t_.lock();
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

  mtx_t_.unlock();

  traj_compiled.bbox = traj.bbox;
  traj_compiled.id = traj.id;
  traj_compiled.time_received = traj.time_received;  // ros::Time::now().toSec();

  traj_compiled.is_static =
      ((traj.is_agent == false) &&                           // is an obstacle and
       (traj.function[0].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate x
       (traj.function[1].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate y
       (traj.function[2].find("t") == std::string::npos))    // there is no dependence on t in the coordinate z
      ||                                                     // OR
      (traj.is_agent == true && fabs(traj.pwp.times.back() - traj.pwp.times.front()) < 1e-7);

  traj_compiled.pwp = traj.pwp;
}
// Note that we need to compile the trajectories inside mader.cpp because t_ is in mader.hpp
void Mader::updateTrajObstacles(dynTraj traj)
{
  // for (auto coeff : traj.pwp.coeff_z)
  // {
  //   std::cout << on_magenta << "coeff.transpose()= " << coeff.transpose() << reset << std::endl;
  // }

  std::cout << "In updateTrajObstacles" << std::endl;

  MyTimer tmp_t(true);

  if (started_check_ == true && traj.is_agent == true)
  {
    have_received_trajectories_while_checking_ = true;
  }

  std::cout << "In updateTrajObstacles, waiting before mtx" << std::endl;
  mtx_trajs_.lock();
  std::cout << on_magenta << "[UTO] mtx_trajs_ is locked" << reset << std::endl;
  // std::cout << "In updateTrajObstacles 0.6" << std::endl;

  std::vector<dynTrajCompiled>::iterator obs_ptr = std::find_if(
      trajs_.begin(), trajs_.end(), [=](const dynTrajCompiled& traj_compiled) { return traj_compiled.id == traj.id; });

  // std::cout << "In updateTrajObstacles 0.7" << std::endl;

  bool exists_in_local_map = (obs_ptr != std::end(trajs_));

  dynTrajCompiled traj_compiled;
  // std::cout << "In updateTrajObstacles 0.8" << std::endl;

  dynTraj2dynTrajCompiled(traj, traj_compiled);

  // std::cout << "In updateTrajObstacles 0.9" << std::endl;

  // std::cout << bold << on_green << "[F]traj_compiled.pwp.times.size()=" << traj_compiled.pwp.times.size() << reset
  //           << std::endl;

  if (exists_in_local_map)
  {  // if that object already exists, substitute its trajectory
    // std::cout << red << "Updating " << traj_compiled.id << " at t=" << std::setprecision(12) << traj.time_received
    //           << reset << std::endl;
    *obs_ptr = traj_compiled;
  }
  else
  {  // if it doesn't exist, add it to the local map
    trajs_.push_back(traj_compiled);
    // ROS_WARN_STREAM("Adding " << traj_compiled.id);
    // std::cout << red << "Adding " << traj_compiled.id << " at t=" << std::setprecision(12) << traj.time_received
    //           << reset << std::endl;
  }

  // and now let's delete those trajectories of the obs/agents whose current positions are outside the local map
  // Note that these positions are obtained with the trajectory stored in the past in the local map
  std::vector<int> ids_to_remove;

  // std::cout << "In updateTrajObstacles 2" << std::endl;

  for (int index_traj = 0; index_traj < trajs_.size(); index_traj++)
  {
    // std::cout << "In updateTrajObstacles 3" << std::endl;

    bool traj_affects_me = false;

    mtx_t_.lock();
    t_ = ros::Time::now().toSec();

    Eigen::Vector3d center_obs;
    center_obs << trajs_[index_traj].function[0].value(),  ////////////////////
        trajs_[index_traj].function[1].value(),            ////////////////
        trajs_[index_traj].function[2].value();            /////////////////

    mtx_t_.unlock();

    // mtx_t_.unlock();
    if (((traj_compiled.is_static == true) && (center_obs - state_.pos).norm() > 2 * par_.Ra) ||  ////
        ((traj_compiled.is_static == false) && (center_obs - state_.pos).norm() > 4 * par_.Ra))
    // #### Static Obstacle: 2*Ra because: traj_{k-1} is inside a sphere of Ra.
    // Then, in iteration k the point A (which I don't
    // know yet)  is taken along that trajectory, and
    // another trajectory of radius Ra will be obtained.
    // Therefore, I need to take 2*Ra to make sure the
    // extreme case (A taken at the end of traj_{k-1} is
    // covered).

    // #### Dynamic Agent: 4*Ra. Same reasoning as above, but with two agets
    // #### Dynamic Obstacle: 4*Ra, it's a heuristics.

    // ######IMPORTANT######
    // Note that removeTrajsThatWillNotAffectMe will later
    // on take care of deleting the ones I don't need once
    // I know A
    {
      ids_to_remove.push_back(trajs_[index_traj].id);
    }
  }

  // std::cout << "In updateTrajObstacles 4" << std::endl;

  for (auto id : ids_to_remove)
  {
    // std::cout << red << "Removing " << id << " at t=" << std::setprecision(12) << traj.time_received;
    // ROS_WARN_STREAM("Removing " << id);

    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  std::cout << "In updateTrajObstacles 5" << std::endl;
  mtx_trajs_.unlock();
  std::cout << on_magenta << "[UTO] mtx_trajs_ is unlocked" << reset << std::endl;
  std::cout << "In updateTrajObstacles 6" << std::endl;

  have_received_trajectories_while_checking_ = false;
  // std::cout << bold << blue << "updateTrajObstacles took " << tmp_t << reset << std::endl;
}

std::vector<Eigen::Vector3d> Mader::vertexesOfInterval(PieceWisePol& pwp, double t_start, double t_end,
                                                       const Eigen::Vector3d& delta)
{
  std::vector<Eigen::Vector3d> points;

  // std::cout << "This is an agent!" << std::endl;
  std::vector<double>::iterator low = std::lower_bound(pwp.times.begin(), pwp.times.end(), t_start);
  std::vector<double>::iterator up = std::upper_bound(pwp.times.begin(), pwp.times.end(), t_end);

  // Example: times=[1 2 3 4 5 6 7]
  // t_start=1.5;
  // t_end=5.5
  // then low points to "2" (low - pwp.times.begin() is 1)
  // and up points to "6" (up - pwp.times.begin() is 5)

  int index_first_interval = low - pwp.times.begin() - 1;  // index of the interval [1,2]
  int index_last_interval = up - pwp.times.begin() - 1;    // index of the interval [5,6]

  saturate(index_first_interval, 0, (int)(pwp.coeff_x.size() - 1));
  saturate(index_last_interval, 0, (int)(pwp.coeff_x.size() - 1));

  Eigen::Matrix<double, 3, 4> P;
  Eigen::Matrix<double, 3, 4> V;

  // push all the complete intervals
  for (int i = index_first_interval; i <= index_last_interval; i++)
  {
    P.row(0) = pwp.coeff_x[i];
    P.row(1) = pwp.coeff_y[i];
    P.row(2) = pwp.coeff_z[i];

    V = P * A_rest_pos_basis_inverse_;

    for (int j = 0; j < V.cols(); j++)
    {
      double x = V(0, j);
      double y = V(1, j);
      double z = V(2, j);  //[x,y,z] is the point

      if (delta.norm() < 1e-6)
      {  // no inflation
        points.push_back(Eigen::Vector3d(x, y, z));
      }
      else
      {
        // points.push_back(Eigen::Vector3d(V(1, j), V(2, j), V(3, j)));  // x,y,z
        points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z + delta.z()));
        points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z + delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z + delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z - delta.z()));
        points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z + delta.z()));
      }
    }
  }

  return points;
}

// return a vector that contains all the vertexes of the polyhedral approx of an interval.
std::vector<Eigen::Vector3d> Mader::vertexesOfInterval(dynTrajCompiled& traj, double t_start, double t_end)
{
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  if (traj.is_agent == false)
  {
    std::vector<Eigen::Vector3d> points;
    delta = traj.bbox / 2.0 + (par_.drone_radius + par_.beta + par_.alpha) *
                                  Eigen::Vector3d::Ones();  // every side of the box will be increased by 2*delta
                                                            //(+delta on one end, -delta on the other)
    // Will always have a sample at the beginning of the interval, and another at the end.
    for (double t = t_start;                           /////////////
         (t < t_end) ||                                /////////////
         ((t > t_end) && ((t - t_end) < par_.gamma));  /////// This is to ensure we have a sample a the end
         t = t + par_.gamma)
    {
      mtx_t_.lock();
      t_ = std::min(t, t_end);  // this min only has effect on the last sample

      std::cout << "traj.function.size()=" << traj.function.size() << std::endl;
      // std::cout << "traj.function[0]" << traj.function[0] << std::endl;
      std::cout << "Computing x" << std::endl;
      double x = traj.function[0].value();
      std::cout << "Computing y" << std::endl;
      double y = traj.function[1].value();
      std::cout << "Computing z" << std::endl;
      double z = traj.function[2].value();
      mtx_t_.unlock();

      //"Minkowski sum along the trajectory: box centered on the trajectory"
      points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z + delta.z()));
      points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x + delta.x(), y + delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x + delta.x(), y - delta.y(), z + delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z + delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y + delta.y(), z - delta.z()));
      points.push_back(Eigen::Vector3d(x - delta.x(), y - delta.y(), z + delta.z()));
    }

    return points;
  }
  else
  {  // is an agent --> use the pwp field

    delta = traj.bbox / 2.0 + (par_.drone_radius) * Eigen::Vector3d::Ones();
    // std::cout << "****traj.bbox = " << traj.bbox << std::endl;
    // std::cout << "****par_.drone_radius = " << par_.drone_radius << std::endl;
    // std::cout << "****Inflation by delta= " << delta.transpose() << std::endl;

    return vertexesOfInterval(traj.pwp, t_start, t_end, delta);
  }
}

// See https://doc.cgal.org/Manual/3.7/examples/Convex_hull_3/quickhull_3.cpp
CGAL_Polyhedron_3 Mader::convexHullOfInterval(dynTrajCompiled& traj, double t_start, double t_end)
{
  std::vector<Eigen::Vector3d> points = vertexesOfInterval(traj, t_start, t_end);

  std::vector<Point_3> points_cgal;
  for (auto point_i : points)
  {
    points_cgal.push_back(Point_3(point_i.x(), point_i.y(), point_i.z()));
  }
  // CGAL_Polyhedron_3 poly = ;
  return convexHullOfPoints(points_cgal);
}

// trajs_ is already locked when calling this function
void Mader::removeTrajsThatWillNotAffectMe(const state& A, double t_start, double t_end)
{
  std::vector<int> ids_to_remove;

  for (auto traj : trajs_)
  {
    bool traj_affects_me = false;

    // STATIC OBSTACLES/AGENTS
    if (traj.is_static == true)
    {
      // mtx_t_.lock();
      mtx_t_.lock();
      t_ = t_start;  // which is constant along the trajectory

      Eigen::Vector3d center_obs;
      if (traj.is_agent == false)
      {
        center_obs << traj.function[0].value(), traj.function[1].value(), traj.function[2].value();
      }
      else
      {
        center_obs = traj.pwp.eval(t_);
      }

      mtx_t_.unlock();
      // mtx_t_.unlock();
      Eigen::Vector3d positive_half_diagonal;
      positive_half_diagonal << traj.bbox[0] / 2.0, traj.bbox[1] / 2.0, traj.bbox[2] / 2.0;

      Eigen::Vector3d c1 = center_obs - positive_half_diagonal;
      Eigen::Vector3d c2 = center_obs + positive_half_diagonal;

      traj_affects_me = boxIntersectsSphere(A.pos, par_.Ra, c1, c2);
    }
    else
    {                                                            // DYNAMIC OBSTACLES/AGENTS
      double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
      for (int i = 0; i < par_.num_pol; i++)                     // for each interval
      {
        std::vector<Eigen::Vector3d> points =
            vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);

        for (auto point_i : points)  // for every vertex of each interval
        {
          if ((point_i - A.pos).norm() <= par_.Ra)
          {
            traj_affects_me = true;
            goto exit;
          }
        }
      }
    }

  exit:
    if (traj_affects_me == false)
    {
      // std::cout << red << bold << "Going to  delete t raj " << trajs_[index_traj].id << reset << std::endl;
      ids_to_remove.push_back(traj.id);
    }
    /*    else
        {
          std::cout << green << bold << "Going to delete traj " << trajs_[index_traj].id << reset << std::endl;
        }*/
  }

  for (auto id : ids_to_remove)
  {
    ROS_INFO_STREAM("traj " << id << " doesn't affect me");
    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  /*  std::cout << "After deleting the trajectory, we have that ids= " << std::endl;

    for (auto traj : trajs_)
    {
      std::cout << traj.id << std::endl;
    }*/
}

bool Mader::IsTranslating()
{
  return (drone_status_ == DroneStatus::GOAL_SEEN || drone_status_ == DroneStatus::TRAVELING);
}

ConvexHullsOfCurve Mader::convexHullsOfCurve(dynTrajCompiled& traj, double t_start, double t_end)
{
  ConvexHullsOfCurve convexHulls;

  double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
  // std::cout << "deltaT= " << deltaT << std::endl;
  for (int i = 0; i < par_.num_pol; i++)
  {
    // std::cout << "i= " << i << std::endl;
    convexHulls.push_back(convexHullOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT));
  }

  // std::cout << "Done with convexHullsOfCurve" << std::endl;

  return convexHulls;
}

ConvexHullsOfCurves Mader::convexHullsOfCurves(double t_start, double t_end)
{
  ConvexHullsOfCurves result;

  for (auto traj : trajs_)
  {
    // std::cout << "Computing convex hull of curve " << traj.id << std::endl;
    // std::cout << "above, traj.function.size()= " << traj.function.size() << std::endl;
    // std::cout << on_blue << "going to call convexHullsOfCurve" << reset << std::endl;
    // for (auto coeff : traj.pwp.coeff_z)
    // {
    //   std::cout << on_blue << "traj.pwp.coeff_z.transpose()= " << coeff.transpose() << reset << std::endl;
    // }
    result.push_back(convexHullsOfCurve(traj, t_start, t_end));
    // std::cout << "called convexHullsOfCurve" << std::endl;
  }
  // std::cout << "end of convexHullsOfCurves" << std::endl;

  return result;
}

void Mader::setTerminalGoal(state& term_goal)
{
  mtx_G_term.lock();
  mtx_G.lock();
  mtx_state.lock();
  mtx_planner_status_.lock();

  G_term_.pos = term_goal.pos;
  Eigen::Vector3d temp = state_.pos;
  G_.pos = G_term_.pos;
  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    // std::cout << bold << green << "[Mader] state_.yaw=" << state_.yaw << reset << std::endl;
    changeDroneStatus(DroneStatus::YAWING);  // not done when drone_status==traveling
  }
  if (drone_status_ == DroneStatus::GOAL_SEEN)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }
  terminal_goal_initialized_ = true;

  // std::cout << bold << red << "[FA] Received Term Goal=" << G_term_.pos.transpose() << reset << std::endl;
  // std::cout << bold << red << "[FA] Received Proj Goal=" << G_.pos.transpose() << reset << std::endl;

  mtx_state.unlock();
  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_planner_status_.unlock();
}

void Mader::getG(state& G)
{
  G = G_;
}

void Mader::getState(state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

void Mader::updateState(state data)
{
  state_ = data;

  if (state_initialized_ == false)
  {
    state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    plan_.push_back(tmp);
    previous_yaw_ = tmp.yaw;
  }

  state_initialized_ = true;
}

bool Mader::initializedAllExceptPlanner()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

bool Mader::initializedStateAndTermGoal()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    return false;
  }
  return true;
}

bool Mader::initialized()
{
  if (!state_initialized_ || !terminal_goal_initialized_ || !planner_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;
        std::cout << "planner_initialized_= " << planner_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

// check wheter a dynTrajCompiled and a pwp_optimized are in collision in the interval [t_start, t_end]
bool Mader::trajsAndPwpAreInCollision(dynTrajCompiled traj, PieceWisePol pwp_optimized, double t_start, double t_end)
{
  Eigen::Vector3d n_i;  // won't be used
  double d_i;           // won't be used

  double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
  for (int i = 0; i < par_.num_pol; i++)                     // for each interval
  {
    // This is my trajectory (no inflation)
    std::vector<Eigen::Vector3d> pointsA =
        vertexesOfInterval(pwp_optimized, t_start + i * deltaT, t_start + (i + 1) * deltaT, Eigen::Vector3d::Zero());

    // This is the trajectory of the other agent/obstacle
    std::vector<Eigen::Vector3d> pointsB = vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);

    // std::cout << "Going to solve model with pointsA.size()= " << pointsA.size() << std::endl;
    // for (auto point_i : pointsA)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    // std::cout << "Going to solve model with pointsB.size()= " << pointsB.size() << std::endl;
    // for (auto point_i : pointsB)
    // {
    //   std::cout << point_i.transpose() << std::endl;
    // }

    if (separator_solver_->solveModel(n_i, d_i, pointsA, pointsB) == false)
    {
      return true;  // There is not a solution --> they collide
    }
  }

  // if reached this point, they don't colide
  return false;
}
// Checks that I have not received new trajectories that affect me while doing the optimization
bool Mader::safetyCheckAfterOpt(PieceWisePol pwp_optimized)
{
  started_check_ = true;

  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_optimized, pwp_optimized.times.front(), pwp_optimized.times.back()))
      {
        ROS_ERROR_STREAM("Traj collides with " << traj.id);

        std::cout << "My traj collides with traj of " << traj.id << ", received at " << std::setprecision(12)
                  << traj.time_received << ", opt at " << time_init_opt_ << reset << std::endl;
        result = false;  // will have to redo the optimization
        break;
      }
    }
  }

  // and now do another check in case I've received anything while I was checking. Note that mtx_trajs_ is locked!
  if (have_received_trajectories_while_checking_ == true)
  {
    ROS_ERROR_STREAM("Recvd traj while checking ");

    std::cout << "Received a trajectory while I was checking" << std::endl;
    result = false;
  }
  started_check_ = false;

  ROS_INFO_STREAM("Returning " << result);
  return result;
  // traj_compiled.time_received = ros::Time::now().toSec();
}

bool Mader::replan(mader_types::Edges& edges_obstacles_out, std::vector<state>& X_safe_out,
                   std::vector<Hyperplane3D>& planes_guesses, int& num_of_LPs_run, int& num_of_QCQPs_run,
                   PieceWisePol& pwp_out)
{
  MyTimer replanCB_t(true);

  if (initializedStateAndTermGoal() == false)
  {
    // std::cout << "Not Replanning" << std::endl;
    return false;
  }

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// G <-- Project GTerm ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_state.lock();
  mtx_G.lock();
  mtx_G_term.lock();

  state state_local = state_;

  state G_term = G_term_;  // Local copy of the terminal terminal goal

  state G = G_term;

  mtx_G.unlock();
  mtx_G_term.unlock();
  mtx_state.unlock();

  // Check if we have reached the goal
  double dist_to_goal = (G_term.pos - state_local.pos).norm();
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    exists_previous_pwp_ = false;
  }

  // Check if we have seen the goal in the last replan
  mtx_plan_.lock();
  double dist_last_plan_to_goal = (G_term.pos - plan_.back().pos).norm();
  mtx_plan_.unlock();
  if (dist_last_plan_to_goal < par_.goal_radius && drone_status_ == DroneStatus::TRAVELING)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
    std::cout << "Status changed to GOAL_SEEN!" << std::endl;
    exists_previous_pwp_ = false;
  }

  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING) ||
      (drone_status_ == DroneStatus::GOAL_SEEN))
  {
    // std::cout << "No replanning needed because" << std::endl;
    // print_status();
    return false;
  }
  ROS_INFO_STREAM("_________________________");

  std::cout << bold << on_white << "**********************IN REPLAN CB*******************" << reset << std::endl;

  /////////////////////////////////// DEBUGGING ///////////////////////////////////
  // mtx_trajs_.lock();
  // std::cout << bold << blue << "Trajectories in the local map: " << reset << std::endl;

  // std::vector<double> all_ids;
  // /*  traj_compiled.id = traj.id;
  //   trajs_.push_back(traj_compiled);*/
  // int tmp_index_traj = 0;
  // for (auto traj : trajs_)
  // {
  //   std::cout << traj.id << ", ";
  //   // double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  //   all_ids.push_back(traj.id);
  //   // all_ids = all_ids + " " + std::to_string(traj.id);

  //   // t_ = time_now;

  //   // Eigen::Vector3d center_obs;
  //   // center_obs << trajs_[tmp_index_traj].function[0].value(),  ////////////////////
  //   //     trajs_[tmp_index_traj].function[1].value(),            ////////////////
  //   //     trajs_[tmp_index_traj].function[2].value();            /////////////////

  //   // std::cout << traj.id << ", which is in " << center_obs.transpose() << std::endl;

  //   // tmp_index_traj = tmp_index_traj + 1;
  // }

  // sort(all_ids.begin(), all_ids.end());

  // if (all_ids.size() >= 1)
  // {
  //   std::ostringstream oss;

  //   if (!all_ids.empty())
  //   {
  //     // Convert all but the last element to avoid a trailing ","
  //     std::copy(all_ids.begin(), all_ids.end() - 1, std::ostream_iterator<double>(oss, ","));

  //     // Now add the last element with no delimiter
  //     oss << all_ids.back();
  //   }

  //   ROS_INFO_STREAM("Trajs used: " << oss.str());
  // }
  // else
  // {
  //   ROS_INFO_STREAM("Trajs used: - ");
  // }

  // std::cout << std::endl;
  // mtx_trajs_.unlock();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  state A;
  int k_index_end, k_index;

  // If k_index_end=0, then A = plan_.back() = plan_[plan_.size() - 1]

  mtx_plan_.lock();

  k_index_end = std::max((int)(plan_.size() - deltaT_), 0);

  if (plan_.size() < 5)
  {
    k_index_end = 0;
  }

  k_index = plan_.size() - 1 - k_index_end;
  A = plan_.get(k_index);

  mtx_plan_.unlock();

  // std::cout << blue << "Have chosen:" << reset << std::endl;
  // std::cout << blue << "k_index:" << k_index << reset << std::endl;
  // std::cout << blue << "k_index_end:" << k_index_end << reset << std::endl;
  // std::cout << blue << "plan_.size():" << plan_.size() << reset << std::endl;

  double runtime_snlopt;

  if (k_index_end != 0)
  {
    runtime_snlopt = k_index * par_.dc;  // std::min(, par_.upper_bound_runtime_snlopt);
  }
  else
  {
    runtime_snlopt = par_.upper_bound_runtime_snlopt;  // I'm stopped at the end of the trajectory --> take my
                                                       // time to replan
  }
  saturate(runtime_snlopt, par_.lower_bound_runtime_snlopt, par_.upper_bound_runtime_snlopt);

  std::cout << green << "Runtime snlopt= " << runtime_snlopt << reset << std::endl;

  /*  if (k_index_end == 0)
    {
      exists_previous_pwp_ = false;
    }*/

  std::cout << "Selected state A= " << A.pos.transpose() << std::endl;
  std::cout << "Selected state G_term= " << G_term.pos.transpose() << std::endl;

  /////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get global plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  std::vector<Eigen::Vector3d> global_plan;
  global_plan.push_back(A.pos);
  global_plan.push_back(G.pos);

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get point E ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  double distA2TermGoal = (A.pos - G_term.pos).norm();
  double ra = std::min((distA2TermGoal - 0.001), par_.Ra);  // radius of the sphere S
  bool noPointsOutsideS;
  int li1;  // last index inside the sphere of global_plan
  state E;
  // std::cout << bold << std::setprecision(3) << "A.pos= " << A.pos.transpose() << reset << std::endl;
  // std::cout << "A= " << A.pos.transpose() << std::endl;
  // std::cout << "G= " << G.pos.transpose() << std::endl;
  // std::cout << "ra= " << ra << std::endl;
  E.pos = getFirstIntersectionWithSphere(global_plan, ra, global_plan[0], &li1, &noPointsOutsideS);
  if (noPointsOutsideS == true)  // if G is inside the sphere
  {
    E.pos = G.pos;
  }

  state initial = A;
  state final = E;

  // std::cout << green << "================================" << reset << std::endl;

  // std::cout << "Initial.pos= " << initial.pos << std::endl;
  // std::cout << "Final.pos= " << final.pos << std::endl;
  // std::cout << green << "================================" << reset << std::endl;

  // std::cout << "norm= " << (initial.pos - final.pos).norm() << std::endl;

  ////////////////////////////////

  snlopt_->setMaxRuntimeKappaAndMu(runtime_snlopt, par_.kappa, par_.mu);

  //////////////////////
  double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_start = k_index * par_.dc + time_now;

  double factor_v_max_tmp = par_.factor_v_max;

  // when it's near the terminal goal --> use a small factor_v_max (if not it will oscillate)
  if (distA2TermGoal < 1.5)  // TODO: Put this as a param
  {
    factor_v_max_tmp = 0.4;  // TODO: Put this as a param
  }

  double t_final = t_start + (initial.pos - final.pos).array().abs().maxCoeff() /
                                 (factor_v_max_tmp * par_.v_max.x());  // time to execute the optimized path

  bool correctInitialCond =
      snlopt_->setInitStateFinalStateInitTFinalT(initial, final, t_start,
                                                 t_final);  // note that here t_final may have been updated

  if (correctInitialCond == false)
  {
    std::cout << bold << red << "The solver cannot guarantee feasibility for v1" << std::endl;
    return false;
  }

  ////////////////

  mtx_trajs_.lock();
  std::cout << on_magenta << "[RC] mtx_trajs_ is locked" << reset << std::endl;

  time_init_opt_ = ros::Time::now().toSec();
  removeTrajsThatWillNotAffectMe(A, t_start, t_final);
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_start, t_final);
  mtx_trajs_.unlock();
  std::cout << on_magenta << "[RC] mtx_trajs_ is unlocked" << reset << std::endl;

  ConvexHullsOfCurves_Std hulls_std = vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = vectorGCALPol2vectorJPSPol(hulls);
  edges_obstacles_out = vectorGCALPol2edges(hulls);

  snlopt_->setHulls(hulls_std);

  //////////////////////
  std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
            << std::endl;

  std::cout << "[FA] Calling NL" << std::endl;

  bool result = snlopt_->optimize();

  num_of_LPs_run += snlopt_->getNumOfLPsRun();
  num_of_QCQPs_run += snlopt_->getNumOfQCQPsRun();

  total_replannings_++;
  if (result == false)
  {
    int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                               // would have been needed for
                                                                               // the last replan
    deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
    deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
    return false;
  }

  snlopt_->getGuessForPlanes(planes_guesses);

  solutions_found_++;

  av_improvement_nlopt_ = ((solutions_found_ - 1) * av_improvement_nlopt_ + snlopt_->improvement_) / solutions_found_;

  std::cout << blue << "Average improvement so far" << std::setprecision(5) << av_improvement_nlopt_ << reset
            << std::endl;

  PieceWisePol pwp_now;
  snlopt_->getSolution(pwp_now);

  MyTimer check_t(true);
  mtx_trajs_.lock();
  std::cout << on_magenta << "[RC2] mtx_trajs_ is locked" << reset << std::endl;
  bool is_safe_after_opt = safetyCheckAfterOpt(pwp_now);
  mtx_trajs_.unlock();
  std::cout << on_magenta << "[RC2] mtx_trajs_ is unlocked" << reset << std::endl;
  std::cout << bold << "Check Timer=" << check_t << std::endl;

  if (is_safe_after_opt == false)
  {
    std::cout << bold << red << "safetyCheckAfterOpt is not satisfied!" << reset << std::endl;
    ROS_ERROR_STREAM("safetyCheckAfterOpt is not satisfied, returning");
    return false;
  }

  /////// END OF DEBUGGING

  M_ = G_term;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Append to plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  mtx_plan_.lock();

  // std::cout << std::endl;
  // std::cout << "************ PLAN BEFORE***************" << std::endl;
  // plan_.print();

  // std::cout << std::endl;
  // std::cout << "************ TRAJ FOUND***************" << std::endl;
  // for (auto xi : snlopt_->traj_solution_)
  // {
  //   xi.printHorizontal();
  // }

  // std::cout << "Erasing" << std::endl;

  int plan_size = plan_.size();
  //  std::cout << "plan_.size()= " << plan_.size() << std::endl;
  //  std::cout << "plan_size - k_index_end = " << plan_size - k_index_end << std::endl;
  if ((plan_size - 1 - k_index_end) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    mtx_plan_.unlock();
    return false;
  }
  else
  {
    // std::cout << "k_index_end= " << k_index_end << std::endl;

    plan_.erase(plan_.end() - k_index_end - 1, plan_.end());  // this deletes also the initial condition...

    for (int i = 0; i < (snlopt_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      plan_.push_back(snlopt_->traj_solution_[i]);
    }
    // std::cout << "Pushed everything back" << std::endl;
  }

  // std::cout << "************ PLAN AFTER***************" << std::endl;
  // plan_.print();

  mtx_plan_.unlock();

  ////////////////////
  ////////////////////

  // std::cout << std::setprecision(30) << "pwp_now.times[0]=" << pwp_now.times[0] << std::endl;
  // std::cout << "t_min= " << t_min << std::endl;

  if (exists_previous_pwp_ == true)
  {
    pwp_out = composePieceWisePol(time_now, par_.dc, pwp_prev_, pwp_now);
    pwp_prev_ = pwp_out;
  }
  else
  {  //
    // std::cout << "exists_previous_pwp_ = false" << std::endl;
    pwp_out = pwp_now;
    pwp_prev_ = pwp_now;
    exists_previous_pwp_ = true;
  }

  X_safe_out = plan_.toStdVector();

  //######################### End of solve with the NLOPT solver: //#########################

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  // Check if we have planned until G_term
  state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
  // std::cout << "F is " << std::endl;
  // F.print();
  double dist = (G_term_.pos - F.pos).norm();

  if (dist < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
  }

  mtx_offsets.lock();

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                             // would have been needed for
                                                                             // the last replan
  deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
  // std::max(par_.alpha * states_last_replan,(double)par_.min_states_deltaT);  // Delta_t
  mtx_offsets.unlock();

  planner_initialized_ = true;

  return true;
}

void Mader::resetInitialization()
{
  planner_initialized_ = false;
  state_initialized_ = false;

  terminal_goal_initialized_ = false;
}

void Mader::yaw(double diff, state& next_goal)
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

void Mader::getDesiredYaw(state& next_goal)
{
  double diff = 0.0;
  double desired_yaw = 0.0;

  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      desired_yaw = atan2(G_term_.pos[1] - next_goal.pos[1], G_term_.pos[0] - next_goal.pos[0]);
      // std::cout << bold << green << "[Mader] state_.yaw=" << state_.yaw << reset << std::endl;
      diff = desired_yaw - state_.yaw;
      // std::cout << bold << green << "[Mader] desired_yaw=" << desired_yaw << reset << std::endl;
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

bool Mader::getNextGoal(state& next_goal)
{
  /*  if (initializedAllExceptPlanner() == false)
    {
      std::cout << "Not publishing new goal!!" << std::endl;
      return false;
    }*/
  if (initializedStateAndTermGoal() == false || (drone_status_ == DroneStatus::GOAL_REACHED && plan_.size() == 1))
  {
    // std::cout << "Not publishing new goal!!" << std::endl;
    return false;
  }

  mtx_goals.lock();
  mtx_plan_.lock();

  next_goal.setZero();
  next_goal = plan_.front();

  // std::cout << bold << green << "[Mader] next_goal.yaw=" << next_goal.yaw << reset << std::endl;

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
void Mader::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
  {
    return;
  }

  std::cout << "Changing DroneStatus from ";
  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "GOAL_REACHED" << reset;
      break;
  }
  std::cout << " to ";

  switch (new_status)
  {
    case DroneStatus::YAWING:
      std::cout << bold << "YAWING" << reset;
      break;
    case DroneStatus::TRAVELING:
      std::cout << bold << "TRAVELING" << reset;
      break;
    case DroneStatus::GOAL_SEEN:
      std::cout << bold << "GOAL_SEEN" << reset;
      break;
    case DroneStatus::GOAL_REACHED:
      std::cout << bold << "GOAL_REACHED" << reset;
      break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

void Mader::print_status()
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