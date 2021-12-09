/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>

#include "mader.hpp"
#include "timer.hpp"
#include "termcolor.hpp"

#if USE_GUROBI_FLAG
#else
#include "nlopt_utils.hpp"
#endif

using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;getNextGoal
// typedef ROSWallTimer MyTimer;
typedef MADER_timers::Timer MyTimer;

Mader::Mader(mt::parameters par) : par_(par)
{
  drone_status_ == DroneStatus::YAWING;
  G_.pos << 0, 0, 0;
  G_term_.pos << 0, 0, 0;

  mtx_initial_cond.lock();
  stateA_.setZero();
  mtx_initial_cond.unlock();

#if USE_GUROBI_FLAG
#else
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
#endif

  changeDroneStatus(DroneStatus::GOAL_REACHED);
  resetInitialization();

  ms::par_solver par_for_solver;

  par_for_solver.x_min = par_.x_min;
  par_for_solver.x_max = par_.x_max;

  par_for_solver.y_min = par_.y_min;
  par_for_solver.y_max = par_.y_max;

  par_for_solver.z_min = par_.z_min;
  par_for_solver.z_max = par_.z_max;

  par_for_solver.Ra = par_.Ra;
  par_for_solver.v_max = par_.v_max;
  par_for_solver.a_max = par_.a_max;
  par_for_solver.j_max = par_.j_max;
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

  mt::basisConverter basis_converter;

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

#if USE_GUROBI_FLAG
  solver_ = std::unique_ptr<SolverGurobi>(new SolverGurobi(par_for_solver));
#else
  solver_ = std::unique_ptr<SolverNlopt>(new SolverNlopt(par_for_solver));
#endif

  // solver_ = new SolverNlopt(par_for_solver);
  // solver_ = new SolverGurobi(par_for_solver);

  separator_solver_ = new separator::Separator();
}

void Mader::dynTraj2dynTrajCompiled(const mt::dynTraj& traj, mt::dynTrajCompiled& traj_compiled)
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
void Mader::updateTrajObstacles(mt::dynTraj traj)
{
  MyTimer tmp_t(true);

  if (started_check_ == true && traj.is_agent == true)
  {
    have_received_trajectories_while_checking_ = true;
  }

  mtx_trajs_.lock();

  std::vector<mt::dynTrajCompiled>::iterator obs_ptr =
      std::find_if(trajs_.begin(), trajs_.end(),
                   [=](const mt::dynTrajCompiled& traj_compiled) { return traj_compiled.id == traj.id; });

  bool exists_in_local_map = (obs_ptr != std::end(trajs_));

  mt::dynTrajCompiled traj_compiled;
  dynTraj2dynTrajCompiled(traj, traj_compiled);

  if (exists_in_local_map)
  {  // if that object already exists, substitute its trajectory
    *obs_ptr = traj_compiled;
  }
  else
  {  // if it doesn't exist, add it to the local map
    trajs_.push_back(traj_compiled);
    // ROS_WARN_STREAM("Adding " << traj_compiled.id);
  }

  // and now let's delete those trajectories of the obs/agents whose current positions are outside the local map
  // Note that these positions are obtained with the trajectory stored in the past in the local map
  std::vector<int> ids_to_remove;

  for (int index_traj = 0; index_traj < trajs_.size(); index_traj++)
  {
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

    // ######REMEMBER######
    // Note that removeTrajsThatWillNotAffectMe will later
    // on take care of deleting the ones I don't need once
    // I know A
    {
      ids_to_remove.push_back(trajs_[index_traj].id);
    }
  }

  for (auto id : ids_to_remove)
  {
    // ROS_WARN_STREAM("Removing " << id);
    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](mt::dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  mtx_trajs_.unlock();

  have_received_trajectories_while_checking_ = false;
  // std::cout << bold << blue << "updateTrajObstacles took " << tmp_t << reset << std::endl;
}

std::vector<Eigen::Vector3d> Mader::vertexesOfInterval(mt::PieceWisePol& pwp, double t_start, double t_end,
                                                       const Eigen::Vector3d& delta)
{
  std::vector<Eigen::Vector3d> points;

  std::vector<double>::iterator low = std::lower_bound(pwp.times.begin(), pwp.times.end(), t_start);
  std::vector<double>::iterator up = std::upper_bound(pwp.times.begin(), pwp.times.end(), t_end);

  // Example: times=[1 2 3 4 5 6 7]
  // t_start=1.5;
  // t_end=5.5
  // then low points to "2" (low - pwp.times.begin() is 1)
  // and up points to "6" (up - pwp.times.begin() is 5)

  int index_first_interval = low - pwp.times.begin() - 1;  // index of the interval [1,2]
  int index_last_interval = up - pwp.times.begin() - 1;    // index of the interval [5,6]

  mu::saturate(index_first_interval, 0, (int)(pwp.coeff_x.size() - 1));
  mu::saturate(index_last_interval, 0, (int)(pwp.coeff_x.size() - 1));

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
std::vector<Eigen::Vector3d> Mader::vertexesOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end)
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

      double x = traj.function[0].value();
      double y = traj.function[1].value();
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
CGAL_Polyhedron_3 Mader::convexHullOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  std::vector<Eigen::Vector3d> points = vertexesOfInterval(traj, t_start, t_end);

  std::vector<Point_3> points_cgal;
  for (auto point_i : points)
  {
    points_cgal.push_back(Point_3(point_i.x(), point_i.y(), point_i.z()));
  }

  return cu::convexHullOfPoints(points_cgal);
}

// trajs_ is already locked when calling this function
void Mader::removeTrajsThatWillNotAffectMe(const mt::state& A, double t_start, double t_end)
{
  std::vector<int> ids_to_remove;

  std::vector<Eigen::Vector3d> pointsB;
  Eigen::Vector3d delta = par_.Ra * Eigen::Vector3d::Ones();

  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() + delta.y(), A.pos.z() + delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() - delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() + delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() + delta.x(), A.pos.y() - delta.y(), A.pos.z() + delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() - delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() + delta.y(), A.pos.z() + delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() + delta.y(), A.pos.z() - delta.z()));
  pointsB.push_back(Eigen::Vector3d(A.pos.x() - delta.x(), A.pos.y() - delta.y(), A.pos.z() + delta.z()));

  for (auto traj : trajs_)
  {
    bool traj_affects_me = false;

    // STATIC OBSTACLES/AGENTS
    if (traj.is_static == true)
    {
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
      traj_affects_me = mu::boxIntersectsSphere(A.pos, par_.Ra, c1, c2);
    }
    else
    {                                                            // DYNAMIC OBSTACLES/AGENTS
      double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals
      for (int i = 0; i < par_.num_pol; i++)                     // for each interval
      {
        std::vector<Eigen::Vector3d> pointsA =
            vertexesOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT);
        // Now we check whether the bbox centered in A (and total_sizes=2*Ra) collides with the polyhedron whose
        // vertexes are pointsA

        // To do that, we solve a linear feasibility program to find a plane that separates that box and the
        // polyhedron

        Eigen::Vector3d n;
        double d;

        bool solved = separator_solver_->solveModel(n, d, pointsA, pointsB);

        if (solved == false)
        {
          traj_affects_me = true;
          goto exit;
        }
      }
    }

  exit:
    if (traj_affects_me == false)
    {
      // std::cout << red << bold << "Going to  delete traj " << trajs_[index_traj].id << reset << std::endl;
      ids_to_remove.push_back(traj.id);
    }
  }

  for (auto id : ids_to_remove)
  {
    // ROS_INFO_STREAM("traj " << id << " doesn't affect me");
    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](mt::dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  /*  std::cout << "After deleting the trajectory, we have these ids= " << std::endl;

    for (auto traj : trajs_)
    {
      std::cout << traj.id << std::endl;
    }*/
}

bool Mader::IsTranslating()
{
  return (drone_status_ == DroneStatus::GOAL_SEEN || drone_status_ == DroneStatus::TRAVELING);
}

ConvexHullsOfCurve Mader::convexHullsOfCurve(mt::dynTrajCompiled& traj, double t_start, double t_end)
{
  ConvexHullsOfCurve convexHulls;
  double deltaT = (t_end - t_start) / (1.0 * par_.num_pol);  // num_pol is the number of intervals

  for (int i = 0; i < par_.num_pol; i++)
  {
    convexHulls.push_back(convexHullOfInterval(traj, t_start + i * deltaT, t_start + (i + 1) * deltaT));
  }

  return convexHulls;
}

ConvexHullsOfCurves Mader::convexHullsOfCurves(double t_start, double t_end)
{
  ConvexHullsOfCurves result;

  for (auto traj : trajs_)
  {
    result.push_back(convexHullsOfCurve(traj, t_start, t_end));
  }

  return result;
}

void Mader::setTerminalGoal(mt::state& term_goal)
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
    changeDroneStatus(DroneStatus::YAWING);
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

void Mader::getG(mt::state& G)
{
  G = G_;
}

void Mader::getState(mt::state& data)
{
  mtx_state.lock();
  data = state_;
  mtx_state.unlock();
}

void Mader::updateState(mt::state data)
{
  state_ = data;

  if (state_initialized_ == false)
  {
    mt::state tmp;
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

// check wheter a mt::dynTrajCompiled and a pwp_optimized are in collision in the interval [t_start, t_end]
bool Mader::trajsAndPwpAreInCollision(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized, double t_start,
                                      double t_end)
{
  Eigen::Vector3d n_i;
  double d_i;

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

  // if reached this point, they don't collide
  return false;
}
// Checks that I have not received new trajectories that affect me while doing the optimization
bool Mader::safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized)
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
        result = false;  // will have to redo the optimization
        break;
      }
    }
  }

  // and now do another check in case I've received anything while I was checking. Note that mtx_trajs_ is locked!
  if (have_received_trajectories_while_checking_ == true)
  {
    ROS_ERROR_STREAM("Recvd traj while checking ");
    result = false;
  }
  started_check_ = false;

  return result;
}

bool Mader::isReplanningNeeded()
{
  if (initializedStateAndTermGoal() == false)
  {
    return false;
  }

  //////////////////////////////////////////////////////////////////////////
  mtx_G_term.lock();

  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal

  mtx_G_term.unlock();

  // Check if we have reached the goal
  double dist_to_goal = (G_term.pos - plan_.front().pos).norm();
  // std::cout << "dist_to_goal= " << dist_to_goal << std::endl;
  if (dist_to_goal < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    exists_previous_pwp_ = false;
  }

  // Check if we have seen the goal in the last replan
  mtx_plan_.lock();
  double dist_last_plan_to_goal = (G_term.pos - plan_.back().pos).norm();
  // std::cout << "dist_last_plan_to_goal= " << dist_last_plan_to_goal << std::endl;
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
    // printDroneStatus();
    return false;
  }
  return true;
}

bool Mader::replan(mt::Edges& edges_obstacles_out, std::vector<mt::state>& X_safe_out,
                   std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                   mt::PieceWisePol& pwp_out)
{
  if (isReplanningNeeded() == false)
  {
    return false;
  }

  MyTimer replanCB_t(true);

  std::cout << bold << on_white << "**********************IN REPLAN CB*******************" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select mt::state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  mt::state A;
  int k_index_end, k_index;

  // If k_index_end=0, then A = plan_.back() = plan_[plan_.size() - 1]

  mtx_plan_.lock();

  mu::saturate(deltaT_, par_.lower_bound_runtime_snlopt / par_.dc, par_.upper_bound_runtime_snlopt / par_.dc);

  k_index_end = std::max((int)(plan_.size() - deltaT_), 0);

  if (plan_.size() < 5)
  {
    k_index_end = 0;
  }

  k_index = plan_.size() - 1 - k_index_end;
  A = plan_.get(k_index);

  mtx_plan_.unlock();

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
  mu::saturate(runtime_snlopt, par_.lower_bound_runtime_snlopt, par_.upper_bound_runtime_snlopt);

  // std::cout << green << "Runtime snlopt= " << runtime_snlopt << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get point G ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double distA2TermGoal = (G_term.pos - A.pos).norm();
  double ra = std::min((distA2TermGoal - 0.001), par_.Ra);  // radius of the sphere S
  mt::state G;
  G.pos = A.pos + ra * (G_term.pos - A.pos).normalized();

  mt::state initial = A;
  mt::state final = G;

  mtx_G.lock();
  G_ = G;
  mtx_G.unlock();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Solve optimization! ////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  solver_->setMaxRuntimeKappaAndMu(runtime_snlopt, par_.kappa, par_.mu);

  //////////////////////
  double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_start = k_index * par_.dc + time_now;

  double factor_alloc = (distA2TermGoal > par_.dist_factor_alloc_close) ? par_.factor_alloc : par_.factor_alloc_close;

  double time_allocated = factor_alloc * mu::getMinTimeDoubleIntegrator3D(initial.pos, initial.vel, final.pos,
                                                                          final.vel, par_.v_max, par_.a_max);

  std::cout << "time_allocated= " << time_allocated << std::endl;

  double t_final = t_start + time_allocated;

  bool correctInitialCond =
      solver_->setInitStateFinalStateInitTFinalT(initial, final, t_start,
                                                 t_final);  // note that here t_final may have been updated

  if (correctInitialCond == false)
  {
    std::cout << bold << red << "The solver cannot guarantee feasibility for v1" << reset << std::endl;
    return false;
  }

  ////////////////

  mtx_trajs_.lock();

  time_init_opt_ = ros::Time::now().toSec();
  removeTrajsThatWillNotAffectMe(A, t_start, t_final);
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_start, t_final);
  mtx_trajs_.unlock();

  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = cu::vectorGCALPol2vectorJPSPol(hulls);
  edges_obstacles_out = cu::vectorGCALPol2edges(hulls);

  solver_->setHulls(hulls_std);

  //////////////////////
  std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
            << std::endl;

  std::cout << "[FA] Calling NL" << std::endl;

  bool result = solver_->optimize();

  num_of_LPs_run = solver_->getNumOfLPsRun();
  num_of_QCQPs_run = solver_->getNumOfQCQPsRun();

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

  solver_->getPlanes(planes);

  solutions_found_++;

  // av_improvement_nlopt_ = ((solutions_found_ - 1) * av_improvement_nlopt_ + solver_->improvement_) /
  // solutions_found_;

  // std::cout << blue << "Average improvement so far" << std::setprecision(5) << av_improvement_nlopt_ << reset
  //          << std::endl;

  mt::PieceWisePol pwp_now;
  solver_->getSolution(pwp_now);

  MyTimer check_t(true);
  mtx_trajs_.lock();
  bool is_safe_after_opt = safetyCheckAfterOpt(pwp_now);
  mtx_trajs_.unlock();
  // std::cout << bold << "Check Timer=" << check_t << std::endl;

  if (is_safe_after_opt == false)
  {
    ROS_ERROR_STREAM("safetyCheckAfterOpt is not satisfied, returning");
    return false;
  }

  M_ = G_term;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Append to plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////
  mtx_plan_.lock();

  int plan_size = plan_.size();

  if ((plan_size - 1 - k_index_end) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    // std::cout << "plan_size= " << plan_size << std::endl;
    // std::cout << "k_index_end= " << k_index_end << std::endl;
    mtx_plan_.unlock();
    return false;
  }
  else
  {
    // std::cout << "Appending" << std::endl;
    // std::cout << "before, plan_size=" << plan_.size() << std::endl;
    plan_.erase(plan_.end() - k_index_end - 1, plan_.end());  // this deletes also the initial condition...
    // std::cout << "middle, plan_size=" << plan_.size() << " sol.size()=" << (solver_->traj_solution_).size()
    // << std::endl;
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      plan_.push_back(solver_->traj_solution_[i]);
    }
    // std::cout << "after, plan_size=" << plan_.size() << std::endl;
  }

  mtx_plan_.unlock();

  ////////////////////
  ////////////////////

  if (exists_previous_pwp_ == true)
  {
    pwp_out = mu::composePieceWisePol(time_now, par_.dc, pwp_prev_, pwp_now);
    pwp_prev_ = pwp_out;
  }
  else
  {  //
    pwp_out = pwp_now;
    pwp_prev_ = pwp_now;
    exists_previous_pwp_ = true;
  }

  X_safe_out = plan_.toStdVector();

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  // Check if we have planned until G_term
  mt::state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
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

void Mader::yaw(double diff, mt::state& next_goal)
{
  mu::saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  double dyaw_not_filtered;

  dyaw_not_filtered = copysign(1, diff) * par_.w_max;

  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered + par_.alpha_filter_dyaw * dyaw_filtered_;
  next_goal.dyaw = dyaw_filtered_;
  next_goal.yaw = previous_yaw_ + dyaw_filtered_ * par_.dc;
}

void Mader::getDesiredYaw(mt::state& next_goal)
{
  double diff = 0.0;
  double desired_yaw = 0.0;

  switch (drone_status_)
  {
    case DroneStatus::YAWING:
      desired_yaw = atan2(G_term_.pos[1] - next_goal.pos[1], G_term_.pos[0] - next_goal.pos[0]);
      diff = desired_yaw - state_.yaw;
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

  mu::angle_wrap(diff);
  if (fabs(diff) < 0.04 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }
  yaw(diff, next_goal);
}

bool Mader::getNextGoal(mt::state& next_goal)
{
  if (initializedStateAndTermGoal() == false)  // || (drone_status_ == DroneStatus::GOAL_REACHED && plan_.size() == 1))
  {
    // std::cout << "Not publishing new goal!!" << std::endl;
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

void Mader::printDroneStatus()
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