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
  // drone_status_ == DroneStatus::YAWING;
  drone_status_ == DroneStatus::TRAVELING;
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
    // std::cout << red << "Basis " << par.basis << " not implemented yet" << reset << std::endl;
    // std::cout << red << "============================================" << reset << std::endl;
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
  traj_compiled.time_created = traj.time_created;

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

void Mader::updateTrajObstacles(mt::dynTraj traj, const mt::PieceWisePol& pwp_now, const bool& is_in_DC, bool& delay_check_result, const double& headsup_time)
{
  
  delay_check_result = true;

  MyTimer tmp_t(true);

  if (started_check_ == true && traj.is_agent == true)
  {
    have_received_trajectories_while_checking_ = true;
  }

  mtx_trajs_.lock();

  // std::vector<mt::dynTrajCompiled>::iterator obs_ptr =
  //     std::find_if(trajs_.begin(), trajs_.end(),
  //                  [=](const mt::dynTrajCompiled& traj_compiled) { return traj_compiled.id == traj.id; });

  // bool exists_in_local_map = (obs_ptr != std::end(trajs_));

  mt::dynTrajCompiled traj_compiled;
  dynTraj2dynTrajCompiled(traj, traj_compiled);

  if (is_in_DC){ 

    // do delay check for the new traj
    if (traj_compiled.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj_compiled, pwp_now, pwp_now.times.front(), pwp_now.times.back()))
      {
        ROS_ERROR_STREAM("In delay check traj_compiled collides with " << traj_compiled.id);
        delay_check_result = false;  // will have to redo the optimization
        mtx_trajs_.unlock();
        have_received_trajectories_while_checking_ = false;
        return;
      } else if (traj_compiled.time_created == headsup_time) // tie breaking: compare x, y, z and bigger one wins
      {
        Eigen::Vector3d center_obs;
        center_obs << traj_compiled.function[0].value(), traj_compiled.function[1].value(), traj_compiled.function[2].value();
        if (center_obs[0] > state_.pos[0])
        {
          delay_check_result = false;
          mtx_trajs_.unlock();
          have_received_trajectories_while_checking_ = false;
          return;
        } else if (center_obs[1] > state_.pos[1])
        {
          delay_check_result = false;
          mtx_trajs_.unlock();
          have_received_trajectories_while_checking_ = false;
          return;
        } else if (center_obs[2] > state_.pos[2])
        {
          delay_check_result = false;
          mtx_trajs_.unlock();
          have_received_trajectories_while_checking_ = false;
          return;
        }
      // center_obs[0] == state_.pos[0] &&  center_obs[1] == state_.pos[1] &&  center_obs[2] == state_.pos[2] won't happen bc it's the same position and collision
      }
    }
  }

  // if (exists_in_local_map && traj.is_committed)
  if (traj.is_committed)
  {  // if that object already exists, substitute its trajectory    
    // clean

    // std::cout << "clean!" << std::endl;

    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](mt::dynTrajCompiled const& traj) { return traj.id == traj_compiled.id; }),
        trajs_.end());

    // std::cout << "traj_compiled.id is " << traj_compiled.id << std::endl;
    // for (auto traj : trajs_){std::cout << "traj.id is " << traj.id << std::endl;}
  
    trajs_.push_back(traj_compiled);
  }
  else
  {  // if it doesn't exist or traj.is_committed == false, then add it to the local map
    trajs_.push_back(traj_compiled);
    // std::cout << "traj_compiled.id is " << traj_compiled.id << std::endl;
    // for (auto traj : trajs_){std::cout << "traj.id is " << traj.id << std::endl;}
    // ROS_WARN_STREAM("Adding " << traj_compiled.id);
  }

  // and now let's delete those trajectories of the obs/agents whose current positions are outside the local map
  // Note that these positions are obtained with the trajectory stored in the past in the local map

  std::vector<mt::dynTrajCompiled> local_trajs;

  for (auto traj_compiled : trajs_)
  {
    mtx_t_.lock();
    t_ = ros::Time::now().toSec();

    Eigen::Vector3d center_obs;
    center_obs << traj_compiled.function[0].value(), traj_compiled.function[1].value(), traj_compiled.function[2].value(); 

    mtx_t_.unlock();

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
      // if it's too far away then do nothing
    } else {
      local_trajs.push_back(traj_compiled);
    }
  }

  trajs_ = local_trajs;

  mtx_trajs_.unlock();

  have_received_trajectories_while_checking_ = false;
  // std::cout << bold << blue << "updateTrajObstacles took " << tmp_t << reset << std::endl;
}

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
  std::vector<mt::dynTrajCompiled> local_trajs;

  for (auto traj_compiled : trajs_)
  {
    mtx_t_.lock();
    t_ = ros::Time::now().toSec();

    Eigen::Vector3d center_obs;
    center_obs << traj_compiled.function[0].value(), traj_compiled.function[1].value(), traj_compiled.function[2].value(); 

    mtx_t_.unlock();

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
      // if it's too far away then do nothing
    } else {
      local_trajs.push_back(traj_compiled);
    }
  }

  trajs_ = local_trajs;

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
  Eigen::Vector3d drone_boundarybox = par_.drone_bbox;
  
  if (traj.is_agent == false)
  {
    std::vector<Eigen::Vector3d> points;
    // delta = traj.bbox / 2.0 + (par_.drone_radius + par_.beta + par_.alpha) *
    //                            Eigen::Vector3d::Ones();  // every side of the box will be increased by 2*delta
    //(+delta on one end, -delta on the other)

    // changeBBox(drone_boundarybox);

    delta = traj.bbox / 2.0 + drone_boundarybox / 2.0 + (par_.beta + par_.alpha) * Eigen::Vector3d::Ones();
    // std::cout << "boundary box size" << std::endl;
    // std::cout << drone_boundarybox[0] << std::endl;
    // std::cout << drone_boundarybox[1] << std::endl;
    // std::cout << drone_boundarybox[2] << std::endl;

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

    // delta = traj.bbox / 2.0 + (par_.drone_radius) * Eigen::Vector3d::Ones();
    // delta = traj.bbox / 2.0 + par_.drone_bbox / 2.0;  // instad of using drone_radius

    // changeBBox(drone_boundarybox);

    delta = traj.bbox / 2.0 + drone_boundarybox / 2.0 + (par_.beta + par_.alpha) * Eigen::Vector3d::Ones();
    // std::cout << "boundary box size" << std::endl;
    // std::cout << drone_boundarybox[0] << std::endl;
    // std::cout << drone_boundarybox[1] << std::endl;
    // std::cout << drone_boundarybox[2] << std::endl;

    // std::cout << "****traj.bbox = " << traj.bbox << std::endl;
    // std::cout << "****par_.drone_radius = " << par_.drone_radius << std::endl;
    // std::cout << "****Inflation by delta= " << delta.transpose() << std::endl;

    return vertexesOfInterval(traj.pwp, t_start, t_end, delta);
  }
}

void Mader::changeBBox(Eigen::Vector3d& drone_boundarybox){

  Eigen::Vector3d v(1e-5, 1e-5, 1e-5); // how much you make the box small?
  Eigen::Vector3d bbox_min_possible(par_.drone_bbox[0]-0.02, par_.drone_bbox[1]-0.02, par_.drone_bbox[2]-0.02);
  // double bbox_change_time = 1; //seconds, used for timer
  double unstuck_dist = 1.0; //meters
  Eigen::Vector3d dist(1e5, 1e5, 1e5); //random big numer, need to initialized before used 

  if (par_.is_stuck || if_bbox_change_ || is_A_star_failed_30_){

      if (!if_bbox_change_){
        // timer_bbox_.Reset(); // start timer
        stuck_state_for_bbox_ = state_; // get the current position 
        // measure the progress
      }

      dist = state_.pos - stuck_state_for_bbox_.pos;
      // std::cout << "boundary box change" << std::endl;
      // std::cout << "dist is " << dist << std::endl;
      // std::cout << "state_ is " << state_.pos << std::endl;
      // std::cout << "stuck_state_for_bbox_ is " << stuck_state_for_bbox_.pos << std::endl;

      // if (timer_bbox_.ElapsedMs() < bbox_change_time * 1000){ //using timer
      // if (!par_.is_stuck){ // TODO: measure the progress like position and if the drone moves certain distance, put the bbox size back 

      if (dist.norm() < unstuck_dist){
        drone_boundarybox = par_.drone_bbox - (stuck_count_for_bbox_ + 1) * v;
        // std::cout << "drone_bbox[0] is " << drone_boundarybox[0] << std::endl;
        // std::cout << "drone_bbox[1] is " << drone_boundarybox[1] << std::endl;
        // std::cout << "drone_bbox[2] is " << drone_boundarybox[2] << std::endl;

        for(int i = 0; i < 3; i++){
          drone_boundarybox[i] = std::max(drone_boundarybox[i], bbox_min_possible[i]);
        };
        // std::cout << "using smaller bbox" << std::endl;
        // std::cout << "stuck count is " << stuck_count_for_bbox_ << std::endl;
        // std::cout << "par_.is_stuck is " << par_.is_stuck << std::endl;
        stuck_count_for_bbox_ = stuck_count_for_bbox_ + 1;
        if_bbox_change_ = true;         
      } else {
        stuck_count_for_bbox_ = 0;
        if_bbox_change_ = false;
      }
    } else {
      drone_boundarybox = par_.drone_bbox;
      stuck_count_for_bbox_ = 0;
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

  std::vector<mt::dynTrajCompiled> local_trajs;

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
      // ids_to_remove.push_back(traj.id);
    } else {
      local_trajs.push_back(traj);
    }
  }

  trajs_ = local_trajs;

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
    // changeDroneStatus(DroneStatus::YAWING);
    changeDroneStatus(DroneStatus::TRAVELING);  // skip yawing
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

// in case drones are stuck we need a new G to detour
void Mader::getDetourG(mt::state& G)
{
  Eigen::Vector2d v = G_when_stuck_.pos.head(2) - stuck_state_.pos.head(2);
  Eigen::Vector2d v2;
  // v2 << v[1], -v[0]; // rotate a vector negative 90 deg
  // v2 << -v[0], -v[1]; // rotate a vector 180 deg
  v2 = RotationMatrix(v,-135 * M_PI / 180); // rotate a vector -135 deg
  v2.normalize();

  double v2_mag = 4.0 * par_.drone_bbox[0];
  // G.pos.head(2) = G.pos.head(2) + v2_mag * v2; // for -90deg rotatoin
  // G.pos.head(2) = stuck_state_.pos.head(2) + v2_mag * v2; // for 180 deg rotation
  detoured_G_.pos.head(2) = stuck_state_.pos.head(2) + v2_mag * v2;
  detoured_G_.pos[2] = G_when_stuck_.pos[2];

  // if new G is outside of highbay boundary, the project new G to highbay boundary
  if (detoured_G_.pos[0] > par_.x_max - par_.drone_bbox[0]) {
    detoured_G_.pos[0] = par_.x_max - par_.drone_bbox[0];
  }
  if (detoured_G_.pos[0] < par_.x_min + par_.drone_bbox[0]) {
    detoured_G_.pos[0] = par_.x_min + par_.drone_bbox[0];
  } 

  if (detoured_G_.pos[1] > par_.y_max - par_.drone_bbox[1]) {
    detoured_G_.pos[1] = par_.y_max - par_.drone_bbox[1];
  }
  if (detoured_G_.pos[1] < par_.y_min + par_.drone_bbox[1]) {
    detoured_G_.pos[1] = par_.y_min + par_.drone_bbox[1];
  } 

  G = detoured_G_;

  //     
  //      new G
  //        ^
  //        |            
  //        |            
  //     v2 | magnitude is v2_mag            
  //        |            
  //        |            
  //        |            v          
  //        G <----------------------- A (drone's current initial planning position)
  // 
  // 

}

void Mader::moveAtowardG(mt::state& A, mt::state& G){
  Eigen::Vector2d v = G.pos.head(2) - stuck_state_.pos.head(2);
  A.pos[0] = stuck_state_.pos[0] + 0.01 * v[0];
  A.pos[1] = stuck_state_.pos[1] + 0.01 * v[1];
}

Eigen::Vector2d Mader::RotationMatrix(Eigen::Vector2d& vec, const double& angle){
  // angle is radian
  Eigen::Vector2d vec2;
  vec2[0] = cos(angle)*vec[0] - sin(angle)*vec[1];
  vec2[1] = sin(angle)*vec[0] + cos(angle)*vec[1];
  return vec2;  
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

  if (!state_initialized_)
  {
    mt::state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    plan_.erase(plan_.begin(), plan_.end());
    plan_.push_back(tmp);  // plan_ should be empty
    // std::cout << "in updateState function ";
    // plan_.print();
    // previous_yaw_ = tmp.yaw;
    initial_yaw_ = tmp.yaw;
  }

  if (drone_status_ == DroneStatus::TRAVELING)
  {
    state_initialized_ = true;
  }

  /* skip yawing process
  if (drone_status_ == DroneStatus::YAWING)
  {
    state_initialized_ = true;
  }
  */
  // state_initialized_ = true;

  // mt::state tmp;
  // tmp.pos = data.pos;
  // tmp.yaw = data.yaw;
  // plan_.pop_front();
  // plan_.push_back(tmp); // plan_ should be empty
  // std::cout << "plan size " << plan_.size() << std::endl;
  // std::cout << "in updateState function ";
  // plan_.print();
  // previous_yaw_ = tmp.yaw;

  // state_.print();
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
// Check period and Recheck period is defined here
bool Mader::safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized, double& headsup_time)
{
  started_check_ = true;

  bool result = true;
  for (auto &traj : trajs_)
  {
    if (traj.time_received > headsup_time && traj.is_agent == true)
    // if (traj.is_agent == true) // need to include the trajs that came in the last delay check
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
  // This is Recheck
  if (have_received_trajectories_while_checking_ == true)
  {
    ROS_ERROR_STREAM("Recvd traj while checking ");
    result = false;
  }

  started_check_ = false;

  return result;
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

// Delay check
bool Mader::everyTrajCheck(mt::PieceWisePol pwp_optimized)
{
  // std::cout << "bef mtx_trajs_.lock() in delayCheck" << std::endl;
  mtx_trajs_.lock(); // this function is called in mader_ros.cpp so need to lock in the function
  // std::cout << "aft mtx_trajs_.lock() in delayCheck" << std::endl;

  bool result = true;
  for (auto &traj : trajs_)
  {
    if (traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_optimized, pwp_optimized.times.front(), pwp_optimized.times.back()))
      {
        ROS_ERROR_STREAM("[First Delay Check] In delay check traj collides with " << traj.id);
        result = false;  // will have to redo the optimization
        break;
      }
    } 
  }

  // std::cout << "bef mtx_trajs_.unlock() in delayCheck" << std::endl;
  mtx_trajs_.unlock();
  // std::cout << "aft mtx_trajs_.unlock() in delayCheck" << std::endl;

  return result;
}

// this is just Check in case A* failed
bool Mader::safetyCheck_for_A_star_failure(mt::PieceWisePol pwp_prev)
{
  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_prev, pwp_prev.times.front(), pwp_prev.times.back()))
      {
        if (id_ < traj.id){ // tie breaking mechanism
          ROS_ERROR_STREAM("my previous traj collides with " << traj.id << " and they have higher number, so I need to change my traj.");
          result = false;  // will have to redo the optimization  
        }
        break;
      }
    }
  }
  return result;
}

// this is just Check in case A* failed
bool Mader::safetyCheck_for_A_star_failure_pwp_now(mt::PieceWisePol pwp_now)
{
  bool result = true;
  for (auto traj : trajs_)
  {
    if (traj.time_received > time_init_opt_ && traj.is_agent == true)
    {
      if (trajsAndPwpAreInCollision(traj, pwp_now, pwp_now.times.front(), pwp_now.times.back()))
      {
        if (id_ < traj.id){ // tie breaking mechanism
          ROS_ERROR_STREAM("my pwp_now collides with " << traj.id << " and they have higher number, so I need to change my traj.");
          result = false;  // will have to redo the optimization  
        }
        break;
      }
    }
  }
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
    // std::cout << "Status changed to GOAL_SEEN!" << std::endl;
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

bool Mader::replan_with_delaycheck(mt::Edges& edges_obstacles_out, std::vector<mt::state>& headsup_plan,
                   std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                   mt::PieceWisePol& pwp_now, double& headsup_time)
{
  if (isReplanningNeeded() == false)
  {
    // std::cout << "replan is not needed" << std::endl;
    return false;
  }

  // Pop-up for demos

  // if (is_z_max_increased_ && !is_going_back_to_normal_z_max_){
  //   // following popped up trajectory and haven't yet reached to pop_up_last_state_in_plan_ 
  //   Eigen::Vector3d diff = state_.pos - pop_up_last_state_in_plan_.pos;
  //   pwp_out = mu::constPosition2pwp(pop_up_last_state_in_plan_.pos);
  //   if (diff.norm() > 0.1){
  //     return true;
  //   }
  //   std::cout << "reached pop_up_last_state_in_plan_" << "\n";
  //   is_going_back_to_normal_z_max_ = true;
  // } 

  // if (is_z_max_increased_ && is_going_back_to_normal_z_max_){
  //   // once you reached pop_up_last_state_in_plan_, then you start planning new traj in extended z_max space
  //   // std::cout << "state_.pos[2] " << state_.pos[2] << "\n";
  //   // std::cout << "par_.z_max_ " << par_.z_max << "\n";
  //   // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";
  //   std::cout << "going back to the nominal space" << "\n";
  //   if (state_.pos[2] < par_.z_max - 0.3){ // if you go back to the nominal z_max, then put z_max back. 0.3 is a buffer
  //     std::cout << "got back to the nomial space!!" << "\n";
  //     solver_->changeZmax(par_.z_max); // put the z_max back to the nominal value
  //     is_z_max_increased_ = false;
  //     is_going_back_to_normal_z_max_ = false;
  //   }
  // }
  // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";

  MyTimer replanCB_t(true);

  std::cout << bold << on_white << "**********************IN REPLAN CB*******************" << reset << std::endl;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select mt::state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  mt::state A;
  int k_index;

  // If k_index_end_=0, then A = plan_.back() = plan_[plan_.size() - 1]

  mtx_plan_.lock();

  mu::saturate(deltaT_, par_.lower_bound_runtime_snlopt / par_.dc, par_.upper_bound_runtime_snlopt / par_.dc);

  k_index_end_ = std::max((int)(plan_.size() - deltaT_), 0);

  if (plan_.size() < 5)
  {
    k_index_end_ = 0;
  }

  k_index = plan_.size() - 1 - k_index_end_;
  A = plan_.get(k_index);

  // std::cout << "in replan_with_delaycheck before opt" << std::endl;
  // plan_.print();

  mtx_plan_.unlock();

  // std::cout << blue << "k_index:" << k_index << reset << std::endl;
  // std::cout << blue << "k_index_end_:" << k_index_end_ << reset << std::endl;
  // std::cout << blue << "plan_.size():" << plan_.size() << reset << std::endl;

  double runtime_snlopt;

  if (k_index_end_ != 0)
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
  
  // detoured G

  // Eigen::Vector3d dist_prog(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_to_goal(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_from_state__to_goal(1e5, 1e5, 1e5);
  // double unstuck_dist = 2.0; //meters
  // double reached_goal_dist = par_.drone_bbox[0]+0.1; // avoid the detoured goal is within others bbox and cannot move at all sitution.
  // double how_much_to_detoured_G = 0.7; //0.1 means 10%
  // double vel_stuck_detect = 0.1; // m/s

  // double detour_max_time = 5; //seconds, how long want to detour
  
  // if (par_.is_stuck || if_detour_ || is_A_star_failed_30_){
    
  //   if (par_.is_stuck && !if_detour_){
  //     // timer_detour_.Reset();
  //     stuck_state_ = state_;
  //     A_when_stuck_ = A;
  //     G_when_stuck_ = G;
  //   }

  //   dist_prog = state_.pos - stuck_state_.pos;
  //   dist_to_goal = detoured_G_.pos - stuck_state_.pos;
  //   dist_from_state__to_goal = detoured_G_.pos - state_.pos;

  //   // std::cout << "dist_prog is " << dist_prog << std::endl;
  //   // std::cout << "state_.pos is " << state_.pos << std::endl;
  //   // std::cout << "stuck_state_.pos is " << stuck_state_.pos << std::endl;

  //   //if (timer_detour_.ElapsedMs() < detour_max_time * 1000){
  //   if (state_.vel.norm() < vel_stuck_detect &&
  //    dist_from_state__to_goal.norm() > reached_goal_dist &&
  //    dist_prog.norm() < unstuck_dist && 
  //    dist_prog.norm() < how_much_to_detoured_G * dist_to_goal.norm()){
      
  //     getDetourG(G); // if stuck, make a new G for detour
  //     // if (!if_A_moveback_){
  //     //   moveAtowardG(A, G);
  //     //   if_A_moveback_ = true;
  //     // }
  //     std::cout << "using detoured G" << std::endl;
  //     stuck_count_for_detour_ = stuck_count_for_detour_ + 1;
  //     if_detour_ = true;
  //     if (stuck_count_for_detour_ > 10){
  //       if_detour_ = false; // maybe the detour G is also causing stuck, so go back to the original one
  //       if_A_moveback_ = false;
  //     }
  //   } else {
  //     stuck_count_for_detour_ = 0;
  //     if_detour_ = false;
  //     if_A_moveback_ = false;
  //   //   std::cout << "stop using detoured G" << std::endl;
  //   //   std::cout << "dist_prog.norm() is " << dist_prog.norm() << std::endl;
  //   } 
  // }

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
  time_now_ = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_start = k_index * par_.dc + time_now_;

  double factor_alloc = (distA2TermGoal > par_.dist_factor_alloc_close) ? par_.factor_alloc : par_.factor_alloc_close;

  double time_allocated = factor_alloc * mu::getMinTimeDoubleIntegrator3D(initial.pos, initial.vel, final.pos,
                                                                          final.vel, par_.v_max, par_.a_max);

  std::cout << "time_allocated= " << time_allocated << std::endl;

  // std::cout << "initial is " << initial.pos.transpose() << std::endl;
  // std::cout << "Pos of A is" << A.pos.transpose() << std::endl;

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
  // std::cout << "bef mtx_trajs_.lock() in replan" << std::endl;
  mtx_trajs_.lock();
  // std::cout << "aft mtx_trajs_.lock() in replan" << std::endl;

  time_init_opt_ = ros::Time::now().toSec();
  removeTrajsThatWillNotAffectMe(A, t_start, t_final);
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_start, t_final);

  // std::cout << "bef mtx_trajs_.unlock() in replan" << std::endl;
  mtx_trajs_.unlock();
  // std::cout << "aft mtx_trajs_.unlock() in replan" << std::endl;

  mt::ConvexHullsOfCurves_Std hulls_std = cu::vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = cu::vectorGCALPol2vectorJPSPol(hulls);
  edges_obstacles_out = cu::vectorGCALPol2edges(hulls);

  solver_->setHulls(hulls_std);

  //////////////////////
  // std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
  //           << std::endl;

  // std::cout << "[FA] Calling NL" << std::endl;

  bool is_stuck;
  bool is_A_star_failed;
  bool result = solver_->optimize(is_stuck, is_A_star_failed);  // calling the solver

  // right after taking off, sometims drones cannot find a path
  // sometimes the very initial path search takes more than how_many_A_star_failure counts and fails
  // if (planner_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed && !is_pwp_prev_feasible_){
  //     A_star_fail_count_ += 1;
  //     is_A_star_failed_30_ = (A_star_fail_count_ > 30);
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_ is " << A_star_fail_count_ << "\n"; 
  //     if (is_A_star_failed_30_){
  //       if(!safetyCheck_for_A_star_failure(pwp_prev_)){
  //         std::cout << "previous pwp collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone 
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         A_star_fail_count_ = 0;
  //         return false; //abort mader
  //       } else {
  //         // std::cout << "previous pwp doesn't collide!\n";
  //         is_pwp_prev_feasible_ = true;
  //         is_pop_up_ = false;
  //         A_star_fail_count_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_ = 0;
  //   }

  // }

  // // check if drones are stuck or not
  // if (is_stuck){
  //   par_.is_stuck = true;
  // } else {
  //   par_.is_stuck = false;
  // }

  num_of_LPs_run = solver_->getNumOfLPsRun();
  num_of_QCQPs_run = solver_->getNumOfQCQPsRun();

  total_replannings_++;
  if (result == false)
  {
    int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000) + par_.expected_comm_delay / par_.dc);  // Number of states that
                                                                               // would have been needed for
                                                                               // the last replan
    deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
    deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
    std::cout << "solver couldn't find optimal path" << std::endl;
    return false;
  }
  
  solver_->getPlanes(planes);

  solutions_found_++;

  // av_improvement_nlopt_ = ((solutions_found_ - 1) * av_improvement_nlopt_ + solver_->improvement_) /
  // solutions_found_;

  // std::cout << blue << "Average improvement so far" << std::setprecision(5) << av_improvement_nlopt_ << reset
  //          << std::endl;

  solver_->getSolution(pwp_now);

  // // check if the current position/plan is colliding
  // if (is_pop_up_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed_){
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     A_star_fail_count_pwp_now_ = A_star_fail_count_pwp_now_ + 1;
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_pwp_now_ is " << A_star_fail_count_pwp_now_ << "\n"; 
  //     double how_many_A_star_failure_now = 30;
  //     if (A_star_fail_count_pwp_now_ > how_many_A_star_failure_now){
  //       if(!safetyCheck_for_A_star_failure_pwp_now(pwp_now)){
  //         std::cout << "pwp now collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone 
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         return false; //abort mader
  //       } else {
  //         is_pop_up_ = false;
  //         A_star_fail_count_pwp_now_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_pwp_now_ = 0;
  //   }

  // }

  MyTimer check_t(true);

  // std::cout << "bef mtx_trajs_.lock() in replan (safetyCheckAfterOpt)" << std::endl;
  mtx_trajs_.lock();
  // std::cout << "aft mtx_trajs_.lock() in replan (safetyCheckAfterOpt)" << std::endl;

  // first make sure none of the trajectory is checked in this optimization
  // for (auto &traj : trajs_){
  //   traj.is_checked = false;
  // }

  // check and recheck are done in safetyChechAfterOpt() 
  bool is_safe_after_opt = safetyCheckAfterOpt(pwp_now, headsup_time);

  // std::cout << "bef mtx_trajs_.unlock() in replan (safetyCheckAfterOpt)" << std::endl;
  mtx_trajs_.unlock();
  // std::cout << "aft mtx_trajs_.unlock() in replan (safetyCheckAfterOpt)" << std::endl;

  // To deal with comm delay, now we publish two trajectories(pwp_prev_ and pwp_now)

  if (is_safe_after_opt == false)
  {
    ROS_ERROR_STREAM("safetyCheckAfterOpt is not satisfied, returning");
    return false;
  }

  ///////////////////////////////////////////////////////////
  ///////////////       OTHER STUFF    //////////////////////
  //////////////////////////////////////////////////////////

  mtx_plan_.lock();

  // Check if we have planned until G_term
  mt::state F = plan_.back();  // Final point of the safe path (\equiv final point of the comitted path)
  
  mtx_plan_.unlock();

  double dist = (G_term_.pos - F.pos).norm();

  if (dist < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN);
  }

  mtx_offsets.lock();

  int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000) + par_.expected_comm_delay / par_.dc);  // Number of states that
                                                                               // would have been needed for
                                                                               // the last replan
  deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);
  mtx_offsets.unlock();

  // std::cout << "end of replan" << "\n";
  // std::cout << "k_index_end_ is " << k_index_end_ << "\n";
  // std::cout << "deltaT_ is " << deltaT_ << "\n";
  // std::cout << "plan_.size() " << plan_.size() << "\n";

  planner_initialized_ = true;

  mtx_plan_.lock();

  // headsup trajectory
  headsup_plan = plan_.toStdVector();
  
  int headsup_plan_size = headsup_plan.size();

  if ((headsup_plan_size - 1 - k_index_end_) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    std::cout << "headsup_plan_size= " << headsup_plan_size << std::endl;
    std::cout << "k_index_end_= " << k_index_end_ << std::endl;
    mtx_plan_.unlock();
    return false;
  } else {
    headsup_plan.erase(headsup_plan.end() - k_index_end_ - 1, headsup_plan.end());  // this deletes also the initial condition...
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      headsup_plan.push_back(solver_->traj_solution_[i]);
    }
  }

  mtx_plan_.unlock();

  return true;

}

bool Mader::addTrajToPlan_with_delaycheck(mt::PieceWisePol& pwp){

  // std::cout << bold << "Check Timer=" << check_t << std::endl;

  mtx_G_term.lock();
  mt::state G_term = G_term_;  // Local copy of the terminal terminal goal
  mtx_G_term.unlock();

  M_ = G_term;

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Append to plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  mtx_plan_.lock();

  int plan_size = plan_.size();

  if ((plan_size - 1 - k_index_end_) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    std::cout << "plan_size= " << plan_size << std::endl;
    std::cout << "k_index_end_= " << k_index_end_ << std::endl;
    mtx_plan_.unlock();
    return false;
  }
  else
  {
    // std::cout << "Appending" << std::endl;
    // std::cout << "before, plan_size=" << plan_.size() << std::endl;
    plan_.erase(plan_.end() - k_index_end_ - 1, plan_.end());  // this deletes also the initial condition...
    // std::cout << "middle, plan_size=" << plan_.size() << " sol.size()=" << (solver_->traj_solution_).size()
    // << std::endl;
    for (int i = 0; i < (solver_->traj_solution_).size(); i++)  //... which is included in traj_solution_[0]
    {
      plan_.push_back(solver_->traj_solution_[i]);
    }
    // std::cout << "after, plan_size=" << plan_.size() << std::endl;
  }
  
  // std::cout << "in addTrajToPlan_with_delaycheck" << std::endl;
  // plan_.print();
  mtx_plan_.unlock();

  ////////////////////
  ////////////////////

  if (exists_previous_pwp_ == true)
  {
    pwp_prev_ = mu::composePieceWisePol(time_now_, par_.dc, pwp_prev_, pwp);
  }
  else
  {
    pwp_prev_ = pwp;
    exists_previous_pwp_ = true;
  }

  is_pwp_prev_feasible_ = false; // we don't know for sure if this traj is feasible until you run opt in the next step and see A* fails

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

  // if (is_z_max_increased_ && !is_going_back_to_normal_z_max_){
  //   // following popped up trajectory and haven't yet reached to pop_up_last_state_in_plan_ 
  //   Eigen::Vector3d diff = state_.pos - pop_up_last_state_in_plan_.pos;
  //   pwp_out = mu::constPosition2pwp(pop_up_last_state_in_plan_.pos);
  //   if (diff.norm() > 0.1){
  //     return true;
  //   }
  //   std::cout << "reached pop_up_last_state_in_plan_" << "\n";
  //   is_going_back_to_normal_z_max_ = true;
  // } 

  // if (is_z_max_increased_ && is_going_back_to_normal_z_max_){
  //   // once you reached pop_up_last_state_in_plan_, then you start planning new traj in extended z_max space
  //   // std::cout << "state_.pos[2] " << state_.pos[2] << "\n";
  //   // std::cout << "par_.z_max_ " << par_.z_max << "\n";
  //   // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";
  //   std::cout << "going back to the nominal space" << "\n";
  //   if (state_.pos[2] < par_.z_max - 0.3){ // if you go back to the nominal z_max, then put z_max back. 0.3 is a buffer
  //     std::cout << "got back to the nomial space!!" << "\n";
  //     solver_->changeZmax(par_.z_max); // put the z_max back to the nominal value
  //     is_z_max_increased_ = false;
  //     is_going_back_to_normal_z_max_ = false;
  //   }
  // }
  // std::cout << "par_for_solver.z_max " << solver_->printZmax() << "\n";

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
  
  // Eigen::Vector3d dist_prog(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_to_goal(1e5, 1e5, 1e5);
  // Eigen::Vector3d dist_from_state__to_goal(1e5, 1e5, 1e5);
  // double unstuck_dist = 2.0; //meters
  // double reached_goal_dist = par_.drone_bbox[0]+0.1; // avoid the detoured goal is within others bbox and cannot move at all sitution.
  // double how_much_to_detoured_G = 0.7; //0.1 means 10%
  // double vel_stuck_detect = 0.1; // m/s

  // // double detour_max_time = 5; //seconds, how long want to detour
  
  // if (par_.is_stuck || if_detour_ || is_A_star_failed_30_){
    
  //   if (par_.is_stuck && !if_detour_){
  //     // timer_detour_.Reset();
  //     stuck_state_ = state_;
  //     A_when_stuck_ = A;
  //     G_when_stuck_ = G;
  //   }

  //   dist_prog = state_.pos - stuck_state_.pos;
  //   dist_to_goal = detoured_G_.pos - stuck_state_.pos;
  //   dist_from_state__to_goal = detoured_G_.pos - state_.pos;

  //   // std::cout << "dist_prog is " << dist_prog << std::endl;
  //   // std::cout << "state_.pos is " << state_.pos << std::endl;
  //   // std::cout << "stuck_state_.pos is " << stuck_state_.pos << std::endl;

  //   //if (timer_detour_.ElapsedMs() < detour_max_time * 1000){
  //   if (state_.vel.norm() < vel_stuck_detect &&
  //    dist_from_state__to_goal.norm() > reached_goal_dist &&
  //    dist_prog.norm() < unstuck_dist && 
  //    dist_prog.norm() < how_much_to_detoured_G * dist_to_goal.norm()){
      
  //     getDetourG(G); // if stuck, make a new G for detour
  //     // if (!if_A_moveback_){
  //     //   moveAtowardG(A, G);
  //     //   if_A_moveback_ = true;
  //     // }
  //     std::cout << "using detoured G" << std::endl;
  //     stuck_count_for_detour_ = stuck_count_for_detour_ + 1;
  //     if_detour_ = true;
  //     if (stuck_count_for_detour_ > 10){
  //       if_detour_ = false; // maybe the detour G is also causing stuck, so go back to the original one
  //       if_A_moveback_ = false;
  //     }
  //   } else {
  //     stuck_count_for_detour_ = 0;
  //     if_detour_ = false;
  //     if_A_moveback_ = false;
  //   //   std::cout << "stop using detoured G" << std::endl;
  //   //   std::cout << "dist_prog.norm() is " << dist_prog.norm() << std::endl;
  //   } 
  // }

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

  // std::cout << "initial is " << initial.pos.transpose() << std::endl;
  // std::cout << "Pos of A is" << A.pos.transpose() << std::endl;

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
  // std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
  //           << std::endl;

  // std::cout << "[FA] Calling NL" << std::endl;

  bool is_stuck;
  bool is_A_star_failed;
  bool result = solver_->optimize(is_stuck, is_A_star_failed);  // calling the solver

  // right after taking off, sometims drones cannot find a path
  // sometimes the very initial path search takes more than how_many_A_star_failure counts and fails
  // if (planner_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed && !is_pwp_prev_feasible_){
  //     A_star_fail_count_ += 1;
  //     is_A_star_failed_30_ = (A_star_fail_count_ > 30);
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_ is " << A_star_fail_count_ << "\n"; 
  //     if (is_A_star_failed_30_){
  //       if(!safetyCheck_for_A_star_failure(pwp_prev_)){
  //         std::cout << "previous pwp collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone 
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         A_star_fail_count_ = 0;
  //         return false; //abort mader
  //       } else {
  //         // std::cout << "previous pwp doesn't collide!\n";
  //         is_pwp_prev_feasible_ = true;
  //         is_pop_up_ = false;
  //         A_star_fail_count_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_ = 0;
  //   }

  // }

  // // check if drones are stuck or not
  // if (is_stuck){
  //   par_.is_stuck = true;
  // } else {
  //   par_.is_stuck = false;
  // }

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

  // // check if the current position/plan is colliding
  // if (is_pop_up_initialized_){

  //   // check if A_star is failed and see if the previous plan is feasible
  //   if (is_A_star_failed_){
  //     // need to check if my previous traj collides with others. and if that's the case pop it up
  //     A_star_fail_count_pwp_now_ = A_star_fail_count_pwp_now_ + 1;
  //     std::cout << "A_star is failing\n";
  //     std::cout << "A_star_fail_count_pwp_now_ is " << A_star_fail_count_pwp_now_ << "\n"; 
  //     double how_many_A_star_failure_now = 30;
  //     if (A_star_fail_count_pwp_now_ > how_many_A_star_failure_now){
  //       if(!safetyCheck_for_A_star_failure_pwp_now(pwp_now)){
  //         std::cout << "pwp now collide!" << "\n";
  //         // if previous pwp is not feasible pop up the drone 
  //         // this only happens when two agents commit traj at the very same time (or in Recheck period)
  //         is_pop_up_ = true;
  //         return false; //abort mader
  //       } else {
  //         is_pop_up_ = false;
  //         A_star_fail_count_pwp_now_ = 0;
  //       }
  //     }
  //   } else {
  //     is_pop_up_ = false;
  //     A_star_fail_count_pwp_now_ = 0;
  //   }

  // }

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
    // std::cout << bold << red << "Already published the point A" << reset << std::endl;
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

  // plan_.print();
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
  is_pwp_prev_feasible_ = false; // we don't know for sure if this traj is feasible until you run opt in the next step and see A* fails

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
  next_goal.dyaw = 0.0;
  next_goal.yaw = initial_yaw_;
  // next_goal.yaw = state_.yaw;
  /*
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
  */
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

  // if (is_pop_up_ && !is_z_max_increased_){
  //   // we need to pop up the drone
  //   std::cout << "pop up!!!!" << std::endl;
  //   double pop_up_alt = par_.drone_bbox[2] + 0.1;
  //   pop_up_state_ = state_;
  //   std::cout << "plan_.size() is " << plan_.size() << "\n";
  //   if (plan_.size() != 0){
  //     for (int i = 0; i < plan_.size(); i++){
  //     // increment z axis value
  //     double pop_up_increment = pop_up_alt / plan_.size();
  //     plan_.content[i].pos[2] = plan_.content[i].pos[2] + i * pop_up_increment;
  //     } 
  //   } else {
  //     double pop_up_increment = pop_up_alt / 400;
  //     for (int i = 0; i < 400; i++){
  //       mt::state temporaty_state = state_;
  //       temporaty_state.pos[2] += i * pop_up_increment;
  //       plan_.push_back(temporaty_state);
  //     }
  //   }
    
  //   is_pop_up_ = false;
  //   pop_up_last_state_in_plan_ = plan_.back();
  //   double new_zmax = par_.z_max + pop_up_alt;
  //   solver_->changeZmax(new_zmax);
  //   is_z_max_increased_ = true;
  // }

  if (plan_.size() > 1)
  {
    plan_.pop_front();
  }
  getDesiredYaw(next_goal);  // we don't need to control yaw

  // previous_yaw_ = next_goal.yaw;

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

  // std::cout << "Changing DroneStatus from ";
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

  // std::cout << std::endl;

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

void Mader::getID(int& id){
  id_ = id;
}