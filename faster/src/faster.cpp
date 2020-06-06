// Jesus Tordesillas Torres, jtorde@mit.edu
#include <Eigen/StdVector>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdlib.h>

#include "faster.hpp"
#include "timer.hpp"
#include "termcolor.hpp"

#include "solvers/nlopt/nlopt_utils.hpp"

using namespace JPS;
using namespace termcolor;

// Uncomment the type of timer you want:
// typedef ROSTimer MyTimer;
// typedef ROSWallTimer MyTimer;
typedef Timer MyTimer;

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

  double max_values[3] = { par_.v_max, par_.a_max, par_.j_max };

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

  ////
  bool success_service_call = system("rosservice call /change_mode 'mode: 1'");  // to avoid having to click on the GUI
  ////
}

void Faster::dynTraj2dynTrajCompiled(dynTraj& traj, dynTrajCompiled& traj_compiled)
{
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
  traj_compiled.bbox = traj.bbox;
  traj_compiled.id = traj.id;
  traj_compiled.time_received = traj.time_received;  // ros::Time::now().toSec();

  traj_compiled.is_static =
      (traj.function[0].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate x
      (traj.function[1].find("t") == std::string::npos) &&  // there is no dependence on t in the coordinate y
      (traj.function[2].find("t") == std::string::npos);    // there is no dependence on t in the coordinate z
}
// Note that we need to compile the trajectories inside faster.cpp because t_ is in faster.hpp
void Faster::updateTrajObstacles(dynTraj traj)
{
  MyTimer tmp_t(true);

  if (started_check_ == true && traj.is_agent == true)
  {
    have_received_trajectories_while_checking_ = true;
  }

  mtx_trajs_.lock();

  std::vector<dynTrajCompiled>::iterator obs_ptr = std::find_if(
      trajs_.begin(), trajs_.end(), [=](const dynTrajCompiled& traj_compiled) { return traj_compiled.id == traj.id; });

  bool exists_in_local_map = (obs_ptr != std::end(trajs_));

  dynTrajCompiled traj_compiled;
  dynTraj2dynTrajCompiled(traj, traj_compiled);

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

  for (int index_traj = 0; index_traj < trajs_.size(); index_traj++)
  {
    bool traj_affects_me = false;

    t_ = ros::Time::now().toSec();

    Eigen::Vector3d center_obs;
    center_obs << trajs_[index_traj].function[0].value(),  ////////////////////
        trajs_[index_traj].function[1].value(),            ////////////////
        trajs_[index_traj].function[2].value();            /////////////////

    if ((center_obs - state_.pos).norm() > 2 * par_.R_local_map)  // 2*Ra because: traj_{k-1} is inside a sphere of Ra.
                                                                  // Then, in iteration k the point A (which I don't
                                                                  // know yet)  is taken along that trajectory, and
                                                                  // another trajectory of radius Ra will be obtained.
                                                                  // Therefore, I need to take 2*Ra to make sure the
                                                                  // extreme case (A taken at the end of traj_{k-1} is
                                                                  // covered). removeTrajsThatWillNotAffectMe will later
                                                                  // on take care of deleting the ones I don't need once
                                                                  // I know A
    {
      ids_to_remove.push_back(trajs_[index_traj].id);
    }
  }

  for (auto id : ids_to_remove)
  {
    // std::cout << red << "Removing " << id << " at t=" << std::setprecision(12) << traj.time_received;
    // ROS_WARN_STREAM("Removing " << id);

    trajs_.erase(
        std::remove_if(trajs_.begin(), trajs_.end(), [&](dynTrajCompiled const& traj) { return traj.id == id; }),
        trajs_.end());
  }

  // First let's check if the object is near me:
  // if (near_me)
  // {
  // }
  // else  // not near me
  // {
  //   int distance =

  //       if (exists_in_local_map && ((par_.impose_fov == true && in_local_map == false) || (par_.impose_fov ==
  //       false)))
  //   {  // remove
  //     trajs_.erase(obs_ptr);
  //     std::cout << red << "Erasing " << (*obs_ptr).id << " at t=" << std::setprecision(12) << traj.time_received
  //               << reset << std::endl;
  //   }
  //   else
  //   {
  //     // if it doesn't exist, don't do anything
  //     // don't erase from trajs (but it hasn't been updated \implies it is local map)
  //   }

  //   // if (exists)  // remove if from the list if it exists
  //   // {
  //   //   trajs_.erase(obs_ptr);
  //   //   std::cout << red << "Erasing " << (*obs_ptr).id << " at t=" << std::setprecision(12) << traj.time_received
  //   //             << reset << std::endl;
  //   // }

  /// Debugging
  /*  std::cout << "[FA] Ids que tengo:" << std::endl;
    for (auto traj : trajs_)
    {
      std::cout << traj.id << ", ";
    }
    std::cout << std::endl;*/

  mtx_trajs_.unlock();
  have_received_trajectories_while_checking_ = false;
  // std::cout << bold << blue << "updateTrajObstacles took " << tmp_t << reset << std::endl;
}

// See https://doc.cgal.org/Manual/3.7/examples/Convex_hull_3/quickhull_3.cpp
CGAL_Polyhedron_3 Faster::convexHullOfInterval(dynTrajCompiled& traj, double t_start, double t_end)
{
  int samples_per_interval = std::max(par_.samples_per_interval, 4);  // at least 4 samples per interval
  double inc = (t_end - t_start) / (1.0 * (samples_per_interval - 1));

  std::vector<Point_3> points;

  double side_box_drone = (2 * par_.drone_radius);

  // Will always have a sample at the beginning of the interval, and another at the end.
  for (int i = 0; i < samples_per_interval; i++)
  {
    // Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    t_ = t_start + i * inc;
    // std::cout << "calling value(), traj_compiled.function.size()= " << traj.function.size() << std::endl;

    double x = traj.function[0].value();
    double y = traj.function[1].value();
    double z = traj.function[2].value();

    Eigen::Vector3d traj_bbox_with_uncertainty;

    if (traj.is_agent)
    {
      traj_bbox_with_uncertainty = traj.bbox;
    }
    else
    {
      if (traj.is_static)
      {  // static obstacle
        traj_bbox_with_uncertainty = par_.beta * traj.bbox;
      }
      else
      {  // dynamic obstacle
        traj_bbox_with_uncertainty =
            par_.beta * traj.bbox + par_.gamma * (t_end - time_init_opt_) *
                                        Eigen::Vector3d::Ones();  // note that by using t_end, we are taking the
                                                                  // highest uncertainty within that interval
      }
    }

    double delta_x = (traj_bbox_with_uncertainty(0) / 2.0) + (side_box_drone / 2.0);
    double delta_y = (traj_bbox_with_uncertainty(1) / 2.0) + (side_box_drone / 2.0);
    double delta_z = (traj_bbox_with_uncertainty(2) / 2.0) + (side_box_drone / 2.0);

    //"Minkowski sum along the trajectory: box centered on the trajectory"

    Point_3 p0(x + delta_x, y + delta_y, z + delta_z);
    Point_3 p1(x + delta_x, y - delta_y, z - delta_z);
    Point_3 p2(x + delta_x, y + delta_y, z - delta_z);
    Point_3 p3(x + delta_x, y - delta_y, z + delta_z);

    Point_3 p4(x - delta_x, y - delta_y, z - delta_z);
    Point_3 p5(x - delta_x, y + delta_y, z + delta_z);
    Point_3 p6(x - delta_x, y + delta_y, z - delta_z);
    Point_3 p7(x - delta_x, y - delta_y, z + delta_z);

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    points.push_back(p4);
    points.push_back(p5);
    points.push_back(p6);
    points.push_back(p7);
  }

  CGAL_Polyhedron_3 poly = convexHullOfPoints(points);

  return poly;
}

// trajs_ is already locked when calling this function
void Faster::removeTrajsThatWillNotAffectMe(const state& A, double t_start, double t_end)
{
  int samples_per_traj = 10;
  double inc = (t_end - t_start) / (1.0 * samples_per_traj);

  std::vector<int> ids_to_remove;

  for (int index_traj = 0; index_traj < trajs_.size(); index_traj++)
  {
    bool traj_affects_me = false;
    for (int i = 0; i < samples_per_traj; i++)
    {
      t_ = t_start + i * inc;

      Eigen::Vector3d center_obs;
      center_obs << trajs_[index_traj].function[0].value(),  ////////////////////
          trajs_[index_traj].function[1].value(),            ////////////////
          trajs_[index_traj].function[2].value();            /////////////////

      Eigen::Vector3d positive_half_diagonal;
      positive_half_diagonal << trajs_[index_traj].bbox[0] / 2.0,  //////////////////
          trajs_[index_traj].bbox[1] / 2.0,                        ////////////////
          trajs_[index_traj].bbox[2] / 2.0;

      Eigen::Vector3d c1 = center_obs - positive_half_diagonal;
      Eigen::Vector3d c2 = center_obs + positive_half_diagonal;

      //  std::cout << "Traj " << trajs_[index_traj].id << " is in " << center_obs.transpose() << std::endl;

      if (boxIntersectsSphere(A.pos, par_.Ra, c1, c2) == true)
      {
        traj_affects_me = true;
        break;  // go out from the inner-most loop
      }
    }

    if (traj_affects_me == false)
    {
      // std::cout << red << bold << "Going to  delete t raj " << trajs_[index_traj].id << reset << std::endl;

      ids_to_remove.push_back(trajs_[index_traj].id);
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

bool Faster::IsTranslating()
{
  return (drone_status_ == DroneStatus::GOAL_SEEN || drone_status_ == DroneStatus::TRAVELING);
}

ConvexHullsOfCurve Faster::convexHullsOfCurve(dynTrajCompiled& traj, double t_start, double t_end)
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

ConvexHullsOfCurves Faster::convexHullsOfCurves(double t_start, double t_end)
{
  ConvexHullsOfCurves result;

  for (auto traj : trajs_)
  {
    // std::cout << "Computing convex hull of curve " << traj.id << std::endl;
    // std::cout << "above, traj.function.size()= " << traj.function.size() << std::endl;
    // std::cout << "going to call convexHullsOfCurve" << std::endl;
    result.push_back(convexHullsOfCurve(traj, t_start, t_end));
    // std::cout << "called convexHullsOfCurve" << std::endl;
  }
  // std::cout << "end of convexHullsOfCurves" << std::endl;

  return result;
}

void Faster::setTerminalGoal(state& term_goal)
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
    // std::cout << bold << green << "[Faster] state_.yaw=" << state_.yaw << reset << std::endl;
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

void Faster::updateState(state data)
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

bool Faster::initializedAllExceptPlanner()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    /*    std::cout << "state_initialized_= " << state_initialized_ << std::endl;
        std::cout << "terminal_goal_initialized_= " << terminal_goal_initialized_ << std::endl;*/
    return false;
  }
  return true;
}

bool Faster::initializedStateAndTermGoal()
{
  if (!state_initialized_ || !terminal_goal_initialized_)
  {
    return false;
  }
  return true;
}

bool Faster::initialized()
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

// check wheter a dynTrajCompiled and a pwp_optimized are in collision in the interval [t_init, t_end]
bool Faster::trajsAndPwpAreInCollision(dynTrajCompiled traj, PieceWisePol pwp_optimized, double t_init, double t_end)
{
  int samples_per_traj = 10;
  double inc = (t_end - t_init) / samples_per_traj;
  for (int i = 0; i < samples_per_traj; i++)
  {
    t_ = t_init + i * inc;

    Eigen::Vector3d pos_1;
    pos_1 << traj.function[0].value(),  ////////////////////
        traj.function[1].value(),       ////////////////
        traj.function[2].value();       /////////////////

    Eigen::Vector3d pos_2 = pwp_optimized.eval(t_);

    double side_box_drone = (2 * par_.drone_radius);

    Eigen::Vector3d positive_half_diagonal1;
    positive_half_diagonal1 << traj.bbox[0] / 2.0, traj.bbox[1] / 2.0, traj.bbox[2] / 2.0;
    Eigen::Vector3d min1 = pos_1 - positive_half_diagonal1;
    Eigen::Vector3d max1 = pos_1 + positive_half_diagonal1;

    Eigen::Vector3d positive_half_diagonal2;
    positive_half_diagonal2 << side_box_drone / 2.0, side_box_drone / 2.0, side_box_drone / 2.0;
    Eigen::Vector3d min2 = pos_2 - positive_half_diagonal2;
    Eigen::Vector3d max2 = pos_2 + positive_half_diagonal2;

    // Now check if the two bounding boxes overlap
    // https://stackoverflow.com/questions/20925818/algorithm-to-check-if-two-boxes-overlap
    if (min1.x() <= max2.x() && min2.x() <= max1.x() &&  /////////////////////
        min1.y() <= max2.y() && min2.y() <= max1.y() &&  /////////////////////
        min1.z() <= max2.z() && min2.z() <= max1.z())
    {
      // std::cout << bold << blue << "These two bounding boxes overlap" << reset << std::endl;
      // std::cout << "1 pos and diagonal:" << std::endl;
      // std::cout << bold << blue << pos_1.transpose() << reset << std::endl;
      // std::cout << bold << blue << positive_half_diagonal1.transpose() << reset << std::endl;
      // std::cout << "2 pos and diagonal:" << std::endl;
      // std::cout << bold << blue << pos_2.transpose() << reset << std::endl;
      // std::cout << bold << blue << positive_half_diagonal2.transpose() << reset << std::endl;
      return true;  // the two bounding boxes overlap
    }
  }

  return false;
}
// Checks that I have not received new trajectories that affect me while doing the optimization
bool Faster::safetyCheckAfterOpt(PieceWisePol pwp_optimized)
{
  started_check_ = true;

  mtx_trajs_.lock();
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

  mtx_trajs_.unlock();
  return result;
  // traj_compiled.time_received = ros::Time::now().toSec();
}

bool Faster::replan(faster_types::Edges& edges_obstacles_out, std::vector<state>& X_safe_out,
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
  mtx_trajs_.lock();
  std::cout << bold << blue << "Trajectories in the local map: " << reset << std::endl;

  std::vector<double> all_ids;
  /*  traj_compiled.id = traj.id;
    trajs_.push_back(traj_compiled);*/
  int tmp_index_traj = 0;
  for (auto traj : trajs_)
  {
    std::cout << traj.id << ", ";
    // double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

    all_ids.push_back(traj.id);
    // all_ids = all_ids + " " + std::to_string(traj.id);

    // t_ = time_now;

    // Eigen::Vector3d center_obs;
    // center_obs << trajs_[tmp_index_traj].function[0].value(),  ////////////////////
    //     trajs_[tmp_index_traj].function[1].value(),            ////////////////
    //     trajs_[tmp_index_traj].function[2].value();            /////////////////

    // std::cout << traj.id << ", which is in " << center_obs.transpose() << std::endl;

    // tmp_index_traj = tmp_index_traj + 1;
  }

  sort(all_ids.begin(), all_ids.end());

  if (all_ids.size() >= 1)
  {
    std::ostringstream oss;

    if (!all_ids.empty())
    {
      // Convert all but the last element to avoid a trailing ","
      std::copy(all_ids.begin(), all_ids.end() - 1, std::ostream_iterator<double>(oss, ","));

      // Now add the last element with no delimiter
      oss << all_ids.back();
    }

    ROS_INFO_STREAM("Trajs used: " << oss.str());
  }
  else
  {
    ROS_INFO_STREAM("Trajs used: - ");
  }

  std::cout << std::endl;
  mtx_trajs_.unlock();

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Select state A /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  state A;
  int k_safe, k_end_whole, k_whole;

  // If k_end_whole=0, then A = plan_.back() = plan_[plan_.size() - 1]
  k_end_whole = std::max((int)(plan_.size() - deltaT_), 0);
  k_whole = plan_.size() - 1 - k_end_whole;
  A = plan_.get(k_whole);

  /*  if (k_end_whole == 0)
    {
      exists_previous_pwp_ = false;
    }*/

  /////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get global plan /////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  std::vector<Eigen::Vector3d> global_plan;
  global_plan.push_back(A.pos);
  global_plan.push_back(G.pos);

  //////////////////////////////////////////////////////////////////////////
  ///////////////////////// Get point E ////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////

  double ra = std::min((dist_to_goal - 0.001), par_.Ra);  // radius of the sphere S
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

  int samples_per_interval = par_.samples_per_interval;

  state initial = A;
  state final = E;

  // std::cout << "Initial.pos= " << initial.pos << std::endl;
  // std::cout << "Final.pos= " << final.pos << std::endl;
  // std::cout << "norm= " << (initial.pos - final.pos).norm() << std::endl;

  ////////////////////////////////

  par_snlopt par_for_solver;

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

  snlopt_ = new SolverNlopt(par_for_solver);

  ///////////////////////////////

  double runtime_snlopt;

  if (k_whole != 0)
  {
    runtime_snlopt = std::min(k_whole * par_.dc, par_.upper_bound_runtime_snlopt);
  }
  else
  {
    runtime_snlopt = std::min(1.0,
                              par_.upper_bound_runtime_snlopt);  // I'm stopped at the end of the trajectory --> take my
                                                                 // time to replan
  }
  snlopt_->setMaxRuntimeKappaAndMu(runtime_snlopt, par_.kappa, par_.mu);

  //////////////////////
  double time_now = ros::Time::now().toSec();  // TODO this ros dependency shouldn't be here

  double t_init = k_whole * par_.dc + time_now;
  double t_final = t_init + (initial.pos - final.pos).array().abs().maxCoeff() /
                                (par_.factor_v_max * par_.v_max);  // time to execute the optimized path

  bool correctInitialCond =
      snlopt_->setInitStateFinalStateInitTFinalT(initial, final, t_init,
                                                 t_final);  // note that here t_final may have been updated

  if (correctInitialCond == false)
  {
    std::cout << bold << red << "The solver cannot guarantee feasibility for v1" << std::endl;
    return false;
  }

  ////////////////

  mtx_trajs_.lock();

  time_init_opt_ = ros::Time::now().toSec();

  removeTrajsThatWillNotAffectMe(A, t_init, t_final);
  ConvexHullsOfCurves hulls = convexHullsOfCurves(t_init, t_final);
  ConvexHullsOfCurves_Std hulls_std = vectorGCALPol2vectorStdEigen(hulls);
  // poly_safe_out = vectorGCALPol2vectorJPSPol(hulls);
  edges_obstacles_out = vectorGCALPol2edges(hulls);
  mtx_trajs_.unlock();

  snlopt_->setHulls(hulls_std);

  //////////////////////
  std::cout << on_cyan << bold << "Solved so far" << solutions_found_ << "/" << total_replannings_ << reset
            << std::endl;

  std::cout << "[FA] Calling NL" << std::endl;

  bool result = snlopt_->optimize();

  num_of_LPs_run += snlopt_->getNumOfLPsRun();
  num_of_QCQPs_run += snlopt_->getNumOfQCQPsRun();

  snlopt_->getGuessForPlanes(planes_guesses);

  total_replannings_++;
  if (result == false)
  {
    int states_last_replan = ceil(replanCB_t.ElapsedMs() / (par_.dc * 1000));  // Number of states that
                                                                               // would have been needed for
                                                                               // the last replan
    deltaT_ = std::max(par_.alpha * states_last_replan, 1.0);
    deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
    return false;
  }

  solutions_found_++;

  av_improvement_nlopt_ = ((solutions_found_ - 1) * av_improvement_nlopt_ + snlopt_->improvement_) / solutions_found_;

  std::cout << blue << "Average improvement so far" << std::setprecision(5) << av_improvement_nlopt_ << reset
            << std::endl;

  PieceWisePol pwp_now;
  snlopt_->getSolution(pwp_now);

  MyTimer check_t(true);
  bool is_safe_after_opt = safetyCheckAfterOpt(pwp_now);
  std::cout << bold << "Check Timer=" << check_t << std::endl;

  if (is_safe_after_opt == false)
  {
    std::cout << bold << red << "safetyCheckAfterOpt is not satisfied!" << reset << std::endl;
    return false;
  }

  /////// END OF DEBUGGING

  std::vector<state> dummy_vector;
  dummy_vector.push_back(A);
  // sg_whole_.X_temp_ = dummy_vector;

  M_ = G_term;

  k_safe = 0;

  bool result_appending = appendToPlan(k_end_whole, dummy_vector, k_safe, snlopt_->X_temp_);

  // std::cout << "After appendToPlan, plan_= " << std::endl;
  // plan_.print();

  if (result_appending != true)
  {
    return false;
  }

  // std::cout << std::setprecision(30) << "pwp_now.times[0]=" << pwp_now.times[0] << std::endl;
  // std::cout << "t_min= " << t_min << std::endl;

  if (exists_previous_pwp_ == true)
  {
    // std::cout << "k_whole= " << k_whole << std::endl;
    // std::cout << "k_end_whole= " << k_end_whole << std::endl;
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
  deltaT_ = std::max(par_.alpha * states_last_replan, 1.0);
  // std::max(par_.alpha * states_last_replan,(double)par_.min_states_deltaT);  // Delta_t
  mtx_offsets.unlock();

  planner_initialized_ = true;

  return true;
}

void Faster::resetInitialization()
{
  planner_initialized_ = false;
  state_initialized_ = false;

  terminal_goal_initialized_ = false;
}

bool Faster::appendToPlan(int k_end_whole, const std::vector<state>& whole, int k_safe, const std::vector<state>& safe)
{
  mtx_plan_.lock();

  // std::cout << "Erasing" << std::endl;
  bool output;
  int plan_size = plan_.size();
  //  std::cout << "plan_.size()= " << plan_.size() << std::endl;
  //  std::cout << "plan_size - k_end_whole = " << plan_size - k_end_whole << std::endl;
  if ((plan_size - 1 - k_end_whole) < 0)
  {
    std::cout << bold << red << "Already published the point A" << reset << std::endl;
    output = false;
  }
  else
  {
    // std::cout << "k_end_whole= " << k_end_whole << std::endl;
    // std::cout << "k_safe = " << k_safe << std::endl;

    plan_.erase(plan_.end() - k_end_whole - 1, plan_.end());

    //    std::cout << "Erased" << std::endl;
    //    std::cout << "whole.size() = " << whole.size() << std::endl;
    //    std::cout << "safe.size() = " << safe.size() << std::endl;
    for (int i = 0; i <= k_safe; i++)
    {
      plan_.push_back(whole[i]);
    }

    for (int i = 1; i < safe.size(); i++)
    {
      plan_.push_back(safe[i]);
    }
    // std::cout << "Pushed everything back" << std::endl;

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
      // std::cout << bold << green << "[Faster] state_.yaw=" << state_.yaw << reset << std::endl;
      diff = desired_yaw - state_.yaw;
      // std::cout << bold << green << "[Faster] desired_yaw=" << desired_yaw << reset << std::endl;
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
  /*  if (initializedAllExceptPlanner() == false)
    {
      std::cout << "Not publishing new goal!!" << std::endl;
      return false;
    }*/
  if (initializedStateAndTermGoal() == false)
  {
    // std::cout << "Not publishing new goal!!" << std::endl;
    return false;
  }

  mtx_goals.lock();
  mtx_plan_.lock();

  next_goal.setZero();
  next_goal = plan_.front();

  // std::cout << bold << green << "[Faster] next_goal.yaw=" << next_goal.yaw << reset << std::endl;

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