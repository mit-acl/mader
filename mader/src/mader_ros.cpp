/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "mader_ros.hpp"

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>

#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros
#include <decomp_geometry/polyhedron.h>       //For hyperplane
#include <Eigen/Geometry>

#include <jsk_rviz_plugins/OverlayText.h>

#include <random>

typedef MADER_timers::Timer MyTimer;

// this object is created in the mader_ros_node
MaderRos::MaderRos(ros::NodeHandle nh1, ros::NodeHandle nh2, ros::NodeHandle nh3, ros::NodeHandle nh4,
                   ros::NodeHandle nh5)
  : nh1_(nh1), nh2_(nh2), nh3_(nh3), nh4_(nh4), nh5_(nh5)
{
  bool sim;  // if this is simulation or hardware. Used to check if we need SQ or HX
  mu::safeGetParam(nh1_, "sim", sim);
  sim_ = sim;
  mu::safeGetParam(nh1_, "is_delaycheck", is_delaycheck_);
  int max_agent_number;
  mu::safeGetParam(nh1_, "max_agent_number", max_agent_number);
  bool is_take_off;
  mu::safeGetParam(nh1_, "is_take_off", is_take_off);

  mu::safeGetParam(nh1_, "delay_check", par_.delay_check);
  delay_check_ = par_.delay_check;
  mu::safeGetParam(nh1_, "simulated_comm_delay", simulated_comm_delay_);

  mu::safeGetParam(nh1_, "comm_delay_param", par_.comm_delay_param);

  mu::safeGetParam(nh1_, "use_ff", par_.use_ff);
  mu::safeGetParam(nh1_, "visual", par_.visual);
  mu::safeGetParam(nh1_, "color_type", par_.color_type);
  mu::safeGetParam(nh1_, "n_agents", par_.n_agents);

  mu::safeGetParam(nh1_, "dc", par_.dc);
  mu::safeGetParam(nh1_, "goal_radius", par_.goal_radius);
  mu::safeGetParam(nh1_, "drone_radius", par_.drone_radius);

  // since drone_bbox is a vector, you need a workaround like this
  std::vector<double> drone_bbox_tmp;
  mu::safeGetParam(nh1_, "drone_bbox", drone_bbox_tmp);
  par_.drone_bbox << drone_bbox_tmp[0], drone_bbox_tmp[1], drone_bbox_tmp[2];

  mu::safeGetParam(nh1_, "Ra", par_.Ra);

  mu::safeGetParam(nh1_, "w_max", par_.w_max);
  mu::safeGetParam(nh1_, "alpha_filter_dyaw", par_.alpha_filter_dyaw);

  mu::safeGetParam(nh1_, "x_min", par_.x_min);
  mu::safeGetParam(nh1_, "x_max", par_.x_max);

  mu::safeGetParam(nh1_, "y_min", par_.y_min);
  mu::safeGetParam(nh1_, "y_max", par_.y_max);

  mu::safeGetParam(nh1_, "z_min", par_.z_min);
  mu::safeGetParam(nh1_, "z_max", par_.z_max);

  std::vector<double> v_max_tmp;
  std::vector<double> a_max_tmp;
  std::vector<double> j_max_tmp;

  mu::safeGetParam(nh1_, "v_max", v_max_tmp);
  mu::safeGetParam(nh1_, "a_max", a_max_tmp);
  mu::safeGetParam(nh1_, "j_max", j_max_tmp);

  par_.v_max << v_max_tmp[0], v_max_tmp[1], v_max_tmp[2];
  par_.a_max << a_max_tmp[0], a_max_tmp[1], a_max_tmp[2];
  par_.j_max << j_max_tmp[0], j_max_tmp[1], j_max_tmp[2];

  mu::safeGetParam(nh1_, "factor_alloc", par_.factor_alloc);
  mu::safeGetParam(nh1_, "factor_alloc_close", par_.factor_alloc_close);
  mu::safeGetParam(nh1_, "dist_factor_alloc_close", par_.dist_factor_alloc_close);

  mu::safeGetParam(nh1_, "factor_alpha", par_.factor_alpha);

  mu::safeGetParam(nh1_, "num_pol", par_.num_pol);
  mu::safeGetParam(nh1_, "deg_pol", par_.deg_pol);
  mu::safeGetParam(nh1_, "weight", par_.weight);
  mu::safeGetParam(nh1_, "epsilon_tol_constraints", par_.epsilon_tol_constraints);
  mu::safeGetParam(nh1_, "xtol_rel", par_.xtol_rel);
  mu::safeGetParam(nh1_, "ftol_rel", par_.ftol_rel);
  mu::safeGetParam(nh1_, "solver", par_.solver);

  mu::safeGetParam(nh1_, "upper_bound_runtime_snlopt", par_.upper_bound_runtime_snlopt);
  mu::safeGetParam(nh1_, "lower_bound_runtime_snlopt", par_.lower_bound_runtime_snlopt);
  mu::safeGetParam(nh1_, "kappa", par_.kappa);
  mu::safeGetParam(nh1_, "mu", par_.mu);

  mu::safeGetParam(nh1_, "a_star_samp_x", par_.a_star_samp_x);
  mu::safeGetParam(nh1_, "a_star_samp_y", par_.a_star_samp_y);
  mu::safeGetParam(nh1_, "a_star_samp_z", par_.a_star_samp_z);
  mu::safeGetParam(nh1_, "a_star_fraction_voxel_size", par_.a_star_fraction_voxel_size);
  mu::safeGetParam(nh1_, "allow_infeasible_guess", par_.allow_infeasible_guess);

  mu::safeGetParam(nh1_, "a_star_bias", par_.a_star_bias);

  mu::safeGetParam(nh1_, "basis", par_.basis);

  mu::safeGetParam(nh1_, "res_plot_traj", par_.res_plot_traj);

  mu::safeGetParam(nh1_, "alpha", par_.alpha);
  mu::safeGetParam(nh1_, "beta", par_.beta);
  mu::safeGetParam(nh1_, "gamma", par_.gamma);

  mu::safeGetParam(nh1_, "alpha_shrink", par_.alpha_shrink);

  mu::safeGetParam(nh1_, "fov_horiz_deg", par_.fov_horiz_deg);
  mu::safeGetParam(nh1_, "fov_vert_deg", par_.fov_vert_deg);
  mu::safeGetParam(nh1_, "fov_depth", par_.fov_depth);

  std::cout << "Parameters obtained" << std::endl;

  // CHECK parameters
  std::cout << bold << "Parameters obtained, checking them..." << reset << std::endl;

  verify((par_.gamma > 0), "Not satisfied: (par_.gamma > 0)");
  verify((par_.beta >= 0 || par_.alpha >= 0), "Not satisfied: (par_.beta >= 0 || par_.alpha >= 0)");
  // verify((par_.a_max.z() <= 9.81), "par_.a_max.z() >= 9.81, the drone will flip");
  verify((par_.factor_alloc >= 1.0), "Not satisfied: (par_.factor_alloc >= 1.0)");
  verify((par_.kappa > 0 || par_.mu > 0), "Not satisfied: (par_.kappa > 0 || par_.mu > 0)");
  verify(((par_.kappa + par_.mu) <= 1), "Not satisfied: ((par_.kappa + par_.mu) <= 1)");
  verify((par_.a_star_fraction_voxel_size >= 0.0 || par_.a_star_fraction_voxel_size <= 1.0), "a_star_fraction_voxel_"
                                                                                             "size must be in [0,1]");

  verify((par_.epsilon_tol_constraints < 0.02), "The tolerance on the constraints is too big -->  there will be "
                                                "jumps in accel/vel");
  std::cout << bold << "Parameters checked" << reset << std::endl;
  /////////////////////

  mader_ptr_ = std::unique_ptr<Mader>(new Mader(par_));

  // get my namespace
  std::string myns = ros::this_node::getNamespace();

  // Publishers
  pub_goal_ = nh1_.advertise<snapstack_msgs::Goal>("goal", 1);
  pub_setpoint_ = nh1_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_point_G_ = nh1_.advertise<geometry_msgs::PointStamped>("point_G", 1);
  pub_point_G_term_ = nh1_.advertise<geometry_msgs::PointStamped>("point_G_term", 1);
  pub_point_A_ = nh1_.advertise<visualization_msgs::Marker>("point_A", 1);
  pub_actual_traj_ = nh1_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  poly_safe_pub_ = nh1_.advertise<decomp_ros_msgs::PolyhedronArray>("poly_safe", 1, true);
  pub_text_ = nh1_.advertise<jsk_rviz_plugins::OverlayText>("text", 1);
  pub_traj_safe_colored_ = nh1_.advertise<visualization_msgs::MarkerArray>("traj_safe_colored", 1);
  pub_traj_safe_colored_bef_commit_ =
      nh1_.advertise<visualization_msgs::MarkerArray>("traj_safe_colored_bef_commit", 1);
  pub_text_ = nh1_.advertise<jsk_rviz_plugins::OverlayText>("text", 1);
  pub_fov_ = nh1_.advertise<visualization_msgs::Marker>("fov", 1);
  pub_obstacles_ = nh1_.advertise<visualization_msgs::Marker>("obstacles", 1);
  pub_traj_ = nh1_.advertise<mader_msgs::DynTraj>("trajs", 1, true);  // The last boolean is latched or not
  pub_comm_delay_ = nh1_.advertise<mader_msgs::CommDelay>("comm_delay", 1);
  pub_missed_msgs_cnt_ = nh1_.advertise<mader_msgs::MissedMsgsCnt>("missed_msgs_cnt", 1);

  // Subscribers
  sub_term_goal_ = nh1_.subscribe("term_goal", 1, &MaderRos::terminalGoalCB, this);
  // sub_mode_ = nh1_.subscribe("mode", 1, &MaderRos::modeCB, this);
  sub_whoplans_ = nh1_.subscribe("who_plans", 1, &MaderRos::whoPlansCB, this);
  sub_state_ = nh1_.subscribe("state", 1, &MaderRos::stateCB, this);

  // Subscribers for each agents

  // if it's simulation
  // TODO:: make more general/robust
  if (sim)
  {
    for (int i = 1; i <= max_agent_number; ++i)
    {
      std::string agent;
      if (i <= 9)
      {
        agent = "SQ0" + std::to_string(i) + "s";
      }
      else
      {
        agent = "SQ" + std::to_string(i) + "s";
      }
      if (myns != agent)
      {  // if my namespace is the same as the agent, then it's you
        sub_traj_.push_back(
            nh5_.subscribe("/" + agent + "/mader/trajs", 20, &MaderRos::trajCB, this));  // The number is the queue size
      }
    }
  }
  else
  {  // if it's hardware
    for (int i = 1; i <= max_agent_number; ++i)
    {
      std::string agent;
      if (i <= 9)
      {
        agent = "NX0" + std::to_string(i);
      }
      else
      {
        agent = "NX" + std::to_string(i);
      }
      if (myns != agent)
      {  // if my namespace is the same as the agent, then it's you
        sub_traj_.push_back(
            nh5_.subscribe("/" + agent + "/mader/trajs", 20, &MaderRos::trajCB, this));  // The number is the queue size
      }
    }
  }

  // Timers
  pubCBTimer_ = nh2_.createTimer(ros::Duration(par_.dc), &MaderRos::pubCB, this);
  replanCBTimer_ = nh3_.createTimer(ros::Duration(par_.dc), &MaderRos::replanCB, this);

  // For now stop all these subscribers/timers until we receive GO
  // // sub_state_.shutdown();
  // pubCBTimer_.stop();
  // replanCBTimer_.stop();
  sub_state_.shutdown();
  sub_term_goal_.shutdown();
  pubCBTimer_.stop();
  replanCBTimer_.stop();

  if (!is_take_off)
  {                                                                                    // no need to take off
    sub_term_goal_ = nh1_.subscribe("term_goal", 1, &MaderRos::terminalGoalCB, this);  // TODO: duplicated from above
    sub_state_ = nh1_.subscribe("state", 1, &MaderRos::stateCB, this);                 // TODO: duplicated from above
    pubCBTimer_.start();
    replanCBTimer_.start();
    is_mader_running_ = true;
    std::cout << on_blue << "**************MADER STARTED" << reset << std::endl;
  }

  // Rviz_Visual_Tools
  visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
  visual_tools_->loadMarkerPub();  // create publisher before waitin
  ros::Duration(0.5).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // Markers
  setpoint_ = mu::getMarkerSphere(0.35, mu::orange_trans);
  E_ = mu::getMarkerSphere(0.35, mu::red_normal);
  A_ = mu::getMarkerSphere(0.35, mu::red_normal);

  // If you want another thread for the replanCB: replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc),
  // &MaderRos::replanCB, this);

  name_drone_ = ros::this_node::getNamespace();  // Return also the slashes (2 in Kinetic, 1 in Melodic)
  name_drone_.erase(std::remove(name_drone_.begin(), name_drone_.end(), '/'), name_drone_.end());  // Remove the slashes

  std::string id = name_drone_;
  id.erase(0, 2);  // Erase SQ or HX i.e. SQ12 --> 12  HX8621 --> 8621 # TODO Hard-coded for this this convention
  id_ = std::stoi(id);

  // mtx_mader_ptr_.lock();
  mader_ptr_->getID(id_);
  // mtx_mader_ptr_.unlock();

  timer_stop_.Reset();

  clearMarkerActualTraj();

  ////// to avoid having to click on the GUI (TODO)
  // mader_msgs::Mode tmp;
  // tmp.mode = 1;
  // modeCB(tmp);
  //// ros::Duration(1.0).sleep();  // TODO
  //// bool success_service_call = system("rosservice call /change_mode 'mode: 1'");
  ////

  ROS_INFO("Planner initialized");
}

MaderRos::~MaderRos()
{
  sub_state_.shutdown();
  pubCBTimer_.stop();
  replanCBTimer_.stop();
}

void MaderRos::pubObstacles(mt::Edges edges_obstacles)
{
  pub_obstacles_.publish(mu::edges2Marker(edges_obstacles, mu::color(mu::red_normal)));

  return;
}

void MaderRos::trajCB(const mader_msgs::DynTraj& msg)
{
  // std::cout << "TrajCB is called" << std::endl;
  // test pop_up scheme (SQ06 doesn't listen /trajs)
  // if (id_ == 6){
  //   return;
  // }

  if (msg.id == id_)
  {  // This is my own trajectory
    return;
  }

  Eigen::Vector3d W_pos(msg.pos.x, msg.pos.y, msg.pos.z);  // position in world frame
  double dist = (state_.pos - W_pos).norm();

  bool can_use_its_info;

  // if (par_.impose_fov == false)
  // {  // same as 360 deg of FOV
  //   can_use_its_info = dist <= par_.R_local_map;
  // }
  // else
  // {  // impose_fov==true

  //   Eigen::Vector3d B_pos = W_T_B_.inverse() * W_pos;  // position of the obstacle in body frame
  //   // check if it's inside the field of view.
  //   can_use_its_info =
  //       B_pos.x() < par_.fov_depth &&                                                       //////////////////////
  //       fabs(atan2(B_pos.y(), B_pos.x())) < ((par_.fov_horiz_deg * M_PI / 180.0) / 2.0) &&  //////////////////////
  //       fabs(atan2(B_pos.z(), B_pos.x())) < ((par_.fov_vert_deg * M_PI / 180.0) / 2.0);     ///////////////////////

  //   // std::cout << "inFOV= " << can_use_its_info << std::endl;
  // }

  can_use_its_info = (dist <= 4 * par_.Ra);  // See explanation of 4*Ra in Mader::updateTrajObstacles

  if (can_use_its_info == false)
  {
    return;
  }

  mt::dynTraj tmp;
  tmp.function.push_back(msg.function[0]);
  tmp.function.push_back(msg.function[1]);
  tmp.function.push_back(msg.function[2]);

  tmp.is_committed = msg.is_committed;

  tmp.bbox << msg.bbox[0], msg.bbox[1], msg.bbox[2];

  tmp.id = msg.id;

  tmp.is_agent = msg.is_agent;

  // tmp.time_sent = msg.time_sent;

  tmp.time_created = msg.time_created;

  if (msg.is_agent)
  {
    tmp.pwp = mu::pwpMsg2Pwp(msg.pwp);
  }

  tmp.time_received = ros::Time::now().toSec();

  // double comm_delay = tmp.time_received - tmp.time_created;
  // if (comm_delay > delay_check_ - simulated_comm_delay_){
  //   std::cout << "comm delay is " << comm_delay << " [s]" << std::endl;
  //   std::cout << "comm delay is huge!!!!" << std::endl;
  // }

  if (sim_ && is_term_goal_initialized_)
  {
    //****** Communication delay introduced in the simulation
    // save all the trajectories into alltrajs_ and create a timer corresponding to that
    // std::cout << "bef alltrajs_ and alltrajsTimers_ are locked() in TrajCB" << std::endl;
    // mtx_alltrajs_.lock();
    // mtx_alltrajsTimers_.lock();
    // std::cout << "aft alltrajs_ and alltrajsTimers_ are locked() in TrajCB" << std::endl;

    alltrajs_.push_back(tmp);
    ros::Timer alltrajs_timer =
        nh4_.createTimer(ros::Duration(simulated_comm_delay_), &MaderRos::allTrajsTimerCB, this, true);
    alltrajsTimers_.push_back(alltrajs_timer);

    // std::cout << "bef alltrajs_ and alltrajsTimers_ are unlocked() in TrajCB" << std::endl;
    // mtx_alltrajs_.unlock();
    // mtx_alltrajsTimers_.unlock();
    // std::cout << "bef alltrajs_ and alltrajsTimers_ are unlocked() in TrajCB" << std::endl;
  }
  else
  {
    // mtx_mader_ptr_.lock();
    mader_ptr_->updateTrajObstacles(tmp);
    // mtx_mader_ptr_.unlock();
  }
}

void MaderRos::allTrajsTimerCB(const ros::TimerEvent& e)
{
  // std::cout << "delayed enough" << std::endl;

  // just measure how long it takes to check collisions

  // if (is_in_DC_)
  // {
  //   // MyTimer each_collision_check_t(true);
  //   mader_ptr_->updateTrajObstacles(alltrajs_[0], pwp_new_, is_in_DC_, headsup_time_, delay_check_result_);
  //   // std::cout << "each collision check takes " << each_collision_check_t.ElapsedUs() << "us" << std::endl;
  // } else {
  //   mader_ptr_->updateTrajObstacles(alltrajs_[0], pwp_new_, is_in_DC_, headsup_time_, delay_check_result_);
  // }
  // std::cout << "bef alltrajs_ and alltrajsTimers_ are locked() in allTrajsTimerCB" << std::endl;
  // std::cout << "aft alltrajs_ and alltrajsTimers_ are locked() in allTrajsTimerCB" << std::endl;

  // mtx_alltrajs_.lock();
  // mtx_alltrajsTimers_.lock();

  mt::dynTraj tmp = alltrajs_[0];

  alltrajs_.pop_front();
  alltrajsTimers_.pop_front();

  // mtx_alltrajs_.unlock();
  // mtx_alltrajsTimers_.unlock();
  // mtx_mader_ptr_.lock();
  mader_ptr_->updateTrajObstacles_with_delaycheck(tmp);
  // mtx_mader_ptr_.unlock();

  double time_now = ros::Time::now().toSec();
  double supposedly_simulated_comm_delay = time_now - tmp.time_created;

  // supposedly_simulated_time_delay should be simulated_comm_delay_
  if (supposedly_simulated_comm_delay > delay_check_)
  {
    // std::cout << "supposedly_simulated_comm_delay is too big " << supposedly_simulated_comm_delay << " s"
    // << std::endl;
    missed_msgs_cnt_++;
  }

  msgs_cnt_++;

  mader_msgs::CommDelay msg;
  msg.comm_delay = supposedly_simulated_comm_delay;
  pub_comm_delay_.publish(msg);


  // std::cout << "bef alltrajs_ and alltrajsTimers_ are unlocked() in allTrajsTimerCB" << std::endl;
  // std::cout << "aft alltrajs_ and alltrajsTimers_ are unlocked() in allTrajsTimerCB" << std::endl;
}

// This trajectory contains all the future trajectory (current_pos --> A --> final_point_of_traj), because it's the
// composition of pwp
void MaderRos::publishOwnTraj(const mt::PieceWisePol& pwp, const bool& is_committed, const double& headsup_time)
{
  std::vector<std::string> s;  // mu::pieceWisePol2String(pwp); The rest of the agents will use the pwp field, not the
                               // string
  s.push_back("");
  s.push_back("");
  s.push_back("");

  mader_msgs::DynTraj msg;
  msg.function = s;
  msg.bbox.push_back(par_.drone_bbox[0]);
  msg.bbox.push_back(par_.drone_bbox[1]);
  msg.bbox.push_back(par_.drone_bbox[2]);
  // msg.bbox.push_back(2 * par_.drone_radius);
  // msg.bbox.push_back(2 * par_.drone_radius);
  // msg.bbox.push_back(2 * par_.drone_radius);
  msg.pos.x = state_.pos.x();
  msg.pos.y = state_.pos.y();
  msg.pos.z = state_.pos.z();
  msg.id = id_;

  msg.is_agent = true;

  msg.pwp = mu::pwp2PwpMsg(pwp);

  msg.time_created = ros::Time::now().toSec();

  // msg.time_sent = ros::Time::now().toSec();  // to measure comm delay between agents

  msg.is_committed = is_committed;

  // std::cout<<"msg.pwp.times[0]= "<<msg.pwp.times[0]

  pub_traj_.publish(msg);
}

void MaderRos::publishOwnTraj(const mt::PieceWisePol& pwp, const bool& is_committed)
{
  std::vector<std::string> s;  // mu::pieceWisePol2String(pwp); The rest of the agents will use the pwp field, not the
                               // string
  s.push_back("");
  s.push_back("");
  s.push_back("");

  mader_msgs::DynTraj msg;
  msg.function = s;
  msg.bbox.push_back(par_.drone_bbox[0]);
  msg.bbox.push_back(par_.drone_bbox[1]);
  msg.bbox.push_back(par_.drone_bbox[2]);
  // msg.bbox.push_back(2 * par_.drone_radius);
  // msg.bbox.push_back(2 * par_.drone_radius);
  // msg.bbox.push_back(2 * par_.drone_radius);
  msg.pos.x = state_.pos.x();
  msg.pos.y = state_.pos.y();
  msg.pos.z = state_.pos.z();
  msg.id = id_;

  msg.is_agent = true;

  msg.pwp = mu::pwp2PwpMsg(pwp);

  msg.time_created = ros::Time::now().toSec();

  msg.is_committed = is_committed;

  // std::cout<<"msg.pwp.times[0]= "<<msg.pwp.times[0]

  pub_traj_.publish(msg);
}

void MaderRos::replanCB(const ros::TimerEvent& e)
{
  if (ros::ok() && published_initial_position_ == true && is_mader_running_)
  {
    replanCBTimer_.stop();  // to avoid blockage

    // introduce random wait time in the beginning
    if (!is_replanCB_called_)
    {
      // to avoid initial path search congestions add some random sleep here
      std::random_device rd;
      std::default_random_engine eng(rd());
      std::uniform_real_distribution<float> distr(0, 1);  // sleep between 0 and 1 sec
      ros::Duration(distr(eng)).sleep();
      is_replanCB_called_ = true;
    }

    // mtx_mader_ptr_.lock();
    if (mader_ptr_->isGoalSeen())
    {
      std::cout << "goal is reached so no need to replan" << std::endl;
      is_mader_running_ = false;
      mader_msgs::MissedMsgsCnt msg;
      msg.missed_msgs_cnt = missed_msgs_cnt_;
      msg.msgs_cnt = msgs_cnt_;
      pub_missed_msgs_cnt_.publish(msg);
      // mtx_mader_ptr_.unlock();
      return;
    }
    // mtx_mader_ptr_.unlock();

    // initialization
    mt::Edges edges_obstacles;
    std::vector<mt::state> traj_plan;
    std::vector<Hyperplane3D> planes;

    // replan
    bool replanned = false;

    if (is_delaycheck_)
    {
      // mtx_mader_ptr_.lock();
      replanned = mader_ptr_->replan_with_delaycheck(edges_obstacles, traj_plan, planes, num_of_LPs_run_,
                                                     num_of_QCQPs_run_, pwp_now_, headsup_time_);
      // mtx_mader_ptr_.unlock();

      if (par_.visual)
      {
        // Delete markers to publish stuff
        visual_tools_->deleteAllMarkers();
        visual_tools_->enableBatchPublishing();
        if (edges_obstacles.size() > 0)
        {
          pubObstacles(edges_obstacles);
        }
      }

      if (replanned)
      {
        // is_in_DC_ = true;

        // let others know my new trajectory
        publishOwnTraj(pwp_now_, false, headsup_time_);

        // visualization
        if (par_.visual)
        {
          // Delete markers to publish stuff
          // visual_tools_->deleteAllMarkers();
          // visual_tools_->enableBatchPublishing();
          // if (edges_obstacles.size() > 0){pubObstacles(edges_obstacles);}
          pubTraj(traj_plan, false);
        }

        // std::cout << "headsup" << std::endl;
        // for (auto state : traj_plan) {state.print();}

        // delay check *******************************************************
        // start
        MyTimer delay_check_t(true);
        while (delay_check_t.ElapsedMs() / 1000.0 < delay_check_)
        {
          // wait while trajCB() is checking new trajs
          // TODO make this as a timer so that i can move onto the next optimization
          // std::cout << "waiting in DC" << std::endl;
          // mtx_mader_ptr_.lock();
          delay_check_result_ = mader_ptr_->delayCheck(pwp_now_, headsup_time_);
          // mtx_mader_ptr_.unlock();
          ros::Duration(0.01).sleep();
          if (delay_check_result_ == false)
          {
            break;
          }
        }
        delay_check_result_ = mader_ptr_->delayCheck(pwp_now_, headsup_time_);
        // ros::Duration(delay_check_).sleep();
        // is_in_DC_ = false;
        // end of delay check *******************************************************

        // std::cout << "after delay check" << std::endl;

        if (delay_check_result_)
        {
          // execute the new trajectory
          // if it's the first time to add traj to plan_ then don't execute it. you don't have a back-up traj
          // (pwp_last_) if (!is_add_plan_initialized_){
          //   pwp_last_ = pwp_now_;
          //   is_add_plan_initialized_ = true;
          //   return;
          // }

          // mtx_mader_ptr_.lock();
          bool successful_to_add_to_plan = mader_ptr_->addTrajToPlan_with_delaycheck(pwp_now_);
          // mtx_mader_ptr_.unlock();
          if (successful_to_add_to_plan)
          {
            // successful

            publishOwnTraj(pwp_now_, true, headsup_time_);
            pwp_last_ = pwp_now_;
            // std::cout << "Success!" << std::endl;
            if (par_.visual)
            {
              // Delete markers to publish stuff
              // visual_tools_->deleteAllMarkers();
              // visual_tools_->enableBatchPublishing();
              // if (edges_obstacles.size() > 0){pubObstacles(edges_obstacles);}
              pubTraj(traj_plan, true);
              // last_traj_plan_ = traj_plan;
              // last_edges_obstacles_ = edges_obstacles;
            }
          }
          else
          {
            // int time_ms = int(ros::Time::now().toSec() * 1000);

            if (timer_stop_.ElapsedMs() > 500.0)
            {
              publishOwnTraj(pwp_last_, true,
                             headsup_time_);  // This is needed because is drone DRONE1 stops, it needs to keep publishing
                                              // his last planned trajectory, so that other drones can avoid it (even if
                                              // DRONE1 was very far from the other drones with it last successfully
                                              // planned a trajectory). Note that these trajectories are time-indexed, and
                                              // the last position is taken if t>times.back(). See eval() function in the
                                              // pwp struct
              timer_stop_.Reset();
            }
            // visualization
            if (par_.visual)
            {
              // Delete markers to publish stuff
              // visual_tools_->deleteAllMarkers();
              // visual_tools_->enableBatchPublishing();
              // if (edges_obstacles.size() > 0){pubObstacles(edges_obstacles);}
              // pubTraj(traj_plan, true);
            }

          }
        }
        else
        {
          // int time_ms = int(ros::Time::now().toSec() * 1000);

          if (timer_stop_.ElapsedMs() > 500.0)
          {
            publishOwnTraj(pwp_last_, true,
                           headsup_time_);  // This is needed because is drone DRONE1 stops, it needs to keep publishing
                                            // his last planned trajectory, so that other drones can avoid it (even if
                                            // DRONE1 was very far from the other drones with it last successfully
                                            // planned a trajectory). Note that these trajectories are time-indexed, and
                                            // the last position is taken if t>times.back(). See eval() function in the
                                            // pwp struct
            timer_stop_.Reset();
          }
          // visualization
          if (par_.visual)
          {
            // Delete markers to publish stuff
            // visual_tools_->deleteAllMarkers();
            // visual_tools_->enableBatchPublishing();
            // if (edges_obstacles.size() > 0){pubObstacles(edges_obstacles);}
            // pubTraj(traj_plan, true);
          }
        }
      }
      else
      {
        // int time_ms = int(ros::Time::now().toSec() * 1000);

        if (timer_stop_.ElapsedMs() > 500.0)
        {
          publishOwnTraj(pwp_last_, true,
                         headsup_time_);  // This is needed because is drone DRONE1 stops, it needs to keep publishing
                                          // his last planned trajectory, so that other drones can avoid it (even if
                                          // DRONE1 was very far from the other drones with it last successfully planned
                                          // a trajectory). Note that these trajectories are time-indexed, and the last
                                          // position is taken if t>times.back(). See eval() function in the pwp struct
          timer_stop_.Reset();
        }
        // visualization
        if (par_.visual)
        {
          // Delete markers to publish stuff
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
          if (edges_obstacles.size() > 0)
          {
            pubObstacles(edges_obstacles);
          }
          // pubTraj(traj_plan, true);
        }
      }
    }
    else
    {
      // // std::cout << "I'm using old mader!!!!!" << std::endl;

      // mtx_mader_ptr_.lock();
      replanned = mader_ptr_->replan(edges_obstacles, traj_plan, planes, num_of_LPs_run_, num_of_QCQPs_run_, pwp_now_);
      // mtx_mader_ptr_.unlock();

      if (par_.visual)
      {
        // Delete markers to publish stuff
        visual_tools_->deleteAllMarkers();
        visual_tools_->enableBatchPublishing();

        // visualization
        if (par_.visual)
        {
          // Delete markers to publish stuff
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
          if (edges_obstacles.size() > 0)
          {
            pubObstacles(edges_obstacles);
          }
          pubTraj(traj_plan, true);
        }

        // publishPlanes(planes);
        // publishText();
      }

      if (replanned)
      {
        publishOwnTraj(pwp_now_, true);
        pwp_last_ = pwp_now_;
      }
      else
      {
        // int time_ms = int(ros::Time::now().toSec() * 1000);

        if (timer_stop_.ElapsedMs() > 500.0)
        {
          publishOwnTraj(pwp_last_,
                         true);  // This is needed because is drone DRONE1 stops, it needs to keep
                                 // publishing his
                                 // last planned trajectory, so that other drones can avoid it (even if DRONE1
                                 // was
                                 // very far from the other drones with it last successfully planned a
                                 // trajectory).
                                 // Note that these trajectories are time-indexed, and the last position is
                                 // taken if
                                 // t>times.back(). See eval() function in the pwp struct
          timer_stop_.Reset();
        }
        // visualization
        if (par_.visual)
        {
          // Delete markers to publish stuff
          visual_tools_->deleteAllMarkers();
          visual_tools_->enableBatchPublishing();
          if (last_edges_obstacles_.size() > 0)
          {
            pubObstacles(last_edges_obstacles_);
          }
          pubTraj(last_traj_plan_, true);
        }
      }
    }

    replanCBTimer_.start();  // to avoid blockage
  }                          // std::cout << "[Callback] Leaving replanCB" << std::endl;

  mt::state G;  // projected goal
  // mtx_mader_ptr_.lock();
  mader_ptr_->getG(G);
  // mtx_mader_ptr_.unlock();
  pubState(G, pub_point_G_);
}

void MaderRos::publishText()
{
  jsk_rviz_plugins::OverlayText text;
  text.width = 600;
  text.height = 133;
  text.left = 1600;
  text.top = 10;
  text.text_size = 17;
  text.line_width = 2;
  text.font = "DejaVu Sans Mono";
  text.text = "Num of LPs run= " + std::to_string(num_of_LPs_run_) + "\n" +  ///////////////////
              "Num of QCQPs run= " + std::to_string(num_of_QCQPs_run_);

  text.fg_color = mu::color(mu::teal_normal);
  text.bg_color = mu::color(mu::black_trans);

  pub_text_.publish(text);
}

void MaderRos::publishPlanes(std::vector<Hyperplane3D>& planes)
{
  //  int num_of_intervals = planes.size() / poly_safe.size();

  auto color = visual_tools_->getRandColor();

  int i = 0;
  for (auto plane_i : planes)
  {
    if ((i % par_.num_pol) == 0)  // planes for a new obstacle --> new color
    {
      color = visual_tools_->getRandColor();  // rviz_visual_tools::TRANSLUCENT_LIGHT;  //
    }
    Eigen::Isometry3d pose;
    pose.translation() = plane_i.p_;

    // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
    Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, plane_i.n_);
    pose.linear() = q.toRotationMatrix();

    double height = 0.001;  // very thin
    double x_width = 2;     // very thin
    double y_width = 2;     // very thin
    visual_tools_->publishCuboid(pose, x_width, y_width, height, color);
    i++;

    /*    double d_i = -plane_i.n_.dot(plane_i.p_);
        std::cout << bold << "Publishing plane, d_i= " << d_i << reset << std::endl;
        visual_tools_->publishABCDPlane(plane_i.n_.x(), plane_i.n_.y(), plane_i.n_.z(), d_i, rvt::MAGENTA, 2, 2);*/
  }
  visual_tools_->trigger();
}

void MaderRos::publishPoly(const vec_E<Polyhedron<3>>& poly)
{
  // std::cout << "Going to publish= " << (poly[0].hyperplanes())[0].n_ << std::endl;
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly);
  poly_msg.header.frame_id = world_name_;

  poly_safe_pub_.publish(poly_msg);
}

// this function won't be called in simulations

void MaderRos::whoPlansCB(const mader_msgs::WhoPlans& msg)
{
  if (msg.value != msg.MADER)
  {  // MADER does nothing
    sub_state_.shutdown();
    sub_term_goal_.shutdown();
    pubCBTimer_.stop();
    replanCBTimer_.stop();
    // mtx_mader_ptr_.lock();
    mader_ptr_->resetInitialization();
    // mtx_mader_ptr_.unlock();
    is_mader_running_ = false;
    std::cout << on_blue << "**************MADER STOPPED" << reset << std::endl;
  }
  else
  {  // MADER is the one who plans now (this happens when the take-off is finished)
    sub_term_goal_ = nh1_.subscribe("term_goal", 1, &MaderRos::terminalGoalCB, this);  // TODO: duplicated from above
    sub_state_ = nh1_.subscribe("state", 1, &MaderRos::stateCB, this);                 // TODO: duplicated from above
    pubCBTimer_.start();
    replanCBTimer_.start();
    is_mader_running_ = true;
    std::cout << on_blue << "**************MADER STARTED" << reset << std::endl;
  }
}

void MaderRos::stateCB(const snapstack_msgs::State& msg)
{
  mt::state state_tmp;  // this gets state from ROS
  state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  // std::cout << bold << red << "MSG_QUAT= " << msg.quat << reset << std::endl;
  double roll, pitch, yaw;
  mu::quaternion2Euler(msg.quat, roll, pitch, yaw);
  state_tmp.setYaw(yaw);
  state_ = state_tmp;
  // std::cout << bold << red << "STATE_YAW= " << state_.yaw << reset << std::endl;

  // std::cout << "Updating state to" << std::endl;
  // state_tmp.print();

  // mtx_mader_ptr_.lock();
  mader_ptr_->updateState(state_tmp);  // this updates state //mader_ptr_ is a pointer to mader object
  // mtx_mader_ptr_.unlock();

  W_T_B_ = Eigen::Translation3d(msg.pos.x, msg.pos.y, msg.pos.z) *
           Eigen::Quaterniond(msg.quat.w, msg.quat.x, msg.quat.y, msg.quat.z);

  if (published_initial_position_ == false)
  {
    pwp_last_ = mu::createPwpFromStaticPosition(state_);
    publishOwnTraj(pwp_last_, true);
    published_initial_position_ = true;
  }
  // mtx_mader_ptr_.lock();
  if (mader_ptr_->IsTranslating() == true && par_.visual)
  {
    pubActualTraj();
  }
  // mtx_mader_ptr_.unlock();

  // publishFOV();
}

// void MaderRos::modeCB(const mader_msgs::Mode& msg)
// {
//   // mader_ptr_->changeMode(msg.mode);

//   if (msg.mode != msg.GO)
//   {  // MADER DOES NOTHING
//     // sub_state_.shutdown();
//     pubCBTimer_.stop();
//     replanCBTimer_.stop();
//     // std::cout << on_blue << "**************stopping replanCBTimer" << reset << std::endl;
//     mader_ptr_->resetInitialization();
//   }
//   else
//   {  // The mode changed to GO (the mode changes to go when takeoff is finished)
//     // sub_state_ = nh_.subscribe("state", 1, &MaderRos::stateCB, this);  // TODO duplicated from above
//     // std::cout << on_blue << "**************starting replanCBTimer" << reset << std::endl;
//     pubCBTimer_.start();
//     replanCBTimer_.start();
//   }
// }

void MaderRos::pubCB(const ros::TimerEvent& e)
{
  mt::state next_goal;
  // mtx_mader_ptr_.lock();
  if (mader_ptr_->getNextGoal(next_goal))
  {
    snapstack_msgs::Goal quadGoal;

    quadGoal.p = mu::eigen2point(next_goal.pos);  // Kota changed it from eigen2rosvector July 26, 2021

    // printf("terminal goal x %f \n", next_goal.pos.x());
    // printf("terminal goal y %f \n", next_goal.pos.y());
    // printf("terminal goal z %f \n", next_goal.pos.z());

    quadGoal.p = mu::eigen2point(next_goal.pos);  // Kota changed it from eigen2rosvector July 26, 2021

    // printf("terminal goal x %f \n", next_goal.pos.x());
    // printf("terminal goal y %f \n", next_goal.pos.y());
    // printf("terminal goal z %f \n", next_goal.pos.z());

    quadGoal.v = mu::eigen2rosvector(next_goal.vel);
    quadGoal.a = mu::eigen2rosvector(next_goal.accel);
    quadGoal.j = mu::eigen2rosvector(next_goal.jerk);

    quadGoal.dpsi = next_goal.dyaw;  // no need to control dyaw
    quadGoal.psi = next_goal.yaw;    // no need to contol yaw

    quadGoal.header.stamp = ros::Time::now();
    quadGoal.header.frame_id = world_name_;

    quadGoal.power = true;  // kota added July 27, 2021

    pub_goal_.publish(quadGoal);

    setpoint_.header.stamp = ros::Time::now();
    setpoint_.pose.position.x = quadGoal.p.x;
    setpoint_.pose.position.y = quadGoal.p.y;
    setpoint_.pose.position.z = quadGoal.p.z;

    pub_setpoint_.publish(setpoint_);
  }
  // mtx_mader_ptr_.unlock();
}

void MaderRos::clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher)
{
  if ((*tmp).markers.size() == 0)
  {
    return;
  }
  int id_begin = (*tmp).markers[0].id;

  for (int i = 0; i < (*tmp).markers.size(); i++)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::DELETE;
    m.id = i + id_begin;
    m.scale.x = 0.15;
    m.scale.y = 0;
    m.scale.z = 0;
    (*tmp).markers[i] = m;
  }

  (*publisher).publish(*tmp);
  (*tmp).markers.clear();
}

void MaderRos::pubTraj(const std::vector<mt::state>& data, const bool& is_committed)
{
  // Trajectory
  nav_msgs::Path traj;
  traj.poses.clear();
  traj.header.stamp = ros::Time::now();
  traj.header.frame_id = world_name_;

  geometry_msgs::PoseStamped temp_path;

  int increm = (int)std::max(data.size() / par_.res_plot_traj, 1.0);  // this is to speed up rviz

  for (int i = 0; i < data.size(); i = i + increm)
  {
    temp_path.pose.position.x = data[i].pos(0);
    temp_path.pose.position.y = data[i].pos(1);
    temp_path.pose.position.z = data[i].pos(2);
    temp_path.pose.orientation.w = 1;
    temp_path.pose.orientation.x = 0;
    temp_path.pose.orientation.y = 0;
    temp_path.pose.orientation.z = 0;
    traj.poses.push_back(temp_path);
  }

  pub_traj_safe_.publish(traj);

  double scale = 0.15;

  if (!is_committed)
  {
    clearMarkerArray(&traj_safe_colored_bef_commit_, &pub_traj_safe_colored_bef_commit_);
    traj_safe_colored_bef_commit_ = mu::trajectory2ColoredMarkerArray(data, par_.v_max.maxCoeff(), increm, name_drone_,
                                                                      scale, "bef_DC", id_, par_.n_agents);
    pub_traj_safe_colored_bef_commit_.publish(traj_safe_colored_bef_commit_);
  }
  else
  {  // if commit is successful then delete the old one
    clearMarkerArray(&traj_safe_colored_, &pub_traj_safe_colored_);
    clearMarkerArray(&traj_safe_colored_bef_commit_, &pub_traj_safe_colored_bef_commit_);
    traj_safe_colored_ = mu::trajectory2ColoredMarkerArray(data, par_.v_max.maxCoeff(), increm, name_drone_, scale,
                                                           par_.color_type, id_, par_.n_agents);
    pub_traj_safe_colored_.publish(traj_safe_colored_);
  }
}

void MaderRos::pubActualTraj()
{
  static geometry_msgs::Point p_last = mu::pointOrigin();

  mt::state current_state;
  // mtx_mader_ptr_.lock();
  mader_ptr_->getState(current_state);
  // mtx_mader_ptr_.unlock();
  Eigen::Vector3d act_pos = current_state.pos;

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.id = actual_trajID_;  // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
  m.ns = "ActualTraj_" + name_drone_;
  actual_trajID_++;
  // m.color = mu::getColorJet(current_state.vel.norm(), 0, par_.v_max.maxCoeff());  // mu::color(mu::red_normal);

  if (par_.color_type == "vel")
  {
    m.color = mu::getColorJet(current_state.vel.norm(), 0, par_.v_max.maxCoeff());  // note that par_.v_max is per axis!
  }
  else
  {
    m.color = mu::getColorJet(id_, 0, par_.n_agents);  // note that par_.v_max is per axis!
  }

  m.scale.x = 0.15;
  m.scale.y = 0.0001;
  m.scale.z = 0.0001;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = world_name_;

  // pose is actually not used in the marker, but if not RVIZ complains about the quaternion
  m.pose.position = mu::pointOrigin();
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  geometry_msgs::Point p;
  p = mu::eigen2point(act_pos);
  m.points.push_back(p_last);
  m.points.push_back(p);
  p_last = p;

  if (m.id == 0)
  {
    return;
  }

  pub_actual_traj_.publish(m);
}

void MaderRos::clearMarkerActualTraj()
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
  actual_trajID_ = 0;
}

void MaderRos::clearMarkerColoredTraj()
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 1;
  m.scale.y = 1;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
}

void MaderRos::pubState(const mt::state& data, const ros::Publisher pub)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = world_name_;
  p.point = mu::eigen2point(data.pos);
  pub.publish(p);
}

void MaderRos::terminalGoalCB(const geometry_msgs::PoseStamped& msg)
{
  mt::state G_term;
  double z;

  if (!is_term_goal_initialized_)
  {
    is_term_goal_initialized_ = true;
    ros::Duration(0.1).sleep();  // wait to receive other's trajs
  }

  if (fabs(msg.pose.position.z) < 1e-5)  // This happens when you click in RVIZ (msg.z is 0.0)
  {
    z = 1.0;
  }
  else  // This happens when you publish by yourself the goal (should always be above the ground)
  {
    z = msg.pose.position.z;
  }

  G_term.setPos(msg.pose.position.x, msg.pose.position.y, z);
  // mtx_mader_ptr_.lock();
  mader_ptr_->setTerminalGoal(G_term);
  // mtx_mader_ptr_.unlock();

  pubState(G_term, pub_point_G_term_);

  clearMarkerActualTraj();
}

void MaderRos::publishFOV()
{
  visualization_msgs::Marker marker_fov;
  marker_fov.header.frame_id = name_drone_;
  marker_fov.header.stamp = ros::Time::now();
  marker_fov.ns = "marker_fov";
  marker_fov.id = 0;
  marker_fov.type = marker_fov.LINE_LIST;
  marker_fov.action = marker_fov.ADD;
  marker_fov.pose = mu::identityGeometryMsgsPose();

  double delta_y = par_.fov_depth * fabs(tan((par_.fov_horiz_deg * M_PI / 180) / 2.0));
  double delta_z = par_.fov_depth * fabs(tan((par_.fov_vert_deg * M_PI / 180) / 2.0));

  geometry_msgs::Point v0 = mu::eigen2point(Eigen::Vector3d(0.0, 0.0, 0.0));
  geometry_msgs::Point v1 = mu::eigen2point(Eigen::Vector3d(par_.fov_depth, delta_y, -delta_z));
  geometry_msgs::Point v2 = mu::eigen2point(Eigen::Vector3d(par_.fov_depth, -delta_y, -delta_z));
  geometry_msgs::Point v3 = mu::eigen2point(Eigen::Vector3d(par_.fov_depth, -delta_y, delta_z));
  geometry_msgs::Point v4 = mu::eigen2point(Eigen::Vector3d(par_.fov_depth, delta_y, delta_z));

  marker_fov.points.clear();

  // Line
  marker_fov.points.push_back(v0);
  marker_fov.points.push_back(v1);

  // Line
  marker_fov.points.push_back(v0);
  marker_fov.points.push_back(v2);

  // Line
  marker_fov.points.push_back(v0);
  marker_fov.points.push_back(v3);

  // Line
  marker_fov.points.push_back(v0);
  marker_fov.points.push_back(v4);

  // Line
  marker_fov.points.push_back(v1);
  marker_fov.points.push_back(v2);

  // Line
  marker_fov.points.push_back(v2);
  marker_fov.points.push_back(v3);

  // Line
  marker_fov.points.push_back(v3);
  marker_fov.points.push_back(v4);

  // Line
  marker_fov.points.push_back(v4);
  marker_fov.points.push_back(v1);

  marker_fov.scale.x = 0.03;
  marker_fov.scale.y = 0.00001;
  marker_fov.scale.z = 0.00001;
  marker_fov.color.a = 1.0;
  marker_fov.color.r = 0.0;
  marker_fov.color.g = 1.0;
  marker_fov.color.b = 0.0;

  pub_fov_.publish(marker_fov);

  ////
  /// https://github.com/PickNikRobotics/rviz_visual_tools/blob/80212659be877f221cf23528b4e4887eaf0c08a4/src/rviz_visual_tools.cpp#L957

  return;
}

void MaderRos::verify(bool cond, std::string info_if_false)
{
  if (cond == false)
  {
    std::cout << termcolor::bold << termcolor::red << info_if_false << termcolor::reset << std::endl;
    std::cout << termcolor::red << "Aborting" << termcolor::reset << std::endl;
    abort();
  }
}