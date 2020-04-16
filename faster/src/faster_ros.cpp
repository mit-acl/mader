#include "faster_ros.hpp"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>  //For DecompROS::polyhedron_array_to_ros
#include <decomp_geometry/polyhedron.h>       //For hyperplane
#include <Eigen/Geometry>

#include <jsk_rviz_plugins/OverlayText.h>

typedef Timer MyTimer;

// this object is created in the faster_ros_node
FasterRos::FasterRos(ros::NodeHandle nh, ros::NodeHandle nh_replan_CB, ros::NodeHandle nh_pub_CB)
  : nh_(nh), nh_replan_CB_(nh_replan_CB), nh_pub_CB_(nh_pub_CB)
{
  safeGetParam(nh_, "use_ff", par_.use_ff);
  safeGetParam(nh_, "visual", par_.visual);

  safeGetParam(nh_, "dc", par_.dc);
  safeGetParam(nh_, "goal_radius", par_.goal_radius);
  safeGetParam(nh_, "drone_radius", par_.drone_radius);

  safeGetParam(nh_, "N_safe", par_.N_safe);
  safeGetParam(nh_, "N_whole", par_.N_whole);

  safeGetParam(nh_, "Ra", par_.Ra);
  safeGetParam(nh_, "R_consider_others", par_.R_consider_others);
  safeGetParam(nh_, "w_max", par_.w_max);
  safeGetParam(nh_, "alpha_filter_dyaw", par_.alpha_filter_dyaw);

  safeGetParam(nh_, "z_ground", par_.z_ground);
  safeGetParam(nh_, "z_max", par_.z_max);
  safeGetParam(nh_, "inflation_jps", par_.inflation_jps);
  safeGetParam(nh_, "factor_jps", par_.factor_jps);

  safeGetParam(nh_, "v_max", par_.v_max);
  safeGetParam(nh_, "a_max", par_.a_max);
  safeGetParam(nh_, "j_max", par_.j_max);

  safeGetParam(nh_, "gamma_whole", par_.gamma_whole);
  safeGetParam(nh_, "gammap_whole", par_.gammap_whole);
  safeGetParam(nh_, "increment_whole", par_.increment_whole);
  safeGetParam(nh_, "gamma_safe", par_.gamma_safe);
  safeGetParam(nh_, "gammap_safe", par_.gammap_safe);
  safeGetParam(nh_, "alpha", par_.alpha);

  safeGetParam(nh_, "increment_safe", par_.increment_safe);

  safeGetParam(nh_, "delta_a", par_.delta_a);
  safeGetParam(nh_, "delta_H", par_.delta_H);

  safeGetParam(nh_, "max_poly_whole", par_.max_poly_whole);
  safeGetParam(nh_, "max_poly_safe", par_.max_poly_safe);
  safeGetParam(nh_, "dist_max_vertexes", par_.dist_max_vertexes);

  safeGetParam(nh_, "gurobi_threads", par_.gurobi_threads);
  safeGetParam(nh_, "gurobi_verbose", par_.gurobi_verbose);

  safeGetParam(nh_, "use_faster", par_.use_faster);

  safeGetParam(nh_, "n_pol", par_.n_pol);
  safeGetParam(nh_, "deg", par_.deg);
  safeGetParam(nh_, "samples_per_interval", par_.samples_per_interval);
  safeGetParam(nh_, "weight", par_.weight);
  safeGetParam(nh_, "epsilon_tol_constraints", par_.epsilon_tol_constraints);
  safeGetParam(nh_, "xtol_rel", par_.xtol_rel);
  safeGetParam(nh_, "ftol_rel", par_.ftol_rel);
  safeGetParam(nh_, "solver", par_.solver);

  safeGetParam(nh_, "kappa", par_.kappa);
  safeGetParam(nh_, "mu", par_.mu);

  safeGetParam(nh_, "a_star_samp_x", par_.a_star_samp_x);
  safeGetParam(nh_, "a_star_samp_y", par_.a_star_samp_y);
  safeGetParam(nh_, "a_star_samp_z", par_.a_star_samp_z);

  safeGetParam(nh_, "a_star_bias", par_.a_star_bias);

  safeGetParam(nh_, "basis", par_.basis);

  safeGetParam(nh_, "res_plot_traj", par_.res_plot_traj);

  // Parameters for the ground robot (jackal):
  /*  safeGetParam(nh_,"kw", par_.kw);
    safeGetParam(nh_,"kyaw", par_.kyaw);
    safeGetParam(nh_,"kdalpha", par_.kdalpha);
    safeGetParam(nh_,"kv", par_.kv);
    safeGetParam(nh_,"kdist", par_.kdist);
    safeGetParam(nh_,"kalpha", par_.kalpha);*/

  // And now obtain the parameters from the mapper
  std::vector<double> world_dimensions;
  safeGetParam(nh_, "mapper/world_dimensions", world_dimensions);
  safeGetParam(nh_, "mapper/resolution", par_.res);

  par_.wdx = world_dimensions[0];
  par_.wdy = world_dimensions[1];
  par_.wdz = world_dimensions[2];

  std::cout << "par_.wdx= " << par_.wdx << std::endl;
  std::cout << "par_.wdy= " << par_.wdy << std::endl;
  std::cout << "par_.wdz= " << par_.wdz << std::endl;

  std::cout << bold << green << "world_dimensions=" << world_dimensions << reset << std::endl;
  std::cout << bold << green << "resolution=" << par_.res << reset << std::endl;

  std::cout << "Parameters obtained" << std::endl;

  if (par_.N_safe <= par_.max_poly_safe + 2)
  {
    std::cout << bold << red << "Needed: N_safe>=max_poly+ 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }
  if (par_.N_whole <= par_.max_poly_whole + 2)
  {
    std::cout << bold << red << "Needed: N_whole>=max_poly + 2 at least" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }

  if (par_.factor_jps * par_.res / 2.0 > par_.inflation_jps)
  {
    std::cout << bold << red << "Needed: par_.factor_jps * par_.res / 2 <= par_.inflation_jps" << reset
              << std::endl;  // If not JPS will find a solution between the voxels.
    abort();
  }

  if ((par_.kappa + par_.mu) > 1)
  {
    std::cout << bold << red << "Needed: (par_.kappa + par_.mu) <= 1" << reset
              << std::endl;  // To decrease the probability of not finding a solution
    abort();
  }

  if (par_.R_consider_others < 2 * par_.Ra)
  {
    std::cout << bold << red << "Needed: par_.R_consider_others < 2 * par_.Ra" << reset << std::endl;
    abort();
  }

  /*  if (par_.Ra_max > (par_.wdx / 2.0) || (par_.Ra_max > par_.wdy / 2.0))
    {
      std::cout << bold << red << "Needed: par_.Ra_max > par_.wdx/2.0|| par_.Ra_max > par_.wdy/2.0" << reset
                << std::endl;  // To decrease the probability of not finding a solution
      abort();f
    }*/

  /*  if (par_.drone_radius <= 2 * par_.res)
    {
      std::cout << bold << red << "Needed: par_.drone_radius > 2*par_.res" << reset
                << std::endl;  // If not the convex decomposition finds polytopes between the voxels of the obstacles
      abort();
    }*/

  /*  if (par_.inflation_jps <= par_.res/2.0 + par_.drone_radius)
    {
      std::cout << bold << red << "Needed: par_.inflation_jps > par_.res/2.0 + par_.drone_radius" << reset
                << std::endl; //JPS should be run with at least drone_radius + half of the size of a voxel
      abort();
    }
  */

  // Publishers
  // pub_goal_jackal_ = nh_.advertise<geometry_msgs::Twist>("goal_jackal", 1);
  pub_goal_ = nh_.advertise<snapstack_msgs::QuadGoal>("goal", 1);
  pub_traj_whole_ = nh_.advertise<nav_msgs::Path>("traj_whole", 1);
  pub_traj_safe_ = nh_.advertise<nav_msgs::Path>("traj_safe", 1);
  pub_setpoint_ = nh_.advertise<visualization_msgs::Marker>("setpoint", 1);
  pub_intersectionI_ = nh_.advertise<visualization_msgs::Marker>("intersection_I", 1);
  pub_point_G_ = nh_.advertise<geometry_msgs::PointStamped>("point_G", 1);
  pub_point_G_term_ = nh_.advertise<geometry_msgs::PointStamped>("point_G_term", 1);
  pub_point_E_ = nh_.advertise<visualization_msgs::Marker>("point_E", 1);
  pub_point_R_ = nh_.advertise<visualization_msgs::Marker>("point_R", 1);
  pub_point_M_ = nh_.advertise<visualization_msgs::Marker>("point_M", 1);
  pub_point_H_ = nh_.advertise<visualization_msgs::Marker>("point_H", 1);
  pub_point_A_ = nh_.advertise<visualization_msgs::Marker>("point_A", 1);
  pub_actual_traj_ = nh_.advertise<visualization_msgs::Marker>("actual_traj", 1);
  pub_path_jps1_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps1", 1);
  pub_path_jps2_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps2", 1);
  pub_path_jps_whole_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_whole", 1);
  pub_path_jps_safe_ = nh_.advertise<visualization_msgs::MarkerArray>("path_jps_safe", 1);
  poly_whole_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_whole", 1, true);
  poly_safe_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("poly_safe", 1, true);
  pub_jps_inters_ = nh_.advertise<geometry_msgs::PointStamped>("jps_intersection", 1);
  pub_text_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("text", 1);
  // pub_log_ = nh_.advertise<snapstack_msgs::Cvx>("log_topic", 1);
  pub_traj_committed_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_committed_colored", 1);
  pub_traj_whole_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_whole_colored", 1);
  pub_traj_safe_colored_ = nh_.advertise<visualization_msgs::MarkerArray>("traj_safe_colored", 1);
  pub_cloud_jps_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_jps", 1);
  pub_traj_ = nh_.advertise<faster_msgs::DynTraj>("/trajs", 1, true);  // The last parameter is latched or not

  // Subscribers
  occup_grid_sub_.subscribe(nh_, "occup_grid", 1);
  unknown_grid_sub_.subscribe(nh_, "unknown_grid", 1);
  sync_.reset(new Sync(MySyncPolicy(1), occup_grid_sub_, unknown_grid_sub_));
  sync_->registerCallback(boost::bind(&FasterRos::mapCB, this, _1, _2));
  sub_goal_ = nh_.subscribe("term_goal", 1, &FasterRos::terminalGoalCB, this);
  sub_mode_ = nh_.subscribe("mode", 1, &FasterRos::modeCB, this);
  sub_state_ = nh_.subscribe("state", 1, &FasterRos::stateCB, this);
  sub_traj_ = nh_.subscribe("/trajs", 20, &FasterRos::trajCB, this);  // The number is the queue size
  // sub_odom_ = nh_.subscribe("odom", 1, &FasterRos::odomCB, this);

  // Timers
  pubCBTimer_ = nh_pub_CB_.createTimer(ros::Duration(par_.dc), &FasterRos::pubCB, this);
  replanCBTimer_ = nh_replan_CB_.createTimer(ros::Duration(par_.dc), &FasterRos::replanCB, this);

  // For now stop all these subscribers/timers until we receive GO
  occup_grid_sub_.unsubscribe();
  unknown_grid_sub_.unsubscribe();
  sub_state_.shutdown();
  pubCBTimer_.stop();
  replanCBTimer_.stop();

  // Rviz_Visual_Tools
  visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
  visual_tools_->loadMarkerPub();  // create publisher before waitin
  ros::Duration(0.5).sleep();
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  // Markers
  setpoint_ = getMarkerSphere(0.35, ORANGE_TRANS);
  R_ = getMarkerSphere(0.35, ORANGE_TRANS);
  I_ = getMarkerSphere(0.35, YELLOW_NORMAL);
  E_ = getMarkerSphere(0.35, RED_NORMAL);
  M_ = getMarkerSphere(0.35, BLUE_NORMAL);
  H_ = getMarkerSphere(0.35, GREEN_NORMAL);
  A_ = getMarkerSphere(0.35, RED_NORMAL);

  // If you want another thread for the replanCB: replanCBTimer_ = nh_.createTimer(ros::Duration(par_.dc),
  // &FasterRos::replanCB, this);

  name_drone_ = ros::this_node::getNamespace();  // Return also the slashes (2 in Kinetic, 1 in Melodic)

  name_drone_.erase(std::remove(name_drone_.begin(), name_drone_.end(), '/'), name_drone_.end());  // Remove the slashes

  // name_drone_.erase(0, 2);  // Erase slashes

  std::string id = name_drone_;
  id.erase(0, 2);  // Erase SQ or HX i.e. SQ12 --> 12  HX8621 --> 8621
  id_ = std::stoi(id);

  tfListener = new tf2_ros::TransformListener(tf_buffer_);
  // wait for body transform to be published before initializing

  // COMMENTED TO BE ABLE TO LAUNCH SIMULATIONS WITHOUT GAZEBO
  /*  ROS_INFO("Waiting for world to camera transform...");
    while (true)
    {
      try
      {
        tf_buffer_.lookupTransform(world_name_, name_drone_ + "/camera", ros::Time::now(), ros::Duration(3.0));  //
        break;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }*/
  // END OF COMMENTED
  clearMarkerActualTraj();

  faster_ptr_ = std::unique_ptr<Faster>(new Faster(par_));

  ROS_INFO("Planner initialized");
}

FasterRos::~FasterRos()
{
  occup_grid_sub_.unsubscribe();
  unknown_grid_sub_.unsubscribe();
  sub_state_.shutdown();
  pubCBTimer_.stop();
  replanCBTimer_.stop();
}

void FasterRos::trajCB(const faster_msgs::DynTraj& msg)
{
  if (msg.id == id_)
  {  // This is my own trajectory
    return;
  }

  std::vector<dynTraj>::iterator obs_ptr =
      std::find_if(trajs_.begin(), trajs_.end(), [=](const dynTraj& traj) { return traj.id == msg.id; });
  bool exists = (obs_ptr != std::end(trajs_));

  Eigen::Vector3d pos(msg.pos.x, msg.pos.y, msg.pos.z);
  bool near_me = ((state_.pos - pos).norm() < par_.R_consider_others);
  // std::cout << "dist= " << (state_.pos - pos).norm() << std::endl;

  dynTraj tmp;
  tmp.function.push_back(msg.function[0]);
  tmp.function.push_back(msg.function[1]);
  tmp.function.push_back(msg.function[2]);

  tmp.bbox.push_back(msg.bbox[0]);
  tmp.bbox.push_back(msg.bbox[1]);
  tmp.bbox.push_back(msg.bbox[2]);

  tmp.id = msg.id;

  tmp.time_received = ros::Time::now().toSec();

  // First let's check if the object is near me:
  if (near_me)
  {
    if (exists)
    {  // if that object already exists, substitute its trajectory
      std::cout << red << "Updating " << tmp.id << reset << std::endl;
      *obs_ptr = tmp;
    }
    else
    {  // if it doesn't exist, create it
      trajs_.push_back(tmp);
      std::cout << red << "Adding " << tmp.id << reset << std::endl;
    }
  }
  else  // not near me
  {
    if (exists)  // remove if fromg the list if it exists
    {
      trajs_.erase(obs_ptr);
      std::cout << red << "Erasing " << (*obs_ptr).id << reset << std::endl;
    }
  }

  // print elements for debugging:
  // std::cout << red << bold << "========================" << reset << std::endl;
  // std::cout << "trajs_ has " << trajs_.size() << " obstacles:" << std::endl;
  /*  for (auto traj : trajs_)
    {
      std::cout << traj.id << ", " << std::endl;
    }*/
  // std::cout << red << bold << "========================" << reset << std::endl;

  faster_ptr_->updateTrajObstacles(trajs_);

  // std::cout << "End of trajCB" << reset << std::endl;
}

// This trajectory is published when the agent arrives at A
void FasterRos::publishOwnTraj(const PieceWisePol& pwp)
{
  std::vector<std::string> s = pieceWisePol2String(pwp);

  faster_msgs::DynTraj msg;
  msg.function = s;
  msg.bbox.push_back(2 * par_.drone_radius);
  msg.bbox.push_back(2 * par_.drone_radius);
  msg.bbox.push_back(2 * par_.drone_radius);
  msg.pos.x = state_.pos.x();
  msg.pos.y = state_.pos.y();
  msg.pos.z = state_.pos.z();
  msg.id = id_;

  pub_traj_.publish(msg);
}

void FasterRos::replanCB(const ros::TimerEvent& e)
{
  if (ros::ok())
  {
    vec_Vecf<3> JPS_safe;
    vec_Vecf<3> JPS_whole;
    vec_E<Polyhedron<3>> poly_safe;
    vec_E<Polyhedron<3>> poly_whole;
    std::vector<state> X_safe;
    std::vector<state> X_whole;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud_jps(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<Hyperplane3D> planes_guesses;
    PieceWisePol pwp;

    bool replanned = faster_ptr_->replan(JPS_safe, JPS_whole, poly_safe, poly_whole, X_safe, X_whole, pcloud_jps,
                                         planes_guesses, num_of_LPs_run_, num_of_QCQPs_run_, pwp);

    // Delete markers to publish stuff
    // visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    pcl::PCLPointCloud2 pcloud_jps2;
    pcl::toPCLPointCloud2(*pcloud_jps.get(), pcloud_jps2);
    sensor_msgs::PointCloud2 cloud_jps_msg;
    pcl_conversions::fromPCL(pcloud_jps2, cloud_jps_msg);
    // cloud_jps_msg.header = mesh_cloud_msg_->header;
    cloud_jps_msg.header.frame_id = "world";
    pub_cloud_jps_.publish(cloud_jps_msg);

    clearJPSPathVisualization(2);
    publishJPSPath(JPS_safe, JPS_SAFE);
    publishJPSPath(JPS_whole, JPS_WHOLE);

    publishPoly(poly_safe, SAFE);
    publishPoly(poly_whole, WHOLE);
    pubTraj(X_safe, SAFE_COLORED);

    pubTraj(X_whole, WHOLE_COLORED);
    // std::cout << "Plane Guesses has" << planes_guesses.size() << std::endl;
    publishPlanes(planes_guesses);

    pub_text_ = nh_.advertise<jsk_rviz_plugins::OverlayText>("text", 1);
    jsk_rviz_plugins::OverlayText text;
    text.width = 600;
    text.height = 133;
    text.left = 10;
    text.top = 10;
    text.text_size = 17;
    text.line_width = 2;
    text.font = "DejaVu Sans Mono";
    text.text = "Num of LPs run= " + std::to_string(num_of_LPs_run_) + "\n" +  ///////////////////
                "Num of QCQPs run= " + std::to_string(num_of_QCQPs_run_);

    text.fg_color = color(TEAL_NORMAL);
    text.bg_color = color(BLACK_TRANS);

    if (replanned)
    {
      publishOwnTraj(pwp);
    }

    pub_text_.publish(text);
  }
}

void FasterRos::publishPlanes(std::vector<Hyperplane3D>& planes)
{
  //  int num_of_intervals = planes.size() / poly_safe.size();

  auto color = visual_tools_->getRandColor();

  int i = 0;
  for (auto plane_i : planes)
  {
    if ((i % par_.n_pol) == 0)  // planes for a new obstacle --> new color
    {
      color = visual_tools_->getRandColor();
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

void FasterRos::publishPoly(const vec_E<Polyhedron<3>>& poly, int type)
{
  // std::cout << "Going to publish= " << (poly[0].hyperplanes())[0].n_ << std::endl;
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(poly);
  poly_msg.header.frame_id = world_name_;

  switch (type)
  {
    case SAFE:
      poly_safe_pub_.publish(poly_msg);
      break;
    case WHOLE:
      poly_whole_pub_.publish(poly_msg);
      break;
  }
}

void FasterRos::stateCB(const snapstack_msgs::State& msg)
{
  state state_tmp;
  state_tmp.setPos(msg.pos.x, msg.pos.y, msg.pos.z);
  state_tmp.setVel(msg.vel.x, msg.vel.y, msg.vel.z);
  state_tmp.setAccel(0.0, 0.0, 0.0);
  // std::cout << bold << red << "MSG_QUAT= " << msg.quat << reset << std::endl;
  double roll, pitch, yaw;
  quaternion2Euler(msg.quat, roll, pitch, yaw);
  state_tmp.setYaw(yaw);
  state_ = state_tmp;
  // std::cout << bold << red << "STATE_YAW= " << state_.yaw << reset << std::endl;
  faster_ptr_->updateState(state_tmp);

  if (published_initial_position_ == false)
  {
    PieceWisePol pwp;
    pwp.times = { ros::Time::now().toSec(), ros::Time::now().toSec() + 1e10 };

    Eigen::Matrix<double, 4, 1> coeff_x_interv0;  // [a b c d]' of the interval 0
    Eigen::Matrix<double, 4, 1> coeff_y_interv0;  // [a b c d]' of the interval 0
    Eigen::Matrix<double, 4, 1> coeff_z_interv0;  // [a b c d]' of the interval 0

    coeff_x_interv0 << 0.0, 0.0, 0.0, state_.pos.x();
    coeff_y_interv0 << 0.0, 0.0, 0.0, state_.pos.y();
    coeff_z_interv0 << 0.0, 0.0, 0.0, state_.pos.z();

    pwp.coeff_x.push_back(coeff_x_interv0);
    pwp.coeff_y.push_back(coeff_y_interv0);
    pwp.coeff_z.push_back(coeff_z_interv0);

    publishOwnTraj(pwp);
    published_initial_position_ = true;
  }
  if (faster_ptr_->IsTranslating() == true)
  {
    pubActualTraj();
  }
}

void FasterRos::modeCB(const faster_msgs::Mode& msg)
{
  // faster_ptr_->changeMode(msg.mode);

  if (msg.mode != msg.GO)
  {  // FASTER DOES NOTHING
    occup_grid_sub_.unsubscribe();
    unknown_grid_sub_.unsubscribe();
    sub_state_.shutdown();
    pubCBTimer_.stop();
    replanCBTimer_.stop();
    std::cout << "stopping replanCBTimer" << std::endl;
    faster_ptr_->resetInitialization();
  }
  else
  {  // The mode changed to GO (the mode changes to go when takeoff is finished)
    occup_grid_sub_.subscribe();
    unknown_grid_sub_.subscribe();

    sub_state_ = nh_.subscribe("state", 1, &FasterRos::stateCB, this);  // TODO duplicated from above

    pubCBTimer_.start();
    replanCBTimer_.start();
  }
}

void FasterRos::pubCB(const ros::TimerEvent& e)
{
  state next_goal;
  if (faster_ptr_->getNextGoal(next_goal))
  {
    snapstack_msgs::QuadGoal quadGoal;
    // visualization_msgs::Marker setpoint;
    // Pub setpoint maker.  setpoint_ is the last quadGoal sent to the drone

    // printf("Publicando Goal=%f, %f, %f\n", quadGoal_.pos.x, quadGoal_.pos.y, quadGoal_.pos.z);

    quadGoal.pos = eigen2rosvector(next_goal.pos);
    quadGoal.vel = eigen2rosvector(next_goal.vel);
    quadGoal.accel = eigen2rosvector(next_goal.accel);
    quadGoal.jerk = eigen2rosvector(next_goal.jerk);
    quadGoal.dyaw = next_goal.dyaw;
    quadGoal.yaw = next_goal.yaw;
    quadGoal.header.stamp = ros::Time::now();
    quadGoal.header.frame_id = world_name_;

    // std::cout << bold << blue << "[FasterRos] publishing goal= " << quadGoal.yaw << reset << std::endl;
    pub_goal_.publish(quadGoal);

    setpoint_.header.stamp = ros::Time::now();
    setpoint_.pose.position.x = quadGoal.pos.x;
    setpoint_.pose.position.y = quadGoal.pos.y;
    setpoint_.pose.position.z = quadGoal.pos.z;

    pub_setpoint_.publish(setpoint_);
  }
}

void FasterRos::clearJPSPathVisualization(int i)
{
  switch (i)
  {
    case JPSk_NORMAL:
      clearMarkerArray(&path_jps1_, &pub_path_jps1_);
      break;
    case JPS2_NORMAL:
      clearMarkerArray(&path_jps2_, &pub_path_jps2_);
      break;
    case JPS_WHOLE:
      clearMarkerArray(&path_jps_whole_, &pub_path_jps_whole_);
      break;
    case JPS_SAFE:
      clearMarkerArray(&path_jps_safe_, &pub_path_jps_safe_);
      break;
  }
}

void FasterRos::clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher)
{
  if ((*tmp).markers.size() == 0)
  {
    return;
  }
  int id_begin = (*tmp).markers[0].id;
  // int id_end = (*path).markers[markers.size() - 1].id;

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

void FasterRos::publishJPSPath(vec_Vecf<3>& path, int i)
{
  /*vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array*/
  clearJPSPathVisualization(i);
  switch (i)
  {
    case JPSk_NORMAL:
      vectorOfVectors2MarkerArray(path, &path_jps1_, color(BLUE_NORMAL));
      pub_path_jps1_.publish(path_jps1_);
      break;

    case JPS2_NORMAL:
      vectorOfVectors2MarkerArray(path, &path_jps2_, color(RED_NORMAL));
      pub_path_jps2_.publish(path_jps2_);
      break;
    case JPS_WHOLE:
      vectorOfVectors2MarkerArray(path, &path_jps_whole_, color(GREEN_NORMAL));
      pub_path_jps_whole_.publish(path_jps_whole_);
      break;
    case JPS_SAFE:
      vectorOfVectors2MarkerArray(path, &path_jps_safe_, color(YELLOW_NORMAL));
      pub_path_jps_safe_.publish(path_jps_safe_);
      break;
  }
}

void FasterRos::pubTraj(const std::vector<state>& data, int type)
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

  // std::cout << "here4" << std::endl;

  if (type == WHOLE)
  {
    pub_traj_whole_.publish(traj);
  }

  if (type == SAFE)
  {
    pub_traj_safe_.publish(traj);
  }

  // clearMarkerColoredTraj();
  clearMarkerArray(&traj_committed_colored_, &pub_traj_committed_colored_);
  clearMarkerArray(&traj_whole_colored_, &pub_traj_whole_colored_);
  clearMarkerArray(&traj_safe_colored_, &pub_traj_safe_colored_);

  // std::cout << "here5" << std::endl;

  if (type == COMMITTED_COLORED)
  {
    traj_committed_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max, increm, name_drone_);
    pub_traj_committed_colored_.publish(traj_committed_colored_);
  }

  if (type == WHOLE_COLORED)
  {
    traj_whole_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max, increm, name_drone_);
    pub_traj_whole_colored_.publish(traj_whole_colored_);
  }

  if (type == SAFE_COLORED)
  {
    traj_safe_colored_ = stateVector2ColoredMarkerArray(data, type, par_.v_max, increm, name_drone_);
    pub_traj_safe_colored_.publish(traj_safe_colored_);
  }

  // std::cout << "here6" << std::endl;
}

void FasterRos::pubJPSIntersection(Eigen::Vector3d& inters)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = world_name_;
  p.point = eigen2point(inters);
  pub_jps_inters_.publish(p);
}

void FasterRos::pubActualTraj()
{
  static geometry_msgs::Point p_last = pointOrigin();

  state current_state;
  faster_ptr_->getState(current_state);
  Eigen::Vector3d act_pos = current_state.pos;

  /*  // mtx_G.lock();
    Eigen::Vector3d t_goal = G_;
    // mtx_G.unlock();
    float dist_to_goal = (t_goal - act_pos).norm();

    if (dist_to_goal < 2 * par_.goal_radius)
    {
      return;
    }
  */

  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.id = actual_trajID_;  // % 3000;  // Start the id again after ___ points published (if not RVIZ goes very slow)
  m.ns = "ActualTraj_" + name_drone_;
  actual_trajID_++;
  m.color = getColorJet(current_state.vel.norm(), 0, par_.v_max);  // color(RED_NORMAL);
  m.scale.x = 0.15;
  m.scale.y = 0.0001;
  m.scale.z = 0.0001;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = world_name_;

  // pose is actually not used in the marker, but if not RVIZ complains about the quaternion
  m.pose.position = pointOrigin();
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  geometry_msgs::Point p;
  p = eigen2point(act_pos);
  m.points.push_back(p_last);
  m.points.push_back(p);
  p_last = p;

  if (m.id == 0)
  {
    return;
  }

  pub_actual_traj_.publish(m);
}

/*void FasterRos::pubG(state G)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id =world_name_;
  // mtx_G.lock();
  p.point = eigen2point(G.pos);
  // mtx_G.unlock();
  pub_point_G_.publish(p);
}*/

void FasterRos::clearMarkerActualTraj()
{
  // printf("In clearMarkerActualTraj\n");

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

void FasterRos::clearMarkerColoredTraj()
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 1;
  m.scale.y = 1;
  m.scale.z = 1;
  pub_actual_traj_.publish(m);
  // actual_trajID_ = 0;
}

// Occupied CB
void FasterRos::mapCB(const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_map_ros,
                      const sensor_msgs::PointCloud2::ConstPtr& pcl2ptr_unk_ros)
{
  // Occupied Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_map_ros, *pclptr_map);
  // Unknown Space Point Cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_unk_ros, *pclptr_unk);

  faster_ptr_->updateMap(pclptr_map, pclptr_unk);
}

void FasterRos::pubState(const state& data, const ros::Publisher pub)
{
  geometry_msgs::PointStamped p;
  p.header.frame_id = world_name_;
  p.point = eigen2point(data.pos);
  pub.publish(p);
}

void FasterRos::terminalGoalCB(const geometry_msgs::PoseStamped& msg)
{
  state G_term;
  double z;
  if (fabs(msg.pose.position.z) < 1e-5)  // This happens when you click in RVIZ (msg.z is 0.0)
  {
    z = 1.0;
  }
  else  // This happens when you publish by yourself the goal (should always be above the ground)
  {
    z = msg.pose.position.z;
  }
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, z);
  faster_ptr_->setTerminalGoal(G_term);

  state G;  // projected goal
  faster_ptr_->getG(G);

  pubState(G_term, pub_point_G_term_);
  pubState(G, pub_point_G_);

  clearMarkerActualTraj();
  // std::cout << "Exiting from goalCB\n";
}

// Odometry Callback (for the Jackal)
/*void FasterRos::odomCB(const nav_msgs::Odometry& msg)
{
  // ROS_ERROR("In state CB");
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x, msg.vel.y,
  //       msg.vel.z);

  mtx_state.lock();

  state state_;
  state_.setPos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  state_.setPos(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  state_.setAccel(0.0, 0.0, 0.0);

  double roll, pitch, yaw;
  quaternion2Euler(msg.pose.pose.orientation, roll, pitch, yaw);

  if (state_initialized_ == false)
  {
    quadGoal_.pos.x = msg.pose.pose.position.x;
    quadGoal_.pos.y = msg.pose.pose.position.y;
    quadGoal_.pos.z = msg.pose.pose.position.z;

    quadGoal_.vel.x = msg.twist.twist.linear.x;
    quadGoal_.vel.y = msg.twist.twist.linear.y;
    quadGoal_.vel.z = msg.twist.twist.linear.z;

    quadGoal_.yaw = yaw;
  }

  state_initialized_ = true;
  // printf("(State): %0.2f  %0.2f  %0.2f %0.2f  %0.2f  %0.2f\n", msg.pos.x, msg.pos.y, msg.pos.z, msg.vel.x,
  //     msg.vel.y, msg.vel.z);

  std::cout << bold << red << "IN ODOM CB:" << msg.pose.pose.orientation << reset << std::endl;
  std::cout << bold << red << "Yaw=" << yaw * 180 / 3.14 << reset << std::endl;
  current_yaw_ = yaw;

  mtx_state.unlock();
  // Stop updating when we get GO
  if (flight_mode_.mode == flight_mode_.NOT_FLYING || flight_mode_.mode == flight_mode_.KILL)
  {
    quadGoal_.pos.x = msg.pose.pose.position.x;
    quadGoal_.pos.y = msg.pose.pose.position.y;
    quadGoal_.pos.z = msg.pose.pose.position.z;

    quadGoal_.vel.x = msg.twist.twist.linear.x;
    quadGoal_.vel.y = msg.twist.twist.linear.y;
    quadGoal_.vel.z = msg.twist.twist.linear.z;

    double roll, pitch, yaw;
    quaternion2Euler(msg.pose.pose.orientation, roll, pitch, yaw);
    current_yaw_ = yaw;
    quadGoal_.yaw = yaw;
    z_start_ = msg.pose.pose.position.z;
    z_start_ = std::max(0.0, z_start_);
    mtx_initial_cond.lock();
    stateA_.setPos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    mtx_initial_cond.unlock();
  }

  static int i = 0;
  i++;

  if (status_ != GOAL_REACHED && par_.visual == true)
  {
    pubActualTraj();
  }

  if (i % 10 == 0 && status_ != GOAL_REACHED && i != 0)
  {
    Eigen::Vector3d actual_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    // Don't use the state to compute the total distance (it's very noisy)
    // log_.total_dist = log_.total_dist + (actual_pos - pos_old_).norm();
    // pos_old_ = actual_pos;
  }
  Eigen::Vector3d vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
  log_.veloc_norm = vel.norm();
}*/