/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef MADER_HPP
#define MADER_HPP

#include <vector>
#include "cgal_utils.hpp"

#include <mutex>

#include "mader_types.hpp"

#if USE_GUROBI_FLAG
#include "solver_gurobi.hpp"
#else
#include "solver_nlopt.hpp"
#endif

// status_ : YAWING-->TRAVELING-->GOAL_SEEN-->GOAL_REACHED-->YAWING-->TRAVELING-->...

enum DroneStatus
{
  YAWING = 0,
  TRAVELING = 1,
  GOAL_SEEN = 2,
  GOAL_REACHED = 3
};

enum PlannerStatus
{
  FIRST_PLAN = 0,
  START_REPLANNING = 1,
  REPLANNED = 2
};

using namespace termcolor;

class Mader
{
public:
  Mader(mt::parameters par);
  bool replan(mt::Edges& edges_obstacles_out, std::vector<mt::state>& X_safe_out, std::vector<Hyperplane3D>& planes,
              int& num_of_LPs_run, int& num_of_QCQPs_run, mt::PieceWisePol& pwp_out);
  bool replan_with_delaycheck(mt::Edges& edges_obstacles_out, std::vector<mt::state>& headsup_plan,
                              std::vector<Hyperplane3D>& planes, int& num_of_LPs_run, int& num_of_QCQPs_run,
                              mt::PieceWisePol& pwp_now, double& headsup_time);
  bool addTrajToPlan_with_delaycheck(mt::PieceWisePol& pwp);
  bool delayCheck(mt::PieceWisePol pwp_optimized, const double& headsup_time);

  void updateState(mt::state data);

  bool getNextGoal(mt::state& next_goal);
  void getState(mt::state& data);
  void getG(mt::state& G);
  void getDetourG(mt::state& G);
  void moveAtowardG(mt::state& A, mt::state& G);
  void setTerminalGoal(mt::state& term_goal);
  void resetInitialization();

  bool IsTranslating();
  void updateTrajObstacles_with_delaycheck(mt::dynTraj traj);
  void updateTrajObstacles(mt::dynTraj traj);

  Eigen::Vector2d RotationMatrix(Eigen::Vector2d& vec, const double& angle);
  void getID(int& id);
  mt::state moveAoutOfBbox(const mt::state& A);
  int getMissedMsgsCnt();
  mt::state getGterm();
  bool isGoalSeen();
  bool isGoalReached();

  std::vector<mt::dynTrajCompiled> getTrajs();

private:
  mt::state M_;
  mt::committedTrajectory plan_;

  double previous_yaw_ = 0.0;
  double initial_yaw_ = 0.0;

  bool isReplanningNeeded();

  void dynTraj2dynTrajCompiled(const mt::dynTraj& traj, mt::dynTrajCompiled& traj_compiled);

  bool initializedStateAndTermGoal();

  bool safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized, bool& is_q0_fail);
  bool safetyCheckAfterOpt(mt::PieceWisePol pwp_optimized);

  bool safetyCheck_for_A_star_failure(mt::PieceWisePol pwp_prev);
  bool safetyCheck_for_A_star_failure_pwp_now(mt::PieceWisePol pwp_now);

  bool trajsAndPwpAreInCollision_with_inflation(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized,
                                                double t_start, double t_end, bool& is_q0_fail);

  bool trajsAndPwpAreInCollision(mt::dynTrajCompiled traj, mt::PieceWisePol pwp_optimized, double t_start,
                                 double t_end);

  void removeTrajsThatWillNotAffectMe(const mt::state& A, double t_start, double t_end);

  /*  vec_E<Polyhedron<3>> cu::vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves);
    mt::ConvexHullsOfCurves_Std cu::vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls);*/
  ConvexHullsOfCurves convexHullsOfCurves(double t_start, double t_end);
  ConvexHullsOfCurve convexHullsOfCurve(mt::dynTrajCompiled& traj, double t_start, double t_end);
  CGAL_Polyhedron_3 convexHullOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end);

  std::vector<Eigen::Vector3d> vertexesOfInterval(mt::PieceWisePol& pwp, double t_start, double t_end,
                                                  const Eigen::Vector3d& delta_inflation);
  std::vector<Eigen::Vector3d> vertexesOfInterval(mt::dynTrajCompiled& traj, double t_start, double t_end);
  void yaw(double diff, mt::state& next_goal);

  void changeBBox(Eigen::Vector3d& drone_boundarybox);

  void getDesiredYaw(mt::state& next_goal);

  void updateInitialCond(int i);

  void changeDroneStatus(int new_status);

  bool appendToPlan(int k_end_whole, const std::vector<mt::state>& whole, int k_safe,
                    const std::vector<mt::state>& safe);

  bool initialized();
  bool initializedAllExceptPlanner();

  void printDroneStatus();

  mt::parameters par_;

  double t_;  // variable where the expressions of the trajs of the dyn obs are evaluated

  std::mutex mtx_trajs_;
  std::vector<mt::dynTrajCompiled> trajs_;

  bool state_initialized_ = false;
  bool planner_initialized_ = false;

  int deltaT_ = 75;

  int k_index_end_ = 0;

  double time_now_ = 0;

  bool terminal_goal_initialized_ = false;

  int drone_status_ = DroneStatus::TRAVELING;  // status_ can be TRAVELING, GOAL_SEEN, GOAL_REACHED
  int planner_status_ = PlannerStatus::FIRST_PLAN;

  double dyaw_filtered_ = 0;

  std::mutex mtx_goals;

  std::mutex mtx_k;

  std::mutex mtx_planner_status_;
  std::mutex mtx_initial_cond;
  std::mutex mtx_state;
  std::mutex mtx_offsets;
  std::mutex mtx_plan_;
  // std::mutex mtx_factors;

  std::mutex mtx_G;
  std::mutex mtx_G_term;
  std::mutex mtx_t_;

  std::mutex mtx_hrtwch_;

  mt::state stateA_;  // It's the initial condition for the solver

  mt::state state_;
  mt::state stuck_state_;
  mt::state stuck_state_for_bbox_;
  mt::state pop_up_state_;
  mt::state pop_up_last_state_in_plan_;
  mt::state G_;       // This goal is always inside of the map
  mt::state G_term_;  // This goal is the clicked goal
  mt::state detoured_G_;

  int solutions_found_ = 0;
  int total_replannings_ = 0;

  mt::PieceWisePol pwp_prev_;

  bool exists_previous_pwp_ = false;

  bool started_check_ = false;

  bool have_received_trajectories_while_checking_ = false;

  double time_init_opt_;

  bool if_detour_ = false;

  MyTimer timer_detour_;

  mt::state A_when_stuck_;
  mt::state G_when_stuck_;

  bool if_bbox_change_ = false;

  MyTimer timer_bbox_;

  int stuck_count_for_bbox_ = 0;
  int stuck_count_for_detour_ = 0;

  bool if_A_moveback_ = false;

  bool is_pwp_prev_feasible_ = false;

  bool is_pop_up_ = false;

  int id_;

  bool is_z_max_increased_ = false;

  bool is_going_back_to_normal_z_max_ = false;

  int A_star_fail_count_ = 0;

  int A_star_fail_count_pwp_now_ = 0;

  bool is_pop_up_initialized_ = false;

  bool is_A_star_failed_30_ = false;

  int q0_fail_count_ = 0;
  int opt_fail_count_ = 0;
  int safetycheck_fail_count_ = 0;

  bool is_movingAoutOfBbox_ = false;
  mt::state movedA_;

  // double av_improvement_nlopt_ = 0.0;

#if USE_GUROBI_FLAG
  std::unique_ptr<SolverGurobi> solver_;  // pointer to the optimization solver
#else
  std::unique_ptr<SolverNlopt> solver_;  // pointer to the optimization solver
#endif

  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_;
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_inverse_;

  separator::Separator* separator_solver_;
};

#endif