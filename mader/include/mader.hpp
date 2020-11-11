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
#include "solver_nlopt.hpp"
#include "solver_gurobi.hpp"

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
  Mader(parameters par);
  bool replan(mader_types::Edges& edges_obstacles_out, std::vector<state>& X_safe_out,
              std::vector<Hyperplane3D>& planes_guesses, int& num_of_LPs_run, int& num_of_QCQPs_run,
              PieceWisePol& pwp_out);
  void updateState(state data);

  bool getNextGoal(state& next_goal);
  void getState(state& data);
  void getG(state& G);
  void setTerminalGoal(state& term_goal);
  void resetInitialization();

  bool IsTranslating();
  void updateTrajObstacles(dynTraj traj);

private:
  state M_;
  committedTrajectory plan_;

  double previous_yaw_ = 0.0;

  void dynTraj2dynTrajCompiled(const dynTraj& traj, dynTrajCompiled& traj_compiled);

  bool initializedStateAndTermGoal();

  bool safetyCheckAfterOpt(PieceWisePol pwp_optimized);

  bool trajsAndPwpAreInCollision(dynTrajCompiled traj, PieceWisePol pwp_optimized, double t_start, double t_end);

  void removeTrajsThatWillNotAffectMe(const state& A, double t_start, double t_end);

  /*  vec_E<Polyhedron<3>> vectorGCALPol2vectorJPSPol(ConvexHullsOfCurves& convex_hulls_of_curves);
    ConvexHullsOfCurves_Std vectorGCALPol2vectorStdEigen(ConvexHullsOfCurves& convexHulls);*/
  ConvexHullsOfCurves convexHullsOfCurves(double t_start, double t_end);
  ConvexHullsOfCurve convexHullsOfCurve(dynTrajCompiled& traj, double t_start, double t_end);
  CGAL_Polyhedron_3 convexHullOfInterval(dynTrajCompiled& traj, double t_start, double t_end);

  std::vector<Eigen::Vector3d> vertexesOfInterval(PieceWisePol& pwp, double t_start, double t_end,
                                                  const Eigen::Vector3d& delta_inflation);
  std::vector<Eigen::Vector3d> vertexesOfInterval(dynTrajCompiled& traj, double t_start, double t_end);
  void yaw(double diff, state& next_goal);

  void getDesiredYaw(state& next_goal);

  void updateInitialCond(int i);

  void changeDroneStatus(int new_status);

  bool appendToPlan(int k_end_whole, const std::vector<state>& whole, int k_safe, const std::vector<state>& safe);

  bool initialized();
  bool initializedAllExceptPlanner();

  void print_status();

  parameters par_;

  double t_;  // variable where the expressions of the trajs of the dyn obs are evaluated

  std::mutex mtx_trajs_;
  std::vector<dynTrajCompiled> trajs_;

  bool state_initialized_ = false;
  bool planner_initialized_ = false;

  int deltaT_ = 75;

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

  state stateA_;  // It's the initial condition for the solver

  state state_;
  state G_;       // This goal is always inside of the map
  state G_term_;  // This goal is the clicked goal

  int solutions_found_ = 0;
  int total_replannings_ = 0;

  PieceWisePol pwp_prev_;

  bool exists_previous_pwp_ = false;

  bool started_check_ = false;

  bool have_received_trajectories_while_checking_ = false;

  double time_init_opt_;

  double av_improvement_nlopt_ = 0.0;

  // SolverNlopt* solver_;  // pointer to the optimization solver
  SolverGurobi* solver_;  // pointer to the optimization solver

  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_;
  Eigen::Matrix<double, 4, 4> A_rest_pos_basis_inverse_;

  separator::Separator* separator_solver_;
};

#endif