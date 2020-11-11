
#include "solver_gurobi.hpp"
#include "termcolor.hpp"
#include "bspline_utils.hpp"
#include "ros/ros.h"
#include "solver_gurobi_utils.hpp"

#include <decomp_util/ellipsoid_decomp.h>  //For Polyhedron definition
#include <unsupported/Eigen/Splines>
#include <iostream>
#include <list>
#include <random>
#include <iostream>
#include <vector>

using namespace termcolor;

bool SolverGurobi::generateAStarGuess()
{
  std::cout << "[NL] Running A* from" << q0_.transpose() << " to " << final_state_.pos.transpose()
            << ", allowing time = " << kappa_ * max_runtime_ * 1000 << " ms" << std::endl;

  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();
  planes_.clear();

  generateStraightLineGuess();  // If A* doesn't succeed --> use straight lineGuess
  /*  generateRandomN(n_guess_);
    generateRandomD(d_guess_);
    generateRandomQ(q_guess_);*/

  // std::cout << "The StraightLineGuess is" << std::endl;
  // printStd(q_guess_);

  myAStarSolver_->setUp(t_init_, t_final_, hulls_);

  myAStarSolver_->setq0q1q2(q0_, q1_, q2_);
  myAStarSolver_->setGoal(final_state_.pos);

  double goal_size = 0.05;  //[meters]

  myAStarSolver_->setXYZMinMaxAndRa(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_,
                                    Ra_);                 // limits for the search, in world frame
  myAStarSolver_->setBBoxSearch(2000.0, 2000.0, 2000.0);  // limits for the search, centered on q2
  myAStarSolver_->setMaxValuesAndSamples(v_max_, a_max_, a_star_samp_x_, a_star_samp_y_, a_star_samp_z_,
                                         a_star_fraction_voxel_size_);

  myAStarSolver_->setRunTime(kappa_ * max_runtime_);  // hack, should be kappa_ * max_runtime_
  myAStarSolver_->setGoalSize(goal_size);
  myAStarSolver_->setBias(a_star_bias_);
  myAStarSolver_->setVisual(false);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  bool is_feasible = myAStarSolver_->run(q, n, d);

  num_of_LPs_run_ = myAStarSolver_->getNumOfLPsRun();

  // fillPlanesFromNDQ(n_guess_, d_guess_, q_guess_);

  if (is_feasible)
  {
    ROS_INFO_STREAM("[NL] A* found a feasible solution!");
    q_guess_ = q;
    n_guess_ = n;
    d_guess_ = d;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[NL] A* didn't find a feasible solution, using straight line guess");
    return false;
  }
}

void SolverGurobi::generateRandomD(std::vector<double> &d)
{
  d.clear();
  for (int k = k_min_; k <= k_max_; k++)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    d.push_back(r1);
  }
}

void SolverGurobi::generateRandomN(std::vector<Eigen::Vector3d> &n)
{
  n.clear();
  for (int j = j_min_; j < j_max_; j = j + 3)
  {
    double r1 = ((double)rand() / (RAND_MAX));
    double r2 = ((double)rand() / (RAND_MAX));
    double r3 = ((double)rand() / (RAND_MAX));
    n.push_back(Eigen::Vector3d(r1, r2, r3));
  }

  // std::cout << "After Generating RandomN, n has size= " << n.size() << std::endl;
}

void SolverGurobi::generateRandomQ(std::vector<Eigen::Vector3d> &q)
{
  q.clear();

  std::default_random_engine generator;
  generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist_x(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_y(0, 1);  // TODO
  std::uniform_real_distribution<double> dist_z(z_min_, z_max_);

  for (int i = 0; i <= N_; i++)
  {
    q.push_back(Eigen::Vector3d(dist_x(generator), dist_y(generator), dist_z(generator)));
  }

  saturateQ(q);  // make sure is inside the bounds specified
}

void SolverGurobi::generateRandomGuess()
{
  n_guess_.clear();
  q_guess_.clear();
  d_guess_.clear();

  generateRandomN(n_guess_);
  generateRandomD(d_guess_);
  generateRandomQ(q_guess_);
}

void SolverGurobi::generateStraightLineGuess()
{
  // std::cout << "Using StraightLineGuess" << std::endl;
  q_guess_.clear();
  n_guess_.clear();
  d_guess_.clear();

  q_guess_.push_back(q0_);  // Not a decision variable
  q_guess_.push_back(q1_);  // Not a decision variable
  q_guess_.push_back(q2_);  // Not a decision variable

  for (int i = 1; i < (N_ - 2 - 2); i++)
  {
    Eigen::Vector3d q_i = q2_ + i * (final_state_.pos - q2_) / (N_ - 2 - 2);
    q_guess_.push_back(q_i);
  }

  q_guess_.push_back(qNm2_);  // three last cps are the same because of the vel/accel final conditions
  q_guess_.push_back(qNm1_);
  q_guess_.push_back(qN_);
  // Now q_guess_ should have (N_+1) elements
  saturateQ(q_guess_);  // make sure is inside the bounds specified

  // std::vector<Eigen::Vector3d> q_guess_with_qNm1N = q_guess_;
  // q_guess_with_qNm1N.push_back(qNm1_);
  // q_guess_with_qNm1N.push_back(qN_);
  //////////////////////

  for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  {
    for (int i = 0; i < num_of_segments_; i++)
    {
      std::vector<Eigen::Vector3d> last4Cps(4);

      Eigen::Matrix<double, 3, 4> Qbs;  // b-spline
      Eigen::Matrix<double, 3, 4> Qmv;  // minvo. each column contains a MINVO control point
      Qbs.col(0) = q_guess_[i];
      Qbs.col(1) = q_guess_[i + 1];
      Qbs.col(2) = q_guess_[i + 2];
      Qbs.col(3) = q_guess_[i + 3];

      transformPosBSpline2otherBasis(Qbs, Qmv, i);

      Eigen::Vector3d n_i;
      double d_i;

      bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, hulls_[obst_index][i], Qmv);

      n_guess_.push_back(n_i);
      d_guess_.push_back(d_i);
    }
  }
}