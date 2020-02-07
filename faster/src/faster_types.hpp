#pragma once

#include <iostream>
#include <iomanip>  // std::setprecision
#include <deque>
#include "exprtk.hpp"
#include "termcolor.hpp"

typedef std::vector<Eigen::Vector3d> Polyhedron_Std;
typedef std::vector<Polyhedron_Std> ConvexHullsOfCurve_Std;
typedef std::vector<ConvexHullsOfCurve_Std> ConvexHullsOfCurves_Std;

struct dynTraj
{
  std::vector<std::string> function;
  std::vector<double> bbox;
  int id;
};

struct dynTrajCompiled
{
  std::vector<exprtk::expression<double>> function;
  std::vector<double> bbox;
  int id;
};

struct polytope
{
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
};

struct parameters
{
  bool use_ff;
  bool visual;

  double dc;
  double goal_radius;
  double drone_radius;

  int N_whole;
  int N_safe;

  double Ra;
  double w_max;
  double alpha_filter_dyaw;

  double z_ground;
  double z_max;
  double inflation_jps;
  double factor_jps;

  double v_max;
  double a_max;
  double j_max;

  double gamma_whole;
  double gammap_whole;
  double increment_whole;
  double gamma_safe;
  double gammap_safe;
  double increment_safe;

  double alpha;

  double delta_a;
  double delta_H;

  int max_poly_whole;
  int max_poly_safe;
  double dist_max_vertexes;

  int gurobi_threads;
  int gurobi_verbose;

  bool use_faster;

  double wdx;
  double wdy;
  double wdz;
  double res;

  int n_pol;
  int deg;
  int samples_per_interval;
  double weight;
  double epsilon_tol_constraints;
  std::string solver;

  /*  double kw;
    double kyaw;
    double kdalpha;
    double kv;
    double kdist;
    double kalpha;*/
};

struct state
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();

  double yaw = 0;
  double dyaw = 0;

  void setPos(const double x, const double y, const double z)
  {
    pos << x, y, z;
  }
  void setVel(const double x, const double y, const double z)
  {
    vel << x, y, z;
  }
  void setAccel(const double x, const double y, const double z)
  {
    accel << x, y, z;
  }

  void setJerk(const double x, const double y, const double z)
  {
    jerk << x, y, z;
  }

  void setPos(const Eigen::Vector3d& data)
  {
    pos << data.x(), data.y(), data.z();
  }

  void setVel(const Eigen::Vector3d& data)
  {
    vel << data.x(), data.y(), data.z();
  }

  void setAccel(const Eigen::Vector3d& data)
  {
    accel << data.x(), data.y(), data.z();
  }

  void setJerk(const Eigen::Vector3d& data)
  {
    jerk << data.x(), data.y(), data.z();
  }

  void setState(const Eigen::Matrix<double, 9, 1>& data)
  {
    pos << data(0, 0), data(1, 0), data(2, 0);
    vel << data(3, 0), data(4, 0), data(5, 0);
    accel << data(6, 0), data(7, 0), data(8, 0);
  }

  void setYaw(const double& data)
  {
    yaw = data;
  }
  void setZero()
  {
    pos = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    accel = Eigen::Vector3d::Zero();
    jerk = Eigen::Vector3d::Zero();
    yaw = 0;
    dyaw = 0;
  }

  void printPos()
  {
    std::cout << "Pos= " << pos.transpose() << std::endl;
  }

  void print()
  {
    std::cout << std::setprecision(3) << "Pos= " << pos.transpose() << std::endl;
    std::cout << std::setprecision(3) << "Vel= " << vel.transpose() << std::endl;
    std::cout << std::setprecision(3) << "Accel= " << accel.transpose() << std::endl;
  }

  void printHorizontal()
  {
    using namespace termcolor;
    std::cout << std::setprecision(3) << "Pos, Vel, Accel, Jerk= " << red << pos.transpose() << reset;
    std::cout << " " << blue << vel.transpose() << reset;
    std::cout << " " << green << accel.transpose() << reset;
    std::cout << " " << jerk.transpose() << std::endl;
  }
};

struct trajectory
{
  std::deque<state> content;

  void print()
  {
    for (auto state_i : content)
    {
      state_i.printHorizontal();
    }
  }

  // now define the functions operating on the member of this struct directly
  int size()
  {
    return content.size();
  }

  void push_back(state tmp)
  {
    content.push_back(tmp);
  }

  void erase(std::deque<state>::iterator a, std::deque<state>::iterator b)
  {
    content.erase(a, b);
  }

  state front()
  {
    return content.front();
  }

  state back()
  {
    return content.back();
  }

  void pop_front()
  {
    content.pop_front();
  }

  std::deque<state>::iterator end()
  {
    return content.end();
  }

  std::deque<state>::iterator begin()
  {
    return content.begin();
  }

  state get(int i)
  {
    return content[i];
  }

  std::vector<state> toStdVector()
  {
    std::vector<state> my_vector;
    std::copy(content.begin(), content.end(), std::back_inserter(my_vector));
    return my_vector;
  }
};