
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

void SolverGurobi::printQND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  std::cout << std::setprecision(5) << std::endl;
  std::cout << "   control points:" << std::endl;
  for (Eigen::Vector3d q_i : q)
  {
    std::cout << q_i.transpose() << std::endl;
  }
  std::cout << "   normals:" << std::endl;
  for (Eigen::Vector3d n_i : n)
  {
    std::cout << n_i.transpose() << std::endl;
  }
  std::cout << "   d coeffs:" << std::endl;
  for (double d_i : d)
  {
    std::cout << d_i << std::endl;
  }
  std::cout << reset << std::endl;
}

void SolverGurobi::transformPosBSpline2otherBasis(const GRBMatrix &Qbs, GRBMatrix &Qmv, int interval)
{
  Qmv = matrixMultiply(Qbs, eigenMatrix2std(M_pos_bs2basis_[interval]));
  // Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverGurobi::transformVelBSpline2otherBasis(const GRBMatrix &Qbs, GRBMatrix &Qmv, int interval)
{
  Qmv = matrixMultiply(Qbs, eigenMatrix2std(M_vel_bs2basis_[interval]));
  // Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverGurobi::transformPosBSpline2otherBasis(const Eigen::Matrix<double, 3, 4> &Qbs,
                                                  Eigen::Matrix<double, 3, 4> &Qmv, int interval)
{
  Qmv = Qbs * M_pos_bs2basis_[interval];
}

void SolverGurobi::transformVelBSpline2otherBasis(const Eigen::Matrix<double, 3, 3> &Qbs,
                                                  Eigen::Matrix<double, 3, 3> &Qmv, int interval)
{
  Qmv = Qbs * M_vel_bs2basis_[interval];
}

void SolverGurobi::saturateQ(std::vector<Eigen::Vector3d> &q)
{
  for (int i = 0; i < q.size(); i++)
  {
    q[i].z() = std::max(q[i].z(), par_.z_min);  // Make sure it's within the limits
    q[i].z() = std::min(q[i].z(), par_.z_max);  // Make sure it's within the limits
  }
}

// void SolverGurobi::printIndexesVariables()
// {
//   std::cout << "_______________________" << std::endl;
//   std::cout << "Indexes variables" << std::endl;
//   std::cout << "q: " << i_min_ << "-->" << i_max_ << std::endl;
//   std::cout << "n: " << j_min_ << "-->" << j_max_ << std::endl;
//   std::cout << "d: " << k_min_ << "-->" << k_max_ << std::endl;

//   std::cout << "Total number of Variables: " << num_of_variables_ << std::endl;
//   std::cout << "_______________________" << std::endl;
// }

// void SolverGurobi::printIndexesConstraints()
// {
//   std::cout << "_______________________" << std::endl;
//   std::cout << "Indexes constraints" << std::endl;
//   std::cout << "Obstacles: " << index_const_obs_ << "-->" << index_const_vel_ - 1 << std::endl;
//   std::cout << "Velocity: " << index_const_vel_ << "-->" << index_const_accel_ - 1 << std::endl;
//   std::cout << "Accel: " << index_const_accel_ << "-->" << index_const_normals_ - 1 << std::endl;
//   // std::cout << "Normals: " << index_const_normals_ << "-->" << num_of_constraints_ << std::endl;
//   std::cout << "Total number of Constraints: " << num_of_constraints_ << std::endl;
//   std::cout << "_______________________" << std::endl;
// }

void SolverGurobi::findCentroidHull(const mt::Polyhedron_Std &hull, Eigen::Vector3d &centroid)
{
  centroid = Eigen::Vector3d::Zero();

  for (int i = 0; i < hull.cols(); i++)
  {
    centroid += hull.col(i);
  }
  if (hull.cols() > 0)
  {
    centroid = centroid / hull.cols();
  }
}