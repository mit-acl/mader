#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include "faster_types.hpp"

void CPs2TrajAndPwp(std::vector<Eigen::Vector3d> &q, std::vector<state> &traj, PieceWisePol &solution_, int N, int p,
                    int num_pol, Eigen::RowVectorXd &knots, double dc);

Eigen::Spline3d findInterpolatingBsplineNormalized(const std::vector<double> &times,
                                                   const std::vector<Eigen::Vector3d> &positions);