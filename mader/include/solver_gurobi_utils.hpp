/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#ifndef SOLVER_GUROBI_UTILS_HPP
#define SOLVER_GUROBI_UTILS_HPP

#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
// using namespace std;

// custom typedefs
typedef std::vector<GRBLinExpr> GRBVector;
typedef std::vector<std::vector<GRBLinExpr>> GRBMatrix;

inline double minPositiveElement(std::vector<double> v)
{
  std::sort(v.begin(), v.end());  // sorted in ascending order
  double min_value = 0;
  for (int i = 0; i < v.size(); i++)
  {
    if (v[i] > 0)
    {
      min_value = v[i];
      break;
    }
  }
  return min_value;
}

template <typename T>
GRBQuadExpr getNorm2(const std::vector<T>& x)  // Return the squared norm of a vector
{
  GRBQuadExpr result = 0;
  for (int i = 0; i < x.size(); i++)
  {
    result = result + x[i] * x[i];
  }
  return result;
}

inline void addVectorEqConstraint(GRBModel& m, const GRBVector a, const Eigen::Vector3d& b)
{
  for (int i = 0; i < a.size(); i++)
  {
    m.addConstr(a[i] == b[i]);
  }
}

inline void addVectorLessEqualConstraint(GRBModel& m, const GRBVector a, const Eigen::Vector3d& b)
{
  for (int i = 0; i < a.size(); i++)
  {
    m.addConstr(a[i] <= b[i]);
  }
}

inline void addVectorGreaterEqualConstraint(GRBModel& m, const GRBVector a, const Eigen::Vector3d& b)
{
  for (int i = 0; i < a.size(); i++)
  {
    m.addConstr(a[i] >= b[i]);
  }
}

inline void resetCompleteModel(GRBModel& m)
{
  GRBConstr* c = 0;
  c = m.getConstrs();
  for (int i = 0; i < m.get(GRB_IntAttr_NumConstrs); ++i)
  {
    m.remove(c[i]);
  }

  GRBQConstr* cq = 0;
  cq = m.getQConstrs();
  for (int i = 0; i < m.get(GRB_IntAttr_NumQConstrs); ++i)
  {
    m.remove(cq[i]);
  }

  GRBGenConstr* gc = 0;
  gc = m.getGenConstrs();
  for (int i = 0; i < m.get(GRB_IntAttr_NumGenConstrs); ++i)
  {
    m.remove(gc[i]);
  }

  GRBVar* vars = 0;
  vars = m.getVars();
  for (int i = 0; i < m.get(GRB_IntAttr_NumVars); ++i)
  {
    m.remove(vars[i]);
  }

  m.reset();  // Note that this function, only by itself, does NOT remove vars or constraints
}

// See https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html#sec:StatusCodes
inline void printGurobiStatus(int status)
{
  switch (status)
  {
    case GRB_LOADED:
      std::cout << "GUROBI Status: GRB_LOADED" << std::endl;
      break;
    case GRB_OPTIMAL:
      std::cout << "GUROBI Status: GRB_OPTIMAL" << std::endl;
      break;
    case GRB_INFEASIBLE:
      std::cout << "GUROBI Status: GRB_INFEASIBLE" << std::endl;
      break;
    case GRB_INF_OR_UNBD:
      std::cout << "GUROBI Status: GRB_INF_OR_UNBD" << std::endl;
      break;
    case GRB_UNBOUNDED:
      std::cout << "GUROBI Status: GRB_UNBOUNDED" << std::endl;
      break;
    case GRB_CUTOFF:
      std::cout << "GUROBI Status: GRB_CUTOFF" << std::endl;
      break;
    case GRB_ITERATION_LIMIT:
      std::cout << "GUROBI Status: GRB_ITERATION_LIMIT" << std::endl;
      break;
    case GRB_NODE_LIMIT:
      std::cout << "GUROBI Status: GRB_NODE_LIMIT" << std::endl;
      break;
    case GRB_TIME_LIMIT:
      std::cout << "GUROBI Status: GRB_TIME_LIMIT" << std::endl;
      break;
    case GRB_SOLUTION_LIMIT:
      std::cout << "GUROBI Status: GRB_SOLUTION_LIMIT" << std::endl;
      break;
    case GRB_INTERRUPTED:
      std::cout << "GUROBI Status: GRB_INTERRUPTED" << std::endl;
      break;
    case GRB_NUMERIC:
      std::cout << "GUROBI Status: GRB_NUMERIC" << std::endl;
      break;
    case GRB_SUBOPTIMAL:
      std::cout << "GUROBI Status: GRB_SUBOPTIMAL" << std::endl;
      break;
    case GRB_INPROGRESS:
      std::cout << "GUROBI Status: GRB_INPROGRESS" << std::endl;
      break;
    case GRB_USER_OBJ_LIMIT:
      std::cout << "GUROBI Status: GRB_USER_OBJ_LIMIT" << std::endl;
      break;
    default:
      std::cout << "GUROBI Status Code=: " << status << std::endl;
  }
}

template <typename T, typename R>
GRBVector matrixMultiply(const std::vector<std::vector<R>>& A, const std::vector<T>& x)
{
  GRBVector result;

  for (int i = 0; i < A.size(); i++)
  {
    GRBLinExpr lin_exp = 0;
    for (int m = 0; m < x.size(); m++)
    {
      lin_exp = lin_exp + A[i][m] * x[m];
    }
    result.push_back(lin_exp);
  }
  return result;
}

template <typename T>
std::vector<GRBVector> matrixMultiply(const std::vector<std::vector<T>>& A, const std::vector<std::vector<double>>& B)
{
  std::vector<GRBVector> result(A.size(), GRBVector(B[0].size(), 0.0));  // Initialize all the
                                                                         // elements to zero

  for (int i = 0; i < A.size(); i++)  // multiply row if of A
  {
    for (int j = 0; j < B[0].size(); j++)  // times column j of B
    {
      GRBLinExpr lin_exp = 0;
      for (int m = 0; m < B.size(); m++)
      {
        lin_exp += A[i][m] * B[m][j];
      }
      result[i][j] = lin_exp;
    }
  }
  return result;
}

template <typename T>  // Overload + to sum Elementwise std::vectors
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
  assert(a.size() == b.size());

  std::vector<T> result;
  result.reserve(a.size());

  std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result), std::plus<T>());
  return result;
}

template <typename T>  // Overload - to substract Elementwise std::vectors
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b)
{
  assert(a.size() == b.size());

  std::vector<T> result;
  result.reserve(a.size());

  std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(result), std::minus<T>());
  return result;
}

template <typename T>  // Overload *
std::vector<T> operator*(const double& a, const std::vector<T>& b)
{
  std::vector<T> result;

  for (int i = 0; i < b.size(); i++)
  {
    result.push_back(a * b[i]);
  }

  return result;
}

template <typename T>
GRBVector operator-(const std::vector<T>& x, const std::vector<double>& b)
{
  GRBVector result;
  for (int i = 0; i < x.size(); i++)
  {
    GRBLinExpr tmp = x[i] - b[i];
    result.push_back(tmp);
  }
  return result;
}

template <typename T>
std::vector<T> eigenVector2std(const Eigen::Matrix<T, -1, 1>& x)
{
  std::vector<T> result;
  for (int i = 0; i < x.rows(); i++)
  {
    result.push_back(x(i, 1));
  }
  return result;
}

template <typename T>
std::vector<T> eigenVector2std(const Eigen::Matrix<T, 3, 1>& x)  // TODO: Merge with the previous one?
{
  std::vector<T> result;
  for (int i = 0; i < x.rows(); i++)
  {
    result.push_back(x(i, 1));
  }
  return result;
}

inline std::vector<std::vector<double>> eigenMatrix2std(const Eigen::Matrix<double, -1, -1>& x)
{
  std::vector<std::vector<double>> result;

  for (int i = 0; i < x.rows(); i++)
  {
    std::vector<double> row;
    for (int j = 0; j < x.cols(); j++)
    {
      row.push_back(x(i, j));
    }
    result.push_back(row);
  }
  return result;
}

template <typename T>
std::vector<T> getColumn(std::vector<std::vector<T>> x, int column)
{
  std::vector<T> result;

  for (int i = 0; i < x.size(); i++)
  {
    result.push_back(x[i][column]);
  }
  return result;
}

// // GRBLinExpr novale = A[i][m] * B[m][j];
// // std::cout << "B[m][j]=" << B[m][j] << std::endl;
// // std::cout << "novale.size() =" << novale.size() << std::endl;

// // std::cout << "novale.getCoeff(0) =" << novale.getCoeff(0) << std::endl;
// // std::cout << "novale.getCoeff(0) =" << novale.getCoeff(1) << std::endl;

// std::vector<std::vector<double>> matrixMultiply(const std::vector<std::vector<double>>& A,
//                                                 const std::vector<std::vector<double>>& B)
// {
//   std::vector<std::vector<double>> result(A.size(), std::vector<double>(B[0].size(), 0.0));  // Initialize all the
//                                                                                              // elements to zero

//   for (int i = 0; i < A.size(); i++)  // multiply row if of A
//   {
//     for (int j = 0; j < B[0].size(); j++)  // times column j of B
//     {
//       double lin_exp = 0;
//       for (int m = 0; m < B.size(); m++)
//       {
//         lin_exp+ = + A[i][m] * B[m][j];
//       }
//       result[i][j] = lin_exp;
//     }
//   }
//   return result;
// }

// std::vector<std::vector<double>> tmp;

// std::vector<double> row1;
// row1.push_back(2.0);
// row1.push_back(1.0);

// std::vector<double> row2;
// row2.push_back(3.0);
// row2.push_back(-5.0);

// tmp.push_back(row1);
// tmp.push_back(row2);

// std::cout << "MINI TEST sizes" << std::endl;
// std::cout << "tmp.size()=" << tmp.size() << std::endl;
// std::cout << "tmp[0].size()=" << tmp[0].size() << std::endl;

// std::vector<std::vector<double>> result = matrixMultiply(tmp, tmp);

// std::cout << "MINI TEST result" << std::endl;
// for (auto tmp : result)
// {
//   std::cout << tmp[0] << ", " << tmp[1] << std::endl;
// }

// template <typename T, typename R>
// GRBVector matrixMultiply(const std::vector<std::vector<R>>& A, const std::vector<std::vector<R>>& B)
// {
//   std::vector<GRBVector> result;

//   for (int i = 0; i < A.size(); i++)
//   {
//     GRBLinExpr lin_exp = 0;
//     for (int m = 0; m < x.size(); m++)
//     {
//       lin_exp = lin_exp + A[i][m] * x[m];
//     }
//     result.push_back(lin_exp);
//   }
//   return result;
// }

// // template <typename T>
// Eigen::Matrix<GRBLinExpr, -1, -1> myMultDV(const Eigen::Matrix<double, -1, -1>& A,
//                                            const Eigen::Matrix<GRBVar, -1, 1>& b)
// {
//   Eigen::Matrix<GRBLinExpr, -1, -1> result(A.rows(), b.cols());

//   for (int i = 0; i < A.rows(); i++)
//   {
//     GRBLinExpr exp = 0;
//     for (int m = 0; m < b.rows(); m++)
//     {
//       exp = exp + A(i, m) * b[m];
//     }
//     result(i, 1) = exp;
//   }
//   return result;
// }

// Eigen::Matrix<GRBLinExpr, -1, -1> myMultVD(const Eigen::Matrix<GRBVar, -1, -1>& A,
//                                            const Eigen::Matrix<double, -1, 1>& b)
// {
//   Eigen::Matrix<GRBLinExpr, -1, -1> result(A.rows(), b.cols());

//   for (int i = 0; i < A.rows(); i++)
//   {
//     GRBLinExpr exp = 0;
//     for (int m = 0; m < b.rows(); m++)
//     {
//       exp = exp + A(i, m) * b[m];
//     }
//     result(i, 1) = exp;
//   }
//   return result;
// }

// // squared norm of a vector or matrix
// GRBLinExpr getNorm2(const Eigen::Matrix<GRBLinExpr, -1, -1>& x)
// {
//   GRBLinExpr result = 0;
//   for (int i = 0; i < x.rows(); i++)
//   {
//     for (int j = 0; j < x.cols(); j++)
//     {
//       result = result + x(i, j) * x(i, j);
//     }
//   }
//   return result;
// }

// // Overload - to substract Elementwise std::vectors
// Eigen::Matrix<GRBLinExpr, -1, -1> operator-(const Eigen::Matrix<GRBLinExpr, -1, -1>& a,
//                                             const Eigen::Matrix<GRBLinExpr, -1, -1>& b)
// {
//   assert(a.rows() == b.rows());
//   assert(a.cols() == b.cols());

//   Eigen::Matrix<GRBLinExpr, -1, -1> result(a.cols(), a.rows());

//   for (int i = 0; i < a.rows(); i++)
//   {
//     for (int j = 0; j < a.cols(); j++)
//     {
//       result(i, j) = a(i, j) + b(i, j);
//     }
//   }

//   return result;
// }

// Eigen::Matrix<GRBLinExpr, -1, -1> myMult(const Eigen::Matrix<GRBVar, -1, -1>& A, const Eigen::Matrix<GRBVar, -1, 1>&
// b)
// {
//   Eigen::Matrix<GRBLinExpr, -1, -1> result(A.rows(), b.cols());

//   for (int i = 0; i < A.rows(); i++)
//   {
//     GRBLinExpr exp = 0;
//     for (int m = 0; m < b.rows(); m++)
//     {
//       exp = exp + A(i, m) * b[m];
//     }
//     result(i, 1) = exp;
//   }
//   return result;
// }

// Eigen::Matrix<GRBLinExpr, 2, 1> operator*(const Eigen::Matrix<double, 2, 2>& A, const Eigen::Matrix<GRBVar, 2, 1>& b)
// {
//   Eigen::Matrix<GRBLinExpr, 2, 1> result;

//   for (int i = 0; i < A.rows(); i++)
//   {
//     GRBLinExpr exp = 0;
//     for (int m = 0; m < b.rows(); m++)
//     {
//       exp = exp + A(i, m) * b[m];
//     }
//     result(i, 1) = exp;
//   }
//   return result;
// }

// template<typename Derived>
// void printFirstRow(const Eigen::MatrixBase<Derived>& x)
// {

// template <typename T>
// Eigen::Matrix<GRBLinExpr, 2, 1> operator*(const Eigen::MatrixBase<Derived1>& A, const Eigen::MatrixBase<Derived2>& b)
// {
//   Eigen::Matrix<GRBLinExpr, 2, 1> result;

//   for (int i = 0; i < A.rows(); i++)
//   {
//     GRBLinExpr exp = 0;
//     for (int m = 0; m < b.rows(); m++)
//     {
//       exp = exp + A(i, m) * b[m];
//     }
//     result(i, 1) = exp;
//   }
//   return result;
// }

/*GRBVector MatrixMultiply(const std::vector<std::vector<double>>& A, const GRBVector& x)
{
  GRBVector result;
  for (int i = 0; i < A.size(); i++)
  {
    GRBLinExpr lin_exp = 0;
    for (int m = 0; m < x.size(); m++)
    {
      lin_exp = lin_exp + A[i][m] * x[m];
    }
    result.push_back(lin_exp);
  }
  return result;
}*/

#endif
