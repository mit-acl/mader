/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "utils.hpp"
#include "exprtk.hpp"
/*Eigen::Vector3d eval(mt::PieceWisePol piecewisepol, double t)
{
  Eigen::Vector3d result;

  if (t >= piecewisepol.times[piecewisepol.times.size() - 1])
  {  // return the last value of the polynomial in the last interval
    Eigen::Matrix<double, 4, 1> tmp;
    double u = 1;
    tmp << u * u * u, u * u, u, 1.0;
    result.x() = piecewisepol.coeff_x.back().transpose() * tmp;
    result.y() = piecewisepol.coeff_y.back().transpose() * tmp;
    result.z() = piecewisepol.coeff_z.back().transpose() * tmp;
    return result;
  }
  //(piecewisepol.piecewisepol.times - 1) is the number of intervals
  for (int i = 0; i < (piecewisepol.times.size() - 1); i++)
  {
    if (piecewisepol.times[i] <= t && t < piecewisepol.times[i + 1])
    {
      double u = (t - piecewisepol.times[i]) / (piecewisepol.times[i + 1] - piecewisepol.times[i]);

      // TODO: This is hand-coded for a third-degree polynomial
      Eigen::Matrix<double, 4, 1> tmp;
      tmp << u * u * u, u * u, u, 1.0;

      result.x() = piecewisepol.coeff_x[i].transpose() * tmp;
      result.y() = piecewisepol.coeff_y[i].transpose() * tmp;
      result.z() = piecewisepol.coeff_z[i].transpose() * tmp;

      break;
    }
  }
  return result;
}*/
int main()
{
  mt::PieceWisePol piecewisepol;
  piecewisepol.times.push_back(0.0);
  piecewisepol.times.push_back(1.0);
  piecewisepol.times.push_back(2.0);
  piecewisepol.times.push_back(3.0);

  for (auto tmp : piecewisepol.times)
  {
    std::cout << tmp << std::endl;
  }

  Eigen::Matrix<double, 4, 1> coeffs;
  coeffs << 1.0, 2.0, 3.0, 4.0;
  piecewisepol.coeff_x.push_back(coeffs);
  piecewisepol.coeff_y.push_back(coeffs);
  piecewisepol.coeff_z.push_back(coeffs);
  coeffs << 5.0, 6.0, 7.0, 8.0;
  piecewisepol.coeff_x.push_back(coeffs);
  piecewisepol.coeff_y.push_back(coeffs);
  piecewisepol.coeff_z.push_back(coeffs);
  coeffs << 9.0, 10.0, 11.0, 12.0;
  piecewisepol.coeff_x.push_back(coeffs);
  piecewisepol.coeff_y.push_back(coeffs);
  piecewisepol.coeff_z.push_back(coeffs);

  std::vector<std::string> s = mu::pieceWisePol2String(piecewisepol);

  // std::cout << "The string is " << std::endl;
  // std::cout << s << std::endl;

  // Check if it leads the the good result
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

  double t;

  symbol_table_t symbol_table;
  symbol_table.add_variable("t", t);

  std::vector<expression_t> expressions(3);
  expressions[0].register_symbol_table(symbol_table);
  expressions[1].register_symbol_table(symbol_table);
  expressions[2].register_symbol_table(symbol_table);
  /*  expression_t expression_x;
    expression_x.register_symbol_table(symbol_table);
    expression_t expression_x;
    expression_x.register_symbol_table(symbol_table);
    expression_t expression_x;
    expression_x.register_symbol_table(symbol_table);*/

  parser_t parser;

  for (int i = 0; i < s.size(); i++)
  {
    parser.compile(s[i], expressions[i]);
  }

  /*  if (!parser.compile(s[0], expressions[0]) || !parser.compile(s, expressions[1]) || !parser.compile(s,
    expressions[2]))
    {
      printf("Compilation error...\n");
      return 0;
    }*/

  for (t = 0; t < 5; t = t + 0.4)
  {
    std::cout << "*****t= " << t << std::endl;
    std::cout << "result with the string= " << expressions[0].value() << ", " << expressions[1].value() << ", "
              << expressions[2].value() << std::endl;
    std::cout << "result with the eval()= " << piecewisepol.eval(t).transpose() << std::endl;
  }

  // Test composing to pwp

  mt::PieceWisePol piecewisepolB;
  piecewisepolB.times.push_back(2.5);
  piecewisepolB.times.push_back(3.5);
  piecewisepolB.times.push_back(4.5);
  piecewisepolB.times.push_back(5.5);

  for (auto tmp : piecewisepolB.times)
  {
    std::cout << tmp << std::endl;
  }

  coeffs << 13.0, 14.0, 15.0, 16.0;
  piecewisepolB.coeff_x.push_back(coeffs);
  piecewisepolB.coeff_y.push_back(coeffs);
  piecewisepolB.coeff_z.push_back(coeffs);
  coeffs << 17.0, 18.0, 19.0, 20.0;
  piecewisepolB.coeff_x.push_back(coeffs);
  piecewisepolB.coeff_y.push_back(coeffs);
  piecewisepolB.coeff_z.push_back(coeffs);
  coeffs << 21.0, 22.0, 23.0, 24.0;
  piecewisepolB.coeff_x.push_back(coeffs);
  piecewisepolB.coeff_y.push_back(coeffs);
  piecewisepolB.coeff_z.push_back(coeffs);

  for (t = 0; t < 5; t = t + 0.4)
  {
    mt::PieceWisePol piecewisepolAB = mu::composePieceWisePol(t, -0.1, piecewisepol, piecewisepolB);
    std::cout << "==================" << std::endl;
    std::cout << "t= " << t << std::endl;
    std::cout << "==================" << std::endl;
    std::cout << "Polynomial A: " << std::endl;
    piecewisepol.print();
    std::cout << "==================" << std::endl;
    std::cout << "Polynomial B: " << std::endl;
    piecewisepolB.print();
    std::cout << "==================" << std::endl;
    std::cout << "Polynomial AB: " << std::endl;
    piecewisepolAB.print();
  }

  return 0;
}