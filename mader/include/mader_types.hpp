/* ----------------------------------------------------------------------------
 * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <iostream>
#include <iomanip>  // std::setprecision
#include <deque>
#include "exprtk.hpp"
#include "termcolor.hpp"
#include <Eigen/Dense>

namespace mt  // mader_types
{
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Polyhedron_Std;
typedef std::vector<mt::Polyhedron_Std> ConvexHullsOfCurve_Std;
typedef std::vector<mt::ConvexHullsOfCurve_Std> ConvexHullsOfCurves_Std;
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Edge;
typedef std::vector<Edge> Edges;

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

  const void printPos()
  {
    std::cout << "Pos= " << pos.transpose() << std::endl;
  }

  const void print()
  {
    std::cout << std::setprecision(3) << "Pos= " << pos.transpose() << std::endl;
    std::cout << std::setprecision(3) << "Vel= " << vel.transpose() << std::endl;
    std::cout << std::setprecision(3) << "Accel= " << accel.transpose() << std::endl;
  }

  const void printHorizontal()
  {
    using namespace termcolor;
    std::cout << std::setprecision(3) << "Pos, Vel, Accel, Jerk= " << red << pos.transpose() << reset;
    std::cout << " " << std::setprecision(3) << blue << vel.transpose() << reset;
    std::cout << " " << std::setprecision(3) << green << accel.transpose() << reset;
    std::cout << " " << std::setprecision(3) << jerk.transpose() << std::endl;
  }
};

// TODO: move this struct to a class (so that no one can modify these matrices)
struct basisConverter
{
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest;
  Eigen::Matrix<double, 4, 4> A_pos_be_rest;
  Eigen::Matrix<double, 4, 4> A_pos_bs_seg0, A_pos_bs_seg1, A_pos_bs_rest, A_pos_bs_seg_last2, A_pos_bs_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2mv_seg0, M_pos_bs2mv_seg1, M_pos_bs2mv_rest, M_pos_bs2mv_seg_last2,
      M_pos_bs2mv_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2be_seg0, M_pos_bs2be_seg1, M_pos_bs2be_rest, M_pos_bs2be_seg_last2,
      M_pos_bs2be_seg_last;

  Eigen::Matrix<double, 3, 3> M_vel_bs2mv_seg0, M_vel_bs2mv_rest, M_vel_bs2mv_seg_last;
  Eigen::Matrix<double, 3, 3> M_vel_bs2be_seg0, M_vel_bs2be_rest, M_vel_bs2be_seg_last;

  basisConverter()
  {
    // See matlab.
    // This is for t \in [0 1];

    // clang-format off

        //////MATRICES A FOR MINVO POSITION///////// (there is only one)
        A_pos_mv_rest << 

     -3.4416308968564117698463178385282,  6.9895481477801393310755884158425, -4.4622887507045296828778191411402,                   0.91437149978080234369315348885721,
      6.6792587327074839365081970754545, -11.845989901556746914934592496138,  5.2523596690684613008670567069203,                                                    0,
     -6.6792587327074839365081970754545,  8.1917862965657040064115790301003, -1.5981560640774179482548333908198,                  0.085628500219197656306846511142794,
      3.4416308968564117698463178385282, -3.3353445427890959784633650997421, 0.80808514571348655231020075007109, -0.0000000000000000084567769453869345852581318467855;

        //////MATRICES A FOR Bezier POSITION///////// (there is only one)
        A_pos_be_rest << 

           -1.0,  3.0, -3.0, 1.0,
            3.0, -6.0,  3.0,   0,
           -3.0,  3.0,    0,   0,
            1.0,    0,    0,   0;

        //////MATRICES A FOR BSPLINE POSITION/////////
        A_pos_bs_seg0 <<

           -1.0000,    3.0000,   -3.0000,    1.0000,
            1.7500,   -4.5000,    3.0000,         0,
           -0.9167,    1.5000,         0,         0,
            0.1667,         0,         0,         0;

        A_pos_bs_seg1 <<

           -0.2500,    0.7500,   -0.7500,    0.2500,
            0.5833,   -1.2500,    0.2500,    0.5833,
           -0.5000,    0.5000,    0.5000,    0.1667,
            0.1667,         0,         0,         0;

        A_pos_bs_rest << 

           -0.1667,    0.5000,   -0.5000,    0.1667,
            0.5000,   -1.0000,         0,    0.6667,
           -0.5000,    0.5000,    0.5000,    0.1667,
            0.1667,         0,         0,         0;

        A_pos_bs_seg_last2 <<
           -0.1667,    0.5000,   -0.5000,    0.1667,
            0.5000,   -1.0000,    0.0000,    0.6667,
           -0.5833,    0.5000,    0.5000,    0.1667,
            0.2500,         0,         0,         0;

        A_pos_bs_seg_last <<

           -0.1667,    0.5000,   -0.5000,   0.1667,
            0.9167,   -1.2500,   -0.2500,   0.5833,
           -1.7500,    0.7500,    0.7500,   0.2500,
            1.0000,         0,         0,        0;


        //////BSPLINE to MINVO POSITION/////////

        M_pos_bs2mv_seg0 <<

         1.1023313949144333268037598827505,   0.34205724556666972091534262290224, -0.092730934245582874453361910127569, -0.032032766697130621302846975595457,
      -0.049683556253749178166501110354147,   0.65780347324677179710050722860615,   0.53053863760186903419935333658941,   0.21181027098212013015654520131648,
      -0.047309044211162346038612724896666,  0.015594436894155586093013710069499,    0.5051827557159349613158383363043,   0.63650059656260427054519368539331,
     -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,   0.18372189915240558222286892942066;

        M_pos_bs2mv_seg1 <<

        0.27558284872860833170093997068761,  0.085514311391667430228835655725561, -0.023182733561395718613340477531892, -0.0080081916742826553257117438988644,
         0.6099042761975865811763242163579,   0.63806904207840509091198555324809,   0.29959938009132258684985572472215,    0.12252106674808682651445224109921,
        0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
     -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066;

        M_pos_bs2mv_rest <<

        0.18372189915240555446729331379174,  0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
        0.70176522577378919187651717948029,   0.66657381254229419731416328431806,   0.29187180223752384744528853843804,    0.11985166952332582113172065874096,
        0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594,
     -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066;


        M_pos_bs2mv_seg_last2 <<

        0.18372189915240569324517139193631,  0.057009540927778309948870116841135, -0.015455155707597145742226985021261, -0.0053387944495218164764338553140988,
        0.70176522577378952494342456702725,   0.66657381254229453038107067186502,   0.29187180223752412500104469472717,    0.11985166952332593215402312125661,
         0.1225210667480875342816304396365,   0.29959938009132280889446064975346,   0.63806904207840497988968309073243,    0.60990427619758624810941682881094,
     -0.0080081916742826154270717964323012, -0.023182733561395621468825822830695,  0.085514311391667444106623463540018,    0.27558284872860833170093997068761;

        M_pos_bs2mv_seg_last <<

       0.18372189915240555446729331379174, 0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
       0.63650059656260415952289122287766,   0.5051827557159349613158383363043,  0.015594436894155294659469745965907,  -0.047309044211162887272337229660479,
       0.21181027098212068526805751389475,  0.53053863760186914522165579910506,   0.65780347324677146403359984105919,  -0.049683556253749622255710960416764,
     -0.032032766697130461708287185729205, -0.09273093424558248587530329132278,   0.34205724556666977642649385416007,     1.1023313949144333268037598827505;


        //////BSPLINE to BEZIER POSITION/////////

        M_pos_bs2be_seg0 <<

            1.0000,    0.0000,   -0.0000,         0,
                 0,    1.0000,    0.5000,    0.2500,
                 0,   -0.0000,    0.5000,    0.5833,
                 0,         0,         0,    0.1667;

        M_pos_bs2be_seg1 <<

            0.2500,    0.0000,   -0.0000,         0,
            0.5833,    0.6667,    0.3333,    0.1667,
            0.1667,    0.3333,    0.6667,    0.6667,
                 0,         0,         0,    0.1667;

        M_pos_bs2be_rest <<

            0.1667,    0.0000,         0,         0,
            0.6667,    0.6667,    0.3333,    0.1667,
            0.1667,    0.3333,    0.6667,    0.6667,
                 0,         0,         0,    0.1667;

        M_pos_bs2be_seg_last2 <<

            0.1667,         0,   -0.0000,         0,
            0.6667,    0.6667,    0.3333,    0.1667,
            0.1667,    0.3333,    0.6667,    0.5833,
                 0,         0,         0,    0.2500;

        M_pos_bs2be_seg_last <<

            0.1667,    0.0000,         0,         0,
            0.5833,    0.5000,         0,         0,
            0.2500,    0.5000,    1.0000,         0,
                 0,         0,         0,    1.0000;

        /////BSPLINE to MINVO VELOCITY
        M_vel_bs2mv_seg0 <<

    1.077349059083916,  0.1666702138890985, -0.07735049175615138,
 -0.03867488648729411,  0.7499977187062712,   0.5386802643920123,
 -0.03867417280506149, 0.08333206631563977,    0.538670227146185;

        M_vel_bs2mv_rest <<

    0.538674529541958, 0.08333510694454926, -0.03867524587807569,
   0.4999996430546639,  0.8333328256508203,   0.5000050185139366,
 -0.03867417280506149, 0.08333206631563977,    0.538670227146185;

        M_vel_bs2mv_seg_last <<

    0.538674529541958, 0.08333510694454926, -0.03867524587807569,
   0.5386738158597254,  0.7500007593351806, -0.03866520863224832,
 -0.07734834561012298,  0.1666641326312795,     1.07734045429237;

      /////BSPLINE to BEZIER VELOCITY
        M_vel_bs2be_seg0 <<

            1.0000,         0,         0,
                 0,    1.0000,    0.5000,
                 0,         0,    0.5000;

        M_vel_bs2be_rest <<

            0.5000,         0,         0,
            0.5000,    1.0000,    0.5000,
                 0,         0,    0.5000;

        M_vel_bs2be_seg_last <<

            0.5000,         0,         0,
            0.5000,    1.0000,         0,
                 0,         0,    1.0000;

    // clang-format on
  }

  //////MATRIX A FOR MINVO POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestMinvo()
  {
    return A_pos_mv_rest;
  }
  //////MATRIX A FOR Bezier POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestBezier()
  {
    return A_pos_be_rest;
  }

  //////MATRIX A FOR BSPLINE POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestBSpline()
  {
    return A_pos_bs_rest;
  }
  //////MATRICES A FOR MINVO POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getAMinvo(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_mv;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      A_pos_mv.push_back(A_pos_mv_rest);
    }
    return A_pos_mv;
  }

  //////MATRICES A FOR Bezier POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getABezier(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_be;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      A_pos_be.push_back(A_pos_be_rest);
    }
    return A_pos_be;
  }

  //////MATRICES A FOR BSPLINE POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getABSpline(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs;  // will have as many elements as num_pol
    A_pos_bs.push_back(A_pos_bs_seg0);
    A_pos_bs.push_back(A_pos_bs_seg1);
    for (int i = 0; i < (num_pol - 4); i++)
    {
      A_pos_bs.push_back(A_pos_bs_rest);
    }
    A_pos_bs.push_back(A_pos_bs_seg_last2);
    A_pos_bs.push_back(A_pos_bs_seg_last);
    return A_pos_bs;
  }

  //////BSPLINE to MINVO POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getMinvoPosConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2mv;  // will have as many elements as num_pol
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg0);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg1);
    for (int i = 0; i < (num_pol - 4); i++)
    {
      M_pos_bs2mv.push_back(M_pos_bs2mv_rest);
    }
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last2);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last);
    return M_pos_bs2mv;
  }

  //////BSPLINE to BEZIER POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getBezierPosConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2be;  // will have as many elements as num_pol
    M_pos_bs2be.push_back(M_pos_bs2be_seg0);
    M_pos_bs2be.push_back(M_pos_bs2be_seg1);
    for (int i = 0; i < (num_pol - 4); i++)
    {
      M_pos_bs2be.push_back(M_pos_bs2be_rest);
    }
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last2);
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last);
    return M_pos_bs2be;
  }

  //////BSPLINE to BSPLINE POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getBSplinePosConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2bs;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      M_pos_bs2bs.push_back(Eigen::Matrix<double, 4, 4>::Identity());
    }
    return M_pos_bs2bs;
  }

  //////BSPLINE to MINVO Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getMinvoVelConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2mv;  // will have as many elements as num_pol
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++)
    {
      M_vel_bs2mv.push_back(M_vel_bs2mv_rest);
    }
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg_last);
    return M_vel_bs2mv;
  }

  //////BSPLINE to BEZIER Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getBezierVelConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2be;  // will have as many elements as segments
    M_vel_bs2be.push_back(M_vel_bs2be_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++)
    {
      M_vel_bs2be.push_back(M_vel_bs2be_rest);
    }
    M_vel_bs2be.push_back(M_vel_bs2be_seg_last);
    return M_vel_bs2be;
  }

  //////BSPLINE to BSPLINE Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getBSplineVelConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2bs;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      M_vel_bs2bs.push_back(Eigen::Matrix<double, 3, 3>::Identity());
    }
    return M_vel_bs2bs;
  }
};

struct PieceWisePol
{
  // Interval 0: t\in[t0, t1)
  // Interval 1: t\in[t1, t2)
  // Interval 2: t\in[t2, t3)
  //...
  // Interval n-1: t\in[tn, tn+1)

  // n intervals in total

  // times has n+1 elements
  std::vector<double> times;  // [t0,t1,t2,...,tn+1]

  // coefficients has n elements
  // The coeffients are such that pol(t)=coeff_of_that_interval*[u^3 u^2 u 1]
  // with u=(t-t_min_that_interval)/(t_max_that_interval- t_min_that_interval)
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_x;  // [a b c d]' of Int0 , [a b c d]' of Int1,...
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_y;  // [a b c d]' of Int0 , [a b c d]' of Int1,...
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_z;  // [a b c d]' of Int0 , [a b c d]' of Int1,...

  void clear()
  {
    times.clear();
    coeff_x.clear();
    coeff_y.clear();
    coeff_z.clear();
  }

  Eigen::Vector3d eval(double t)
  {
    Eigen::Vector3d result;

    if (t >= times[times.size() - 1])
    {  // return the last value of the polynomial in the last interval
      Eigen::Matrix<double, 4, 1> tmp;
      double u = 1;
      tmp << u * u * u, u * u, u, 1.0;
      result.x() = coeff_x.back().transpose() * tmp;
      result.y() = coeff_y.back().transpose() * tmp;
      result.z() = coeff_z.back().transpose() * tmp;
      return result;
    }
    if (t < times[0])
    {  // return the first value of the polynomial in the first interval
      Eigen::Matrix<double, 4, 1> tmp;
      double u = 0;
      tmp << u * u * u, u * u, u, 1.0;
      result.x() = coeff_x.front().transpose() * tmp;
      result.y() = coeff_y.front().transpose() * tmp;
      result.z() = coeff_z.front().transpose() * tmp;
      return result;
    }
    //(times - 1) is the number of intervals
    for (int i = 0; i < (times.size() - 1); i++)
    {
      if (times[i] <= t && t < times[i + 1])
      {
        double u = (t - times[i]) / (times[i + 1] - times[i]);

        // TODO: This is hand-coded for a third-degree polynomial
        Eigen::Matrix<double, 4, 1> tmp;
        tmp << u * u * u, u * u, u, 1.0;

        result.x() = coeff_x[i].transpose() * tmp;
        result.y() = coeff_y[i].transpose() * tmp;
        result.z() = coeff_z[i].transpose() * tmp;

        break;
      }
    }
    return result;
  }

  void print()
  {
    std::cout << "coeff_x.size()= " << coeff_x.size() << std::endl;
    std::cout << "times.size()= " << times.size() << std::endl;

    for (int i = 0; i < (times.size() - 1); i++)
    {
      std::cout << "From " << times[i] << " to " << times[i + 1] << std::endl;
      std::cout << "  Coeff_x= " << coeff_x[i].transpose() << std::endl;
      std::cout << "  Coeff_y= " << coeff_y[i].transpose() << std::endl;
      std::cout << "  Coeff_z= " << coeff_z[i].transpose() << std::endl;
    }
  }
};

struct dynTraj
{
  std::vector<std::string> function;
  Eigen::Vector3d bbox;
  int id;
  double time_received;  // time at which this trajectory was received from an agent
  bool is_agent;         // true for a trajectory of an agent, false for an obstacle
  mt::PieceWisePol pwp;
};

struct dynTrajCompiled
{
  std::vector<exprtk::expression<double>> function;
  Eigen::Vector3d bbox;
  int id;
  double time_received;  // time at which this trajectory was received from an agent
  bool is_agent;         // true for a trajectory of an agent, false for an obstacle
  bool is_static;
  mt::PieceWisePol pwp;
};

// struct mt::PieceWisePolWithInfo
// {
//   mt::PieceWisePol pwp;

//   Eigen::Vector3d bbox;
//   int id;
//   double time_received;  // time at which this trajectory was received from an agent
//   bool is_agent;         // true for a trajectory of an agent, false for an obstacle
//   bool is_static;
// };

struct parameters
{
  bool use_ff;
  bool visual;

  std::string color_type;
  int n_agents;

  double dc;
  double goal_radius;
  double drone_radius;

  double Ra;

  double w_max;
  double alpha_filter_dyaw;

  // bool impose_fov = false;

  double fov_horiz_deg = 60;  //[deg] angle between two faces of the tetrahedron
  double fov_vert_deg = 60;   //[deg] angle between two faces of the tetrahedron
  double fov_depth = 3.0;

  // double R_local_map;

  double x_min;
  double x_max;

  double y_min;
  double y_max;

  double z_min;
  double z_max;

  Eigen::Vector3d v_max;
  Eigen::Vector3d a_max;
  Eigen::Vector3d j_max;

  double factor_alpha;

  int num_pol;
  int deg_pol;
  double weight;
  double epsilon_tol_constraints;
  double xtol_rel;
  double ftol_rel;
  std::string solver;

  double upper_bound_runtime_snlopt;
  double lower_bound_runtime_snlopt;
  double kappa;
  double mu;

  int a_star_samp_x = 7;
  int a_star_samp_y = 7;
  int a_star_samp_z = 7;
  double a_star_fraction_voxel_size = 0.5;
  bool allow_infeasible_guess = false;

  double a_star_bias = 1.0;

  std::string basis;

  double res_plot_traj;

  double factor_alloc = 1.0;
  double factor_alloc_close = 1.0;
  double dist_factor_alloc_close = 1.0;

  double alpha_shrink = 1.0;

  double alpha = 0.0;
  double beta = 0.0;
  double gamma = 0.5;
};

typedef std::vector<mt::state> trajectory;

struct committedTrajectory
{
  std::deque<mt::state> content;

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

  void push_back(mt::state tmp)
  {
    content.push_back(tmp);
  }

  void erase(std::deque<mt::state>::iterator a, std::deque<mt::state>::iterator b)
  {
    content.erase(a, b);
  }

  mt::state front()
  {
    return content.front();
  }

  mt::state back()
  {
    return content.back();
  }

  void pop_front()
  {
    content.pop_front();
  }

  std::deque<mt::state>::iterator end()
  {
    return content.end();
  }

  std::deque<mt::state>::iterator begin()
  {
    return content.begin();
  }

  mt::state get(int i)
  {
    return content[i];
  }

  std::vector<mt::state> toStdVector()
  {
    std::vector<mt::state> my_vector;
    std::copy(content.begin(), content.end(), std::back_inserter(my_vector));
    return my_vector;
  }
};

}  // namespace mt
