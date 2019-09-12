/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/tools/cayley.hpp>


Eigen::Matrix3d
polyview::tools::cayley2rot( const Eigen::Vector3d & cayley)
{
  Eigen::Matrix3d R;
  double scale = 1+pow(cayley[0],2)+pow(cayley[1],2)+pow(cayley[2],2);

  R(0,0) = 1+pow(cayley[0],2)-pow(cayley[1],2)-pow(cayley[2],2);
  R(0,1) = 2*(cayley[0]*cayley[1]-cayley[2]);
  R(0,2) = 2*(cayley[0]*cayley[2]+cayley[1]);
  R(1,0) = 2*(cayley[0]*cayley[1]+cayley[2]);
  R(1,1) = 1-pow(cayley[0],2)+pow(cayley[1],2)-pow(cayley[2],2);
  R(1,2) = 2*(cayley[1]*cayley[2]-cayley[0]);
  R(2,0) = 2*(cayley[0]*cayley[2]-cayley[1]);
  R(2,1) = 2*(cayley[1]*cayley[2]+cayley[0]);
  R(2,2) = 1-pow(cayley[0],2)-pow(cayley[1],2)+pow(cayley[2],2);

  R = (1/scale) * R;
  return R;
}

Eigen::Vector3d
polyview::tools::rot2cayley( const Eigen::Matrix3d & R )
{
  Eigen::Matrix3d C1;
  Eigen::Matrix3d C2;
  Eigen::Matrix3d C;
  C1 = R-Eigen::Matrix3d::Identity();
  C2 = R+Eigen::Matrix3d::Identity();
  C = C1 * C2.inverse();

  Eigen::Vector3d cayley;
  cayley[0] = -C(1,2);
  cayley[1] = C(0,2);
  cayley[2] = -C(0,1);

  return cayley;
}
