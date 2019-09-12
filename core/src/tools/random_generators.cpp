/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/tools/random_generators.hpp>


Eigen::Vector3d
polyview::tools::generateRandomTranslation( double maximumParallax )
{
  Eigen::Vector3d translation;
  translation[0] = (((double) rand())/ ((double) RAND_MAX)-0.5)*2.0;
  translation[1] = (((double) rand())/ ((double) RAND_MAX)-0.5)*2.0;
  translation[2] = (((double) rand())/ ((double) RAND_MAX)-0.5)*2.0;
  return maximumParallax * translation;
}

Eigen::Matrix3d
polyview::tools::generateRandomRotation( double maxAngle )
{
  Eigen::Vector3d rpy;
  rpy[0] = ((double) rand())/ ((double) RAND_MAX);
  rpy[1] = ((double) rand())/ ((double) RAND_MAX);
  rpy[2] = ((double) rand())/ ((double) RAND_MAX);

  rpy[0] = maxAngle*2.0*(rpy[0]-0.5);
  rpy[1] = maxAngle*2.0*(rpy[1]-0.5);
  rpy[2] = maxAngle*2.0*(rpy[2]-0.5);

  Eigen::Matrix3d R1;
  R1(0,0) = 1.0;
  R1(0,1) = 0.0;
  R1(0,2) = 0.0;
  R1(1,0) = 0.0;
  R1(1,1) = cos(rpy[0]);
  R1(1,2) = -sin(rpy[0]);
  R1(2,0) = 0.0;
  R1(2,1) = -R1(1,2);
  R1(2,2) = R1(1,1);

  Eigen::Matrix3d R2;
  R2(0,0) = cos(rpy[1]);
  R2(0,1) = 0.0;
  R2(0,2) = sin(rpy[1]);
  R2(1,0) = 0.0;
  R2(1,1) = 1.0;
  R2(1,2) = 0.0;
  R2(2,0) = -R2(0,2);
  R2(2,1) = 0.0;
  R2(2,2) = R2(0,0);

  Eigen::Matrix3d R3;
  R3(0,0) = cos(rpy[2]);
  R3(0,1) = -sin(rpy[2]);
  R3(0,2) = 0.0;
  R3(1,0) =-R3(0,1);
  R3(1,1) = R3(0,0);
  R3(1,2) = 0.0;
  R3(2,0) = 0.0;
  R3(2,1) = 0.0;
  R3(2,2) = 1.0;

  Eigen::Matrix3d rotation = R3 * R2 * R1;

  rotation.col(0) = rotation.col(0) / rotation.col(0).norm();
  rotation.col(2) = rotation.col(0).cross(rotation.col(1));
  rotation.col(2) = rotation.col(2) / rotation.col(2).norm();
  rotation.col(1) = rotation.col(2).cross(rotation.col(0));
  rotation.col(1) = rotation.col(1) / rotation.col(1).norm();

  return rotation;
}