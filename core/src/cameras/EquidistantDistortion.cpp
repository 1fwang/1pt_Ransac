/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/EquidistantDistortion.hpp>


polyview::cameras::EquidistantDistortion::EquidistantDistortion(
    double k1, double k2, double k3, double k4 ) :
    Distortion(),
    _k1(k1), _k2(k2), _k3(k3), _k4(k4)
{}

polyview::cameras::EquidistantDistortion::~EquidistantDistortion()
{}

void
polyview::cameras::EquidistantDistortion::distort( Eigen::Vector2d & imgPt ) const
{
  double r, theta, theta2, theta4, theta6, theta8, thetad, scaling;

  r = imgPt.norm();
  theta = atan(r);
  theta2 = theta * theta;
  theta4 = theta2 * theta2;
  theta6 = theta4 * theta2;
  theta8 = theta4 * theta4;
  thetad = theta *
      (1.0 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);

  scaling = (r > 1e-8) ? thetad / r : 1.0;
  imgPt *= scaling;
}

void
polyview::cameras::EquidistantDistortion::undistort( Eigen::Vector2d & imgPt ) const
{
  double theta, theta2, theta4, theta6, theta8, thetad, scaling;

  thetad = imgPt.norm();
  theta = thetad; // initial guess
  for( int i = 20; i > 0; i-- )
  {
    theta2 = theta * theta;
    theta4 = theta2 * theta2;
    theta6 = theta4 * theta2;
    theta8 = theta4 * theta4;
    theta = thetad /
        (1.0 + _k1 * theta2 + _k2 * theta4 + _k3 * theta6 + _k4 * theta8);
  }
  scaling = tan(theta) / thetad;

  imgPt *= scaling;
}

double
polyview::cameras::EquidistantDistortion::k1() const
{
  return _k1;
}

double
polyview::cameras::EquidistantDistortion::k2() const
{
  return _k2;
}

double
polyview::cameras::EquidistantDistortion::k3() const
{
  return _k3;
}

double
polyview::cameras::EquidistantDistortion::k4() const
{
  return _k4;
}
