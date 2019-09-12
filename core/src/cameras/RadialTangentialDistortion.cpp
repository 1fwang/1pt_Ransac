/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/RadialTangentialDistortion.hpp>


polyview::cameras::RadialTangentialDistortion::RadialTangentialDistortion(
    double k1, double k2, double p1, double p2) : Distortion(),
    _k1(k1), _k2(k2), _p1(p1), _p2(p2)
{}

polyview::cameras::RadialTangentialDistortion::~RadialTangentialDistortion()
{}

void
polyview::cameras::RadialTangentialDistortion::distort( Eigen::Vector2d & imgPt ) const
{
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = imgPt[0] * imgPt[0];
  my2_u = imgPt[1] * imgPt[1];
  mxy_u = imgPt[0] * imgPt[1];
  rho2_u = mx2_u + my2_u;
  rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;
  imgPt[0] += imgPt[0] * rad_dist_u + 2.0 * _p1 * mxy_u + _p2 * (rho2_u + 2.0 * mx2_u);
  imgPt[1] += imgPt[1] * rad_dist_u + 2.0 * _p2 * mxy_u + _p1 * (rho2_u + 2.0 * my2_u);
}

void
polyview::cameras::RadialTangentialDistortion::undistort( Eigen::Vector2d & imgPt ) const
{
  Eigen::Vector2d ybar = imgPt;
  const int n = 5;
  Eigen::Matrix2d F;

  Eigen::Vector2d y_tmp;

  for( int i = 0; i < n; i++ )
  {
    y_tmp = ybar;

    distort(y_tmp, F);

    Eigen::Vector2d e(imgPt - y_tmp);
    Eigen::Vector2d du = (F.transpose() * F).inverse() * F.transpose() * e;

    ybar += du;

    if (e.dot(e) < 1e-15)
      break;
  }
  
  imgPt = ybar;
}


void
polyview::cameras::RadialTangentialDistortion::distort(
    Eigen::Vector2d & imgPt, Eigen::Matrix2d & J) const
{
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
  J.setZero();

  mx2_u = imgPt[0] * imgPt[0];
  my2_u = imgPt[1] * imgPt[1];
  mxy_u = imgPt[0] * imgPt[1];
  rho2_u = mx2_u + my2_u;

  rad_dist_u = _k1 * rho2_u + _k2 * rho2_u * rho2_u;

  J(0, 0) = 1.0 + rad_dist_u + _k1 * 2.0 * mx2_u + _k2 * rho2_u * 4.0 * mx2_u
      + 2.0 * _p1 * imgPt[1] + 6.0 * _p2 * imgPt[0];
  J(1, 0) = _k1 * 2.0 * imgPt[0] * imgPt[1] + _k2 * 4.0 * rho2_u * imgPt[0] * imgPt[1]
      + _p1 * 2.0 * imgPt[0] + 2.0 * _p2 * imgPt[1];
  J(0, 1) = J(1, 0);
  J(1, 1) = 1.0 + rad_dist_u + _k1 * 2.0 * my2_u + _k2 * rho2_u * 4.0 * my2_u
      + 6.0 * _p1 * imgPt[1] + 2.0 * _p2 * imgPt[0];

  imgPt[0] += imgPt[0] * rad_dist_u + 2.0 * _p1 * mxy_u + _p2 * (rho2_u + 2.0 * mx2_u);
  imgPt[1] += imgPt[1] * rad_dist_u + 2.0 * _p2 * mxy_u + _p1 * (rho2_u + 2.0 * my2_u);
}

double
polyview::cameras::RadialTangentialDistortion::k1() const
{
  return _k1;
}

double
polyview::cameras::RadialTangentialDistortion::k2() const
{
  return _k2;
}

double
polyview::cameras::RadialTangentialDistortion::p1() const
{
  return _p1;
}

double
polyview::cameras::RadialTangentialDistortion::p2() const
{
  return _p2;
}