/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/OmniProjection.hpp>
#include <polyview/cameras/NoDistortion.hpp>


polyview::cameras::OmniProjection::OmniProjection(
    double xi,
    double focalLengthU, double focalLengthV,
    double imageCenterU, double imageCenterV,
    int resolutionU, int resolutionV ) :
    Camera(
        resolutionU,
        resolutionV,
        Eigen::Matrix3d::Identity(),
        (focalLengthU+focalLengthV)/2.0),
    _xi(xi),
    _fu(focalLengthU), _fv(focalLengthV),
    _cu(imageCenterU), _cv(imageCenterV),
    _ru(resolutionU), _rv(resolutionV),
    _distortion(NoDistortion::Ptr(new NoDistortion()))
{
  updateTemporaries();
  _K(0,0) = focalLengthU;
  _K(1,1) = focalLengthV;
  _K(0,2) = imageCenterU;
  _K(1,2) = imageCenterV;
}

polyview::cameras::OmniProjection::~OmniProjection()
{}

Eigen::Vector3d
polyview::cameras::OmniProjection::camToWorld( const Eigen::Vector2d & imgPt ) const
{
  Eigen::Vector3d outPoint;

  // Unproject...
  outPoint[0] = _recip_fu * (imgPt[0] - _cu);
  outPoint[1] = _recip_fv * (imgPt[1] - _cv);

  // Re-distort
  Eigen::Vector2d temp = outPoint.block<2,1>(0,0);
  _distortion->undistort(temp);
  outPoint.block<2,1>(0,0) = temp;

  double rho2_d = outPoint[0]*outPoint[0] + outPoint[1]*outPoint[1];
  outPoint[2] = 1.0 - _xi*(rho2_d+1.0) / (_xi+sqrt(1.0+(1.0-_xi*_xi)*rho2_d));

  return outPoint;
}

Eigen::Vector2d
polyview::cameras::OmniProjection::worldToCam( const Eigen::Vector3d & wrlPt ) const
{
  Eigen::Vector2d outKeypoint;

  double d = wrlPt.norm();

  // Check if point will lead to a valid projection
  if( wrlPt[2] <= -(_fov_parameter * d) )
    return outKeypoint;

  double rz = 1.0 / (wrlPt[2]+_xi*d);
  outKeypoint[0] = wrlPt[0] * rz;
  outKeypoint[1] = wrlPt[1] * rz;

  _distortion->distort(outKeypoint);

  outKeypoint[0] = _fu * outKeypoint[0] + _cu;
  outKeypoint[1] = _fv * outKeypoint[1] + _cv;
  return outKeypoint;
}

void
polyview::cameras::OmniProjection::setDistortion(const Distortion::Ptr & distortion)
{
  _distortion = distortion;
}

const polyview::cameras::Distortion &
polyview::cameras::OmniProjection::distortion() const
{
  return *_distortion.get();
}

Eigen::Matrix3d
polyview::cameras::OmniProjection::getCameraMatrix() const
{
  return _K;
}

double
polyview::cameras::OmniProjection::xi() const
{
  return _xi;
}

double
polyview::cameras::OmniProjection::fu() const
{
  return _fu;
}

double
polyview::cameras::OmniProjection::fv() const
{
  return _fv;
}

double
polyview::cameras::OmniProjection::cu() const
{
  return _cu;
}

double
polyview::cameras::OmniProjection::cv() const
{
  return _cv;
}

int
polyview::cameras::OmniProjection::ru() const
{
  return _ru;
}

int
polyview::cameras::OmniProjection::rv() const
{
  return _rv;
}

int
polyview::cameras::OmniProjection::width() const
{
  return _ru;
}

int
polyview::cameras::OmniProjection::height() const
{
  return _rv;
}

double
polyview::cameras::OmniProjection::focalLengthCol() const
{
  return _fu;
}

double
polyview::cameras::OmniProjection::focalLengthRow() const
{
  return _fv;
}

double
polyview::cameras::OmniProjection::opticalCenterCol() const
{
  return _cu;
}

double
polyview::cameras::OmniProjection::opticalCenterRow() const
{
  return _cv;
}

void
polyview::cameras::OmniProjection::updateTemporaries()
{
  _recip_fu = 1.0 / _fu;
  _recip_fv = 1.0 / _fv;
  _fu_over_fv = _fu / _fv;
  _one_over_xixi_m_1 = 1.0 / (_xi * _xi - 1.0);
  _fov_parameter = (_xi <= 1.0) ? _xi : 1.0 / _xi;
}
