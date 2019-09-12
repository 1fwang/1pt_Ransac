/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_OMNIPROJECTION_HPP_
#define POLYVIEW_CAMERAS_OMNIPROJECTION_HPP_

#include <stdlib.h>
#include <polyview/cameras/Camera.hpp>
#include <polyview/cameras/Distortion.hpp>
#include <memory>


namespace polyview
{
namespace cameras
{

class OmniProjection : Camera
{
public:
  typedef std::shared_ptr<OmniProjection> Ptr;

  OmniProjection(
      double xi, double focalLengthU, double focalLengthV,
      double imageCenterU, double imageCenterV,
      int resolutionU, int resolutionV,
      const Distortion::Ptr & distortion );

  OmniProjection(
      double xi, double focalLengthU, double focalLengthV,
      double imageCenterU, double imageCenterV, int resolutionU,
      int resolutionV );

  virtual ~OmniProjection();

  virtual Eigen::Vector3d camToWorld( const Eigen::Vector2d & imgPt ) const;
  virtual Eigen::Vector2d worldToCam( const Eigen::Vector3d & wrlPt ) const;
  
  void setDistortion(const Distortion::Ptr & distortion);  
  const Distortion & distortion() const;
  Eigen::Matrix3d getCameraMatrix() const;

  double xi() const;
  double fu() const;
  double fv() const;
  double cu() const;  
  double cv() const;  
  int ru() const;  
  int rv() const;  
  int width() const;  
  int height() const;

  double focalLengthCol() const;  
  double focalLengthRow() const;  
  double opticalCenterCol() const;  
  double opticalCenterRow() const;
  
private:
  using Camera::_width;
  using Camera::_height;
  using Camera::_K;
  using Camera::_focalLength;

  // The xi parameter that controls the spherical projection.
  double _xi;
  // The horizontal focal length in pixels.
  double _fu;
  // The vertical focal length in pixels.
  double _fv;
  // The horizontal image center in pixels.
  double _cu;
  // The vertical image center in pixels.
  double _cv;
  // The horizontal resolution in pixels.
  int _ru;
  // The vertical resolution in pixels.
  int _rv;

  // some computed values for speeding up computation
  void updateTemporaries();

  double _recip_fu;
  double _recip_fv;
  double _fu_over_fv;
  double _one_over_xixi_m_1;
  double _fov_parameter;
  //PM: is = xi for xi=<1, = 1/xi for x>1. Used for determining valid projections. Better name?

  Distortion::Ptr _distortion;
};

}
}

#endif /* POLYVIEW_CAMERAS_OMNIPROJECTION_HPP_ */
