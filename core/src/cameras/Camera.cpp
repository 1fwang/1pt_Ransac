/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/Camera.hpp>


polyview::cameras::Camera::Camera(
      int width,
      int height,
      const Eigen::Matrix3d & K,
      double focalLength ) :
      _width(width), _height(height), _K(K), _focalLength(focalLength)
{}

polyview::cameras::Camera::~Camera()
{}

const Eigen::Matrix3d &
polyview::cameras::Camera::K() const
{
  return _K;
}

double
polyview::cameras::Camera::focalLength() const
{
  return _focalLength;
}

int
polyview::cameras::Camera::width() const
{
  return _width;
}

int
polyview::cameras::Camera::height() const
{
  return _height;
}
