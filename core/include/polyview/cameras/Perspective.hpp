/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_PERSPECTIVE_HPP_
#define POLYVIEW_CAMERAS_PERSPECTIVE_HPP_

#include <stdlib.h>
#include <vector>
#include <Eigen/Eigen>
#include <memory>
#include <polyview/cameras/Camera.hpp>


namespace polyview
{
namespace cameras
{

class Perspective : public Camera
{  
public:
  typedef std::shared_ptr<Perspective> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  //creates a fake camera, good for tests that only do image processing
  Perspective();
  Perspective(
      int width,
      int height,
      const Eigen::Matrix3d & K );
  Perspective(
      int width,
      int height,
      double focalLength );
  Perspective(
      int width,
      int height,
      double focalLength_mm,
      double sensorWidth_mm,
      double sensorHeight_mm );
  virtual ~Perspective();

  virtual Eigen::Vector3d camToWorld( const Eigen::Vector2d & imgPt ) const;
  virtual Eigen::Vector2d worldToCam( const Eigen::Vector3d & wrlPt ) const;

private:
  using Camera::_width;
  using Camera::_height;
  using Camera::_K;
  using Camera::_focalLength;
  
  Eigen::Matrix3d _invK;
};

}
}

#endif /* POLYVIEW_CAMERAS_PERSPECTIVE_HPP_ */
