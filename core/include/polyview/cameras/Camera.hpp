/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_CAMERA_HPP_
#define POLYVIEW_CAMERAS_CAMERA_HPP_

#include <stdlib.h>
#include <Eigen/Eigen>
#include <memory>
#include <polyview/BaseClass.hpp>


namespace polyview
{
namespace cameras
{

class Camera : public BaseClass
{
public:
  typedef std::shared_ptr<Camera> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Camera(
      int width,
      int height,
      const Eigen::Matrix3d & K,
      double focalLength );
  virtual ~Camera();

  virtual Eigen::Vector3d camToWorld( const Eigen::Vector2d & imgPt ) const = 0;
  virtual Eigen::Vector2d worldToCam( const Eigen::Vector3d & wrlPt ) const = 0;
  
  const Eigen::Matrix3d & K() const;
  double focalLength() const;
  int width() const;
  int height() const;
  
protected:
  int _width;
  int _height;
  Eigen::Matrix3d _K;
  double _focalLength;
};

}
}

#endif /* POLYVIEW_CAMERAS_CAMERA_HPP_ */

