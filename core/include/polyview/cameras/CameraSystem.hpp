/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_CAMERASYSTEM_HPP_
#define POLYVIEW_CAMERAS_CAMERASYSTEM_HPP_

#include <stdlib.h>
#include <vector>
#include <Eigen/Eigen>
#include <memory>
#include <polyview/BaseClass.hpp>
#include <polyview/cameras/Camera.hpp>


namespace polyview
{
namespace cameras
{

class CameraSystem : public BaseClass
{
public:
  typedef std::shared_ptr<CameraSystem> Ptr;
  
  CameraSystem();
  CameraSystem( const Camera::Ptr & camera );
  CameraSystem(
      const std::vector< Camera::Ptr > & cameras,
      const std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > & positions,
      const std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > & orientations );
  CameraSystem(
      const std::vector< Camera::Ptr > & cameras,
      const std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > & positions,
      const std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > & orientations,
      const std::vector< bool > & rgbdFlags );
  virtual ~CameraSystem();

  void addCamera( const Camera::Ptr & camera, bool rgbdFlag = false );
  void addCamera( const Camera::Ptr & camera, const Eigen::Vector3d & position, const Eigen::Matrix3d & orientation, bool rgbdFlag = false );
  size_t size() const;

  const Camera & camera( size_t index ) const;
  const Eigen::Vector3d & position( size_t index ) const;
  const Eigen::Matrix3d & orientation( size_t index ) const;
  bool rgbdFlag( size_t index ) const;

  Camera::Ptr cameraPtr( size_t index ) const;
  
protected:
  std::vector< Camera::Ptr > _cameras;
  std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > _positions;
  std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > _orientations;
  std::vector< bool > _rgbdFlags;
};

}
}

#endif /* POLYVIEW_CAMERAS_CAMERASYSTEM_HPP_ */

