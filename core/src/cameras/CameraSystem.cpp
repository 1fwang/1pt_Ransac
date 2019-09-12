/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/CameraSystem.hpp>


polyview::cameras::CameraSystem::CameraSystem()
{}

polyview::cameras::CameraSystem::CameraSystem( const Camera::Ptr & camera )
{
  _cameras.push_back(camera);
  _positions.push_back(Eigen::Vector3d::Zero());
  _orientations.push_back(Eigen::Matrix3d::Identity());
  _rgbdFlags.push_back(false);
}

polyview::cameras::CameraSystem::CameraSystem(
    const std::vector< Camera::Ptr > & cameras,
    const std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > & positions,
    const std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > & orientations ) :
    _cameras(cameras), _positions(positions), _orientations(orientations)
{
  _rgbdFlags.reserve(cameras.size());
  for( size_t i = 0; i < cameras.size(); i++ )
    _rgbdFlags.push_back(false);
}

polyview::cameras::CameraSystem::CameraSystem(
    const std::vector< Camera::Ptr > & cameras,
    const std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > & positions,
    const std::vector< Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > & orientations,
    const std::vector<bool> & rgbdFlags ) :
    _cameras(cameras), _positions(positions), _orientations(orientations), _rgbdFlags(rgbdFlags)
{}

polyview::cameras::CameraSystem::~CameraSystem()
{}

void
polyview::cameras::CameraSystem::addCamera(
    const Camera::Ptr & camera,
    bool rgbdFlag )
{
  _cameras.push_back(camera);
  _positions.push_back(Eigen::Vector3d::Zero());
  _orientations.push_back(Eigen::Matrix3d::Identity());
  _rgbdFlags.push_back(rgbdFlag);
}

void
polyview::cameras::CameraSystem::addCamera(
    const Camera::Ptr & camera,
    const Eigen::Vector3d & position,
    const Eigen::Matrix3d & orientation,
    bool rgbdFlag )
{
  _cameras.push_back(camera);
  _positions.push_back(position);
  _orientations.push_back(orientation);
  _rgbdFlags.push_back(rgbdFlag);
}

size_t
polyview::cameras::CameraSystem::size() const
{
  return _cameras.size();
}

const polyview::cameras::Camera &
polyview::cameras::CameraSystem::camera( size_t index ) const
{
  return *(_cameras[index]);
}

const Eigen::Vector3d &
polyview::cameras::CameraSystem::position( size_t index ) const
{
  return _positions[index];
}

const Eigen::Matrix3d & 
polyview::cameras::CameraSystem::orientation( size_t index ) const
{
  return _orientations[index];
}

bool
polyview::cameras::CameraSystem::rgbdFlag( size_t index ) const
{
  return _rgbdFlags[index];
}

polyview::cameras::Camera::Ptr
polyview::cameras::CameraSystem::cameraPtr( size_t index ) const
{
  return _cameras[index];
}