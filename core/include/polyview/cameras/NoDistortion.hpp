/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_NODISTORTION_HPP_
#define POLYVIEW_CAMERAS_NODISTORTION_HPP_

#include <stdlib.h>
#include <memory>
#include <polyview/cameras/Distortion.hpp>


namespace polyview
{
namespace cameras
{

class NoDistortion : public Distortion
{
public:
  typedef std::shared_ptr<NoDistortion> Ptr;

  NoDistortion();
  virtual ~NoDistortion();

  virtual void distort( Eigen::Vector2d & imgPt ) const;
  virtual void undistort( Eigen::Vector2d & imgPt ) const;
};

}
}

#endif /* POLYVIEW_CAMERAS_NODISTORTION_HPP */
