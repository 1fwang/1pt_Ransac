/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_DISTORTION_HPP_
#define POLYVIEW_CAMERAS_DISTORTION_HPP_

#include <stdlib.h>
#include <Eigen/Eigen>
#include <memory>
#include <polyview/BaseClass.hpp>


namespace polyview
{
namespace cameras
{

class Distortion : public BaseClass
{
public:
  typedef std::shared_ptr<Distortion> Ptr;
  
  Distortion() {};
  virtual ~Distortion() {};
  
  virtual void distort( Eigen::Vector2d & imgPt ) const = 0;
  virtual void undistort( Eigen::Vector2d & imgPt ) const = 0;
};

}
}

#endif /* POLYVIEW_CAMERAS_DISTORTION_HPP_ */
