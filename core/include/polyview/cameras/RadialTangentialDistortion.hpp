/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_RADIALTANGENTIALDISTORTION_HPP_
#define POLYVIEW_CAMERAS_RADIALTANGENTIALDISTORTION_HPP_

#include <stdlib.h>
#include <memory>
#include <polyview/cameras/Distortion.hpp>


namespace polyview
{
namespace cameras
{

/**
* \class RadialTangentialDistortion
* \brief An implementation of the standard, four parameter distortion model for pinhole cameras.
*
* \todo outline the math here and provide a reference. What is the original reference?
*
*
* The usual model of a pinhole camera follows these steps:
* - Transformation: Transform the point into a coordinate frame associated with the camera
* - Normalization: Project the point onto the normalized image plane: \f$\mathbf y := \left[ x/z,y/z\right] \f$
* - Distortion: apply a nonlinear transformation to \f$y\f$ to account for radial and tangential distortion of the lens
* - Projection: Project the point into the image using a standard \f$3 \time 3\f$ projection matrix
*
* This class represents a standard implementation of the distortion block. The function "distort" applies this nonlinear transformation.
* The function "undistort" applies the inverse transformation. Note that the inverse transformation in this case is not avaialable in
* closed form and so it is computed iteratively.
*
*/
class RadialTangentialDistortion : public Distortion
{
public:
  RadialTangentialDistortion( double k1, double k2, double p1, double p2 );
  virtual ~RadialTangentialDistortion();

  virtual void distort( Eigen::Vector2d & imgPt ) const;
  virtual void undistort( Eigen::Vector2d & imgPt ) const;
  virtual void distort( Eigen::Vector2d & imgPt, Eigen::Matrix2d & J ) const;
  
  double k1() const;
  double k2() const;
  double p1() const;
  double p2() const;

private:
  double _k1;
  double _k2;
  double _p1;
  double _p2;
};

}
}

#endif /* POLYVIEW_CAMERAS_RADIALTANGENTIALDISTORTION_HPP_ */
