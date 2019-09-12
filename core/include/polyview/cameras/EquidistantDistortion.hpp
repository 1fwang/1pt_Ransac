/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_EQUIDISTANTDISTORTION_HPP_
#define POLYVIEW_CAMERAS_EQUIDISTANTDISTORTION_HPP_

#include <stdlib.h>
#include <memory>
#include <polyview/cameras/Distortion.hpp>


namespace polyview
{
namespace cameras
{

/**
* \class EquidistantDistortion
* \brief An implementation of the equidistant distortion model for pinhole cameras.
*
* See "A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses" by Juho Kannala and Sami S. Brandt for further information
*
*
* The usual model of a pinhole camera follows these steps:
* - Transformation: Transform the point into a coordinate frame associated with the camera
* - Normalization: Project the point onto the normalized image plane: \f$\mathbf y := \left[ x/z,y/z\right] \f$
* - Distortion: apply a nonlinear transformation to \f$y\f$ to account for distortions caused by the optics
* - Projection: Project the point into the image using a standard \f$3 \time 3\f$ projection matrix
*
* This class represents a standard implementation of the distortion block. The function "distort" applies this nonlinear transformation.
* The function "undistort" applies the inverse transformation. Note that the inverse transformation in this case is not avaialable in
* closed form and so it is computed iteratively.
*
*/
class EquidistantDistortion : public Distortion
{
public:
  EquidistantDistortion( double k1, double k2, double k3, double k4 );
  virtual ~EquidistantDistortion();

  virtual void distort(Eigen::Vector2d & imgPt ) const;
  virtual void undistort(Eigen::Vector2d & imgPt ) const;

  double k1() const;
  double k2() const;
  double k3() const;
  double k4() const;
  
private:
  double _k1;
  double _k2;
  double _k3;
  double _k4;

};

}
}

#endif /* POLYVIEW_CAMERAS_EQUIDISTANTDISTORTION_HPP_ */
