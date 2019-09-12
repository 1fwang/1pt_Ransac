/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_CAMERAS_OCAM_HPP_
#define POLYVIEW_CAMERAS_OCAM_HPP_

#include <stdlib.h>
#include <vector>
#include <Eigen/Eigen>
#include <memory>
#include <polyview/cameras/Camera.hpp>


namespace polyview
{
namespace cameras
{

struct OcamParams
{
  std::vector<double> pol;      // the polynomial coefficients
  std::vector<double> invPol;   // the inverse polynomial coefficients
  double xc;                    // row coordinate of the center
  double yc;                    // column coordinate of the center
  double c;                     // affine parameter
  double d;                     // affine parameter
  double e;                     // affine parameter
  int width;                    // image width
  int height;                   // image height
};

class Ocam : public Camera
{  
public:
  typedef std::shared_ptr<Ocam> Ptr;
  
  Ocam( const OcamParams & ocamParams );
  virtual ~Ocam();

  virtual Eigen::Vector3d camToWorld( const Eigen::Vector2d & imgPt ) const;
  virtual Eigen::Vector2d worldToCam( const Eigen::Vector3d & wrlPt ) const;

private:
  using Camera::_width;
  using Camera::_height;
  using Camera::_K;
  using Camera::_focalLength;
  
  OcamParams _op;
  double _invDet;
};

}
}

#endif /* POLYVIEW_CAMERAS_OCAM_HPP_ */
