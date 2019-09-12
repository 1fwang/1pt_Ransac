/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/Ocam.hpp>


polyview::cameras::Ocam::Ocam( const OcamParams & ocamParams ) :
    Camera( ocamParams.width, ocamParams.height, Eigen::Matrix3d::Identity(), 1.0 ),
    _op(ocamParams)
{
  //I've tried to figure out what the focal length might be, but without big success,
  //Best guess for the moment is _op.invPol[1]

  _focalLength = fabs(_op.pol[0]);
  _K(0,0) = _focalLength;
  _K(1,1) = _focalLength;
  _K(0,2) = _op.yc;
  _K(1,2) = _op.xc;

  _invDet  = 1.0/( _op.c - _op.d * _op.e );
}

polyview::cameras::Ocam::~Ocam()
{}

Eigen::Vector3d
polyview::cameras::Ocam::camToWorld( const Eigen::Vector2d & imgPt ) const
{
  double inverse_scale = 1 / 0.5;
  Eigen::Vector2d scaled_imgPt = imgPt * inverse_scale;
  const double & x = scaled_imgPt[0] - _op.xc;
  const double & y = scaled_imgPt[1] - _op.yc;

  // Important: we exchange x and y since our convention is to work with x
  // along the columns and y along the rows (different from Ocam convention)
  Eigen::Vector2d worldCoordinates;
  worldCoordinates[0] = _invDet * ( x - _op.d * y );
  worldCoordinates[1] = _invDet * ( -_op.e * x + _op.c * y );

  // r = distance [pixels] of the point from the image center
  double r = worldCoordinates.norm();
  double z = _op.pol[0];
  double rEi = 1.0;

  for( size_t i = 1; i < _op.pol.size(); i++ )
  {
    rEi *= r;
    z += rEi * _op.pol[i];
  }
  
  // change back to our axis convention:  
  Eigen::Vector3d wc;
  
  wc[0] = worldCoordinates[0];
  wc[1] = worldCoordinates[1];
  wc[2] = z;
  
  wc.normalize();
  return wc;
}

Eigen::Vector2d
polyview::cameras::Ocam::worldToCam( const Eigen::Vector3d & wrlPt ) const
{
  Eigen::Vector2d imageCoordinates;
  double scale = 0.5;
  // transform world-coordinates to Davide's camera frame
  Eigen::Vector3d worldCoordinates_bis;
  worldCoordinates_bis[0] =  wrlPt[0];
  worldCoordinates_bis[1] =  wrlPt[1];
  worldCoordinates_bis[2] =  wrlPt[2];

  double norm = sqrt(
      worldCoordinates_bis[0]*worldCoordinates_bis[0] +
      worldCoordinates_bis[1]*worldCoordinates_bis[1] );
  double theta = atan( worldCoordinates_bis[2]/norm );

  // Important: we exchange x and y since the Ocam convention is not common
  if(norm > 1e-10)
  {
    double rho = _op.invPol[0];
    double tEi = 1.0;

    for( size_t i = 1; i < _op.invPol.size(); i++ )
    {
      tEi *= theta;
      rho += tEi * _op.invPol[i];
    }

    double temp = rho/norm;
    double x = worldCoordinates_bis[0] * temp;
    double y = worldCoordinates_bis[1] * temp;

    // we exchange 0 and 1 in order to have our convention again
    imageCoordinates[0] = x * _op.c + y * _op.d + _op.xc;
    imageCoordinates[1] = x * _op.e + y + _op.yc;
  }
  else
  {
    // we exchange 0 and 1 in order to have pinhole model again
    imageCoordinates[0] = _op.xc;
    imageCoordinates[1] = _op.yc;
  }

  imageCoordinates = imageCoordinates * scale;

  return imageCoordinates;
}