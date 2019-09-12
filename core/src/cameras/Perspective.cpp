/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/Perspective.hpp>


polyview::cameras::Perspective::Perspective() : 
    Camera(1,1,Eigen::Matrix3d::Identity(),1.0),
    _invK(Eigen::Matrix3d::Identity())
{}

polyview::cameras::Perspective::Perspective(
    int width,
    int height,
    const Eigen::Matrix3d & K ) :
    Camera(width,height,K,(K(0,0) + K(1,1))/2.0),
    _invK(K.inverse())
{}

polyview::cameras::Perspective::Perspective(
    int width,
    int height,
    double focalLength ) :
    Camera(width,height,Eigen::Matrix3d::Identity(),focalLength)
{
  _K(0,0) = focalLength;
  _K(1,1) = focalLength;
  _K(0,2) = width / 2.0;
  _K(1,2) = height / 2.0;
  
  _invK = _K.inverse();
}

polyview::cameras::Perspective::Perspective(
    int width,
    int height,
    double focalLength_mm,
    double sensorWidth_mm,
    double sensorHeight_mm ) :
    Camera(width,height,Eigen::Matrix3d::Identity(),1.0)
{
  double pixelWidth_mm = sensorWidth_mm / width;
  double pixelHeight_mm = sensorHeight_mm / height;
  double focalLength_x = focalLength_mm / pixelWidth_mm;
  double focalLength_y = focalLength_mm / pixelHeight_mm;
  
  _K(0,0) = focalLength_x;
  _K(1,1) = focalLength_y;
  _K(0,2) = width / 2.0;
  _K(1,2) = height / 2.0;
  _focalLength = (focalLength_x + focalLength_y)/2.0;
  
  _invK = _K.inverse();
}

polyview::cameras::Perspective::~Perspective()
{}

Eigen::Vector3d
polyview::cameras::Perspective::camToWorld( const Eigen::Vector2d & imgPt ) const
{
	Eigen::Vector3d worldCoordinates = _invK.block<3,2>(0,0) * imgPt + _invK.col(2);
	worldCoordinates.normalize();
	return worldCoordinates;
}

Eigen::Vector2d
polyview::cameras::Perspective::worldToCam( const Eigen::Vector3d & wrlPt ) const
{
	Eigen::Vector2d imageCoordinates;
	Eigen::Vector3d imageCoordinatesHom = _K * wrlPt;
	imageCoordinates[0] = imageCoordinatesHom[0] / imageCoordinatesHom[2];
	imageCoordinates[1] = imageCoordinatesHom[1] / imageCoordinatesHom[2];
	return imageCoordinates;
}
