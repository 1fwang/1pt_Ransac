/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/cameras/NoDistortion.hpp>


polyview::cameras::NoDistortion::NoDistortion() : Distortion()
{}

polyview::cameras::NoDistortion::~NoDistortion()
{}

void
polyview::cameras::NoDistortion::distort( Eigen::Vector2d & imgPt ) const
{}

void
polyview::cameras::NoDistortion::undistort( Eigen::Vector2d & imgPt ) const
{}
