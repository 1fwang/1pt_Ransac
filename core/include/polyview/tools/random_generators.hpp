/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_TOOLS_RANDOM_GENERATORS_HPP_
#define POLYVIEW_TOOLS_RANDOM_GENERATORS_HPP_

#include <stdlib.h>
#include <Eigen/Eigen>

namespace polyview
{
namespace tools
{

Eigen::Vector3d generateRandomTranslation( double maximumParallax );
Eigen::Matrix3d generateRandomRotation( double maxAngle );

}
}

#endif /* POLYVIEW_TOOLS_RANDOM_GENERATORS_HPP_ */