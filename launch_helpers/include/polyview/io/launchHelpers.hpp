/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_IO_LAUNCHHELPERS_HPP_
#define POLYVIEW_IO_LAUNCHHELPERS_HPP_

#include <stdlib.h>
#include <polyview/tools/options.hpp>
#include <polyview/cameras/CameraSystem.hpp>
#include <vector>

namespace polyview
{
namespace io
{

polyview::cameras::CameraSystem::Ptr createFakeCamera();
int createCamera( tools::OptionSet & optionSet, cameras::CameraSystem::Ptr & cameraSystem );
int createCameraSystem( std::vector<tools::OptionSet> & optionSets, cameras::CameraSystem::Ptr & cameraSystem );
}
}

#endif /* POLYVIEW_IO_LAUNCHHELPERS_HPP_ */