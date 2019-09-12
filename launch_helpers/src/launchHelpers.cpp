/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/
#include <iostream>
#include <polyview/io/launchHelpers.hpp>
#include <polyview/cameras/Perspective.hpp>
#include <polyview/cameras/Ocam.hpp>
#include <Eigen/Eigen>
#include <string>
#include <dirent.h>


polyview::cameras::CameraSystem::Ptr
polyview::io::createFakeCamera()
{
  polyview::cameras::Camera::Ptr camera( new polyview::cameras::Perspective() );
  polyview::cameras::CameraSystem::Ptr cameraSystem( new polyview::cameras::CameraSystem() );
  cameraSystem->addCamera( camera, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() );
  return cameraSystem;
}

int
polyview::io::createCamera( tools::OptionSet & optionSet, cameras::CameraSystem::Ptr & cameraSystem )
{
  tools::OptionSet::const_iterator it = tools::findOption( optionSet, tools::options::CAM_POSITION );
  if( it == optionSet.end() )
  {
    //the position and orientation have not been provided, so set them to default
    Eigen::MatrixXd camPosition(3,1); camPosition << 0.0, 0.0, 0.0;
    tools::options::OptionType type = tools::options::CAM_POSITION;
    optionSet.push_back( tools::Option(type, camPosition ) );
    Eigen::MatrixXd camOrientation(3,3); camOrientation << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    type = tools::options::CAM_ORIENTATION;
    optionSet.push_back( tools::Option(type, camOrientation) );
  }

  std::vector<tools::OptionSet> optionSets;
  optionSets.push_back( optionSet );
  return createCameraSystem( optionSets, cameraSystem );
}

int
polyview::io::createCameraSystem( std::vector<tools::OptionSet> & optionSets, cameras::CameraSystem::Ptr & cameraSystem )
{
  cameraSystem = cameras::CameraSystem::Ptr( new cameras::CameraSystem() );

  for( size_t c = 0; c < optionSets.size(); c++ )
  {
    tools::OptionSet & optionSet = optionSets[c];

    //an option iterator
    tools::OptionSet::const_iterator it;

    it = tools::findOption( optionSet, tools::options::CAM_POSITION );
    if( it == optionSet.end() )
    {
      std::cout << "Please provide the position of the camera in the camera system\n";
      return -1;
    }
    Eigen::Vector3d camPosition = it->valueMat();

    it = tools::findOption( optionSet, tools::options::CAM_ORIENTATION );
    if( it == optionSet.end() )
    {
      std::cout << "Please provide the orientation of the camera in the camera system\n";
      return -1;
    }
    Eigen::Matrix3d camOrientation = it->valueMat();

    //find out whether this is an omni-cam model
    it = tools::findOption( optionSet, tools::options::OMNICAM_POLY );
    if( it != optionSet.end() )
    {
      //thsi is the case of an omni-camera
      cameras::OcamParams ocamParams;

      it = tools::findOption( optionSet, tools::options::IMAGE_SIZE );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the size of the camera images\n";
        return -1;
      }
      const Eigen::MatrixXd & size = it->valueMat();
      ocamParams.height = floor( size(0,0) + 0.5 );
      ocamParams.width = floor( size(1,0) + 0.5 );

      it = tools::findOption( optionSet, tools::options::PRINCIPAL_POINT );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the location of the principal point\n";
        return -1;
      }
      const Eigen::MatrixXd & pp = it->valueMat();
      ocamParams.xc = pp(0,0);
      ocamParams.yc = pp(1,0);

      it = tools::findOption( optionSet, tools::options::OMNICAM_POLY );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the coefficients for the omnicam polynomial\n";
        return -1;
      }
      const Eigen::MatrixXd & pol = it->valueMat();
      for( size_t i = 0; i < 5; i++ )
        ocamParams.pol.push_back(pol(i,0));

      it = tools::findOption( optionSet, tools::options::OMNICAM_INVPOLY );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the coefficients for the omnicam inverse polynomial\n";
        return -1;
      }
      const Eigen::MatrixXd & invPol = it->valueMat();
      for( size_t i = 0; i < invPol.rows(); i++ )
        ocamParams.invPol.push_back(invPol(i,0));

      it = tools::findOption( optionSet, tools::options::OMNICAM_AFFINE );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the coefficients for the omnicam affine coefficients\n";
        return -1;
      }
      const Eigen::MatrixXd & affine = it->valueMat();
      ocamParams.c = affine(0,0);
      ocamParams.d = affine(1,0);
      ocamParams.e = affine(2,0);

      cameras::Camera::Ptr camera( new cameras::Ocam( ocamParams ));
      cameraSystem->addCamera( camera, camPosition, camOrientation );
    }
    else
    {
      //this is the case of a perspective camera (eventually with radial distortion)
      it = tools::findOption( optionSet, tools::options::IMAGE_SIZE );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the size of the camera images\n";
        return -1;
      }
      const Eigen::MatrixXd & size = it->valueMat();
      int width = floor( size(0,0) + 0.5 );
      int height = floor( size(1,0) + 0.5 );
    
      it = tools::findOption( optionSet, tools::options::PRINCIPAL_POINT );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the location of the principal point\n";
        return -1;
      }
      const Eigen::MatrixXd & pp = it->valueMat();

      it = tools::findOption( optionSet, tools::options::FOCAL_LENGTH );
      if( it == optionSet.end() )
      {
        std::cout << "Please provide the focal length of the camera\n";
        return -1;
      }
      Eigen::Vector2d f = it->valueMat();

      Eigen::Matrix3d K;
      K <<    f(0), 0.0f,  pp(0,0),
           0.0f,    f(1),  pp(1,0),
           0.0f, 0.0f,     1.0f;

      it = tools::findOption( optionSet, tools::options::RADIAL_DISTORTION );
      if( it == optionSet.end() )
      {
        //we have a perspective camera
        cameras::Camera::Ptr camera( new cameras::Perspective(
            width,height,K) );
        cameraSystem->addCamera( camera, camPosition, camOrientation );
      }
      else
      {
        //we have a perspective camera with radial distortion
        /*Eigen::Matrix<double,5,1> distCoeff = it->valueMat();

        cameras::Camera::Ptr camera( new cameras::PerspectiveRad(
            width,height,K,distCoeff) );
        cameraSystem->addCamera( camera, camPosition, camOrientation );*/
        std::cout << "Please provide Undistorded camera images\n";
      }
    }
  }
  return 0;
}
