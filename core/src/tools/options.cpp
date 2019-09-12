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
#include <polyview/tools/options.hpp>
#include <sstream>


polyview::tools::Option::Option()
{
  clear();
}

polyview::tools::Option::Option( options::OptionType & type )
{
  clear();
  _type = type;
}

polyview::tools::Option::Option( options::OptionType & type, std::string & value )
{
  clear();
  _type = type;
  _valueString = value;
}

polyview::tools::Option::Option( options::OptionType & type, int & value )
{
  clear();
  _type = type;
  _valueInt = value;
}

polyview::tools::Option::Option( options::OptionType & type, double & value )
{
  clear();
  _type = type;
  _valueFloat = value;
}

polyview::tools::Option::Option( options::OptionType & type, Eigen::MatrixXd & value )
{
  clear();
  _type = type;
  _valueMat = value;
}

const polyview::tools::options::OptionType &
polyview::tools::Option::type() const
{
  return _type;
}

polyview::tools::options::OptionType &
polyview::tools::Option::type()
{
  return _type;
}

const std::string &
polyview::tools::Option::valueString() const
{
  return _valueString;
}

std::string &
polyview::tools::Option::valueString()
{
  return _valueString;
}

const int &
polyview::tools::Option::valueInt() const
{
  return _valueInt;
}

int &
polyview::tools::Option::valueInt()
{
  return _valueInt;
}

const double &
polyview::tools::Option::valueFloat() const
{
  return _valueFloat;
}

double &
polyview::tools::Option::valueFloat()
{
  return _valueFloat;
}

const Eigen::MatrixXd &
polyview::tools::Option::valueMat() const
{
  return _valueMat;
}

Eigen::MatrixXd &
polyview::tools::Option::valueMat()
{
  return _valueMat;
}

void
polyview::tools::Option::clear()
{
  _type = options::NONE;
  _valueString = std::string("");
  _valueInt = 0;
  _valueFloat = 0.0;
  _valueMat = Eigen::MatrixXd();
}

int
polyview::tools::extractOptions( int argc, char** argv, OptionSet & optionSet )
{
  Option currentOption;
  int argumentCountdown = 0;

  for( size_t i = 1; i < argc; i++ )
  {
    std::string argument(argv[i]);

    if( argumentCountdown == 0 )
    {
      currentOption.clear();

      if( argument.at(0) != '-' )
      {
        std::cout << argument << ": expected option tag (starts with -)\n";
        return -1;
      }
      else
      {
        if( argument == std::string("-tf") )
        {
          currentOption.type() = options::TARGETS_FOLDER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-tp") )
        {
          currentOption.type() = options::TARGETS_PREFIX;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-o") )
        {
          currentOption.type() = options::ISONLINE;
          argumentCountdown = 0;
        }
        else if ( argument == std::string("-s") )
        {
          currentOption.type() = options::SHUTTER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-c") )
        {
          currentOption.type() = options::COLOR;
          argumentCountdown = 0;
        }
        else if ( argument == std::string("-if") )
        {
          currentOption.type() = options::INPUT_FOLDER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-ip") )
        {
          currentOption.type() = options::INPUT_PREFIX;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-i") )
        {
          currentOption.type() = options::IMAGE_SIZE;
          currentOption.valueMat() = Eigen::MatrixXd(2,1);
          argumentCountdown = 2;
        }
        else if ( argument == std::string("-p") )
        {
          currentOption.type() = options::PRINCIPAL_POINT;
          currentOption.valueMat() = Eigen::MatrixXd(2,1);
          argumentCountdown = 2;
        }
        else if ( argument == std::string("-f") )
        {
          currentOption.type() = options::FOCAL_LENGTH;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-r") )
        {
          currentOption.type() = options::RADIAL_DISTORTION;
          currentOption.valueMat() = Eigen::MatrixXd(5,1);
          argumentCountdown = 5;
        }
        else if (argument == std::string("-ocp") )
        {
          currentOption.type() = options::OMNICAM_POLY;
          currentOption.valueMat() = Eigen::MatrixXd(5,1);
          argumentCountdown = 5;
        }
        else if (argument == std::string("-ocl") )
        {
          currentOption.type() = options::OMNICAM_INVPOLYL;
          argumentCountdown = 1;
        }
        else if (argument == std::string("-oci") )
        {
          //Attention: -ocl needs to be provided before -oci!
          currentOption.type() = options::OMNICAM_INVPOLY;
          OptionSet::const_iterator sit = findOption( optionSet, options::OMNICAM_INVPOLYL );
          size_t invPolyLength = sit->valueInt();
          currentOption.valueMat() = Eigen::MatrixXd(invPolyLength,1);
          argumentCountdown = invPolyLength;
        }
        else if (argument == std::string("-oca") )
        {
          currentOption.type() = options::OMNICAM_AFFINE;
          currentOption.valueMat() = Eigen::MatrixXd(3,1);
          argumentCountdown = 3;
        }
        else if ( argument == std::string("-of") )
        {
          currentOption.type() = options::OUTPUT_FOLDER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-op") )
        {
          currentOption.type() = options::OUTPUT_PREFIX;
          argumentCountdown = 1;
        }
        //added by yi zhou
        else if ( argument == std::string("-cf") )
        {
          currentOption.type() = options::CONFIGURATION_FOLDER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-sf") )
        {
          currentOption.type() = options::SEQUENCE_FOLDER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-vf") )
        {
          currentOption.type() = options::VOCABULARY_FOLDER;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-gt") )
        {
          currentOption.type() = options::GROUNDTRUTH;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-ass") )
        {
          currentOption.type() = options::ASSOCIATION;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-v") )
        {
          currentOption.type() = options::VISUALIZATION;
          argumentCountdown = 0;
        }
        else if ( argument == std::string("-st") )
        {
          currentOption.type() = options::SAVE_TRAJECTORY;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-sw") )
        {
          currentOption.type() = options::SLIDING_WINDOW;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-rl") )
        {
          currentOption.type() = options::RELOCALIZATION;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-lc") )
        {
          currentOption.type() = options::LOOP_CLOSING;
          argumentCountdown = 0;
        }
        else if ( argument == std::string("-pl") )
        {
          currentOption.type() = options::PYRAMID_LEVEL;
          argumentCountdown = 1;
        }
        else if ( argument == std::string("-sk") )
        {
          currentOption.type() = options::SPECIFIC_KINECT;
          argumentCountdown = 0;
        }
        else if ( argument == std::string("-offset") )
        {
          currentOption.type() = options::OFFSET;
          argumentCountdown = 1;
        }
        else if (argument == std::string("-rcfg") )
        {
          currentOption.type() = options::RECON_CONFIG;
          argumentCountdown = 1;
        }
        else if (argument == std::string("-rsp") )
        {
          currentOption.type() = options::RECON_SAVE_PATH;
          argumentCountdown = 1;
        }
        else
        {
          std::cout << argument << ": unrecognized option\n";
          return -1;
        }
      } 
    }
    else
    {
      if( currentOption.type() == options::PRINCIPAL_POINT || currentOption.type() == options::IMAGE_SIZE ) //added by yi zhou
      {
        std::stringstream ss(argument); double temp; ss >> temp;
        currentOption.valueMat()( 2-argumentCountdown,0 ) = temp;
      }
      else if( currentOption.type() == options::OMNICAM_AFFINE )
      {
        std::stringstream ss(argument); double temp; ss >> temp;
        currentOption.valueMat()( 3-argumentCountdown,0 ) = temp;
      }
      else if( currentOption.type() == options::RADIAL_DISTORTION || currentOption.type() == options::OMNICAM_POLY )
      {
        std::stringstream ss(argument); double temp; ss >> temp;
        currentOption.valueMat()( 5-argumentCountdown,0 ) = temp;
      }
      else if( currentOption.type() == options::OMNICAM_INVPOLY )
      {
        std::stringstream ss(argument); double temp; ss >> temp;
        currentOption.valueMat()( 6-argumentCountdown,0 ) = temp;
      }
      else if( currentOption.type() == options::FOCAL_LENGTH || currentOption.type() == options::RELOCALIZATION )
      {
        std::stringstream ss(argument); double temp; ss >> temp;
        currentOption.valueFloat() = temp;
      }
      else if( currentOption.type() == options::SHUTTER || currentOption.type() == options::OMNICAM_INVPOLYL ||
        currentOption.type() == options::SLIDING_WINDOW || currentOption.type() == options::PYRAMID_LEVEL ||
        currentOption.type() == options::RECON_CONFIG ) //added by yi zhou
      {
        std::stringstream ss(argument); int temp; ss >> temp;
        currentOption.valueInt() = temp;
      }
      else if( currentOption.type() == options::OFFSET )
      {
        std::stringstream ss(argument); int temp; ss >> temp;
        currentOption.valueInt() = size_t(temp); 
      }
      else
        currentOption.valueString() = argument;
        
      argumentCountdown--;
    }

    if( argumentCountdown == 0 )
      optionSet.push_back( currentOption );
  }

  if( argumentCountdown > 0 )
  {
    std::cout << "incomplete option at the end of instruction\n";
    return -1;
  }
  return 1;
}

polyview::tools::OptionSet::const_iterator
polyview::tools::findOption( const OptionSet & optionSet, const options::OptionType & type )
{
  OptionSet::const_iterator it = optionSet.begin();
  while( it != optionSet.end() )
  {
    if( it->type() == type )
      break;
    it++;
  }
  return it;
}

void
polyview::tools::printOptions( OptionSet & optionSet )
{
  OptionSet::const_iterator it = optionSet.begin();
  while( it != optionSet.end() )
  {
    switch( it->type() )
    {
      case options::NONE:
      {
        std::cout << "Unrecognized option\n";
        break;
      }
      case options::TARGETS_FOLDER:
      {
        std::cout << "Folder of AR-targets: " << it->valueString() << "\n";
        break;
      }
      case options::TARGETS_PREFIX:
      {
        std::cout << "Prefix of AR-target images: " << it->valueString() << "\n";
        break;
      }
      case options::ISONLINE:
      {
        std::cout << "the execution is online\n";
        break;
      }
      case options::SHUTTER:
      {
        std::cout << "the shutter time is: " << it->valueInt() << "\n";
        break;
      }
      case options::COLOR:
      {
        std::cout << "color is enabled\n";
        break;
      }
      case options::INPUT_FOLDER:
      {
        std::cout << "Folder of images: " << it->valueString() << "\n";
        break;
      }
      case options::INPUT_PREFIX:
      {
        std::cout << "Prefix of images: " << it->valueString() << "\n";
        break;
      }
      case options::IMAGE_SIZE:
      {
        std::cout << "The size of the image is: " << Eigen::Ref<const Eigen::MatrixXd>(it->valueMat().transpose()) << "\n";
        break;
      }
      case options::PRINCIPAL_POINT:
      {
        std::cout << "Principal point: " << Eigen::Ref<const Eigen::MatrixXd>(it->valueMat().transpose()) << "\n";
        break;
      }
      case options::FOCAL_LENGTH:
      {
        std::cout << "Focal length: " << it->valueFloat() << "\n";
        break;
      }
      case options::RADIAL_DISTORTION:
      {
        std::cout << "Radial distortion parameters: " << Eigen::Ref<const Eigen::MatrixXd>(it->valueMat().transpose()) << "\n";
        break;
      }
      case options::OMNICAM_POLY:
      {
        std::cout << "Omnicam polynomial coefficients: " << Eigen::Ref<const Eigen::MatrixXd>(it->valueMat().transpose()) << "\n";
        break;
      }
      case options::OMNICAM_INVPOLYL:
      {
        std::cout << "Omnicam inverse polynomial length: " << it->valueInt() << "\n";
        break;
      }
      case options::OMNICAM_INVPOLY:
      {
        std::cout << "Omnicam inverse polynomial coefficients: " << Eigen::Ref<const Eigen::MatrixXd>(it->valueMat().transpose()) << "\n";
        break;
      }
      case options::OMNICAM_AFFINE:
      {
        std::cout << "Omnicam affine coefficients: " << Eigen::Ref<const Eigen::MatrixXd>(it->valueMat().transpose()) << "\n";
        break;
      }
      case options::OUTPUT_FOLDER:
      {
        std::cout << "The folder for the output images is: " << it->valueString() << "\n";
        break;
      }
      case options::OUTPUT_PREFIX:
      {
        std::cout << "The prefix for the output images is: " << it->valueString() << "\n";
        break;
      }
      case options::CONFIGURATION_FOLDER:
      {
        std::cout << "The setting file is at: " << it->valueString() << "\n";
        break;
      }
      case options::SEQUENCE_FOLDER:
      {
        std::cout << "The sequence is at: " << it->valueString() << "\n";
        break;
      }
      case options::VOCABULARY_FOLDER:
      {
        std::cout << "The vocabulary is at: " << it->valueString() << "\n";
        break;
      }
      case options::GROUNDTRUTH:
      {
        std::cout << "The groundtruth is at: " << it->valueString() << "\n";
        break;
      }
      case options::ASSOCIATION:
      {
        std::cout << "The association file is at: " << it->valueString() << "\n";
        break; 
      }
      case options::VISUALIZATION:
      {
        std::cout << "Visualization is enabled!\n";
        break;
      }
      case options::SAVE_TRAJECTORY:
      {
        std::cout << "Save trajectory is enabled!\n";
        break;
      }
      case options::SLIDING_WINDOW:
      {
        std::cout << "Sliding window is enabled, and size of the window is: " << it->valueInt() << "\n";
        break;
      }
      case options::RELOCALIZATION:
      {
        std::cout << "Relocalization is enabled, the lost tracking threshold (in disparity) is: " << it->valueInt() << "\n";
        break;
      }
      case options::LOOP_CLOSING:
      {
        std::cout << "Loop closing is enabled!\n";
        break;
      }
      case options::PYRAMID_LEVEL:
      {
        std::cout << "The number of pyramid level: " << it->valueInt() << "\n";
        break;
      }
      case options::SPECIFIC_KINECT:
      {
        std::cout << "We are using an specific KINECT!\n";
        break;
      }
      case options::OFFSET:
      {
        std::cout << "We are using RSISE sequence with a given offset: " << it->valueInt() << "\n";
        break;
      }
      case options::RECON_CONFIG:
      {
        std::cout << "The vocabulary is at: " << it->valueInt() << "\n";
        break;
      }
      case options::RECON_SAVE_PATH:
      {
        std::cout << "The vocabulary is at: " << it->valueString() << "\n";
        break;
      }
      default:
      {
        std::cout << "Unrecognized option \n";
        break;
      }
    }

    it++;
  }
}
