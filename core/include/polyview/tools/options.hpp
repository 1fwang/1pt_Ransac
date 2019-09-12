/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_TOOLS_OPTIONS_HPP_
#define POLYVIEW_TOOLS_OPTIONS_HPP_

#include <stdlib.h>
#include <Eigen/Eigen>
#include <string>
#include <list>
#include <vector>

namespace polyview
{
namespace tools
{

namespace options
{

enum OptionType
{
  NONE,
  TARGETS_FOLDER,    //-tf
  TARGETS_PREFIX,    //-tp
  ISONLINE,          //-o
  SHUTTER,           //-s
  COLOR,             //-c
  INPUT_FOLDER,      //-if
  INPUT_PREFIX,      //-ip
  IMAGE_SIZE,        //-i
  PRINCIPAL_POINT,   //-p
  FOCAL_LENGTH,      //-f
  FOCAL_LENGTH2,      //-f2 // added by yi zhou, focal length in x and y coordinate respectively.
  RADIAL_DISTORTION, //-r
  OMNICAM_POLY,      //-ocp
  OMNICAM_INVPOLYL,  //-ocl
  OMNICAM_INVPOLY,   //-oci
  OMNICAM_AFFINE,    //-oca
  OUTPUT_FOLDER,     //-of
  OUTPUT_PREFIX,     //-op
  CAM_POSITION,      //not supported from the command line
  CAM_ORIENTATION,   //not supported from the command line
  //added by yi zhou
  CONFIGURATION_FOLDER, //-cf
  SEQUENCE_FOLDER,      //-sf
  VOCABULARY_FOLDER,    //-vf
  // SAVE_FOLDER,          //-svf
  GROUNDTRUTH,          //-gt
  ASSOCIATION,          //-ass
  VISUALIZATION,        //-v
  SAVE_TRAJECTORY,      //-st
  SLIDING_WINDOW,       //-sw
  RELOCALIZATION,       //-rl
  LOOP_CLOSING,         //-lc
  PYRAMID_LEVEL,        //-pl
  SPECIFIC_KINECT,      //-sk
  OFFSET,               //-offset
  RECON_CONFIG,         //-rcfg
  RECON_SAVE_PATH       //-rsp
};

}

struct Option
{
private:
  options::OptionType _type;
  std::string _valueString;
  int _valueInt;
  double _valueFloat;
  Eigen::MatrixXd _valueMat;

public:
  Option();
  Option( options::OptionType & type );
  Option( options::OptionType & type, std::string & value );
  Option( options::OptionType & type, int & value );
  Option( options::OptionType & type, double & value );
  Option( options::OptionType & type, Eigen::MatrixXd & value );

  const options::OptionType & type() const;
  options::OptionType & type();
  const std::string & valueString() const;
  std::string & valueString();
  const int & valueInt() const;
  int & valueInt();
  const double & valueFloat() const;
  double & valueFloat();
  const Eigen::MatrixXd & valueMat() const;
  Eigen::MatrixXd & valueMat();

  void clear();
};

typedef std::list<Option> OptionSet;

int extractOptions( int argc, char** argv, OptionSet & optionSet );
OptionSet::const_iterator findOption( const OptionSet & optionSet, const options::OptionType & type );
void printOptions( OptionSet & optionSet );

}
}

#endif /* POLYVIEW_TOOLS_OPTIONS_HPP_ */