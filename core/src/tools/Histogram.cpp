/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#include <polyview/tools/Histogram.hpp>
#include <algorithm>
#include <math.h>


polyview::tools::Histogram::Histogram( double minVal, double maxVal, size_t bins )
{
  _bins.resize(bins+1,0);
  _rightBorders.reserve(bins);

  _binSize = (maxVal - minVal) / bins;

  for( size_t i = 0; i < bins; i++ )
    _rightBorders.emplace_back(minVal + (i+1)*_binSize);

  _totalElements = 0;
}

polyview::tools::Histogram::Histogram( const Histogram & copy )
{
  _bins.resize(_rightBorders.size()+1,0);
  _rightBorders = copy._rightBorders;
  _binSize = copy._binSize;
  _totalElements = 0;
}

polyview::tools::Histogram::~Histogram()
{}

void
polyview::tools::Histogram::insert( double val )
{  
  size_t index = getRelevantBin(val);
  _bins[index]++;
  _totalElements++;
}

size_t
polyview::tools::Histogram::getRelevantBin( double val ) const
{
  //get some bottom value
  double bottomValue = _rightBorders[0]-_binSize;

  int index = floor( (val-bottomValue) / _binSize );
  if( index < 0 )
    index = 0;
  if( index >= _rightBorders.size() )
    index = _rightBorders.size() - 1;
  return index;
}

void
polyview::tools::Histogram::incrementRelevantBin( size_t bin, size_t increment )
{
  _bins[bin] += increment;
  _totalElements += increment;
}

void
polyview::tools::Histogram::clear()
{
  std::fill(_bins.begin(),_bins.end(),0);
  _totalElements = 0;
}

size_t
polyview::tools::Histogram::binCount( size_t bin ) const
{
  return _bins[bin];
}

size_t
polyview::tools::Histogram::totalCount() const
{
  return _totalElements;
}

double
polyview::tools::Histogram::percentileVal( double percentile ) const
{
  size_t countLimit = floor(percentile * _totalElements + 0.5f);
  size_t count = 0;

  size_t binIndex;
  for( binIndex = 0; binIndex < _bins.size(); binIndex++ )
  {
    count += _bins[binIndex];
    if( count > countLimit )
      break;
  }

  if( binIndex == _rightBorders.size() )
    return _rightBorders.back() + _binSize / 2.0;
  return _rightBorders[binIndex] - _binSize / 2.0;
}

void
polyview::tools::Histogram::getPeaks( std::list< std::pair<double,int> > & peaks ) const
{
  if( _bins[0] > _bins[1] )
    peaks.emplace_back( _rightBorders[0]-_binSize*0.5, _bins[0] );

  for( size_t i = 1; i < _bins.size()-1; i++ )
  {
    if( _bins[i] > _bins[i-1] && _bins[i] > _bins[i+1] )
      peaks.emplace_back( _rightBorders[i]-_binSize*0.5,_bins[i] );
  }

  if( _bins.back() > _bins[_bins.size()-2] )
    peaks.emplace_back( _rightBorders.back() + _binSize*0.5,_bins.back() );
}

void
polyview::tools::Histogram::plot( Eigen::ArrayXXf & histogramPlot ) const
{
  size_t maximum = _bins[0];
  for( size_t i = 1; i < _bins.size(); i++ )
  {
    if( _bins[i] > maximum )
      maximum = _bins[i];
  }

  size_t height = 400;
  float factor = maximum / 300.0f;

  histogramPlot = Eigen::ArrayXXf(height,_bins.size());
  histogramPlot.fill(255.0f);
  for( size_t i = 0; i < _bins.size(); i++ )
  {
    for( size_t h = 0; h < floor( _bins[i] / factor ); h++ )
      histogramPlot( height-1-h, i ) = 0.0f;
  }
}