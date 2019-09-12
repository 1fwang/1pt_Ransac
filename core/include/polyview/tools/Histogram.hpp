/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

#ifndef POLYVIEW_TOOLS_HISTOGRAM_HPP_
#define POLYVIEW_TOOLS_HISTOGRAM_HPP_

#include <stdlib.h>
#include <vector>
#include <list>
#include <Eigen/Eigen>
#include <memory>
#include <polyview/BaseClass.hpp>


namespace polyview
{
namespace tools
{

class Histogram : public BaseClass
{
public:
  typedef std::shared_ptr<Histogram> Ptr;

  Histogram( double minVal = 0.0f, double maxVal = 20.0f, size_t bins = 500 );
  Histogram( const Histogram & copy );
  virtual ~Histogram();

  void insert( double val );
  size_t getRelevantBin( double val ) const;
  void incrementRelevantBin( size_t bin, size_t increment = 1 );
  void clear();
  size_t binCount( size_t bin ) const;
  size_t totalCount() const;
  double percentileVal( double percentile ) const;
  void getPeaks( std::list< std::pair<double,int> > & peaks ) const;

  void plot( Eigen::ArrayXXf & histogramPlot ) const;

private:
  std::vector<size_t> _bins;
  double _binSize;
  std::vector<double> _rightBorders;
  size_t _totalElements;
};

}
}

#endif /* POLYVIEW_TOOLS_HISTOGRAM_HPP_ */