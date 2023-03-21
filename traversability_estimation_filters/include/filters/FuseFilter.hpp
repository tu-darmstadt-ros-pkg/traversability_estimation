/*
 * StepFilter.hpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef FUSEFILTER_HPP
#define FUSEFILTER_HPP

#include <filters/filter_base.h>

#include <string>

namespace filters {

/*!
 * Fuse Filter class to compute the minimum traversability from multiple input layers
 */
template<typename T>
class FuseFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  FuseFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Step map type.
  std::string type_;

  //! Layers to be fused
  std::vector<std::string> layer_strings_;
};

} /* namespace */

#endif
