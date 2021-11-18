/*
 * StepFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/FuseFilter.hpp"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <limits>
#include <vector>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

namespace filters {

template<typename T>
FuseFilter<T>::FuseFilter()
    : max_allowed_step_depth_(0.15),
      sample_distance_cells_(10),
      //firstWindowRadius_(0.08),
      //secondWindowRadius_(0.08),
      nCellCritical_(3),
      type_("traversability")
{

}

template<typename T>
FuseFilter<T>::~FuseFilter()
{

}

template<typename T>
bool FuseFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("max_allowed_step_depth"), max_allowed_step_depth_)) {
    ROS_ERROR("Step filter did not find param max_allowed_step_depth.");
    return false;
  }

  if (max_allowed_step_depth_ < 0.0) {
    ROS_ERROR("Max allowed step depth must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Max allowed step depth = %f.", max_allowed_step_depth_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Step filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("Negative Step map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool FuseFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(type_);

  const std::vector<std::string> layer_strings = { "traversability_slope", "traversability_step",
                                                   "traversability_negative_step", "traversability_roughness" };

  std::vector<Eigen::Ref<grid_map::Matrix>> layers;
  for (const auto& layer : layer_strings)
  {
    if (mapOut.exists(layer))
    {
      layers.emplace_back(mapOut[layer]);
    }
  }

  grid_map::Matrix& traversability_data = mapOut[type_];

  // First iteration through the elevation map.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {

    //grid_map::wrapIndexToRange(cindex, mapOut.getSize());

    if (!mapOut.isValid(*iterator, "elevation"))
      continue;

    grid_map::Index curr_index(*iterator);

    float min = std::numeric_limits<float>::infinity();
    for (const auto& layer : layers)
    {
      const float& value = layer(curr_index.x(), curr_index.y());

      if (value < min)
      {
        min = value;
      }
    }

    traversability_data(curr_index.x(), curr_index.y()) = min;
  }
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(filters::FuseFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
