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
    : type_("traversability")
{}

template<typename T>
bool FuseFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Step filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("Fuse Filter map type = %s.", type_.c_str());

  if (!FilterBase<T>::getParam(std::string("layers"), layer_strings_)) {
    ROS_ERROR("DeletionFilter did not find parameter 'layers'.");
    return false;
  }

  return true;
}

template<typename T>
bool FuseFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add(type_);

  std::vector<Eigen::Ref<grid_map::Matrix>> layers;
  for (const auto& layer : layer_strings_)
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

    if (!mapOut.isValid(*iterator))
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
