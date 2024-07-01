/*
 * StepFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/NegativeStepFilter.hpp"
#include <pluginlib/class_list_macros.h>
#include <algorithm>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

namespace filters {

template<typename T>
NegativeStepFilter<T>::NegativeStepFilter()
    : max_allowed_step_depth_(0.15),
      sample_distance_cells_(10),
      //firstWindowRadius_(0.08),
      //secondWindowRadius_(0.08),
      nCellCritical_(3),
      type_("traversability_negative_step")
{

}

template<typename T>
NegativeStepFilter<T>::~NegativeStepFilter()
{

}

template<typename T>
bool NegativeStepFilter<T>::configure()
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

  if (!FilterBase<T>::getParam(std::string("sample_distance_cells"),
                               sample_distance_cells_)) {
    ROS_ERROR("Step filter did not find param 'sample_distance_cells'.");
    return false;
  }

  if (sample_distance_cells_ < 0) {
    ROS_ERROR("'sample_distance_cells' must be greater than zero.");
    return false;
  }

  if (!FilterBase<T>::getParam(std::string("critical_cell_number"),
                               nCellCritical_)) {
    ROS_ERROR("Step filter did not find param 'critical_cell_number'.");
    return false;
  }

  if (nCellCritical_ <= 0) {
    ROS_ERROR("'critical_cell_number' must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Number of critical cells of step filter = %d.", nCellCritical_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Step filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("Negative Step map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool NegativeStepFilter<T>::update(const T& mapIn, T& mapOut)
{
  int sample_dist_cells = sample_distance_cells_;
  std::vector<Eigen::Array2i> sample_offsets;
  sample_offsets.push_back(Eigen::Array2i(-sample_dist_cells, -sample_dist_cells));
  sample_offsets.push_back(Eigen::Array2i(                 0, -sample_dist_cells));
  sample_offsets.push_back(Eigen::Array2i( sample_dist_cells, -sample_dist_cells));
  sample_offsets.push_back(Eigen::Array2i(-sample_dist_cells,                  0));
  sample_offsets.push_back(Eigen::Array2i( sample_dist_cells,                  0));
  sample_offsets.push_back(Eigen::Array2i(-sample_dist_cells,  sample_dist_cells));
  sample_offsets.push_back(Eigen::Array2i(                 0,  sample_dist_cells));
  sample_offsets.push_back(Eigen::Array2i( sample_dist_cells,  sample_dist_cells));



  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double height, step;

  size_t num_samples = sample_offsets.size();

  const grid_map::Matrix& elevation_data = mapOut["elevation"];

  // First iteration through the elevation map.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {

    //grid_map::Index cindex(*iterator);
    //grid_map::wrapIndexToRange(cindex, mapOut.getSize());

    //const float curr_elevation_data = elevation_data(cindex.x(), cindex.y());
    //std::cout << " el: " << curr_elevation_data << "\n";



    if (!mapOut.isValid(*iterator, "elevation"))
      continue;
    height = mapOut.at("elevation", *iterator);

    const grid_map::Index curr_itr_index(*iterator);

    int num_negative_step_confirm_samples = 0;

    for (size_t i = 0; i < num_samples; ++i){

      grid_map::Index line_end_index = curr_itr_index + sample_offsets[i];
      grid_map::wrapIndexToRange(line_end_index, mapOut.getSize());

      //std::cout << "ci:\n" << curr_itr_index << "\nle:\n" << line_end_index << "\n";


      int unknown_cell_count = 0;
      int line_cell_index = 0;

      for (grid_map::LineIterator line_iterator(mapOut, curr_itr_index, line_end_index);
              !line_iterator.isPastEnd(); ++line_iterator) {

        //First cell is known valid, at height elevation
        if (line_cell_index == 0){
          line_cell_index++;
        }else{

          grid_map::Index curr_line_index(*line_iterator);
          grid_map::wrapIndexToRange(curr_line_index, mapOut.getSize());

          const float curr_elevation_data = elevation_data(curr_line_index.x(), curr_line_index.y());
          //std::cout << line_cell_index << " el: " << curr_elevation_data << "\n" << curr_line_index << "\n";


          if (std::isnan(curr_elevation_data)){
            unknown_cell_count++;
            //std::cout << "unk\n";
          }else{
            if (unknown_cell_count==0){
              // No free neighbor cell, abort
              //std::cout << "abort\n";
              break;
            }else{
              if (unknown_cell_count > 3){
                if (height - curr_elevation_data > max_allowed_step_depth_){
                  num_negative_step_confirm_samples++;
                }

              }
              break;
            }
          }

          line_cell_index++;
        }
      }
    }

    mapOut.at(type_, *iterator) = 1.0 - std::min(1.0f, static_cast<float>(num_negative_step_confirm_samples)/static_cast<float>(nCellCritical_));
  }
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(filters::NegativeStepFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
