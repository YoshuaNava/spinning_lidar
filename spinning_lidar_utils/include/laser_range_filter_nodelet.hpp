#pragma once

// laser range filter
#include "spinning_lidar_utils/laser_range_filter.hpp"

// Nodelets
#include <nodelet/nodelet.h>

namespace spinning_lidar_utils {

class LaserRangeFilterNodelet : public nodelet::Nodelet, public LaserRangeFilter {
 public:
  LaserRangeFilterNodelet() : LaserRangeFilter() {}

  void onInit() override;
};

}  // namespace spinning_lidar_utils