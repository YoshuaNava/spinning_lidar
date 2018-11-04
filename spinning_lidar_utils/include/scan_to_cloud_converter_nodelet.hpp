#pragma once

// scan to cloud converter
#include "spinning_lidar_utils/scan_to_cloud_converter.hpp"

// Nodelets
#include <nodelet/nodelet.h>

namespace spinning_lidar_utils {

class ScanToCloudConverterNodelet : public nodelet::Nodelet, public ScanToCloudConverter {
 public:
  ScanToCloudConverterNodelet() : ScanToCloudConverter() {}

  void onInit() override;
};

}  // namespace spinning_lidar_utils