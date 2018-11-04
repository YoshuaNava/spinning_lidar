#pragma once

// interrupt laser assembler
#include "spinning_lidar_utils/interrupt_laser_assembler.hpp"

// Nodelets
#include <nodelet/nodelet.h>

namespace spinning_lidar_utils {

class InterruptLaserAssemblerNodelet : public nodelet::Nodelet, public InterruptLaserAssembler {
 public:
  InterruptLaserAssemblerNodelet() : InterruptLaserAssembler() {}

  void onInit() override;
};

}  // namespace spinning_lidar_utils