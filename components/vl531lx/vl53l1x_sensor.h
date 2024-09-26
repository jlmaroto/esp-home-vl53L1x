#pragma once

#include <list>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace vl53l1x {


class VL53L1XSensor : public sensor::Sensor, public PollingComponent, public i2c::I2CDevice {
 public:
  VL53L1XSensor();

  void setup() override;

  void dump_config() override;
  void update() override;

  void loop() override;

 protected:
  static std::list<VL53L1XSensor *> vl53_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
  static bool enable_pin_setup_complete;           // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
};

}  // namespace vl53l1x
}  // namespace esphome
