#include "vl53l1x_sensor.h"

#include <Wire.h>
#include "Adafruit_VL53L1X.h"

namespace esphome {
namespace vl53l1x {

static const char *const TAG = "vl53l1x";

std::list<VL53L1XSensor *> VL53L1XSensor::vl53_sensors;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
bool VL53L1XSensor::enable_pin_setup_complete = false;   // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

VL53L1XSensor::VL53L1XSensor() { VL53L1XSensor::vl53_sensors.push_back(this); }

void VL53L1XSensor::dump_config() {
  LOG_SENSOR("", "VL53L1X", this);
}
void VL53L1XSensor::update(){
}
#define IRQ_PIN 12
#define XSHUT_PIN 23
#define I2C_SDA 19
#define I2C_SCL 22

TwoWire I2CWire=TwoWire(1);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN,IRQ_PIN);

void VL53L1XSensor::setup() {
  ESP_LOGD(TAG, "'%s' - setup BEGIN", this->name_.c_str());

  //vl53 = Adafruit_VL53L1X(XSHUT_PIN,IRQ_PIN);

  I2CWire.begin(I2C_SDA,I2C_SCL);

  ESP_LOGD(TAG, "'%s' - wire begin", this->name_.c_str());
  if(!vl53.begin(0x29,&I2CWire)){
    ESP_LOGD(TAG, "'%s' - Error initing VL53L1X", this->name_.c_str());
    ESP_LOGD(TAG, "'%s' - %s", this->name_.c_str(),vl53.vl_status);
    return;
  }

  ESP_LOGD(TAG, "'%s' - VL53L1X sensor OK!", this->name_.c_str());
  ESP_LOGD(TAG, "'%s' - sensor Id! %i", this->name_.c_str(),vl53.sensorID(),HEX);


  if (! vl53.startRanging()) {
    ESP_LOGD(TAG, "'%s' - Couldn't start ranging: %d ", this->name_.c_str(),vl53.vl_status);
    return;
  }

  vl53.setTimingBudget(50);


  ESP_LOGD(TAG, "'%s' - setup END", this->name_.c_str());
}

void VL53L1XSensor::loop() {
  int16_t distance;

  if(vl53.dataReady()){
    distance = vl53.distance();
    if (distance == -1){
       ESP_LOGD(TAG, "'%s' - Couldn't get distance: %d ", this->name_.c_str(),vl53.vl_status);
       ESP_LOGD(TAG, "'%s' - Couldn't get distance: ", this->name_.c_str());
       return;
    }
    float range_m = distance / 1000.0;
 
    this->publish_state(range_m);
    vl53.clearInterrupt();
  }else{
//    ESP_LOGD(TAG, "'%s' - no data", this->name_.c_str());
//    this->publish_state(-1);
  }

}

  
}  // namespace vl53l1x
}  // namespace esphome
