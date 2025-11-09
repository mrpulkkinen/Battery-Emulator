#ifndef MQB_EV_BATTERY_H
#define MQB_EV_BATTERY_H

#include <memory>
#include "../datalayer/datalayer.h"
#include "CanBattery.h"

#define MQB_EV_MAX_PACK_VOLTAGE_dV 4030   // 403.0 V upper limit
#define MQB_EV_MIN_PACK_VOLTAGE_dV 3000   // 300.0 V lower limit
#define MQB_EV_MAX_CELL_DEVIATION_mV 150  // 0.15 V max deviation
#define MQB_EV_MAX_CELL_VOLTAGE_mV 4200
#define MQB_EV_MIN_CELL_VOLTAGE_mV 3100
#define MQB_EV_NOMINAL_CAPACITY_Wh 35800  // e-Golf 35.8 kWh pack
#define MQB_EV_MAX_MODULES 16
#define MQB_EV_CELLS_PER_MODULE 12

class MQBModuleManager;

class MqbEvBattery : public CanBattery {
 public:
  MqbEvBattery();

  void setup(void) override;
  void handle_incoming_can_frame(CAN_frame rx_frame) override;
  void update_values() override;
  void transmit_can(unsigned long currentMillis) override;

  static constexpr const char* Name = "Volkswagen MQB e-Golf modules";

 private:
  void publish_pack_state();
  uint16_t estimate_soc_from_voltage(uint16_t pack_voltage_dV) const;

  std::unique_ptr<MQBModuleManager> module_manager_;
  unsigned long last_poll_ms_ = 0;
  bool poll_toggle_ = false;
  bool datalayer_dirty_ = false;
  CAN_frame poll_frame_a_;
  CAN_frame poll_frame_b_;
};

#endif
