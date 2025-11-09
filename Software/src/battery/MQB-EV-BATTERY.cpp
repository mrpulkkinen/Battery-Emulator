#include "MQB-EV-BATTERY.h"

#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

#include "../communication/can/comm_can.h"

namespace {

constexpr uint32_t kModuleFrameIdMin = 0x1B0;
constexpr uint32_t kModuleFrameIdMax = 0x1EF;
constexpr uint32_t kTempFrameType1Min = 0x1A555401;
constexpr uint32_t kTempFrameType1Max = 0x1A555440;
constexpr uint32_t kTempFrameType2Min = 0x1A5555F0;
constexpr uint32_t kTempFrameType2Max = 0x1A5555FF;
constexpr float kIgnoreCellVoltage = 0.1f;
constexpr float kMaxValidCellVoltage = 5.0f;
constexpr float kInvalidTemperature = -80.0f;
constexpr uint32_t kCellPollIntervalMs = 500;
constexpr uint32_t kDataStaleTimeoutMs = 1000;
constexpr uint16_t kSocMinCell_mV = 3200;
constexpr uint16_t kSocMaxCell_mV = 4150;
constexpr uint16_t kDefaultMaxDischargeA = 2500;  // 250 A
constexpr uint16_t kDefaultMaxChargeA = 1250;     // 125 A
constexpr uint32_t kDefaultMaxDischargePowerW = 80000;
constexpr uint32_t kDefaultMaxChargePowerW = 45000;

}  // namespace

class MQBModule {
 public:
  MQBModule() { reset(); }

  void reset() {
    cell_voltages_.fill(0.0f);
    temperatures_.fill(kInvalidTemperature);
    present_ = false;
    last_update_ms_ = 0;
  }

  void decodeCellBlock(uint8_t block, const CAN_frame& frame) {
    const uint8_t* b = frame.data.u8;
    switch (block) {
      case 0:
        cell_voltages_[0] = (static_cast<uint16_t>(b[1] >> 4) + (static_cast<uint16_t>(b[2]) << 4) + 1000) * 0.001f;
        cell_voltages_[1] = (static_cast<uint16_t>(b[3]) + (static_cast<uint16_t>(b[4] & 0x0F) << 8) + 1000) * 0.001f;
        cell_voltages_[2] = ((static_cast<uint16_t>(b[5]) << 4) + static_cast<uint16_t>(b[4] >> 4) + 1000) * 0.001f;
        cell_voltages_[3] = (static_cast<uint16_t>(b[6]) + (static_cast<uint16_t>(b[7] & 0x0F) << 8) + 1000) * 0.001f;
        break;
      case 1:
        cell_voltages_[4] = (static_cast<uint16_t>(b[1] >> 4) + (static_cast<uint16_t>(b[2]) << 4) + 1000) * 0.001f;
        cell_voltages_[5] =
            (static_cast<uint16_t>(b[3]) + (static_cast<uint16_t>(b[4] & 0x0F) << 8) + 1000) * 0.001f;
        cell_voltages_[6] = ((static_cast<uint16_t>(b[5]) << 4) + static_cast<uint16_t>(b[4] >> 4) + 1000) * 0.001f;
        cell_voltages_[7] = (static_cast<uint16_t>(b[6]) + (static_cast<uint16_t>(b[7] & 0x0F) << 8) + 1000) * 0.001f;
        break;
      case 2:
        cell_voltages_[8] = (static_cast<uint16_t>(b[1] >> 4) + (static_cast<uint16_t>(b[2]) << 4) + 1000) * 0.001f;
        cell_voltages_[9] =
            (static_cast<uint16_t>(b[3]) + (static_cast<uint16_t>(b[4] & 0x0F) << 8) + 1000) * 0.001f;
        cell_voltages_[10] = ((static_cast<uint16_t>(b[5]) << 4) + static_cast<uint16_t>(b[4] >> 4) + 1000) * 0.001f;
        cell_voltages_[11] = (static_cast<uint16_t>(b[6]) + (static_cast<uint16_t>(b[7] & 0x0F) << 8) + 1000) * 0.001f;
        break;
      case 3:
        cell_voltages_[12] = (static_cast<uint16_t>(b[1] >> 4) + (static_cast<uint16_t>(b[2]) << 4) + 1000) * 0.001f;
        break;
      default:
        break;
    }
    present_ = true;
    last_update_ms_ = millis();
  }

  void decodeTempFrameType1(const CAN_frame& frame) {
    const uint8_t* b = frame.data.u8;
    if (b[7] == 0xFD) {
      if (b[2] != 0xFD) {
        temperatures_[0] = static_cast<float>(b[2]) * 0.5f - 40.0f;
      }
    } else {
      if (b[0] < 0xDF) {
        temperatures_[0] = static_cast<float>(b[0]) * 0.5f - 43.0f;
      } else {
        temperatures_[0] = kInvalidTemperature;
      }
    }

    if (b[5] != 0xFF) {
      temperatures_[1] = static_cast<float>(b[5]) * 0.5f - 40.0f;
    } else {
      temperatures_[1] = kInvalidTemperature;
    }
  }

  void decodeTempFrameType2(const CAN_frame& frame) {
    const uint8_t* b = frame.data.u8;
    if (b[0] != 0xFF) {
      temperatures_[2] = static_cast<float>(b[0]) * 0.5f - 40.0f;
    } else {
      temperatures_[2] = kInvalidTemperature;
    }
  }

  float moduleVoltage() const {
    float total = 0.0f;
    uint8_t count = 0;
    for (float v : cell_voltages_) {
      if (v > kIgnoreCellVoltage && v < kMaxValidCellVoltage) {
        total += v;
        count++;
      }
    }
    if (count == 0) {
      return 0.0f;
    }
    return total;
  }

  uint8_t validCellCount() const {
    return static_cast<uint8_t>(std::count_if(cell_voltages_.begin(), cell_voltages_.end(), [](float voltage) {
      return voltage > kIgnoreCellVoltage && voltage < kMaxValidCellVoltage;
    }));
  }

  float minCellVoltage() const {
    float min_v = kMaxValidCellVoltage;
    for (float v : cell_voltages_) {
      if (v < min_v && v > kIgnoreCellVoltage) {
        min_v = v;
      }
    }
    return min_v;
  }

  float maxCellVoltage() const {
    float max_v = 0.0f;
    for (float v : cell_voltages_) {
      if (v > max_v && v < kMaxValidCellVoltage) {
        max_v = v;
      }
    }
    return max_v;
  }

  float averageCellVoltage() const {
    float total = 0.0f;
    uint8_t count = 0;
    for (float v : cell_voltages_) {
      if (v > kIgnoreCellVoltage && v < kMaxValidCellVoltage) {
        total += v;
        count++;
      }
    }
    if (count == 0) {
      return 0.0f;
    }
    return total / static_cast<float>(count);
  }

  float minTemperature() const {
    float min_t = std::numeric_limits<float>::max();
    for (float t : temperatures_) {
      if (t != kInvalidTemperature && t < min_t) {
        min_t = t;
      }
    }
    return min_t;
  }

  float maxTemperature() const {
    float max_t = kInvalidTemperature;
    for (float t : temperatures_) {
      if (t != kInvalidTemperature && t > max_t) {
        max_t = t;
      }
    }
    return max_t;
  }

  void copyCellVoltages(uint16_t& cursor, uint16_t* dest, size_t max_cells) const {
    for (float v : cell_voltages_) {
      if (cursor >= max_cells) {
        return;
      }
      dest[cursor++] = static_cast<uint16_t>(std::lround(v * 1000.0f));
    }
  }

  bool isPresent() const { return present_; }

  uint32_t lastUpdateMs() const { return last_update_ms_; }

 private:
  std::array<float, MQB_EV_CELLS_PER_MODULE> cell_voltages_{};
  std::array<float, 3> temperatures_{};
  bool present_ = false;
  uint32_t last_update_ms_ = 0;
};

class MQBModuleManager {
 public:
  void handleCellFrame(const CAN_frame& frame) {
    if (frame.ext_ID) {
      return;
    }
    if (frame.ID < kModuleFrameIdMin || frame.ID > kModuleFrameIdMax) {
      return;
    }
    uint16_t offset = static_cast<uint16_t>(frame.ID - kModuleFrameIdMin);
    uint8_t module = static_cast<uint8_t>(offset / 4) + 1;
    uint8_t block = static_cast<uint8_t>(offset % 4);
    if (module == 0 || module > MQB_EV_MAX_MODULES) {
      return;
    }
    modules_[module].decodeCellBlock(block, frame);
    last_activity_ms_ = millis();
  }

  void handleTempFrame(const CAN_frame& frame) {
    if (!frame.ext_ID) {
      return;
    }
    const uint32_t id = frame.ID;
    if (id > kTempFrameType1Min && id < kTempFrameType1Max) {
      uint8_t cmu = id & 0xFF;
      if (cmu > 10 && cmu < 60) {
        cmu = static_cast<uint8_t>((cmu & 0x0F) * 0.5f + 1);
      }
      if (cmu > 0 && cmu <= MQB_EV_MAX_MODULES) {
        modules_[cmu].decodeTempFrameType1(frame);
        last_activity_ms_ = millis();
      }
      return;
    }
    if (id > kTempFrameType2Min && id < kTempFrameType2Max) {
      uint8_t cmu = static_cast<uint8_t>(id & 0x0F);
      cmu++;
      if (cmu > 0 && cmu <= MQB_EV_MAX_MODULES) {
        modules_[cmu].decodeTempFrameType2(frame);
        last_activity_ms_ = millis();
      }
    }
  }

  void refreshAggregates() {
    pack_voltage_V_ = 0.0f;
    avg_cell_voltage_V_ = 0.0f;
    min_cell_voltage_V_ = kMaxValidCellVoltage;
    max_cell_voltage_V_ = 0.0f;
    min_temperature_C_ = std::numeric_limits<float>::max();
    max_temperature_C_ = kInvalidTemperature;
    series_cells_ = 0;
    module_count_ = 0;

    float avg_sum = 0.0f;
    uint16_t avg_counter = 0;

    for (uint8_t idx = 1; idx <= MQB_EV_MAX_MODULES; idx++) {
      auto& module = modules_[idx];
      if (!module.isPresent()) {
        continue;
      }
      if ((millis() - module.lastUpdateMs()) > kDataStaleTimeoutMs) {
        continue;
      }
      module_count_++;
      pack_voltage_V_ += module.moduleVoltage();
      series_cells_ += module.validCellCount();

      float module_avg = module.averageCellVoltage();
      if (module_avg > 0.0f) {
        avg_sum += module_avg;
        avg_counter++;
      }

      min_cell_voltage_V_ = std::min(min_cell_voltage_V_, module.minCellVoltage());
      max_cell_voltage_V_ = std::max(max_cell_voltage_V_, module.maxCellVoltage());
      min_temperature_C_ = std::min(min_temperature_C_, module.minTemperature());
      max_temperature_C_ = std::max(max_temperature_C_, module.maxTemperature());
    }

    if (avg_counter > 0) {
      avg_cell_voltage_V_ = avg_sum / static_cast<float>(avg_counter);
    } else {
      avg_cell_voltage_V_ = 0.0f;
    }

    if (module_count_ == 0) {
      pack_voltage_V_ = 0.0f;
      min_cell_voltage_V_ = 0.0f;
      min_temperature_C_ = kInvalidTemperature;
    }
  }

  void copyCellVoltages(uint16_t* dest, size_t max_cells) const {
    uint16_t cursor = 0;
    for (uint8_t idx = 1; idx <= MQB_EV_MAX_MODULES && cursor < max_cells; idx++) {
      modules_[idx].copyCellVoltages(cursor, dest, max_cells);
    }
  }

  float packVoltage() const { return pack_voltage_V_; }
  float minCellVoltage() const { return min_cell_voltage_V_; }
  float maxCellVoltage() const { return max_cell_voltage_V_; }
  float minTemperature() const { return min_temperature_C_; }
  float maxTemperature() const { return max_temperature_C_; }
  uint16_t seriesCells() const { return series_cells_; }
  uint8_t moduleCount() const { return module_count_; }

 private:
  std::array<MQBModule, MQB_EV_MAX_MODULES + 1> modules_{};
  float pack_voltage_V_ = 0.0f;
  float avg_cell_voltage_V_ = 0.0f;
  float min_cell_voltage_V_ = 0.0f;
  float max_cell_voltage_V_ = 0.0f;
  float min_temperature_C_ = kInvalidTemperature;
  float max_temperature_C_ = kInvalidTemperature;
  uint16_t series_cells_ = 0;
  uint8_t module_count_ = 0;
  uint32_t last_activity_ms_ = 0;
};

MqbEvBattery::MqbEvBattery() : CanBattery(), module_manager_(new MQBModuleManager()) {
  poll_frame_a_ = {.FD = false,
                   .ext_ID = false,
                   .DLC = 8,
                   .ID = 0x0BA,
                   .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  poll_frame_b_ = {.FD = false,
                   .ext_ID = false,
                   .DLC = 8,
                   .ID = 0x0BA,
                   .data = {0x45, 0x01, 0x28, 0x00, 0x00, 0x00, 0x00, 0x30}};
}

void MqbEvBattery::setup(void) {
  strncpy(datalayer.system.info.battery_protocol, Name, sizeof(datalayer.system.info.battery_protocol) - 1);
  datalayer.system.info.battery_protocol[sizeof(datalayer.system.info.battery_protocol) - 1] = '\0';

  datalayer.battery.info.max_design_voltage_dV = MQB_EV_MAX_PACK_VOLTAGE_dV;
  datalayer.battery.info.min_design_voltage_dV = MQB_EV_MIN_PACK_VOLTAGE_dV;
  datalayer.battery.info.max_cell_voltage_mV = MQB_EV_MAX_CELL_VOLTAGE_mV;
  datalayer.battery.info.min_cell_voltage_mV = MQB_EV_MIN_CELL_VOLTAGE_mV;
  datalayer.battery.info.max_cell_voltage_deviation_mV = MQB_EV_MAX_CELL_DEVIATION_mV;
  datalayer.battery.info.chemistry = battery_chemistry_enum::NMC;
  datalayer.battery.info.number_of_cells = MQB_EV_MAX_MODULES * MQB_EV_CELLS_PER_MODULE;

  datalayer.battery.status.CAN_battery_still_alive = 0;
  datalayer_dirty_ = true;
}

void MqbEvBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  module_manager_->handleCellFrame(rx_frame);
  module_manager_->handleTempFrame(rx_frame);
  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  datalayer_dirty_ = true;
}

void MqbEvBattery::update_values() {
  publish_pack_state();
  datalayer_dirty_ = false;
}

void MqbEvBattery::transmit_can(unsigned long currentMillis) {
  if (currentMillis - last_poll_ms_ < kCellPollIntervalMs) {
    return;
  }

  last_poll_ms_ = currentMillis;
  poll_toggle_ = !poll_toggle_;

  if (poll_toggle_) {
    transmit_can_frame(&poll_frame_a_);
  } else {
    transmit_can_frame(&poll_frame_b_);
  }
}

void MqbEvBattery::publish_pack_state() {
  module_manager_->refreshAggregates();

  const uint16_t series_cells = module_manager_->seriesCells();
  if (series_cells > 0) {
    datalayer.battery.info.number_of_cells =
        std::min<uint16_t>(series_cells, static_cast<uint16_t>(MAX_AMOUNT_CELLS));
  }

  const uint16_t pack_voltage_dV = static_cast<uint16_t>(std::lround(module_manager_->packVoltage() * 10.0f));
  datalayer.battery.status.voltage_dV = pack_voltage_dV;
  datalayer.battery.status.cell_min_voltage_mV =
      static_cast<uint16_t>(std::lround(module_manager_->minCellVoltage() * 1000.0f));
  datalayer.battery.status.cell_max_voltage_mV =
      static_cast<uint16_t>(std::lround(module_manager_->maxCellVoltage() * 1000.0f));
  datalayer.battery.status.temperature_min_dC =
      static_cast<int16_t>(std::lround(module_manager_->minTemperature() * 10.0f));
  datalayer.battery.status.temperature_max_dC =
      static_cast<int16_t>(std::lround(module_manager_->maxTemperature() * 10.0f));

  datalayer.battery.status.real_soc = estimate_soc_from_voltage(pack_voltage_dV);
  datalayer.battery.status.reported_soc = datalayer.battery.status.real_soc;

  const uint32_t capacity = MQB_EV_NOMINAL_CAPACITY_Wh;
  datalayer.battery.info.total_capacity_Wh = capacity;
  datalayer.battery.status.remaining_capacity_Wh =
      static_cast<uint32_t>((static_cast<uint64_t>(capacity) * datalayer.battery.status.real_soc) / 10000ULL);

  datalayer.battery.status.max_discharge_power_W = kDefaultMaxDischargePowerW;
  datalayer.battery.status.max_charge_power_W = kDefaultMaxChargePowerW;
  datalayer.battery.status.max_discharge_current_dA = kDefaultMaxDischargeA;
  datalayer.battery.status.max_charge_current_dA = kDefaultMaxChargeA;
  datalayer.battery.status.current_dA = 0;
  datalayer.battery.status.active_power_W = 0;

  std::fill_n(datalayer.battery.status.cell_voltages_mV, MAX_AMOUNT_CELLS, 0);
  module_manager_->copyCellVoltages(datalayer.battery.status.cell_voltages_mV, MAX_AMOUNT_CELLS);

  datalayer.system.status.battery_allows_contactor_closing = (module_manager_->moduleCount() > 0);
}

uint16_t MqbEvBattery::estimate_soc_from_voltage(uint16_t pack_voltage_dV) const {
  const uint16_t series_cells = module_manager_->seriesCells();
  if (series_cells == 0) {
    return 5000;
  }
  const float avg_cell_mV = static_cast<float>(pack_voltage_dV * 10) / static_cast<float>(series_cells);
  float soc = (avg_cell_mV - static_cast<float>(kSocMinCell_mV)) / static_cast<float>(kSocMaxCell_mV - kSocMinCell_mV);
  soc = std::clamp(soc, 0.0f, 1.0f);
  return static_cast<uint16_t>(std::lround(soc * 10000.0f));
}
