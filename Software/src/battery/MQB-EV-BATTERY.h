#ifndef MQB_EV_BATTERY_H
#define MQB_EV_BATTERY_H

#include "../include.h"

#define BATTERY_SELECTED

#define MQB_EV_MAX_PACK_VOLTAGE_dV 4030   // 403.0 V upper limit
#define MQB_EV_MIN_PACK_VOLTAGE_dV 3000   // 300.0 V lower limit
#define MQB_EV_MAX_CELL_DEVIATION_mV 150  // 0.15 V max deviation
#define MQB_EV_MAX_CELL_VOLTAGE_mV 4200
#define MQB_EV_MIN_CELL_VOLTAGE_mV 3100
#define MQB_EV_NOMINAL_CAPACITY_Wh 35800  // e-Golf 35.8 kWh pack
#define MQB_EV_MAX_MODULES 16
#define MQB_EV_CELLS_PER_MODULE 12

void setup_battery(void);
void transmit_can_frame(CAN_frame* tx_frame, int interface);

#endif
