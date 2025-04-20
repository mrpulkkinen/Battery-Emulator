#pragma once

#include "Battery.h"
#include "CANInterface.h"

class EGolfBattery : public Battery {
public:
    EGolfBattery();
    void init() override;
    void update() override;
    void processCAN(const CANMessage& msg) override;

private:
    void sendStatus();
    void sendTemperature();
    void sendHealth();
    void sendChargerStatus();
    void sendInsulation();
    void sendContactorState();
    void sendInternalFaults();
    void sendModuleVoltages();
    void handleUDSRequest(const CANMessage& msg);
    void updateState();

    enum BatteryState { IDLE, CHARGING, FAULT };
    BatteryState state;

    float soc;
    float voltage;
    float current;
    float temp1;
    float temp2;
    float soh;
    uint16_t cycles;

    bool chargerConnected;
    float insulationResistance;
    uint8_t contactorState;
    uint32_t faultFlags;
    uint16_t moduleVoltages[27];
};