#include "EGOLF-BATTERY.h"
#include <Arduino.h>
#include <string.h>

EGolfBattery::EGolfBattery() {
    soc = 80.0;
    voltage = 360.0;
    current = 0.0;
    temp1 = 25.0;
    temp2 = 26.0;
    soh = 95.0;
    cycles = 150;
    chargerConnected = false;
    insulationResistance = 500.0;
    contactorState = 0x03;
    faultFlags = 0;
    for (int i = 0; i < 27; ++i)
        moduleVoltages[i] = 360 / 27;
    state = IDLE;
}

void EGolfBattery::init() {
    Serial.println("EGolfBattery initialized");
}

void EGolfBattery::update() {
    updateState();
    static uint32_t last100ms = 0, last500ms = 0, last1000ms = 0;
    uint32_t now = millis();

    if (now - last100ms >= 100) {
        sendStatus();
        last100ms = now;
    }
    if (now - last500ms >= 500) {
        sendTemperature();
        sendChargerStatus();
        sendInsulation();
        sendContactorState();
        sendInternalFaults();
        sendModuleVoltages();
        last500ms = now;
    }
    if (now - last1000ms >= 1000) {
        sendHealth();
        last1000ms = now;
    }
}

void EGolfBattery::updateState() {
    switch (state) {
        case IDLE:
            current = 0;
            contactorState = 0x01;
            break;
        case CHARGING:
            current = -20.0;
            contactorState = 0x03;
            soc += 0.05;
            if (soc >= 99.0) {
                soc = 99.0;
                state = IDLE;
            }
            break;
        case FAULT:
            current = 0;
            contactorState = 0x00;
            faultFlags = 0x01;
            break;
    }
}

void EGolfBattery::processCAN(const CANMessage& msg) {
    if (msg.id == 0x180) {
        chargerConnected = msg.data[0] & 0x01;
        if (chargerConnected && state != CHARGING)
            state = CHARGING;
    }
    if (msg.id == 0x7DF || msg.id == 0x7E0) {
        handleUDSRequest(msg);
    }
}

void EGolfBattery::handleUDSRequest(const CANMessage& msg) {
    if (msg.len < 4 || msg.data[1] != 0x22) return;

    uint16_t pid = (msg.data[2] << 8) | msg.data[3];
    CANMessage response = {0x7E8, 8, {0}};

    const char* text = nullptr;
    switch (pid) {
        case 0xF190: text = "VWGEG123456789000"; break;
        case 0xF187: text = "EV_BMS_Golf"; break;
        case 0xF189: text = "0751"; break;
        case 0xF18C: text = "HL3-BMS-EG"; break;
        default: return;
    }

    response.data[0] = 0x10 | (strlen(text) + 3);
    response.data[1] = 0x62;
    response.data[2] = msg.data[2];
    response.data[3] = msg.data[3];
    strncpy((char*)&response.data[4], text, 4);
    sendCAN(response);

    for (size_t i = 4; i < strlen(text); i += 7) {
        CANMessage cont = {0x7E8, 8, {0}};
        cont.data[0] = 0x21 + ((i - 4) / 7);
        strncpy((char*)&cont.data[1], text + i, 7);
        sendCAN(cont);
    }
}

void EGolfBattery::sendStatus() {
    CANMessage msg = {0x1A0, 8, {}};
    uint16_t soc_raw = soc * 10;
    uint16_t voltage_raw = voltage * 10;
    int16_t current_raw = current * 10;
    msg.data[0] = soc_raw & 0xFF;
    msg.data[1] = soc_raw >> 8;
    msg.data[2] = voltage_raw & 0xFF;
    msg.data[3] = voltage_raw >> 8;
    msg.data[4] = current_raw & 0xFF;
    msg.data[5] = current_raw >> 8;
    msg.data[6] = 0;
    msg.data[7] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendTemperature() {
    CANMessage msg = {0x1A1, 8, {}};
    msg.data[0] = static_cast<int8_t>(temp1 + 40);
    msg.data[1] = static_cast<int8_t>(temp2 + 40);
    for (int i = 2; i < 8; ++i) msg.data[i] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendHealth() {
    CANMessage msg = {0x1A2, 8, {}};
    msg.data[0] = static_cast<uint8_t>(soh);
    msg.data[1] = cycles & 0xFF;
    msg.data[2] = cycles >> 8;
    for (int i = 3; i < 8; ++i) msg.data[i] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendChargerStatus() {
    CANMessage msg = {0x1A3, 8, {}};
    msg.data[0] = chargerConnected ? 1 : 0;
    for (int i = 1; i < 8; ++i) msg.data[i] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendInsulation() {
    CANMessage msg = {0x1A4, 8, {}};
    uint16_t resistance_raw = insulationResistance * 10;
    msg.data[0] = resistance_raw & 0xFF;
    msg.data[1] = resistance_raw >> 8;
    for (int i = 2; i < 8; ++i) msg.data[i] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendContactorState() {
    CANMessage msg = {0x1A5, 8, {}};
    msg.data[0] = contactorState;
    for (int i = 1; i < 8; ++i) msg.data[i] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendInternalFaults() {
    CANMessage msg = {0x1A6, 8, {}};
    msg.data[0] = faultFlags & 0xFF;
    msg.data[1] = (faultFlags >> 8) & 0xFF;
    msg.data[2] = (faultFlags >> 16) & 0xFF;
    msg.data[3] = (faultFlags >> 24) & 0xFF;
    for (int i = 4; i < 8; ++i) msg.data[i] = 0;
    sendCAN(msg);
}

void EGolfBattery::sendModuleVoltages() {
    for (int group = 0; group < 7; ++group) {
        CANMessage msg = {uint16_t(0x1A7 + group), 8, {}};
        for (int i = 0; i < 4; ++i) {
            int idx = group * 4 + i;
            uint16_t v = (idx < 27) ? moduleVoltages[idx] : 0;
            msg.data[i * 2] = v & 0xFF;
            msg.data[i * 2 + 1] = v >> 8;
        }
        sendCAN(msg);
    }
}
