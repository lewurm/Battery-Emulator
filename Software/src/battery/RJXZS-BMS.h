#ifndef RJXZS_BMS_H
#define RJXZS_BMS_H
#include <Arduino.h>
#include "../include.h"

/* Tweak these according to your battery build */
#define MAX_CELL_DEVIATION_MV 250
#define MAX_DISCHARGE_POWER_ALLOWED_W 5000
#define MAX_CHARGE_POWER_ALLOWED_W 5000
#define MAX_CHARGE_POWER_WHEN_TOPBALANCING_W 500
#define RAMPDOWN_SOC 9000  // (90.00) SOC% to start ramping down from max charge power towards 0 at 100.00%

/* Do not modify any rows below*/
#define BATTERY_SELECTED
#define NATIVECAN_250KBPS

void setup_battery(void);
void transmit_can(CAN_frame* tx_frame, int interface);

#endif
