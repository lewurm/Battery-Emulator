#include "../include.h"
#ifdef SMA_TRIPOWER_CAN
#include "../datalayer/datalayer.h"
#include "SMA-TRIPOWER-CAN.h"

/* TODO:
- Figure out the manufacturer info needed in send_tripower_init() CAN messages
  - CAN logs from real system might be needed
- Figure out how cellvoltages need to be displayed
- Figure out if sending send_tripower_init() like we do now is OK
- Figure out how to send the non-cyclic messages when needed
*/

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis500ms = 0;  // will store last time a 100ms CAN Message was send

//Actual content messages
CAN_frame SMA_00D = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x00D,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_00F = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x00F,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_011 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x011,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_013 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x013,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_014 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x014,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_005 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x005,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_007 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x007,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_006 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x006,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_008 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x008,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_015 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x015,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_016 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x016,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_017 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x017,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame SMA_018 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x018,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static uint16_t discharge_current = 0;
static uint16_t charge_current = 0;
static int16_t temperature_average = 0;
static uint16_t ampere_hours_remaining = 0;
static uint16_t ampere_hours_max = 0;
static bool batteryAlarm = false;
static bool BMSevent = false;

enum BatteryState { NA, INIT, BAT_STANDBY, OPERATE, WARNING, FAULTED, UPDATE, BAT_UPDATE };
BatteryState batteryState = OPERATE;
enum InverterControlFlags {
  EMG_CHARGE_REQUEST,
  EMG_DISCHARGE_REQUEST,
  NOT_ENOUGH_ENERGY_FOR_START,
  INVERTER_STAY_ON,
  FORCED_BATTERY_SHUTDOWN,
  RESERVED,
  BATTERY_UPDATE_AVAILABLE,
  NO_BATTERY_UPDATED_BY_INV
};
InverterControlFlags inverterControlFlags = BATTERY_UPDATE_AVAILABLE;
enum Events0 {
  START_SOC_CALIBRATE,
  STOP_SOC_CALIBRATE,
  START_POWERLIMIT,
  STOP_POWERLIMIT,
  PREVENTATIVE_BAT_SHUTDOWN,
  THERMAL_MANAGEMENT,
  START_BALANCING,
  STOP_BALANCING
};
Events0 events0 = START_BALANCING;
enum Events1 { START_BATTERY_SELFTEST, STOP_BATTERY_SELFTEST };
Events1 events1 = START_BATTERY_SELFTEST;
enum Command2Battery { IDLE, RUN, NOT_USED1, NOT_USED2, SHUTDOWN, FIRMWARE_UPDATE, BATSELFUPDATE, NOT_USED3 };
Command2Battery command2Battery = RUN;
enum InvInitState { SYSTEM_FREQUENCY, XPHASE_SYSTEM, BLACKSTART_OPERATION };
InvInitState invInitState = SYSTEM_FREQUENCY;

void update_values_can_inverter() {  //This function maps all the values fetched from battery CAN to the inverter CAN
  //Calculate values

  if (datalayer.battery.status.voltage_dV > 10) {  // Only update value when we have voltage available to avoid div0
    charge_current =
        ((datalayer.battery.status.max_charge_power_W * 10) /
         datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
    charge_current = (charge_current * 10);     //Value needs a decimal before getting sent to inverter (81.0A)
    discharge_current =
        ((datalayer.battery.status.max_discharge_power_W * 10) /
         datalayer.battery.status.voltage_dV);     //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
    discharge_current = (discharge_current * 10);  //Value needs a decimal before getting sent to inverter (81.0A)
  }

  if (charge_current > datalayer.battery.info.max_charge_amp_dA) {
    charge_current =
        datalayer.battery.info
            .max_charge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  if (discharge_current > datalayer.battery.info.max_discharge_amp_dA) {
    discharge_current =
        datalayer.battery.info
            .max_discharge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  temperature_average =
      ((datalayer.battery.status.temperature_max_dC + datalayer.battery.status.temperature_min_dC) / 2);

  ampere_hours_remaining =
      ((datalayer.battery.status.reported_remaining_capacity_Wh / datalayer.battery.status.voltage_dV) *
       100);  //(WH[10000] * V+1[3600])*100 = 270 (27.0Ah)
  ampere_hours_max = ((datalayer.battery.info.total_capacity_Wh / datalayer.battery.status.voltage_dV) *
                      100);  //(WH[10000] * V+1[3600])*100 = 270 (27.0Ah)

  batteryState = OPERATE;
  inverterControlFlags = INVERTER_STAY_ON;

  //Map values to CAN messages
  // Battery Limits
  //Battery Max Charge Voltage (eg 400.0V = 4000 , 16bits long)
  SMA_00D.data.u8[0] = (datalayer.battery.info.max_design_voltage_dV >> 8);
  SMA_00D.data.u8[1] = (datalayer.battery.info.max_design_voltage_dV & 0x00FF);
  //Battery Min Discharge Voltage (eg 300.0V = 3000 , 16bits long)
  SMA_00D.data.u8[2] = (datalayer.battery.info.min_design_voltage_dV >> 8);
  SMA_00D.data.u8[3] = (datalayer.battery.info.min_design_voltage_dV & 0x00FF);
  //Discharge limited current, 500 = 50A, (0.1, A)
  SMA_00D.data.u8[4] = (discharge_current >> 8);
  SMA_00D.data.u8[5] = (discharge_current & 0x00FF);
  //Charge limited current, 125 =12.5A (0.1, A)
  SMA_00D.data.u8[6] = (charge_current >> 8);
  SMA_00D.data.u8[7] = (charge_current & 0x00FF);

  // Battery State
  //SOC (100.00%)
  SMA_00F.data.u8[0] = (datalayer.battery.status.reported_soc >> 8);
  SMA_00F.data.u8[1] = (datalayer.battery.status.reported_soc & 0x00FF);
  //StateOfHealth (100.00%)
  SMA_00F.data.u8[2] = (datalayer.battery.status.soh_pptt >> 8);
  SMA_00F.data.u8[3] = (datalayer.battery.status.soh_pptt & 0x00FF);
  //State of charge (AH, 0.1)
  SMA_00F.data.u8[4] = (ampere_hours_remaining >> 8);
  SMA_00F.data.u8[5] = (ampere_hours_remaining & 0x00FF);
  //Fully charged (AH, 0.1)
  SMA_00F.data.u8[6] = (ampere_hours_max >> 8);
  SMA_00F.data.u8[7] = (ampere_hours_max & 0x00FF);

  // Battery Energy
  //Charged Energy Counter TODO: are these needed?
  //SMA_011.data.u8[0] = (X >> 8);
  //SMA_011.data.u8[1] = (X & 0x00FF);
  //SMA_011.data.u8[2] = (X >> 8);
  //SMA_011.data.u8[3] = (X & 0x00FF);
  //Discharged Energy Counter TODO: are these needed?
  //SMA_011.data.u8[4] = (X >> 8);
  //SMA_011.data.u8[5] = (X & 0x00FF);
  //SMA_011.data.u8[6] = (X >> 8);
  //SMA_011.data.u8[7] = (X & 0x00FF);

  // Battery Measurements
  //Voltage (370.0)
  SMA_013.data.u8[0] = (datalayer.battery.status.voltage_dV >> 8);
  SMA_013.data.u8[1] = (datalayer.battery.status.voltage_dV & 0x00FF);
  //Current (TODO: signed OK?)
  SMA_013.data.u8[2] = (datalayer.battery.status.current_dA >> 8);
  SMA_013.data.u8[3] = (datalayer.battery.status.current_dA & 0x00FF);
  //Temperature average
  SMA_013.data.u8[4] = (temperature_average >> 8);
  SMA_013.data.u8[5] = (temperature_average & 0x00FF);
  //Battery state
  SMA_013.data.u8[6] = batteryState;
  SMA_013.data.u8[6] = inverterControlFlags;

  // Battery Temperature and Cellvoltages
  // Battery max temperature
  SMA_014.data.u8[0] = (datalayer.battery.status.temperature_max_dC >> 8);
  SMA_014.data.u8[1] = (datalayer.battery.status.temperature_max_dC & 0x00FF);
  // Battery min temperature
  SMA_014.data.u8[2] = (datalayer.battery.status.temperature_min_dC >> 8);
  SMA_014.data.u8[3] = (datalayer.battery.status.temperature_min_dC & 0x00FF);
  // Battery Cell Voltage (sum)
  //SMA_014.data.u8[4] = (??? >> 8); //TODO scaling?
  //SMA_014.data.u8[5] = (??? & 0x00FF); //TODO scaling?
  // Cell voltage min
  //SMA_014.data.u8[6] = (??? >> 8); //TODO scaling? 0-255
  // Cell voltage max
  //SMA_014.data.u8[7] = (??? >> 8); //TODO scaling? 0-255

  //SMA_006.data.u8[0] = (ErrorCode >> 8);
  //SMA_006.data.u8[1] = (ErrorCode & 0x00FF);
  //SMA_006.data.u8[2] = ModuleNumber;
  //SMA_006.data.u8[3] = ErrorLevel;
  //SMA_008.data.u8[0] = Events0;
  //SMA_008.data.u8[1] = Events1;

  //SMA_005.data.u8[0] = BMSalarms0;
  //SMA_005.data.u8[1] = BMSalarms1;
  //SMA_005.data.u8[2] = BMSalarms2;
  //SMA_005.data.u8[3] = BMSalarms3;
  //SMA_005.data.u8[4] = BMSalarms4;
  //SMA_005.data.u8[5] = BMSalarms5;
  //SMA_005.data.u8[6] = BMSalarms6;
  //SMA_005.data.u8[7] = BMSalarms7;

  //SMA_007.data.u8[0] = DCDCalarms0;
  //SMA_007.data.u8[1] = DCDCalarms1;
  //SMA_007.data.u8[2] = DCDCalarms2;
  //SMA_007.data.u8[3] = DCDCalarms3;
  //SMA_007.data.u8[4] = DCDCwarnings0;
  //SMA_007.data.u8[5] = DCDCwarnings1;
  //SMA_007.data.u8[6] = DCDCwarnings2;
  //SMA_007.data.u8[7] = DCDCwarnings3;

  //SMA_015.data.u8[0] = BatterySystemVersion;
  //SMA_015.data.u8[1] = BatterySystemVersion;
  //SMA_015.data.u8[2] = BatterySystemVersion;
  //SMA_015.data.u8[3] = BatterySystemVersion;
  //SMA_015.data.u8[4] = BatteryCapacity;
  //SMA_015.data.u8[5] = BatteryCapacity;
  //SMA_015.data.u8[6] = NumberOfModules;
  //SMA_015.data.u8[7] = BatteryManufacturerID;

  //SMA_016.data.u8[0] = SerialNumber;
  //SMA_016.data.u8[1] = SerialNumber;
  //SMA_016.data.u8[2] = SerialNumber;
  //SMA_016.data.u8[3] = SerialNumber;
  //SMA_016.data.u8[4] = ManufacturingDate;
  //SMA_016.data.u8[5] = ManufacturingDate;
  //SMA_016.data.u8[6] = ManufacturingDate;
  //SMA_016.data.u8[7] = ManufacturingDate;

  //SMA_017.data.u8[0] = Multiplex;
  //SMA_017.data.u8[1] = ManufacturerName;
  //SMA_017.data.u8[2] = ManufacturerName;
  //SMA_017.data.u8[3] = ManufacturerName;
  //SMA_017.data.u8[4] = ManufacturerName;
  //SMA_017.data.u8[5] = ManufacturerName;
  //SMA_017.data.u8[6] = ManufacturerName;
  //SMA_017.data.u8[7] = ManufacturerName;

  //SMA_018.data.u8[0] = Multiplex;
  //SMA_018.data.u8[1] = BatteryName;
  //SMA_018.data.u8[2] = BatteryName;
  //SMA_018.data.u8[3] = BatteryName;
  //SMA_018.data.u8[4] = BatteryName;
  //SMA_018.data.u8[5] = BatteryName;
  //SMA_018.data.u8[6] = BatteryName;
  //SMA_018.data.u8[7] = BatteryName;
}

void receive_can_inverter(CAN_frame rx_frame) {
  switch (rx_frame.ID) {
    case 0x00D:  //Inverter Measurements
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x00F:  //Inverter Feedback
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x010:  //Time from inverter
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x015:  //Initialization message from inverter
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      send_tripower_init();
      break;
    case 0x017:  //Initialization message from inverter 2
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      //send_tripower_init();
      break;
    default:
      break;
  }
}

void send_can_inverter() {
  unsigned long currentMillis = millis();

  // Send CAN Message every 500ms
  if (currentMillis - previousMillis500ms >= INTERVAL_500_MS) {
    previousMillis500ms = currentMillis;

    transmit_can(&SMA_00D, can_config.inverter);  //Battery limits
    transmit_can(&SMA_00F, can_config.inverter);  // Battery state
    transmit_can(&SMA_011, can_config.inverter);  // Battery Energy
    transmit_can(&SMA_013, can_config.inverter);  // Battery Measurements
    transmit_can(&SMA_014, can_config.inverter);  // Battery Temperatures and cellvoltages
  }

  if (batteryAlarm) {                             //Non-cyclic
    transmit_can(&SMA_005, can_config.inverter);  // Battery Alarms 1
    transmit_can(&SMA_007, can_config.inverter);  // Battery Alarms 2
  }

  if (BMSevent) {                                 //Non-cyclic
    transmit_can(&SMA_006, can_config.inverter);  // Battery Errorcode
    transmit_can(&SMA_008, can_config.inverter);  // Battery Events
  }
}

void send_tripower_init() {
  transmit_can(&SMA_015, can_config.inverter);  // Battery Data 1
  transmit_can(&SMA_016, can_config.inverter);  // Battery Data 2
  transmit_can(&SMA_017, can_config.inverter);  // Battery Manufacturer
  transmit_can(&SMA_018, can_config.inverter);  // Battery Name
}
#endif
