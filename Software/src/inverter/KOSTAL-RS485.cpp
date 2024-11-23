#include "../include.h"
#ifdef BYD_KOSTAL_RS485
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "KOSTAL-RS485.h"

#define RS485_HEALTHY \
  12  // How many value updates we can go without inverter gets reported as missing \
      // e.g. value set to 12, 12*5sec=60seconds without comm before event is raised
static const uint8_t KOSTAL_FRAMEHEADER[5] = {0x62, 0xFF, 0x02, 0xFF, 0x29};
static const uint8_t KOSTAL_FRAMEHEADER2[5] = {0x63, 0xFF, 0x02, 0xFF, 0x29};
static uint16_t nominal_voltage_dV = 0;
static uint16_t discharge_current_dA = 0;
static uint16_t charge_current_dA = 0;
static int16_t average_temperature_dC = 0;
static uint8_t incoming_message_counter = RS485_HEALTHY;

static int32_t closing_done_count = 0;

#define STATE0_STANDBY 0
#define STATE1_CLOSE_CONTACTORS 1
#define STATE2_CLOSING_DONE 2
#define STATE3_OPERATE 3

static int8_t state = STATE0_STANDBY;

static unsigned long currentMillis;
static unsigned long startupMillis = 0;
static unsigned long contactorMillis = 0;

static boolean RX_allow = false;

union f32b {
  float f;
  byte b[4];
};

static set_state(int next_state) {
    Serial.print("SWITCH STATE: before=");
    Serial.print(state);
    Serial.print(", next -> ");
    Serial.print(next_state);

    if ((state == 0 && next_state == 1) || (state == 1 && next_state == 2) || (state == 2 && next_state == 3)) {
        /* all good */
    } else if (state == 3 && next_state == 0) {
        Serial.println(" -> state reset");
    } else {
        Serial.println(" -> state WARNING");
    }

    state = next_state;
}


uint8_t BATTERY_INFO[40] = {0x00, 0xE2, 0xFF, 0x02, 0xFF, 0x29,  // Frame header
                      0x00, 0x00, 0x80, 0x43,  // 256.063 Nominal voltage / 5*51.2=256      first byte 0x01 or 0x04
                      0xE4, 0x70, 0x8A, 0x5C,  // Manufacture date (Epoch time) (BYD: GetBatteryInfo this[0x10ac])
                      0xB5, 0x00, 0xD3, 0x00,  // Battery Serial number? Modbus register 527 - 0x10b0
                      0x00, 0x00, 0xC8, 0x41,  // 0x10b4
                      0xC2, 0x18,              // Battery Firmware, modbus register 586  (0x10b8)
                      0x00,                    // Static (BYD: GetBatteryInfo this[0x10ba])
                      0x00,                    // ?
                      0x59, 0x42,              // Static (BYD: GetBatteryInfo this[0x10bc]) 
                      0x00, 0x00,              // Static (BYD: GetBatteryInfo this[0x10be]) 
                      0x00, 0x00,
                      0x05, 0x00, 0xA0, 0x00, 0x00, 0x00,
                      0x4D,   // CRC
                      0x00};  //


// example (null descrambled)
   // 00: 00
   // 01: E2 FF 02 FF 29
   // 06: E5 10 82 43   // current voltage, 260.132f V
   // 10: 00 00 8D 43   // max design voltage, 282.0f V
   // 14: CD CC 88 41   // temperature, 17.1f °C
   // 18: 9A 99 99 3F   // peak discharge current, 1.2f A
   // 22: 99 99 99 3F   // avg. current, 1.19f A
   // 26: 00 00 48 42   // max discharge current, 50.0f A
   // 30: 00 00 C8 41   // battery cap Ah, 25.0f Ah
   // 34: 00 00 A0 41   // max charge current , 25.0f A
   // 38: 00 00 88 41   // cell temp max, 17.0f °C
   // 42: 9A 99 81 41   // cell temp min, 16.2f °C
   // 46: 12 83 50 40   // cell voltage max, 3.258f V
   // 50: 29 5C 4F 40   // cell voltage min, 3.240f V
   // 54: 68 00
   // 56: 00
   // 57: 00
   // 58: 14 // SoC
   // 59: 00 00 00
   // 62: 0B // CRC

// values in CyclicData will be overwritten at update_modbus_registers_inverter()

uint8_t CyclicData[64] = {
    0x00,                          // First zero byte pointer
    0xE2, 0xFF, 0x02, 0xFF, 0x29,  // frame Header
    0x1D, 0x1A, 0xB2, 0x43,  // Current Voltage  (float)                        Modbus register 216, Bytes 6-9
    0x00, 0x00, 0x8D, 0x43,  // Max Voltage      (float),                                     Bytes 10-13
    0x00, 0x00, 0xAC, 0x41,  // BAttery Temperature        (float)       Modbus register 214, Bytes 14-17
    0x00, 0x00, 0x00, 0x00,  // Peak Current (1s period?),  Bytes 18-21 - Communication fault seen with some values (>10A?)
    0x00, 0x00, 0x00, 0x00,  // Avg current  (1s period?),  Bytes 22-25  - Communication fault seen with some values (>10A?)
    0x00, 0x00, 0x48, 0x42,  // Max discharge current (float), Bytes 26-29,
                             // Sunspec: ADisChaMax
    0x00, 0x00, 0x50, 0x41,  // Battery gross capacity, Ah (2 byte float) , Bytes 30-33, Modbus 512
    0x00, 0x00, 0x50, 0x41,  // Max charge current (2 byte float) Bit 34-37, ZERO WHEN SOC=100
                             // Sunspec: AChaMax
    0x00, 0x00, 0xC0, 0x40,  // MaxCellTemp (float), Bytes 38-41
    0x00, 0x00, 0xA0, 0x40,  // MinCellTemp (float), Bytes 42-45
    0xdd, 0x24, 0x66, 0x40,  // MaxCellVolt (float), Bytes 46-49
    0x01, 0x00, 0x60, 0x40,  // MinCellVolt (float), Bytes 50-53

    0x39, 0x05,              // Cycle count (uint16), Bytes 54-55
                             //
                             //
    0x00,                    // Byte 56. Maps 1:1 to ModBus 208 "Battery ready flag"
                             //
    0x00,                    // When SOC=100 Byte57 (=0x39), > at startup 0x03 (about 7 times), otherwise 0x02
                             //                               ^^^^ actually I think those have been confused with 0x00 in the past
                             //                                    due to null byte scrambling
                             //
                             // lewurm: Valid bits:
                             //    1: ArriveAlarm
                             //    2: ArriveWarning
                             //    8 | 0x10: ??? but only comes as pair
                             //    0x40: flag set by "CellOverVoltageManage" `(this + 0x115c) & 0x10`, so probably means "full"
                             //
                             //    remaining bits should never be set? Does not match with observations...?
                             //
    0x64,                    // SOC, Byte 58
                             //
    0x00,                    // Unknown,  Byte 59, startup magic?  Can be 2 or 0.  maybe contactors?
                             //        ```
                             //        uVar4 = *(uint *)(*(int *)(this + 0x10cc) + 0x29e8);
                             //        if (((uVar4 & 1) == 0) || ((uVar4 & 4) != 0)) {
                             //          this[0x10a1] = (TKOSTALComm)0x2;
                             //        }
                             //
    0x00,                    // Unknown,  Byte 60, startup magic?  Can be 1 or 0.  or always 0?
                             //        ```
                             //          iVar5 = (**(code **)(this + 0x114c))(*(undefined4 *)(this + 0x10dc),0x10);
                             //          if (iVar5 == 0) {
                             //            this[0x10a2] = (TKOSTALComm)0x0;
                             //          }
                             //          else {
                             //            this[0x10a2] = (TKOSTALComm)0x1;
                             //          }
                             //
    0x00,                    // Unknown, Byte 61
    0x00,                    // CRC, Byte 62
    0x00};

// FE 00 01 40 SOC 00 00 00 CRC (fully charged)
// FE 00 01 00 SOC 01 01 02 CRC (charging or discharging)

uint8_t frame3[9] = {
    0x08, 0xE2, 0xFF, 0x02, 0xFF, 0x29,  //header
    0x06,                                //Unknown
    0xEF,                                //CRC
    0x00                                 //endbyte
};

uint8_t frame4[8]   = {0x07, 0xE3, 0xFF, 0x02, 0xFF, 0x29, 0xF4, 0x00};
uint8_t frameB1[10] = {0x07, 0x63, 0xFF, 0x02, 0xFF, 0x29, 0x5E, 0x02, 0x16, 0x00};

uint8_t RS485_RXFRAME[10];

bool register_content_ok = false;

void float2frame(byte* arr, float value, byte framepointer) {
  f32b g;
  g.f = value;
  arr[framepointer] = g.b[0];
  arr[framepointer + 1] = g.b[1];
  arr[framepointer + 2] = g.b[2];
  arr[framepointer + 3] = g.b[3];
}

void send_kostal(byte* arr, int alen) {
#ifdef DEBUG_KOSTAL_RS485_DATA
  Serial.print("TX: ");
  for (int i = 0; i < alen; i++) {
    if (arr[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(arr[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\n");
#endif
  Serial2.write(arr, alen);
}

void scramble_null_bytes(byte *lfc, int len) {
  int last_null_byte = 0;
  for (int i = 0; i < len; i++) {
    if (lfc[i] == '\0') {
      lfc[last_null_byte] = (byte) (i - last_null_byte);
      last_null_byte = i;
    }
  }
}

byte calculate_kostal_crc(byte *lfc, int len) {
  unsigned int sum = 0;
  if (lfc[0] != 0) {
      printf("WARNING: first byte should be 0, but is 0x%02x\n", lfc[0]);
  }
  for (int i = 1; i < len; i++) {
      sum += lfc[i];
  }
  return (byte) (-sum & 0xff);
}

bool check_kostal_frame_crc() {
  unsigned int sum = 0;
  for (int i = 1; i < 8; ++i) {
    sum += RS485_RXFRAME[i];
  }
  if (((~sum + 1) & 0xff) == (RS485_RXFRAME[8] & 0xff)) {
    return (true);
  } else {
    return (false);
  }
}

void update_RS485_registers_inverter() {

  if (datalayer.battery.status.voltage_dV > 10) {  // Only update value when we have voltage available to avoid div0
    charge_current_dA =
        ((datalayer.battery.status.max_charge_power_W * 10) /
         datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
    //The above calculation results in (30 000*10)/3700=81A
    charge_current_dA = (charge_current_dA * 10);  //Value needs a decimal before getting sent to inverter (81.0A)

    discharge_current_dA =
        ((datalayer.battery.status.max_discharge_power_W * 10) /
         datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
    //The above calculation results in (30 000*10)/3700=81A
    discharge_current_dA = (discharge_current_dA * 10);  //Value needs a decimal before getting sent to inverter (81.0A)
  }

  if (charge_current_dA > datalayer.battery.info.max_charge_amp_dA) {
    charge_current_dA =
        datalayer.battery.info
            .max_charge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  if (discharge_current_dA > datalayer.battery.info.max_discharge_amp_dA) {
    discharge_current_dA =
        datalayer.battery.info
            .max_discharge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  average_temperature_dC =
      ((datalayer.battery.status.temperature_max_dC + datalayer.battery.status.temperature_min_dC) / 2);
  if (datalayer.battery.status.temperature_min_dC < 0) {
    average_temperature_dC = 0;
  }

  // if (datalayer.system.status.battery_allows_contactor_closing & datalayer.system.status.inverter_allows_contactor_closing ) {
  // } else {
  //   float2frame(CyclicData, 0.0, 6);
  // }
  // Set nominal voltage to value between min and max voltage set by battery (Example 400 and 300 results in 350V)
  nominal_voltage_dV =
      (((datalayer.battery.info.max_design_voltage_dV - datalayer.battery.info.min_design_voltage_dV) / 2) +
       datalayer.battery.info.min_design_voltage_dV);
  float2frame(BATTERY_INFO, (float)370.0f, 6);

  float2frame(CyclicData, (float)410.0f, 10); // max voltage

  float2frame(CyclicData, (float)17.1f, 14); // temperature

  if (state == STATE3_OPERATE) {
    float2frame(CyclicData, (float)datalayer.battery.status.current_dA / 10, 18);  // Peak discharge? current (float)
    float2frame(CyclicData, (float)datalayer.battery.status.current_dA / 10, 22);  // avg current (float)
  } else  {
    float2frame(CyclicData, (float)0.0f, 18);  // Peak discharge? current (float)
    float2frame(CyclicData, (float)0.0f, 22);  // avg current (float)
  }

  if (state == STATE2_CLOSING_DONE || state == STATE3_OPERATE) {
    float2frame(CyclicData, (float)13.0f, 26);  // max discharge current (float)
  } else {
    float2frame(CyclicData, (float)0.0f, 26);  // max discharge current (float)
  }
#endif

# if 1
  float2frame(CyclicData, (float)164.3f, 30); // battery Ah
#else
  /* from BYD trace... maybe too large batteries are rejected? */
  // float2frame(CyclicData, (float)25.0f, 30); // battery Ah
#endif

  CyclicData[57] = 0; // clear it
  if (state == STATE2_CLOSING_DONE || state == STATE3_OPERATE) {
      if ((datalayer.battery.status.reported_soc / 100) < 100) {
        float2frame(CyclicData, 13.0f, 34); // max charge current
      } else {
        // When SOC = 100%, drop down allowed charge current down.
        CyclicData[57] = 0x40; // cell overvoltage, aka. full
        float2frame(CyclicData, 0.0, 34);
      }
  } else {
    float2frame(CyclicData, 0.0, 34);
  }

  /* current voltage */
  if (state == STATE2_CLOSING_DONE || state == STATE3_OPERATE) {
    float2frame(CyclicData, (float)datalayer.battery.status.voltage_dV / 10, 6); // Confirmed OK mapping
  } else {
    float2frame(CyclicData, 0.0f, 6);
  }

  if (state == STATE2_CLOSING_DONE || state == STATE3_OPERATE) {
    CyclicData[56] = 0x01;  // Battery ready!  Contactors closed (?)
  } else {
    CyclicData[56] = 0x00;
  }

  if (state == STATE3_OPERATE) {
    CyclicData[59] = 0x00;
  } else {
    CyclicData[59] = 0x02;
  }

  if (state == STATE0_STANDBY) {
    CyclicData[61] = 0x01; // Waiting for inverter to send `07 E3 FF 02 FF 29 F4 00`
  } else {
    CyclicData[61] = 0x00;
  }

  if (state == STATE2_CLOSING_DONE || state == STATE3_OPERATE) {
    float2frame(CyclicData, (float)17.0f, 38); // cell temp max
    float2frame(CyclicData, (float)16.2f, 42); // cell temp min

    float2frame(CyclicData, (float)3.258f, 46); // cell voltage max
    float2frame(CyclicData, (float)3.250f, 50); // cell voltage min
  } else {
    float2frame(CyclicData, (float)0.0f, 38); // cell temp max
    float2frame(CyclicData, (float)0.0f, 42); // cell temp min

    float2frame(CyclicData, (float)0.0f, 46); // cell voltage max
    float2frame(CyclicData, (float)0.0f, 50); // cell voltage min
  }

  CyclicData[58] = (byte)(datalayer.battery.status.reported_soc / 100);  // Confirmed OK mapping

  register_content_ok = true;


  if (incoming_message_counter > 0) {
    incoming_message_counter--;
  }

  if (incoming_message_counter == 0) {
    set_event(EVENT_MODBUS_INVERTER_MISSING, 0);
    set_state(STATE0_STANDBY);
  } else {
    clear_event(EVENT_MODBUS_INVERTER_MISSING);
  }
}

static uint8_t rx_index = 0;

void receive_RS485()  // Runs as fast as possible to handle the serial stream
{
  currentMillis = millis();

  if (datalayer.system.status.battery_allows_contactor_closing & !contactorMillis) {
    contactorMillis = currentMillis;
  }
  if (currentMillis - contactorMillis >= INTERVAL_2_S & !RX_allow) {
    RX_allow = true;
  }

  if (startupMillis) {
    if (((currentMillis - startupMillis) >= INTERVAL_2_S & currentMillis - startupMillis <= 7000) &
      datalayer.system.status.inverter_allows_contactor_closing) {
      // Disconnect allowed only, when curren zero
      if (datalayer.battery.status.current_dA == 0) {
        datalayer.system.status.inverter_allows_contactor_closing = false;
      }
    } else if (((currentMillis - startupMillis) >= 7000) &
               datalayer.system.status.inverter_allows_contactor_closing == false) {
      datalayer.system.status.inverter_allows_contactor_closing = true;
    }
  }

  if (Serial.available()) {
    /* manually signal that the contactors have been closed */
    if (state == STATE1_CLOSE_CONTACTORS) {
      if (Serial.read() == 'd') {
        set_state(STATE2_CLOSING_DONE);
      }
    }
    // TODO: remove me, just testing
    Serial.println("Got something from console!");
    Serial.print(Serial.read());
    Serial.println();
  }

  if (Serial2.available()) {
    RS485_RXFRAME[rx_index] = Serial2.read();
    if (RX_allow) {
      rx_index++;
      if (RS485_RXFRAME[rx_index - 1] == 0x00) {
        // TODO: check that receiving header matches with sending header
        if (rx_index == 10 && (RS485_RXFRAME[0] == 0x09 || RS485_RXFRAME[0] == 0x07) && register_content_ok) {
#ifdef DEBUG_KOSTAL_RS485_DATA
          Serial.print("RX: ");
          for (uint8_t i = 0; i < 10; i++) {
            Serial.print(RS485_RXFRAME[i], HEX);
            Serial.print(" ");
          }
          Serial.println("");
#endif
          rx_index = 0;
          if (RS485_RXFRAME[0] == 0x07 || check_kostal_frame_crc()) {
            incoming_message_counter = RS485_HEALTHY;

            if (RS485_RXFRAME[1] == 'c') {
              if (RS485_RXFRAME[6] == 0x47) {
                // Set time function - Do nothing.
                send_kostal(frame4, 8); // ACK
              }
              if (RS485_RXFRAME[6] == 0x5E) {
                // Set State function
                if (RS485_RXFRAME[7] == 0x01) {
                  // State X
                  send_kostal(frame4, 8); // ACK
                }
                else if (RS485_RXFRAME[7] == 0x02) {
                  // request to apply voltage?
                  Serial.println("let's CLOSE THE CONTACTORS");
                  set_state(STATE1_CLOSE_CONTACTORS);
                  send_kostal(frame4, 8); // ACK
                }
                else if (RS485_RXFRAME[7] == 0x04) {
                  // INVALID
                  send_kostal(frame4, 8); // ACK
                } else {
                  send_kostal(frame4, 8); // ACK
                }
              }
            }
            else if (RS485_RXFRAME[1] == 'b') {
              if (RS485_RXFRAME[6] == 0x50) {
                //Reverse polarity, do nothing
              }
              else {
                int code=RS485_RXFRAME[6] + RS485_RXFRAME[7]*0x100;
                if (code == 0x44a) {
                  if (state == STATE2_CLOSING_DONE) {
                      closing_done_count++;
                      if (closing_done_count >= 5) {
                          set_state(STATE3_OPERATE);
                          closing_done_count = 0;
                      }
                  }
                  //Send cyclic data
                  update_values_battery();
                  update_RS485_registers_inverter();

                  byte tmpframe[64];  //copy values to prevent data manipulation during rewrite/crc calculation
                  memcpy(tmpframe, CyclicData, 64);
                  tmpframe[62] = calculate_kostal_crc(tmpframe, 62);
                  scramble_null_bytes(tmpframe,65);
                  send_kostal(tmpframe, 64);
                }
                else if (code == 0x84a) {
                  //Send  battery info
                  byte tmpframe[40];  //copy values to prevent data manipulation during rewrite/crc calculation
                  memcpy(tmpframe, BATTERY_INFO, 40);
                  tmpframe[38] = calculate_kostal_crc(tmpframe, 38);
                  scramble_null_bytes(tmpframe, 40);
                  send_kostal(tmpframe, 40);
                  if (!startupMillis) {
                    startupMillis = currentMillis;
                  }
                }
                else if (code == 0x353) {
                  //Send  battery error
                  send_kostal(frame3, 9);
                }
                else {
#ifdef DEBUG_KOSTAL_RS485_DATA
                  Serial.print("RX cmd1: ");
                  for (uint8_t i = 0; i < 10; i++) {
                      Serial.print(RS485_RXFRAME[i], HEX);
                      Serial.print(" ");
                  }
                  Serial.println("");
#endif
                }
              }
            }
            else {
#ifdef DEBUG_KOSTAL_RS485_DATA
              Serial.print("RX cmd2: ");
              for (uint8_t i = 0; i < 10; i++) {
                  Serial.print(RS485_RXFRAME[i], HEX);
                  Serial.print(" ");
              }
              Serial.println("");
#endif
            }
          }
          else {
#ifdef DEBUG_KOSTAL_RS485_DATA
            Serial.print("RX failed crc: ");
            for (uint8_t i = 0; i < 10; i++) {
              Serial.print(RS485_RXFRAME[i], HEX);
              Serial.print(" ");
            }
            Serial.println("");
#endif
         }
        }
        else {
#ifdef DEBUG_KOSTAL_RS485_DATA
          Serial.print("RX unknown hdr: ");
          for (uint8_t i = 0; i < 10; i++) {
            Serial.print(RS485_RXFRAME[i], HEX);
            Serial.print(" ");
          }
          Serial.println("");
#endif
        }
        rx_index = 0;
      }
    }
    if (rx_index >= 10) {
      rx_index = 0;
    }
  }
}

#endif
