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
static int16_t average_temperature_dC = 0;
static uint8_t incoming_message_counter = RS485_HEALTHY;
static int8_t f2_startup_count = 0;

static unsigned long B1_last_millis = 0;
static unsigned long currentMillis;
static unsigned long startupMillis = 0;
static unsigned long contactorMillis = 0;

static int32_t closing_done_count = 0;
static int32_t timeout_asking_count = 0;

#define STATE0_STANDBY 0
#define STATE1_ASKING_TO_CLOSE 1
#define STATE2_READY_TO_CLOSE 2
#define STATE3_CLOSING_DONE 3
#define STATE4_OPERATE 4

static int8_t state = STATE0_STANDBY;

union f32b {
  float f;
  byte b[4];
};

uint8_t frame1[40] = {0x00, 0xE2, 0xFF, 0x02, 0xFF, 0x29,  // Frame header
                      0x00, 0x00, 0x80, 0x43,  // 256.063 Nominal voltage / 5*51.2=256      first byte 0x01 or 0x04
                      0xE4, 0x70, 0x8A, 0x5C,  // These might be Umin & Unax, Uint16
                      0xB5, 0x00, 0xD3, 0x00,  // Battery Serial number? Modbus register 527
                      0x00, 0x00, 0xC8, 0x41,  // 25.0024  ?
                      0xC2, 0x18,              // Battery Firmware, modbus register 586
                      0x00, 0x00, 0x59, 0x42,  // 0x00005942 = 54.25 ??
                      0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0xA0, 0x00, 0x00, 0x00,
                      0x4D,   // CRC
                      0x00};  //

// values in frame2 will be overwritten at update_modbus_registers_inverter()

uint8_t frame2[64] = {
    0x00,  // This may also been 0x06, seen at startup when live values not valid, but also occasionally single frames.
    0xE2, 0xFF, 0x02, 0xFF, 0x29,  // frame Header

    0x1D, 0x5A, 0x85, 0x43,  // Current Voltage  (float)                        Modbus register 216, Bytes 6-9
    0x00, 0x00,              // Unknown, 0x03 seen also 0x0F, 0x07, might hava something to do with current
    0x8D, 0x43,              // Max Voltage      (2 byte float),                                     Bytes 12-13
    0x00, 0x00, 0xAC, 0x41,  // BAttery Temperature        (2 byte float)       Modbus register 214, Bytes 16-17
    0x00, 0x00, 0x00,
    0x00,  // Peak Current (1s period?),  Bytes 18-21 - Communication fault seen with some values (>10A?)
    0x00, 0x00, 0x00,
    0x00,  // Avg current  (1s period?), Bytes 22-25  - Communication fault seen with some values (>10A?)

    0x00, 0x00, 0x48, 0x42,  // Max discharge current (2 byte float), Bit 26-29,
                             // Sunspec: ADisChaMax

    0x00, 0x00,  // Unknown
    0xC8, 0x41,  // Battery gross capacity, Ah (2 byte float) , Bytes 30-33, Modbus 512

    0x00,  // Unknown
    0x16,  // This seems to have something to do with cell temperatures

    0xA0, 0x41,  // Max charge current (2 byte float) Bit 36-37, ZERO WHEN SOC=100
                 // Sunspec: AChaMax

    0xCD, 0xCC, 0xB4, 0x41,  // MaxCellTemp (4 byte float) Bit 38-41
    0x01, 0x0C, 0xA4, 0x41,  // MinCellTemp (4 byte float) Bit 42-45

    0xA4, 0x70, 0x55, 0x40,  // MaxCellVolt  (float), Bit 46-49
    0x7D, 0x3F, 0x55, 0x40,  // MinCellVolt  (float), Bit 50-53

    0xFE,  // Cylce count , Bit 54
    0x04,  // Cycle count? , Bit 55
    0x00,  // Byte 56
    0x00,  // When SOC=100 Byte57=0x40, at startup 0x03 (about 7 times), otherwise 0x02
    0x64,  // SOC , Bit 58
    0x00,  // Unknown, when byte 57 = 0x03, this 0x02, otherwise 0x01
    0x00,  // Unknown, Seen only 0x01
    0x00,  // Unknown, Mostly 0x02. seen also 0x01
    0x00,  // CRC (inverted sum of bytes 1-62 + 0xC0), Bit 62
    0x00};

// FE 04 01 40 xx 01 01 02 yy (fully charged)
// FE 02 01 02 xx 01 01 02 yy (charging or discharging)

uint8_t frame3[9] = {
    0x08, 0xE2, 0xFF, 0x02, 0xFF, 0x29,  //header
    0x06,                                //Unknown
    0xEF,                                //CRC
    0x00                                 //endbyte
};

uint8_t frame4[8] = {0x07, 0xE3, 0xFF, 0x02, 0xFF, 0x29, 0xF4, 0x00};

uint8_t frameB1[10] = {0x07, 0x63, 0xFF, 0x02, 0xFF, 0x29, 0x5E, 0x02, 0x16, 0x00};
uint8_t frameB1b[8] = {0x07, 0xE3, 0xFF, 0x02, 0xFF, 0x29, 0xF4, 0x00};

uint8_t RS485_RXFRAME[10];

bool register_content_ok = false;

static void set_state(int next_state, bool force) {
    if (state == 0 && state == next_state) {
        return;
    }
    Serial.print("[");
    Serial.print(millis());
    Serial.print(" ms] SWITCH STATE: before=");
    Serial.print(state);
    Serial.print(", next=");
    Serial.print(next_state);
    Serial.println();

    if (force) {
        Serial.println(" forced transition");
        state = next_state;
    } else if ((state == 0 && next_state == 0) || (state == 0 && next_state == 1) || (state == 1 && next_state == 2) || (state == 2 && next_state == 3) || (state == 3 && next_state == 4)) {
        /* all good */
        state = next_state;
    } else if (state == 3 && next_state == 0) {
        state = next_state;
        Serial.println("    -> state reset");
    } else {
        Serial.println("    -> state transition IGNORED");
    }
}
static void set_state(int next_state) {
    set_state(next_state, false);
}

static void print_state(void) {
    Serial.print("[");
    Serial.print(millis());
    Serial.print(" ms] ");
    if (state == STATE0_STANDBY) {
        Serial.println("  >> STATE0_STANDBY <<");
    } else if (state == STATE1_ASKING_TO_CLOSE) {
        Serial.println("  >> STATE1_ASKING_TO_CLOSE <<");
    } else if (state == STATE2_READY_TO_CLOSE) {
        Serial.println("  >> STATE2_READY_TO_CLOSE <<");
    } else if (state == STATE3_CLOSING_DONE) {
        Serial.println("  >> STATE3_CLOSING_DONE <<");
    } else if (state == STATE4_OPERATE) {
        Serial.println("  >> STATE4_OPERATE <<");
    }
}

static void float2frame(byte* arr, float value, byte framepointer) {
  f32b g;
  g.f = value;
  arr[framepointer] = g.b[0];
  arr[framepointer + 1] = g.b[1];
  arr[framepointer + 2] = g.b[2];
  arr[framepointer + 3] = g.b[3];
}

static void float2frameMSB(byte* arr, float value, byte framepointer) {
  f32b g;
  g.f = value;
  arr[framepointer + 0] = g.b[2];
  arr[framepointer + 1] = g.b[3];
}

static void send_kostal(byte* arr, int alen) {
#ifdef DEBUG_KOSTAL_RS485_DATA
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] TX(state=");
  Serial.print(state);
  Serial.print("): ");
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

static void scramble_null_bytes(byte *lfc, int len) {
  int last_null_byte = 0;
  for (int i = 0; i < len; i++) {
    if (lfc[i] == '\0') {
      lfc[last_null_byte] = (byte) (i - last_null_byte);
      last_null_byte = i;
    }
  }
}

static byte calculate_kostal_crc(byte *lfc, int len) {
  unsigned int sum = 0;
  if (lfc[0] != 0) {
      printf("WARNING: first byte should be 0, but is 0x%02x\n", lfc[0]);
  }
  for (int i = 1; i < len; i++) {
      sum += lfc[i];
  }
  return (byte) (-sum & 0xff);
}

static bool check_kostal_frame_crc(int length) {
  unsigned int sum = 0;

  /* assumption: There is no \0 byte except the terminating one */
  if (RS485_RXFRAME[0] < length) {
      /* except for one case, 07 63 FF 02 FF 29 5E 02 16 00 */
      /* clear that byte at index 7*/
      RS485_RXFRAME[RS485_RXFRAME[0]] = '\0';
  }

  for (int i = 1; i < length; ++i) {
    sum += RS485_RXFRAME[i];
  }
  if (((~sum + 1) & 0xff) == (RS485_RXFRAME[length - 1] & 0xff)) {
    return (true);
  } else {
    return (false);
  }
}

static void reset_state(void) {
  f2_startup_count = 14;
  closing_done_count = 0;
  contactorMillis = 0;
  timeout_asking_count = 0;
  state = STATE0_STANDBY;
  datalayer.system.status.inverter_allows_contactor_closing = false;  // The inverter needs to allow first
}

void update_RS485_registers_inverter() {

  average_temperature_dC =
      ((datalayer.battery.status.temperature_max_dC + datalayer.battery.status.temperature_min_dC) / 2);
  if (datalayer.battery.status.temperature_min_dC < 0) {
    average_temperature_dC = 0;
  }

  if (state == STATE3_CLOSING_DONE || state == STATE4_OPERATE) {
    float2frame(frame2, (float)datalayer.battery.status.voltage_dV / 10, 6);  // Confirmed OK mapping
  } else {
    float2frame(frame2, 0.0, 6);
  }
  // Set nominal voltage to value between min and max voltage set by battery (Example 400 and 300 results in 350V)
  nominal_voltage_dV =
      (((datalayer.battery.info.max_design_voltage_dV - datalayer.battery.info.min_design_voltage_dV) / 2) +
       datalayer.battery.info.min_design_voltage_dV);
  float2frameMSB(frame1, (float)nominal_voltage_dV / 10, 8);

  float2frameMSB(frame2, (float)datalayer.battery.info.max_design_voltage_dV / 10, 12);

  float2frameMSB(frame2, (float)average_temperature_dC / 10, 16);

  if (state == STATE1_ASKING_TO_CLOSE) {
      timeout_asking_count++;

      if (timeout_asking_count > 23) {
        /* fake it for one frame */
        float2frame(frame2, (float)datalayer.battery.status.voltage_dV / 10, 6);

        Serial.println("!!! HACK VOLTAGE BUMP !!!");
        timeout_asking_count = 0;
      }
  }

  //  Some current values causes communication error, must be resolved, why.
  //  float2frameMSB(frame2, (float)datalayer.battery.status.current_dA / 10, 20);  // Peak discharge? current (2 byte float)
  //  float2frameMSB(frame2, (float)datalayer.battery.status.current_dA / 10, 24);

  float2frameMSB(frame2, (float)datalayer.battery.status.max_discharge_current_dA / 10, 28);  // BAttery capacity Ah

  float2frameMSB(frame2, (float)datalayer.battery.status.max_discharge_current_dA / 10, 32);

  frame2[57] = 0;
  if (state == STATE3_CLOSING_DONE || state == STATE4_OPERATE) {
    // When SOC = 100%, drop down allowed charge current down.
    if ((datalayer.battery.status.reported_soc / 100) < 100) {
      float2frameMSB(frame2, (float)datalayer.battery.status.max_charge_current_dA / 10, 36);
    } else {
      float2frameMSB(frame2, 0.0, 36);
      frame2[57] |= 0x40;
    }
  } else {
    float2frameMSB(frame2, 0.0, 36);
  }

  if (state == STATE3_CLOSING_DONE || state == STATE4_OPERATE) {
    frame2[56] = 0x01;  // Battery ready!  Contactors closed (?)
  } else {
    frame2[56] = 0x00;
  }

  if (state == STATE4_OPERATE) {
    frame2[59] = 0x00;
  } else {
    frame2[59] = 0x02;
  }

  if (state == STATE0_STANDBY) {
    frame2[61] = 0x01; // only set on first message with inverter
  } else {
    frame2[61] = 0x00;
  }

  float2frame(frame2, (float)datalayer.battery.status.temperature_max_dC / 10, 38);
  float2frame(frame2, (float)datalayer.battery.status.temperature_min_dC / 10, 42);

  float2frame(frame2, (float)datalayer.battery.status.cell_max_voltage_mV / 1000, 46);
  float2frame(frame2, (float)datalayer.battery.status.cell_min_voltage_mV / 1000, 50);

  frame2[58] = (byte)(datalayer.battery.status.reported_soc / 100);  // Confirmed OK mapping

  register_content_ok = true;

  if (incoming_message_counter > 0) {
    incoming_message_counter--;
  }

  if (incoming_message_counter == 0) {
    set_event(EVENT_MODBUS_INVERTER_MISSING, 0);
    reset_state();
  } else {
    clear_event(EVENT_MODBUS_INVERTER_MISSING);
  }
}

static void dump_rs485_data_rx(const char* prefix) {
#ifdef DEBUG_KOSTAL_RS485_DATA
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] RX");
  Serial.print(prefix);
  Serial.print(": ");
  for (uint8_t i = 0; i < 10; i++) {
    if (RS485_RXFRAME[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(RS485_RXFRAME[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
#endif
}

static uint8_t rx_index = 0;

void receive_RS485()  // Runs as fast as possible to handle the serial stream
{

  if (!contactorMillis && state == STATE2_READY_TO_CLOSE) {
    currentMillis = millis();
    if ((currentMillis - contactorMillis) >= INTERVAL_2_S) {
      set_state(STATE3_CLOSING_DONE);
      contactorMillis = 0;
    }
  }
#if 0
  if (datalayer.system.status.battery_allows_contactor_closing & !contactorMillis) {
    contactorMillis = currentMillis;
  }
  if (currentMillis - contactorMillis >= INTERVAL_2_S & !RX_allow) {
#ifdef DEBUG_KOSTAL_RS485_DATA
    Serial.println("RX_allow -> true");
#endif
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
#ifdef DEBUG_KOSTAL_RS485_DATA
      Serial.println("inverter_allows_contactor_closing -> true");
#endif
    }
  }
#endif

  if (Serial2.available()) {
    RS485_RXFRAME[rx_index] = Serial2.read();
    rx_index++;
    if (RS485_RXFRAME[rx_index - 1] == 0x00 && !datalayer.system.settings.equipment_stop_active) {
      if ((rx_index == 10) && (RS485_RXFRAME[0] == 0x09 || RS485_RXFRAME[0] == 0x07) && register_content_ok) {
        dump_rs485_data_rx("");
        rx_index = 0;
        if (check_kostal_frame_crc(10)) {
          incoming_message_counter = RS485_HEALTHY;
          bool headerA = true;
          bool headerB = true;
          for (uint8_t i = 0; i < 5; i++) {
            if (RS485_RXFRAME[i + 1] != KOSTAL_FRAMEHEADER[i]) {
              headerA = false;
            }
            if (RS485_RXFRAME[i + 1] != KOSTAL_FRAMEHEADER2[i]) {
              headerB = false;
            }
          }

          // // "frame B1", maybe reset request, seen after battery power on/partial data
          // if (headerB && (RS485_RXFRAME[6] == 0x5E) && (RS485_RXFRAME[7] == 0xFF)) {
          //   send_kostal(frameB1, 10);
          //   B1_delay = true;
          //   B1_last_millis = currentMillis;
          // }

          // "frame B1", maybe reset request, seen after battery power on/partial data
          if (headerB && (RS485_RXFRAME[6] == 0x5E) && (RS485_RXFRAME[7] == 0x04)) {
            send_kostal(frame4, 8);
            datalayer.system.status.inverter_allows_contactor_closing = true;
            set_state(STATE2_READY_TO_CLOSE);
#ifdef DEBUG_KOSTAL_RS485_DATA
            Serial.println("Kostal(!!!): inverter_allows_contactor_closing -> true");
            Serial.println("-> state partial");
            Serial.println("");
#endif
            // This needs more reverse engineering, disabled...
          }

          if (RS485_RXFRAME[6] == 0x5E && RS485_RXFRAME[7] == 0x00) {
            // clearance to apply voltage
            send_kostal(frame4, 8); // ACK
            datalayer.system.status.inverter_allows_contactor_closing = true;
            set_state(STATE2_READY_TO_CLOSE);
#ifdef DEBUG_KOSTAL_RS485_DATA
            Serial.println("Kostal(!!!): inverter_allows_contactor_closing -> true");
            Serial.println("-> state fresh");
            Serial.println("");
#endif
          }

          if (headerA && (RS485_RXFRAME[6] == 0x4A) && (RS485_RXFRAME[7] == 0x08)) {  // "frame 1"
            byte tmpframe[40];  //copy values to prevent data manipulation during rewrite/crc calculation
            memcpy(tmpframe, frame1, 40);
            tmpframe[38] = calculate_kostal_crc(tmpframe, 38);
            scramble_null_bytes(tmpframe, 40);
            send_kostal(tmpframe, 40);
            f2_startup_count = 14;
          }
          if (headerA && (RS485_RXFRAME[6] == 0x4A) && (RS485_RXFRAME[7] == 0x04)) {  // "frame 2"
            if (f2_startup_count > 0) {
              f2_startup_count--;
#ifdef DEBUG_KOSTAL_RS485_DATA
              Serial.print("[");
              Serial.print(millis());
              Serial.print(" ms] f2_startup_count: ");
              Serial.print(f2_startup_count);
              Serial.println();
              Serial.println();
#endif
            }
            if (state == STATE3_CLOSING_DONE) {
                closing_done_count++;
                if (closing_done_count >= 8) {
                    set_state(STATE4_OPERATE);
                    closing_done_count = 0;
                }
            }

            update_values_battery();
            update_RS485_registers_inverter();


            byte tmpframe[64];  //copy values to prevent data manipulation during rewrite/crc calculation
            memcpy(tmpframe, frame2, 64);
            tmpframe[62] = calculate_kostal_crc(tmpframe, 62);
            scramble_null_bytes(tmpframe,65);
            send_kostal(tmpframe, 64);

            /* only send one frame in STATE0_STANDBY */
            if (state == STATE0_STANDBY) {
              set_state(STATE1_ASKING_TO_CLOSE);
            }
          }
          if (headerA && (RS485_RXFRAME[6] == 0x53) && (RS485_RXFRAME[7] == 0x03)) {
            /* Battery error. send everything okay */
            send_kostal(frame3, 9);
          }
        }
      } else {
        dump_rs485_data_rx(" (dropped)");
      }
      rx_index = 0;
    }
  }
}

void setup_inverter(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.inverter_protocol, "BYD battery via Kostal RS485", 63);
  datalayer.system.info.inverter_protocol[63] = '\0';
  reset_state();
}

#endif
