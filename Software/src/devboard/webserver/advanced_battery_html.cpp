#include "advanced_battery_html.h"
#include <Arduino.h>
#include "../../datalayer/datalayer.h"
#include "../../datalayer/datalayer_extended.h"

String advanced_battery_processor(const String& var) {
  if (var == "X") {
    String content = "";
    //Page format
    content += "<style>";
    content += "body { background-color: black; color: white; }";
    content += "</style>";

    content += "<button onclick='goToMainPage()'>Back to main page</button>";

    // Start a new block with a specific background color
    content += "<div style='background-color: #303E47; padding: 10px; margin-bottom: 10px;border-radius: 50px'>";

#ifdef BMW_I3_BATTERY
    content += "<h4>SOC raw: " + String(datalayer_extended.bmwi3.SOC_raw) + "</h4>";
    content += "<h4>SOC dash: " + String(datalayer_extended.bmwi3.SOC_dash) + "</h4>";
    content += "<h4>SOC OBD2: " + String(datalayer_extended.bmwi3.SOC_OBD2) + "</h4>";
    static const char* statusText[16] = {
        "Not evaluated", "OK", "Error!", "Invalid signal", "", "", "", "", "", "", "", "", "", "", "", ""};
    content += "<h4>Interlock: " + String(statusText[datalayer_extended.bmwi3.ST_interlock]) + "</h4>";
    content += "<h4>Isolation external: " + String(statusText[datalayer_extended.bmwi3.ST_iso_ext]) + "</h4>";
    content += "<h4>Isolation internal: " + String(statusText[datalayer_extended.bmwi3.ST_iso_int]) + "</h4>";
    content += "<h4>Isolation: " + String(statusText[datalayer_extended.bmwi3.ST_isolation]) + "</h4>";
    content += "<h4>Cooling valve: " + String(statusText[datalayer_extended.bmwi3.ST_valve_cooling]) + "</h4>";
    content += "<h4>Emergency: " + String(statusText[datalayer_extended.bmwi3.ST_EMG]) + "</h4>";
    static const char* prechargeText[16] = {"Not evaluated",
                                            "Not active, closing not blocked",
                                            "Error precharge blocked",
                                            "Invalid signal",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            "",
                                            ""};
    content += "<h4>Precharge: " + String(prechargeText[datalayer_extended.bmwi3.ST_precharge]) +
               "</h4>";  //Still unclear of enum
    static const char* DCSWText[16] = {"Contactors open",
                                       "Precharge ongoing",
                                       "Contactors engaged",
                                       "Invalid signal",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       ""};
    content += "<h4>Contactor status: " + String(DCSWText[datalayer_extended.bmwi3.ST_DCSW]) + "</h4>";
    static const char* contText[16] = {"Contactors OK",
                                       "One contactor welded!",
                                       "Two contactors welded!",
                                       "Invalid signal",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       ""};
    content += "<h4>Contactor weld: " + String(contText[datalayer_extended.bmwi3.ST_WELD]) + "</h4>";
    static const char* valveText[16] = {"OK",
                                        "Short circuit to GND",
                                        "Short circuit to 12V",
                                        "Line break",
                                        "",
                                        "",
                                        "Driver error",
                                        "",
                                        "",
                                        "",
                                        "",
                                        "",
                                        "Stuck",
                                        "Stuck",
                                        "",
                                        "Invalid Signal"};
    content += "<h4>Cold shutoff valve: " + String(contText[datalayer_extended.bmwi3.ST_cold_shutoff_valve]) + "</h4>";

#endif  //BMW_I3_BATTERY

#ifdef CELLPOWER_BMS
    static const char* falseTrue[2] = {"False", "True"};
    content += "<h3>States:</h3>";
    content += "<h4>Discharge: " + String(falseTrue[datalayer_extended.cellpower.system_state_discharge]) + "</h4>";
    content += "<h4>Charge: " + String(falseTrue[datalayer_extended.cellpower.system_state_charge]) + "</h4>";
    content +=
        "<h4>Cellbalancing: " + String(falseTrue[datalayer_extended.cellpower.system_state_cellbalancing]) + "</h4>";
    content +=
        "<h4>Tricklecharging: " + String(falseTrue[datalayer_extended.cellpower.system_state_tricklecharge]) + "</h4>";
    content += "<h4>Idle: " + String(falseTrue[datalayer_extended.cellpower.system_state_idle]) + "</h4>";
    content += "<h4>Charge completed: " + String(falseTrue[datalayer_extended.cellpower.system_state_chargecompleted]) +
               "</h4>";
    content +=
        "<h4>Maintenance charge: " + String(falseTrue[datalayer_extended.cellpower.system_state_maintenancecharge]) +
        "</h4>";
    content += "<h3>IO:</h3>";
    content +=
        "<h4>Main positive relay: " + String(falseTrue[datalayer_extended.cellpower.IO_state_main_positive_relay]) +
        "</h4>";
    content +=
        "<h4>Main negative relay: " + String(falseTrue[datalayer_extended.cellpower.IO_state_main_negative_relay]) +
        "</h4>";
    content +=
        "<h4>Charge enabled: " + String(falseTrue[datalayer_extended.cellpower.IO_state_charge_enable]) + "</h4>";
    content +=
        "<h4>Precharge relay: " + String(falseTrue[datalayer_extended.cellpower.IO_state_precharge_relay]) + "</h4>";
    content +=
        "<h4>Discharge enable: " + String(falseTrue[datalayer_extended.cellpower.IO_state_discharge_enable]) + "</h4>";
    content += "<h4>IO 6: " + String(falseTrue[datalayer_extended.cellpower.IO_state_IO_6]) + "</h4>";
    content += "<h4>IO 7: " + String(falseTrue[datalayer_extended.cellpower.IO_state_IO_7]) + "</h4>";
    content += "<h4>IO 8: " + String(falseTrue[datalayer_extended.cellpower.IO_state_IO_8]) + "</h4>";
    content += "<h3>Errors:</h3>";
    content +=
        "<h4>Cell overvoltage: " + String(falseTrue[datalayer_extended.cellpower.error_Cell_overvoltage]) + "</h4>";
    content +=
        "<h4>Cell undervoltage: " + String(falseTrue[datalayer_extended.cellpower.error_Cell_undervoltage]) + "</h4>";
    content += "<h4>Cell end of life voltage: " +
               String(falseTrue[datalayer_extended.cellpower.error_Cell_end_of_life_voltage]) + "</h4>";
    content +=
        "<h4>Cell voltage misread: " + String(falseTrue[datalayer_extended.cellpower.error_Cell_voltage_misread]) +
        "</h4>";
    content +=
        "<h4>Cell over temperature: " + String(falseTrue[datalayer_extended.cellpower.error_Cell_over_temperature]) +
        "</h4>";
    content +=
        "<h4>Cell under temperature: " + String(falseTrue[datalayer_extended.cellpower.error_Cell_under_temperature]) +
        "</h4>";
    content += "<h4>Cell unmanaged: " + String(falseTrue[datalayer_extended.cellpower.error_Cell_unmanaged]) + "</h4>";
    content +=
        "<h4>LMU over temperature: " + String(falseTrue[datalayer_extended.cellpower.error_LMU_over_temperature]) +
        "</h4>";
    content +=
        "<h4>LMU under temperature: " + String(falseTrue[datalayer_extended.cellpower.error_LMU_under_temperature]) +
        "</h4>";
    content += "<h4>Temp sensor open circuit: " +
               String(falseTrue[datalayer_extended.cellpower.error_Temp_sensor_open_circuit]) + "</h4>";
    content += "<h4>Temp sensor short circuit: " +
               String(falseTrue[datalayer_extended.cellpower.error_Temp_sensor_short_circuit]) + "</h4>";
    content += "<h4>SUB comm: " + String(falseTrue[datalayer_extended.cellpower.error_SUB_communication]) + "</h4>";
    content += "<h4>LMU comm: " + String(falseTrue[datalayer_extended.cellpower.error_LMU_communication]) + "</h4>";
    content +=
        "<h4>Over current In: " + String(falseTrue[datalayer_extended.cellpower.error_Over_current_IN]) + "</h4>";
    content +=
        "<h4>Over current Out: " + String(falseTrue[datalayer_extended.cellpower.error_Over_current_OUT]) + "</h4>";
    content += "<h4>Short circuit: " + String(falseTrue[datalayer_extended.cellpower.error_Short_circuit]) + "</h4>";
    content += "<h4>Leak detected: " + String(falseTrue[datalayer_extended.cellpower.error_Leak_detected]) + "</h4>";
    content +=
        "<h4>Leak detection failed: " + String(falseTrue[datalayer_extended.cellpower.error_Leak_detection_failed]) +
        "</h4>";
    content +=
        "<h4>Voltage diff: " + String(falseTrue[datalayer_extended.cellpower.error_Voltage_difference]) + "</h4>";
    content += "<h4>BMCU supply overvoltage: " +
               String(falseTrue[datalayer_extended.cellpower.error_BMCU_supply_over_voltage]) + "</h4>";
    content += "<h4>BMCU supply undervoltage: " +
               String(falseTrue[datalayer_extended.cellpower.error_BMCU_supply_under_voltage]) + "</h4>";
    content += "<h4>Main positive contactor: " +
               String(falseTrue[datalayer_extended.cellpower.error_Main_positive_contactor]) + "</h4>";
    content += "<h4>Main negative contactor: " +
               String(falseTrue[datalayer_extended.cellpower.error_Main_negative_contactor]) + "</h4>";
    content += "<h4>Precharge contactor: " + String(falseTrue[datalayer_extended.cellpower.error_Precharge_contactor]) +
               "</h4>";
    content +=
        "<h4>Midpack contactor: " + String(falseTrue[datalayer_extended.cellpower.error_Midpack_contactor]) + "</h4>";
    content +=
        "<h4>Precharge timeout: " + String(falseTrue[datalayer_extended.cellpower.error_Precharge_timeout]) + "</h4>";
    content += "<h4>EMG connector override: " +
               String(falseTrue[datalayer_extended.cellpower.error_Emergency_connector_override]) + "</h4>";
    content += "<h3>Warnings:</h3>";
    content +=
        "<h4>High cell voltage: " + String(falseTrue[datalayer_extended.cellpower.warning_High_cell_voltage]) + "</h4>";
    content +=
        "<h4>Low cell voltage: " + String(falseTrue[datalayer_extended.cellpower.warning_Low_cell_voltage]) + "</h4>";
    content +=
        "<h4>High cell temperature: " + String(falseTrue[datalayer_extended.cellpower.warning_High_cell_temperature]) +
        "</h4>";
    content +=
        "<h4>Low cell temperature: " + String(falseTrue[datalayer_extended.cellpower.warning_Low_cell_temperature]) +
        "</h4>";
    content +=
        "<h4>High LMU temperature: " + String(falseTrue[datalayer_extended.cellpower.warning_High_LMU_temperature]) +
        "</h4>";
    content +=
        "<h4>Low LMU temperature: " + String(falseTrue[datalayer_extended.cellpower.warning_Low_LMU_temperature]) +
        "</h4>";
    content +=
        "<h4>SUB comm interf: " + String(falseTrue[datalayer_extended.cellpower.warning_SUB_communication_interfered]) +
        "</h4>";
    content +=
        "<h4>LMU comm interf: " + String(falseTrue[datalayer_extended.cellpower.warning_LMU_communication_interfered]) +
        "</h4>";
    content +=
        "<h4>High current In: " + String(falseTrue[datalayer_extended.cellpower.warning_High_current_IN]) + "</h4>";
    content +=
        "<h4>High current Out: " + String(falseTrue[datalayer_extended.cellpower.warning_High_current_OUT]) + "</h4>";
    content += "<h4>Pack resistance diff: " +
               String(falseTrue[datalayer_extended.cellpower.warning_Pack_resistance_difference]) + "</h4>";
    content +=
        "<h4>High pack resistance: " + String(falseTrue[datalayer_extended.cellpower.warning_High_pack_resistance]) +
        "</h4>";
    content += "<h4>Cell resistance diff: " +
               String(falseTrue[datalayer_extended.cellpower.warning_Cell_resistance_difference]) + "</h4>";
    content +=
        "<h4>High cell resistance: " + String(falseTrue[datalayer_extended.cellpower.warning_High_cell_resistance]) +
        "</h4>";
    content += "<h4>High BMCU supply voltage: " +
               String(falseTrue[datalayer_extended.cellpower.warning_High_BMCU_supply_voltage]) + "</h4>";
    content += "<h4>Low BMCU supply voltage: " +
               String(falseTrue[datalayer_extended.cellpower.warning_Low_BMCU_supply_voltage]) + "</h4>";
    content += "<h4>Low SOC: " + String(falseTrue[datalayer_extended.cellpower.warning_Low_SOC]) + "</h4>";
    content += "<h4>Balancing required: " +
               String(falseTrue[datalayer_extended.cellpower.warning_Balancing_required_OCV_model]) + "</h4>";
    content += "<h4>Charger not responding: " +
               String(falseTrue[datalayer_extended.cellpower.warning_Charger_not_responding]) + "</h4>";
#endif  //CELLPOWER_BMS

#ifdef BYD_ATTO_3_BATTERY
    content += "<h4>SOC estimated: " + String(datalayer_extended.bydAtto3.SOC_estimated) + "</h4>";
    content += "<h4>SOC highprec: " + String(datalayer_extended.bydAtto3.SOC_highprec) + "</h4>";
    content += "<h4>SOC OBD2: " + String(datalayer_extended.bydAtto3.SOC_polled) + "</h4>";
    content += "<h4>Voltage periodic: " + String(datalayer_extended.bydAtto3.voltage_periodic) + "</h4>";
    content += "<h4>Voltage OBD2: " + String(datalayer_extended.bydAtto3.voltage_polled) + "</h4>";
#endif  //BYD_ATTO_3_BATTERY

#ifdef TESLA_BATTERY
    static const char* contactorText[] = {"UNKNOWN(0)",  "OPEN",        "CLOSING",    "BLOCKED", "OPENING",
                                          "CLOSED",      "UNKNOWN(6)",  "WELDED",     "POS_CL",  "NEG_CL",
                                          "UNKNOWN(10)", "UNKNOWN(11)", "UNKNOWN(12)"};
    content += "<h4>Contactor Status: " + String(contactorText[datalayer_extended.tesla.status_contactor]) + "</h4>";
    static const char* hvilStatusState[] = {"NOT OK",
                                            "STATUS_OK",
                                            "CURRENT_SOURCE_FAULT",
                                            "INTERNAL_OPEN_FAULT",
                                            "VEHICLE_OPEN_FAULT",
                                            "PENTHOUSE_LID_OPEN_FAULT",
                                            "UNKNOWN_LOCATION_OPEN_FAULT",
                                            "VEHICLE_NODE_FAULT",
                                            "NO_12V_SUPPLY",
                                            "VEHICLE_OR_PENTHOUSE_LID_OPENFAULT",
                                            "UNKNOWN(10)",
                                            "UNKNOWN(11)",
                                            "UNKNOWN(12)",
                                            "UNKNOWN(13)",
                                            "UNKNOWN(14)",
                                            "UNKNOWN(15)"};
    content += "<h4>HVIL: " + String(hvilStatusState[datalayer_extended.tesla.hvil_status]) + "</h4>";
    static const char* contactorState[] = {"SNA",        "OPEN",       "PRECHARGE",   "BLOCKED",
                                           "PULLED_IN",  "OPENING",    "ECONOMIZED",  "WELDED",
                                           "UNKNOWN(8)", "UNKNOWN(9)", "UNKNOWN(10)", "UNKNOWN(11)"};
    content +=
        "<h4>Negative contactor: " + String(contactorState[datalayer_extended.tesla.packContNegativeState]) + "</h4>";
    content +=
        "<h4>Positive contactor: " + String(contactorState[datalayer_extended.tesla.packContPositiveState]) + "</h4>";
    static const char* falseTrue[] = {"False", "True"};
    content += "<h4>Closing allowed?: " + String(falseTrue[datalayer_extended.tesla.packCtrsClosingAllowed]) + "</h4>";
    content += "<h4>Pyrotest: " + String(falseTrue[datalayer_extended.tesla.pyroTestInProgress]) + "</h4>";
#endif

#ifdef NISSAN_LEAF_BATTERY
    content += "<h4>LEAF generation: " + String(datalayer_extended.nissanleaf.LEAF_gen) + "</h4>";
    content += "<h4>GIDS: " + String(datalayer_extended.nissanleaf.GIDS) + "</h4>";
    content += "<h4>Regen kW: " + String(datalayer_extended.nissanleaf.ChargePowerLimit) + "</h4>";
    content += "<h4>Charge kW: " + String(datalayer_extended.nissanleaf.MaxPowerForCharger) + "</h4>";
    content += "<h4>Interlock: " + String(datalayer_extended.nissanleaf.Interlock) + "</h4>";
    content += "<h4>Relay cut request: " + String(datalayer_extended.nissanleaf.RelayCutRequest) + "</h4>";
    content += "<h4>Failsafe status: " + String(datalayer_extended.nissanleaf.FailsafeStatus) + "</h4>";
    content += "<h4>Fully charged: " + String(datalayer_extended.nissanleaf.Full) + "</h4>";
    content += "<h4>Battery empty: " + String(datalayer_extended.nissanleaf.Empty) + "</h4>";
    content += "<h4>Main relay ON: " + String(datalayer_extended.nissanleaf.MainRelayOn) + "</h4>";
    content += "<h4>Heater present: " + String(datalayer_extended.nissanleaf.HeatExist) + "</h4>";
    content += "<h4>Heating stopped: " + String(datalayer_extended.nissanleaf.HeatingStop) + "</h4>";
    content += "<h4>Heating started: " + String(datalayer_extended.nissanleaf.HeatingStart) + "</h4>";
    content += "<h4>Heating requested: " + String(datalayer_extended.nissanleaf.HeaterSendRequest) + "</h4>";
#endif

#ifdef RENAULT_ZOE_GEN2_BATTERY
    content += "<h4>soc: " + String(datalayer_extended.zoePH2.battery_soc) + "</h4>";
    content += "<h4>usable soc: " + String(datalayer_extended.zoePH2.battery_usable_soc) + "</h4>";
    content += "<h4>soh: " + String(datalayer_extended.zoePH2.battery_soh) + "</h4>";
    content += "<h4>pack voltage: " + String(datalayer_extended.zoePH2.battery_pack_voltage) + "</h4>";
    content += "<h4>max cell voltage: " + String(datalayer_extended.zoePH2.battery_max_cell_voltage) + "</h4>";
    content += "<h4>min cell voltage: " + String(datalayer_extended.zoePH2.battery_min_cell_voltage) + "</h4>";
    content += "<h4>12v: " + String(datalayer_extended.zoePH2.battery_12v) + "</h4>";
    content += "<h4>avg temp: " + String(datalayer_extended.zoePH2.battery_avg_temp) + "</h4>";
    content += "<h4>min temp: " + String(datalayer_extended.zoePH2.battery_min_temp) + "</h4>";
    content += "<h4>max temp: " + String(datalayer_extended.zoePH2.battery_max_temp) + "</h4>";
    content += "<h4>max power: " + String(datalayer_extended.zoePH2.battery_max_power) + "</h4>";
    content += "<h4>interlock: " + String(datalayer_extended.zoePH2.battery_interlock) + "</h4>";
    content += "<h4>kwh: " + String(datalayer_extended.zoePH2.battery_kwh) + "</h4>";
    content += "<h4>current: " + String(datalayer_extended.zoePH2.battery_current) + "</h4>";
    content += "<h4>current offset: " + String(datalayer_extended.zoePH2.battery_current_offset) + "</h4>";
    content += "<h4>max generated: " + String(datalayer_extended.zoePH2.battery_max_generated) + "</h4>";
    content += "<h4>max available: " + String(datalayer_extended.zoePH2.battery_max_available) + "</h4>";
    content += "<h4>current voltage: " + String(datalayer_extended.zoePH2.battery_current_voltage) + "</h4>";
    content += "<h4>charging status: " + String(datalayer_extended.zoePH2.battery_charging_status) + "</h4>";
    content += "<h4>remaining charge: " + String(datalayer_extended.zoePH2.battery_remaining_charge) + "</h4>";
    content +=
        "<h4>balance capacity total: " + String(datalayer_extended.zoePH2.battery_balance_capacity_total) + "</h4>";
    content += "<h4>balance time total: " + String(datalayer_extended.zoePH2.battery_balance_time_total) + "</h4>";
    content +=
        "<h4>balance capacity sleep: " + String(datalayer_extended.zoePH2.battery_balance_capacity_sleep) + "</h4>";
    content += "<h4>balance time sleep: " + String(datalayer_extended.zoePH2.battery_balance_time_sleep) + "</h4>";
    content +=
        "<h4>balance capacity wake: " + String(datalayer_extended.zoePH2.battery_balance_capacity_wake) + "</h4>";
    content += "<h4>balance time wake: " + String(datalayer_extended.zoePH2.battery_balance_time_wake) + "</h4>";
    content += "<h4>bms state: " + String(datalayer_extended.zoePH2.battery_bms_state) + "</h4>";
    content += "<h4>balance switches: " + String(datalayer_extended.zoePH2.battery_balance_switches) + "</h4>";
    content += "<h4>energy complete: " + String(datalayer_extended.zoePH2.battery_energy_complete) + "</h4>";
    content += "<h4>energy partial: " + String(datalayer_extended.zoePH2.battery_energy_partial) + "</h4>";
    content += "<h4>slave failures: " + String(datalayer_extended.zoePH2.battery_slave_failures) + "</h4>";
    content += "<h4>mileage: " + String(datalayer_extended.zoePH2.battery_mileage) + "</h4>";
    content += "<h4>fan speed: " + String(datalayer_extended.zoePH2.battery_fan_speed) + "</h4>";
    content += "<h4>fan period: " + String(datalayer_extended.zoePH2.battery_fan_period) + "</h4>";
    content += "<h4>fan control: " + String(datalayer_extended.zoePH2.battery_fan_control) + "</h4>";
    content += "<h4>fan duty: " + String(datalayer_extended.zoePH2.battery_fan_duty) + "</h4>";
    content += "<h4>temporisation: " + String(datalayer_extended.zoePH2.battery_temporisation) + "</h4>";
    content += "<h4>time: " + String(datalayer_extended.zoePH2.battery_time) + "</h4>";
    content += "<h4>pack time: " + String(datalayer_extended.zoePH2.battery_pack_time) + "</h4>";
    content += "<h4>soc min: " + String(datalayer_extended.zoePH2.battery_soc_min) + "</h4>";
    content += "<h4>soc max: " + String(datalayer_extended.zoePH2.battery_soc_max) + "</h4>";
#endif  //RENAULT_ZOE_GEN2_BATTERY

#if !defined(TESLA_BATTERY) && !defined(NISSAN_LEAF_BATTERY) && !defined(BMW_I3_BATTERY) && \
    !defined(BYD_ATTO_3_BATTERY) && !defined(RENAULT_ZOE_GEN2_BATTERY) && !defined(CELLPOWER_BMS)
    content += "No extra information available for this battery type";
#endif

    content += "</div>";

    content += "<script>";
    content += "function goToMainPage() { window.location.href = '/'; }";
    content += "</script>";
    return content;
  }
  return String();
}
