/*
  Copyright 2012-2016 Benjamin Vedder benjamin@vedder.se

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef VESC_DATATYPES_H_
#define VESC_DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// CAN commands
typedef enum {
  CAN_PACKET_SET_DUTY = 0,                //0
  CAN_PACKET_SET_CURRENT,                 //1
  CAN_PACKET_SET_CURRENT_BRAKE,           //2
  CAN_PACKET_SET_RPM,                     //3
  CAN_PACKET_SET_POS,                     //4
  CAN_PACKET_FILL_RX_BUFFER,              //5
  CAN_PACKET_FILL_RX_BUFFER_LONG,         //6
  CAN_PACKET_PROCESS_RX_BUFFER,           //7
  CAN_PACKET_PROCESS_SHORT_BUFFER,        //8
  CAN_PACKET_STATUS,                      //9
  CAN_PACKET_SET_CURRENT_REL,             //10
  CAN_PACKET_SET_CURRENT_BRAKE_REL,       //11
  CAN_PACKET_SET_CURRENT_HANDBRAKE,       //12
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,   //13
  CAN_PACKET_STATUS_2,                    //14
  CAN_PACKET_STATUS_3,                    //15
  CAN_PACKET_STATUS_4,                    //16
  CAN_PACKET_PING,                        //17
  CAN_PACKET_PONG,                        //18
  CAN_PACKET_DETECT_APPLY_ALL_FOC,        //19
  CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,    //20
  CAN_PACKET_CONF_CURRENT_LIMITS,         //21
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS,   //22
  CAN_PACKET_CONF_CURRENT_LIMITS_IN,      //23
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,//24
  CAN_PACKET_CONF_FOC_ERPMS,              //25
  CAN_PACKET_CONF_STORE_FOC_ERPMS,        //26
  CAN_PACKET_STATUS_5,                    //27
  CAN_PACKET_POLL_TS5700N8501_STATUS,     //28
  CAN_PACKET_CONF_BATTERY_CUT,            //29
  CAN_PACKET_CONF_STORE_BATTERY_CUT,      //30
  CAN_PACKET_SHUTDOWN,                    //31
  CAN_PACKET_IO_BOARD_ADC_1_TO_4,         //32
  CAN_PACKET_IO_BOARD_ADC_5_TO_8,         //33
  CAN_PACKET_IO_BOARD_ADC_9_TO_12,        //34
  CAN_PACKET_IO_BOARD_DIGITAL_IN,         //35
  CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL, //36
  CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,     //37
  CAN_PACKET_BMS_V_TOT,                   //38
  CAN_PACKET_BMS_I,                       //39
  CAN_PACKET_BMS_AH_WH,                   //40
  CAN_PACKET_BMS_V_CELL,                  //41
  CAN_PACKET_BMS_BAL,                     //42
  CAN_PACKET_BMS_TEMPS,                   //43
  CAN_PACKET_BMS_HUM,                     //44
  CAN_PACKET_BMS_SOC_SOH_TEMP_STAT,       //45
  CAN_PACKET_PSW_STAT,                    //46
  CAN_PACKET_PSW_SWITCH,                  //47
  CAN_PACKET_BMS_HW_DATA_1,               //48
  CAN_PACKET_BMS_HW_DATA_2,               //49
  CAN_PACKET_BMS_HW_DATA_3,               //50
  CAN_PACKET_BMS_HW_DATA_4,               //51
  CAN_PACKET_BMS_HW_DATA_5,               //52
  CAN_PACKET_BMS_AH_WH_CHG_TOTAL,         //53
  CAN_PACKET_BMS_AH_WH_DIS_TOTAL,         //54
  CAN_PACKET_UPDATE_PID_POS_OFFSET,       //55
  CAN_PACKET_POLL_ROTOR_POS,              //56
  CAN_PACKET_NOTIFY_BOOT,                 //57
  CAN_PACKET_STATUS_6,                    //58
  CAN_PACKET_GNSS_TIME,                   //59
  CAN_PACKET_GNSS_LAT,                    //60
  CAN_PACKET_GNSS_LON,                    //61
  CAN_PACKET_GNSS_ALT_SPEED_HDOP,         //62
  CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

// TODO implement other can status messages
typedef struct {
  // CAN STATUS 1
  float rpm;
  float current;
  float duty;
  // CAN STATUS 2
  float amp_hours;
  float amp_hours_charged;
  // CAN STATUS 3
  float watt_hours;
  float watt_hours_charged;
  // CAN STATUS 4
  float temp_fet;
  float temp_motor;
  float current_in;
  float pid_pos_now;
  // CAN STATUS 5
  float v_in;
  int32_t tacho_value;
  // CAN STATUS 6
  float adc_1;
  float adc_2;
  float adc_3;
  float ppm;
  // CAN_PACKET_POLL_ROTOR_POS
  float rotor_pos;
} vesc_node_status;

#ifdef __cplusplus
}
#endif

#endif /* VESC_DATATYPES_H_ */
