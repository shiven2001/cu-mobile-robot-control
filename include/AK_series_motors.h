#ifndef AK_series_motors_h
#define AK_series_motors_h

#include <FlexCAN_T4.h>
#include <map>
#include "AK_series_datatypes.h"
#include "buffer.h"

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
class AK_series_motors
{
public:
  AK_series_motors(uint8_t can_id[num_motor], uint8_t poles[num_motor], float ratio[num_motor]);
  void begin();
  void update();

  void can_sniff(const CAN_message_t &msg);

  bool can_send_packet(uint32_t id, uint8_t packet[], int32_t len);
  float get_duty(uint8_t id);
  float get_pos(uint8_t id);
  float get_vel(uint8_t id);
  float get_current(uint8_t id);
  bool set_duty(uint8_t id, float duty);
  bool set_pos(uint8_t id, float pos);
  bool set_vel(uint8_t id, float vel);
  bool set_current(uint8_t id, float current);
  bool set_origin(uint8_t id, uint8_t set_origin_mode);
  bool set_current_brake(uint8_t id, float current);
  bool set_pos_spd(uint8_t id, float pos, int16_t spd, int16_t RPA);

  void config_id(uint8_t id[num_motor]);
  void config_poles(uint8_t poles[num_motor]);
  void config_ratio(float ratio[num_motor]);

  AK_node_status *get_node_status(uint32_t id);

private:
  FlexCAN_T4<can_bus, RX_SIZE_256, TX_SIZE_16> _CAN;
  static AK_series_motors<can_bus, num_motor> *_instance;
  static void can_sniff_wrapper(const CAN_message_t &msg);

  uint8_t _id[num_motor];
  std::map<uint8_t, uint8_t> _idx;
  AK_node_status _motor[num_motor];
  uint8_t _poles[num_motor];
  float _ratio[num_motor];
};

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
AK_series_motors<can_bus, num_motor> *AK_series_motors<can_bus, num_motor>::_instance = nullptr;

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
AK_series_motors<can_bus, num_motor>::AK_series_motors(uint8_t can_id[num_motor], uint8_t poles[num_motor], float ratio[num_motor])
{
  for (int i = 0; i < num_motor; i++)
  {
    _id[i] = can_id[i];
    _idx[can_id[i]] = i;
    _poles[i] = poles[i];
    _ratio[i] = ratio[i];
  }
  _instance = this;
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::begin()
{
  _CAN.begin();
  _CAN.setBaudRate(1000000);
  _CAN.enableFIFO();
  _CAN.enableFIFOInterrupt();
  _CAN.onReceive(can_sniff_wrapper);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::update()
{
  _CAN.events();
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::can_sniff(const CAN_message_t &msg)
{
  if (msg.flags.extended)
  {
    uint8_t id = msg.id & 0xFF; // take the lower 8 bits for the ID
    uint8_t idx = _idx[id];

    CAN_PACKET_ID cmd = (CAN_PACKET_ID)(msg.id >> 8); // Take the upper bits as the command

    int32_t ind = 0;
    int16_t pos_int = buffer_get_int16(msg.buf, &ind);
    int16_t spd_int = buffer_get_int16(msg.buf, &ind);
    int16_t cur_int = buffer_get_int16(msg.buf, &ind);
    _motor[idx].pos = (float)(pos_int * 0.1f);  // motor position
    _motor[idx].spd = (float)(spd_int * 10.0f); // motor speed
    _motor[idx].cur = (float)(cur_int * 0.01f); // motor current
    _motor[idx].temp = (int8_t)msg.buf[6];      // motor temperature
    _motor[idx].error = (int8_t)msg.buf[7];     // motor error mode
  }
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::can_sniff_wrapper(const CAN_message_t &msg)
{
  _instance->can_sniff(msg);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::can_send_packet(uint32_t id, uint8_t packet[], int32_t len)
{
  if (len > 8)
  {
    len = 8;
  }
  CAN_message_t msg;
  msg.flags.extended = true;
  msg.id = id;
  msg.len = len;
  memcpy(msg.buf, packet, len);
  // msg.timeout = 0;
  return (bool)_CAN.write(msg);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
float AK_series_motors<can_bus, num_motor>::get_duty(uint8_t id)
{
  uint8_t idx = _idx[id];
  return _motor[idx].duty;
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
float AK_series_motors<can_bus, num_motor>::get_pos(uint8_t id)
{
  uint8_t idx = _idx[id];
  return _motor[idx].pid_pos_now * _ratio[idx];
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
float AK_series_motors<can_bus, num_motor>::get_vel(uint8_t id)
{
  uint8_t idx = _idx[id];
  return _motor[idx].rpm * _ratio[idx] / _poles[idx];
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
float AK_series_motors<can_bus, num_motor>::get_current(uint8_t id)
{
  uint8_t idx = _idx[id];
  return _motor[idx].current;
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_duty(uint8_t id, float duty)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_pos(uint8_t id, float pos)
{
  uint8_t idx = _idx[id];

  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0 / _ratio[idx]), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_vel(uint8_t id, float vel)
{
  uint8_t idx = _idx[id];

  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(vel * _poles[idx] / _ratio[idx]), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_current(uint8_t id, float current)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_origin(uint8_t id, uint8_t set_origin_mode)
{
  uint8_t idx = _idx[id];

  int32_t send_index = 0;
  uint8_t buffer[4];
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_current_brake(uint8_t id, float current)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool AK_series_motors<can_bus, num_motor>::set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA)
{
  int32_t send_index = 0;
  int16_t send_index1 = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer, spd, &send_index1);
  buffer_append_int16(buffer, RPA, &send_index1);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::config_id(uint8_t id[num_motor])
{
  for (uint8_t idx = 0; idx < num_motor; idx++)
  {
    _id[idx] = id[idx];
  }
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::config_poles(uint8_t poles[num_motor])
{
  for (uint8_t idx = 0; idx < num_motor; idx++)
  {
    _poles[idx] = poles[idx];
  }
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
void AK_series_motors<can_bus, num_motor>::config_ratio(float ratio[num_motor])
{
  for (uint8_t idx = 0; idx < num_motor; idx++)
  {
    _ratio[idx] = ratio[idx];
  }
}

template <CAN_DEV_TABLE can_bus, uint8_t num_motor>
AK_node_status *AK_series_motors<can_bus, num_motor>::get_node_status(uint32_t id)
{
  uint8_t idx = _idx[id];
  return &_motor[idx];
}
#endif // AK_series_motors_h
