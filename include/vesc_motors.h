#ifndef vesc_motors_h
#define vesc_motors_h

#include <FlexCAN_T4.h>
#include <map>
#include "vesc_datatypes.h"
#include "buffer.h"

template<CAN_DEV_TABLE can_bus, uint8_t num_motor> class vesc_motors {
  public:
    vesc_motors(uint8_t can_id[num_motor], uint8_t poles[num_motor], float ratio[num_motor]);
    void begin();
    void update();
    
    void can_sniff(const CAN_message_t &msg);
    bool can_send_packet(uint32_t id, uint8_t packet[], int32_t len);
    
    float get_duty(uint8_t id);
    float get_pos(uint8_t id);
    float get_vel(uint8_t id);
    float get_current(uint8_t id);
    
    bool poll_rotor_pos(uint8_t id);
    
    bool set_duty(uint8_t id, float duty);
    bool set_pos(uint8_t id, float pos);
    bool set_vel(uint8_t id, float vel);
    bool set_current(uint8_t id, float current);
    bool set_offset(uint8_t id, float pos_offset);
    bool set_current_brake(uint8_t id, float current);
    
    bool multi_set_duty(float duty[num_motor]);
    bool multi_set_pos(float pos[num_motor]);
    bool multi_set_vel(float vel[num_motor]);
    bool multi_set_current(float current[num_motor]);
    bool multi_set_offset(float pos_offset[num_motor]);
    bool multi_set_current_brake(float current[num_motor]);

    void config_id(uint8_t id[num_motor]);
    void config_poles(uint8_t poles[num_motor]);
    void config_ratio(float ratio[num_motor]);
    
    vesc_node_status* get_vesc_node_status(uint32_t id);
    
  private:
    FlexCAN_T4<can_bus, RX_SIZE_256, TX_SIZE_16> _CAN;
    static vesc_motors<can_bus, num_motor>* _instance;
    static void can_sniff_wrapper(const CAN_message_t &msg);
    
    uint8_t _id[num_motor];
    std::map<uint8_t, uint8_t> _idx;
    vesc_node_status _motor[num_motor];
    uint8_t _poles[num_motor];
    float _ratio[num_motor];
};

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
vesc_motors<can_bus, num_motor>* vesc_motors<can_bus, num_motor>::_instance = nullptr;

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
vesc_motors<can_bus, num_motor>::vesc_motors(uint8_t can_id[num_motor], uint8_t poles[num_motor], float ratio[num_motor]) {
  for (int i = 0; i < num_motor; i++) {
    _id[i] = can_id[i];
    _idx[can_id[i]] = i;
    _poles[i] = poles[i];
    _ratio[i] = ratio[i];
  }
  _instance = this;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::begin() {
  _CAN.begin();
  _CAN.setBaudRate(1000000);
  _CAN.enableFIFO();
  _CAN.enableFIFOInterrupt();
  _CAN.onReceive(can_sniff_wrapper);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::update() {
  _CAN.events();
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::can_sniff(const CAN_message_t &msg) {
  if (msg.flags.extended) {
    uint8_t id = msg.id & 0xFF; //take the lower 8 bits for the ID
    uint8_t idx = _idx[id];
    
    CAN_PACKET_ID cmd = (CAN_PACKET_ID) (msg.id >> 8); // Take the upper bits as the command

    int32_t ind = 0;
    switch (cmd) {
      case CAN_PACKET_STATUS:
        _motor[idx].rpm = (float)buffer_get_int32(msg.buf, &ind);
        _motor[idx].current = (float)buffer_get_int16(msg.buf, &ind) / 1e2;
        _motor[idx].duty = (float)buffer_get_int16(msg.buf, &ind) / 1e3;
        break;
      case CAN_PACKET_STATUS_2:
        _motor[idx].amp_hours = (float)buffer_get_int32(msg.buf, &ind) / 1e4;
        _motor[idx].amp_hours_charged = (float)buffer_get_int32(msg.buf, &ind) / 1e4;
        break;
      case CAN_PACKET_STATUS_3:
        _motor[idx].watt_hours = (float)buffer_get_int32(msg.buf, &ind) / 1e4;
        _motor[idx].watt_hours_charged = (float)buffer_get_int32(msg.buf, &ind) / 1e4;
        break;
      case CAN_PACKET_STATUS_4:
        _motor[idx].temp_fet = (float)buffer_get_int16(msg.buf, &ind) / 1e1;
        _motor[idx].temp_motor = (float)buffer_get_int16(msg.buf, &ind) / 1e1;
        _motor[idx].current_in = (float)buffer_get_int16(msg.buf, &ind) / 1e1;
        _motor[idx].pid_pos_now = (float)buffer_get_int16(msg.buf, &ind) / 50;
        break;
      case CAN_PACKET_STATUS_5:
        _motor[idx].tacho_value = (int32_t)buffer_get_int16(msg.buf, &ind);
        _motor[idx].v_in = (float)buffer_get_int16(msg.buf, &ind) / 1e1;
        break;
      case CAN_PACKET_STATUS_6:
        _motor[idx].adc_1 = buffer_get_float16(msg.buf, 1e3, &ind);
        _motor[idx].adc_2 = buffer_get_float16(msg.buf, 1e3, &ind);
        _motor[idx].adc_3 = buffer_get_float16(msg.buf, 1e3, &ind);
        _motor[idx].ppm = buffer_get_float16(msg.buf, 1e3, &ind);
        break;
      default:
        break;
    }
  }
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::can_sniff_wrapper(const CAN_message_t &msg) {
  _instance->can_sniff(msg);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::can_send_packet(uint32_t id, uint8_t packet[], int32_t len) {
  if (len > 8) {
    len = 8;
  }
  CAN_message_t  msg;
  msg.flags.extended = true;
  msg.id = id;
  msg.len = len;
  memcpy(msg.buf, packet, len);
  //msg.timeout = 0;
  return (bool) _CAN.write(msg);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
float vesc_motors<can_bus, num_motor>::get_duty(uint8_t id) {
  uint8_t idx = _idx[id];
  return _motor[idx].duty;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
float vesc_motors<can_bus, num_motor>::get_pos(uint8_t id) {
  uint8_t idx = _idx[id];
  return _motor[idx].pid_pos_now * _ratio[idx];
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
float vesc_motors<can_bus, num_motor>::get_vel(uint8_t id) {
  uint8_t idx = _idx[id];
  return _motor[idx].rpm * _ratio[idx] / _poles[idx];
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
float vesc_motors<can_bus, num_motor>::get_current(uint8_t id) {
  uint8_t idx = _idx[id];
  return _motor[idx].current;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::poll_rotor_pos(uint8_t id) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  uint8_t idx = _idx[id];
  _motor[idx].rotor_pos = -1;
  return can_send_packet(id | ((uint32_t)CAN_PACKET_POLL_ROTOR_POS << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::set_duty(uint8_t id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::set_pos(uint8_t id, float pos) {
  uint8_t idx = _idx[id];

  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0 / _ratio[idx]), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::set_vel(uint8_t id, float vel) {
  uint8_t idx = _idx[id];

  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(vel * _poles[idx] / _ratio[idx]), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::set_current(uint8_t id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::set_offset(uint8_t id, float pos_offset) {
  uint8_t idx = _idx[id];

  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos_offset * 10000.0 / _ratio[idx]), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_UPDATE_PID_POS_OFFSET << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::set_current_brake(uint8_t id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  return can_send_packet(id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::multi_set_duty(float duty[num_motor]) {
  bool ret = true;
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty[idx] * 100000.0), &send_index);
    ret &= can_send_packet(_id[idx] | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
  }
  return ret;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::multi_set_pos(float pos[num_motor]) {
  bool ret = true;
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos[idx] * 1000000.0 / _ratio[idx]), &send_index);
    ret &= can_send_packet(_id[idx] | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
  }
  return ret;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::multi_set_vel(float vel[num_motor]) {
  bool ret = true;
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(vel[idx] * _poles[idx] / _ratio[idx]), &send_index);
    ret &= can_send_packet(_id[idx] | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
  }
  return ret;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::multi_set_current(float current[num_motor]) {
  bool ret = true;
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current[idx] * 1000.0), &send_index);
    ret &= can_send_packet(_id[idx] | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
  }
  return ret;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::multi_set_offset(float pos_offset[num_motor]) {
  bool ret = true;
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos_offset[idx] * 10000.0 / _ratio[idx]), &send_index);
    ret &= can_send_packet(_id[idx] | ((uint32_t)CAN_PACKET_UPDATE_PID_POS_OFFSET << 8), buffer, send_index);
  }
  return ret;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
bool vesc_motors<can_bus, num_motor>::multi_set_current_brake(float current[num_motor]) {
  bool ret = true;
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current[idx] * 1000.0), &send_index);
    ret &= can_send_packet(_id[idx] | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
  }
  return ret;
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::config_id(uint8_t id[num_motor]) {
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    _id[idx] = id[idx];
  }
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::config_poles(uint8_t poles[num_motor]) {
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    _poles[idx] = poles[idx];
  }
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
void vesc_motors<can_bus, num_motor>::config_ratio(float ratio[num_motor]) {
  for (uint8_t idx = 0; idx < num_motor; idx++) {
    _ratio[idx] = ratio[idx];
  }
}

template<CAN_DEV_TABLE can_bus, uint8_t num_motor>
vesc_node_status* vesc_motors<can_bus, num_motor>::get_vesc_node_status(uint32_t id) {
  uint8_t idx = _idx[id];
  return &_motor[idx];
}
#endif // vesc_motors_h
