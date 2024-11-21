#ifndef AK_SERIES_DATATYPES_H_
#define AK_SERIES_DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  // CAN commands
  typedef enum
  {
    CAN_PACKET_SET_DUTY = 0,      // Duty cycle mode
    CAN_PACKET_SET_CURRENT,       // Current loop mode
    CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
    CAN_PACKET_SET_RPM,           // Velocity mode
    CAN_PACKET_SET_POS,           // Position mode
    CAN_PACKET_SET_ORIGIN_HERE,   // Set origin mode
    CAN_PACKET_SET_POS_SPD,       // Position velocity loop mode
  } CAN_PACKET_ID;

  // TODO implement other can status messages
  typedef struct
  {
    float pos;
    float spd;
    float cur;
    int8_t temp;
    int8_t error;
  } AK_node_status;

#ifdef __cplusplus
}
#endif

#endif /* AK_SERIES_DATATYPES_H_ */
