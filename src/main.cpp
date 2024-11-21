#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "vesc_motors.h"

#define PRINT_PERIOD 8

#include "waypoints.h"
#define MOTOR_POLES 14
#define POSITION_ANGLE_DIVISION 1.0
#define DRUM_DIAMETER 65.0
#define RATIO POSITION_ANGLE_DIVISION * M_PI * DRUM_DIAMETER / 360.0

uint8_t can_id[NUM_MOTOR] = {1, 2, 3, 4, 5, 6};
uint8_t poles[NUM_MOTOR] = {MOTOR_POLES, MOTOR_POLES};
float ratio[NUM_MOTOR] = {RATIO, RATIO};

vesc_motors<CAN1, NUM_MOTOR> can1_motors(can_id, poles, ratio);

unsigned long time_last = 0;
unsigned long time_start = 0;

void setup() {
  can1_motors.begin();
  delay(100);
  can1_motors.update();
  
  can1_motors.multi_set_offset(init_length);
  Serial.println("Start");
  time_start = millis();
  time_last = time_start;
}

void loop() {
  unsigned long time_now = millis();
  can1_motors.update();
  
  if (time_now - time_last >= 1) {
    current += time_now - time_last;

    // Check if we should process the current waypoint
    if (waypoint < NUM_WAYPOINT - 1 || LOOP) {
        float this_current = (float) current;
        float this_duration = DURATION;

        // Iterate over the motors (0-2 for wheels, 3-5 for direction)
        for (int idx = 0; idx < NUM_MOTOR; idx++) {
            float target = map(
                min(this_current, this_duration),
                0.0,
                this_duration,
                trajectorys[waypoint][idx],
                trajectorys[(waypoint + 1) % NUM_WAYPOINT][idx]
        );
        
        can1_motors.set_pos(can_id[idx], max(0, target));

        if (PRINT_PERIOD && (time_now - time_start) % PRINT_PERIOD == 0) {
          Serial.print("target ");        
          Serial.print(idx);        
          Serial.print(":");
          Serial.print(target);
          Serial.print(",");
          Serial.print("actual ");        
          Serial.print(idx);        
          Serial.print(":");
          Serial.print(can1_motors.get_pos(can_id[idx]));
          Serial.print(idx == NUM_MOTOR - 1 ? "\n" : ",");
        }
      }
      if (this_current >= this_duration) {
        waypoint++;
        waypoint %= NUM_WAYPOINT;
        current = 0;
      }
    }
    time_last = time_now;
  }
}

/*

    
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

*/


