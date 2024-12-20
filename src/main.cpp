#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "vesc_motors.h"
#include "commands.h"

#define PRINT_PERIOD 8

#include "motors.h"
#define MOTOR_POLES 14
#define RATIO 1.0

// #define POSITION_ANGLE_DIVISION 1.0
// #define DRUM_DIAMETER 65.0
// #define RATIO POSITION_ANGLE_DIVISION *M_PI *DRUM_DIAMETER / 360.0

uint8_t can_id[NUM_MOTOR] = {40, 83, 13, 116, 76, 108};
uint8_t poles[NUM_MOTOR] = {MOTOR_POLES, MOTOR_POLES, MOTOR_POLES, MOTOR_POLES, MOTOR_POLES, MOTOR_POLES};
float ratio[NUM_MOTOR] = {RATIO, RATIO, RATIO, RATIO, RATIO, RATIO};
float positions[NUM_MOTOR] = {0, 0, 0, 0, 0, 0};

float init_length[NUM_MOTOR] = {525.0, 525.0, 525.0, 525.0, 525.0, 525.0}; // Initial position of the motors

bool commandReceived = false; // Flag to indicate a command was received

vesc_motors<CAN2, NUM_MOTOR> can2_motors(can_id, poles, ratio);

unsigned long time_last = 0;
unsigned long time_start = 0;

float desired_throttle = 0.0;
float desired_rotate = 0.0;

// Initialize the robot
void setup()
{
  can2_motors.begin();
  delay(100);
  can2_motors.update();
  // Offset initialization of robot motor positions
  can2_motors.multi_set_offset(init_length);
  Serial.println("Start");
  time_start = millis();
  time_last = time_start;
}

// Function to navigate the robot
void navigate(float throttle, float rotate)
{
  // Clamp throttle and rotate values to their respective ranges
  throttle = constrain(throttle, -1.0, 1.0);
  rotate = constrain(rotate, -1.0, 1.0);

  // Set the duty for the propulsion motors (0.0 to 1.0)
  float duty_cycle = throttle * 1.0; // Scale to 0.0 to 1.0

  // Handle direction control based on throttle value
  float duty_sign = (throttle >= 0) ? duty_cycle : -duty_cycle; // Reverse if throttle is negative

  for (int i = 0; i < NUM_MOTOR_PROP; i++)
  {
    can2_motors.set_duty(can_id[i], duty_sign);
    Serial.print("Duty: " + String(can_id[i]) + " Value: " + String(duty_sign) + "\n");
  }

  // Get the initial positions of the direction motors for calibration
  float initial_positions[NUM_MOTOR_DIRC];

  for (int i = 3; i < NUM_MOTOR; i++)
  {
    initial_positions[i - 3] = can2_motors.get_pos(can_id[i]);
    Serial.print("Get POS: " + String(can_id[i]) + " Value: " + String(initial_positions[i - 3]) + "\n");
  }

  // Calculate target positions for direction motors
  for (int i = 0; i < NUM_MOTOR_DIRC; i++)
  {
    float target = initial_positions[i] + rotate * 360.0; // Scale rotation to -360 to +360
    can2_motors.set_pos(can_id[i + 3], target);
    Serial.print("PUSH POS: " + String(can_id[i]) + " Value: " + String(target) + "\n");
  }
}

// In loop function, call navigate with desired throttle and rotate values
void loop()
{
  unsigned long time_now = millis();
  can2_motors.update();

  // Check for incoming serial data
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n'); // Read command until newline
    parseCommand(command);
    commandReceived = true; // Set the flag to true when a command is received
  }

  if (time_now - time_last >= 1)
  {
    if (commandReceived)
    {
      navigate(desired_throttle, desired_rotate); // Use updated throttle and rotate

      // Print debug info...
      for (int idx = 0; idx < NUM_MOTOR; idx++)
      {
        positions[idx] = can2_motors.get_pos(can_id[idx]);
        Serial.print("Positions of Motor " + String(idx) + ": " + String(positions[idx]) + "\n");
      }

      commandReceived = false;
    }

    time_last = time_now;
  }
}
