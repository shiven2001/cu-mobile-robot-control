#define NUM_MOTOR 6      // The rpbot has total of 6 motors
#define NUM_MOTOR_PROP 3 // 3 motors for propulsion
#define NUM_MOTOR_DIRC 3 // 3 motors for direction
#define NUM_WAYPOINT 9000
#define DURATION 1
#define LOOP true

float init_length[NUM_MOTOR] = {525.0, 525.0, 525.0, 525.0, 525.0, 525.0}; // Initial position of the motors

// 2 groups: one for vertical movement (propulsion) and another for horizontal movement (direction control).
// Refer to main.cpp for the float propulsion and direction control.