#define NUM_MOTOR 6      // The robot has total of 6 motors
#define NUM_MOTOR_PROP 3 // 3 motors for propulsion
#define NUM_MOTOR_DIRC 3 // 3 motors for direction
#define NUM_WAYPOINT 9000
#define DURATION 1
#define LOOP true

extern float init_length[NUM_MOTOR];

extern float desired_throttle;
extern float desired_rotate;

// 2 groups: one for vertical movement (propulsion) and another for horizontal movement (direction control).
// Refer to main.cpp for the float propulsion and direction control.