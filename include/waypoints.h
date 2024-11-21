#define NUM_MOTOR 6 // Total 3 motors
#define NUM_MOTOR_PROP 3 // 3 motors for propulsion
#define NUM_MOTOR_DIRC 3 // 3 motors for direction
#define NUM_WAYPOINT 9000
#define DURATION 1
#define LOOP true

unsigned long waypoint = 0;
unsigned long current = 0;
float init_length[NUM_MOTOR] = {525.0, 525.0, 525.0, 525.0, 525.0, 525.0}; 

// 2 groups: one for vertical movement (propulsion) and another for horizontal movement (direction control).

float trajectorys[NUM_WAYPOINT][NUM_MOTOR] = {
    {514.5, 514.5, 514.5, 0.0, 0.0, 0.0}, // Initial positions for wheel motors and direction motors
    {515.0, 515.0, 515.0, 10.0, 10.0, 10.0}, // Example for the next waypoint
};


float propulsion[NUM_WAYPOINT][NUM_MOTOR_PROP] = {
    {514.5, 514.5, 514.5}, // Initial positions for wheel motors
    {515.0, 515.0, 515.0}, // Example for the next waypoint
};

float direction[NUM_WAYPOINT][NUM_MOTOR_DIRC] = {
    {0.0, 0.0, 0.0}, // Initial positions for direction motors
    {10.0, 10.0, 10.0}, // Example for the next waypoint
};
