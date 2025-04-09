// -------------------- MODE SELECTION --------------------
#define MODE "FOLLOW" // Options: "FOLLOW", "LINE", "ARC"

// -------------------- PID PARAMETERS --------------------
// PID for Speed Control
#define SPEED_PID_KP 1.0
#define SPEED_PID_KI 0.2
#define SPEED_PID_KD 0.05
#define SPEED_PID_MAX_INTEGRAL 20

// PID for Angle Control
#define ANGLE_PID_KP 1.5
#define ANGLE_PID_KI 0.1
#define ANGLE_PID_KD 0.1
#define ANGLE_PID_MAX_INTEGRAL 15

// -------------------- PHYSICAL CONSTANTS --------------------
#define WHEEL_DIAMETER 0.065 // meters
#define WHEEL_DISTANCE 0.15  // meters (distance between wheels)
#define NUM_SLITS 20         // Number of slits on the encoder disk

// -------------------- SPEED AND DISTANCE PARAMETERS --------------------
// Base Speed
#define BASE_START_SPEED 50 // Initial base speed (PWM value)

// Follow Mode
#define FOLLOW_TARGET_DISTANCE 5.0 // meters (desired distance to the car in front)

// Line Mode
#define LINE_TARGET_SPEED 1.0 // meters per second (straight-line target speed)
#define WANTED_SPEED 100      // PWM value for desired speed in line mode

// Arc Mode
#define ARC_TURN_RADIUS 1.0          // meters (desired turning radius)
#define ARC_WANTED_SPEED 0.3         // meters per second (desired speed for arc mode)
#define ARC_WANTED_SPEED_BITS 100    // PWM value for arc speed

// -------------------- CONTROL PARAMETERS --------------------
#define CONTROL_SENSITIVITY 1.5 // Sensitivity multiplier for PID adjustments

// -------------------- TIMING PARAMETERS --------------------
#define CONTROL_INTERVAL 200 // milliseconds (interval for control updates)

// -------------------- DEBUGGING --------------------
#define DEBUG_MODE 1 // Set to 1 to enable debug output, 0 to disable