// -------------------- MODE SELECTION --------------------
#define MODE "ARC" // Options: "FOLLOW", "LINE", "ARC", "SPEED_TEST"

// -------------------- PID PARAMETERS --------------------
// PID for Speed Control
#define SPEED_PID_KP 0.5
#define SPEED_PID_KI 0.01
#define SPEED_PID_KD 0.1
#define SPEED_PID_MAX_INTEGRAL 10

// PID for Angle Control
#define ANGLE_PID_KP 1.2
#define ANGLE_PID_KI 0.02
#define ANGLE_PID_KD 0.2
#define ANGLE_PID_MAX_INTEGRAL 15

// -------------------- PHYSICAL CONSTANTS --------------------
#define WHEEL_DIAMETER 0.065 // meters
#define WHEEL_DISTANCE 0.15  // meters (distance between wheels)
#define NUM_SLITS 20         // Number of slits on the encoder disk

// -------------------- SPEED AND DISTANCE PARAMETERS --------------------

// Follow Mode
#define FOLLOW_TARGET_DISTANCE 0.5 // meters (desired distance to the car in front)

// Line Mode
#define WANTED_SPEED_BITS_LINE 150 // PWM value for desired speed in line mode

// Arc Mode
#define ARC_TURN_RADIUS 1.0 // meters (desired turning radius)
#define ARC_WANTED_SPEED 0.7 // meters per second (desired speed for arc mode)
#define BASE_PWM_ARC 150 // pwm 0-255 value
#define LEFT_OUTER true

// -------------------- CONTROL PARAMETERS --------------------
#define CONTROL_SENSITIVITY 1.5 // Sensitivity multiplier for PID adjustments
#define CONTROL_SENSITIVITY_ARC 50 // sensitivity multiplier for ARC PID adjustments

// -------------------- TIMING PARAMETERS --------------------
#define CONTROL_INTERVAL 200 // milliseconds (interval for control updates)

// -------------------- DEBUGGING --------------------
#define DEBUG_MODE 1 // Set to 1 to enable debug output, 0 to disable
#define DEBUG_LEVEL 2 // 0: No debug, 1: Basic debug, 2: Detailed debug