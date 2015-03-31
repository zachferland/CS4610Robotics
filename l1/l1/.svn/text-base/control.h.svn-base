/**
 * Module - Forward and angular control interface
 *
 * Implements a bang-bang controller with a P controller to provide 
 * accurate forward and angular control. Performs incremental odometry to 
 * track the current pose of the robot in its' world. Provides other commands 
 * to determine the current state of the robot. 
 * 
 **/

#if !defined(CONTROL_H) /* protect against multiple inclusion */
#define CONTROL_H

/** for uint8_t etc. **/
 #include <stdint.h>

/**
 * Radius of wheel in mm
 **/
#define WHEEL_RADIUS ((float) 39)

/**
 * Baseline in mm
 **/
#define BASELINE ((float) 205)

/**
 * Fixed forward velocity for bang-bang control
 **/
#define DRIVE_VEL ((float) 120)

/**
 * Fixed angular velocity for bang-bang control
 **/
#define TURN_VEL ((float) 0.9)

/**
 * Module initialization.
 *
 * Sets positions to 0
 **/
void controlInit();

/**
 * Get current forward velocity and angular velocity
 * 
 * v returned here in mm/s forward (negative is reverse) 
 * w returned here in radians/s (ccw is positive)
 **/
void driveGetVW(float *v, float *w); 

/**
 * Sets forward velocity and angular velocity
 * 
 * v - forward velocity in mm/s (negative is reverse)
 * w - angualr velocity in rad/s (ccw is positive)
 **/
void driveSetVW(float v, float w); 

/**
 * Sets forward velocity and turning radius
 *
 * v - forward velocity in mm/s (negative is reverse)
 * l - turning radius in mm
 **/
void driveSetVL(float v, float l); 

/**
 * Implements incremental odometry
 **/
void incOd(); 

/**
 * Gets current pose of robot
 * 
 * x - x coordinate on 2d plane
 * y - y coordinate on 2d plane
 * omega - orientaion of robot
 **/
void driveGetPose(float *x, float *y, float *omega);

/**
 * Resets the pose to world pose (0, 0, 0)
 **/
void driveResetPose();

/**
 * Sets the distance to drive forward
 * 
 * d - distance in mm to drive forward (negative is reverse)
 **/
void driveForward(float d);

/**
 * Implements and bang-bang controller for forward control
 **/
void bangBangForward();

/**
 * Implements a proportional controller for forward control
 **/
void proportionalForward();

/**
 * Sets the amount to change robot orientation
 *
 * t - amount in radians to turn (positive is ccw)
 **/
void driveTurn(float t);

/**
 * Implements and bang-bang controller for turn control
 **/
void bangBangTurn();

/**
 * Implements a proportional controller for turn control
 **/
void proportionalTurn();

/**
 * Calls next command in que if no commands are running or paused
 **/
void runQue();

/**
 * Registers all control related commands
 **/
void controlRegisterCmds();

/**
 * Returns the number of commands in the que
 **/
int8_t driveGetStatus();

/**
 * Pauses que and current command (NOTE - my implementation only pauses que)
 **/
void drivePause();

/**
 * UnPauses que and current command (NOTE - my implementation only unpauses que)
 **/
void driveUnPause();

/**
 * ReInitializes and empties current que
 **/
void driveReInit();

/**
 * Testing command to read out values with HLP_DBG
 **/
void readOut();

#endif /* protect against multiple inclusion */
