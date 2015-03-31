/**
 * See corresponding header file for module documentation.
 *
 * control.h
 **/

#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "control.h"
#include "que.h"

#include "ohmm/mot.h"
#include "ohmm/cmd.h"
#include "ohmm/task.h"
#include "ohmm/hlp.h"

/**
 * Left index.
 **/
#define L 0

/**
 * Right index.
 **/
#define R 1

 /**
 * Pose X
 **/
#define X 0

/**
 * Pose Y
 **/
#define Y 1

 /**
 * Pose OMEGA
 **/
#define OMEGA 2

 /**
 * Current robot pose (x, y, omega)
 **/
static volatile float r_pose[3];

/**
 * Last update of number of revolutions since initializations of L and R
 **/
static volatile float rev_last[2];

 /**
 * Goal Pose  (x, y omega)
 **/
static volatile float g_pose[3];

 /**
 * Estimated time of command 
 **/
static volatile float e_time;

 /**
 * Time passed of command
 **/
static volatile float p_time;

/**
 * Velocity from last update
 **/
static volatile float v_last;

 /**
 * Error from last update
 **/
static volatile float e_last;

 /**
 * Command active? - 1 = true, 0 = false 
 **/
static volatile int8_t cmd_live;

 /**
 * Command paused? - 1 = true, 0 = false
 **/
static volatile int8_t cmd_paused;

 /**
 * Velocity direction, -1 = (reverse or cw), 1 = (forward or ccw)
 **/
static volatile float direction;

 /**
 * Handle the dgvw command.
 *
 * Returns two floats: v - current forward velocity in mm/sec and w - current 
 * CCW angular velocity in rad/sec. 
 **/
static void dgvwCmdHandler();

 /**
 * Handle the dsvw command.
 *
 * Sets forward velocity and angular velocity
 *
 * Takes float v - forward velodity in mm/sec and float w - angular velocity
 * in rad/s 
 **/
static void dsvwCmdHandler();

 /**
 * Handle the dsvl command.
 *
 * Sets forward velocity and turning radius
 *
 * Takes float v - forward velodity in mm/sec and float l - turning radius
 * in mm
 **/
static void dsvlCmdHandler();

 /**
 * Handle the dgp command.
 *
 * Returns robot pose in form (float x, float y, omega) - x and y as 2d plane 
 * coordinates and omega as the orientation of the robot.
 **/
static void dgpCmdHandler();

 /**
 * Handle the drp command.
 *
 * Resets the robot pose to world post (0, 0, 0)
 **/
static void drpCmdHandler();

 /**
 * Handle the df command.
 *
 * Takes a float f to drive the robot foward f mm, or a negative f to drive 
 * in reverse
 **/
static void dfCmdHandler();

 /**
 * Handle the dt command.
 *
 * Takes a float t to turn the robot t radians. Positive radians results in
 * a ccw turn.
 **/
static void dtCmdHandler();

 /**
 * Handle the dgs command.
 *
 * Returns an int, the number of commands in the que
 **/
static void dgsCmdHandler();

 /**
 * Handle the dp command.
 *
 * Calls drivePause();
 **/
static void dpCmdHandler();

 /**
 * Handle the dup command.
 *
 * Calls driveUnPause();
 **/
static void dupCmdHandler();

 /**
 * Handle the dri command.
 *
 * Reinitializes and empties the que
 **/
static void driCmdHandler();

void controlInit() {
	//initialize global variables
	r_pose[0] = 0;
	r_pose[1] = 0;
	r_pose[2] = 0;
	g_pose[0] = 0;
	g_pose[1] = 0;
	g_pose[2] = 0;
	rev_last[0] = 0;
	rev_last[1] = 0;
	e_time = 0;
	p_time = 0;
	v_last = 0;
	e_last = 0;
	cmd_live = 0;
	cmd_paused = 0;
	direction = 0;

	//Register commands in this module
	controlRegisterCmds();

	//Register incremental odometry task at 20hz (20 times per second)
	taskRegister(incOd, MS_TO_TICKS(50), "Incremental Odometry");

	//Register and run que
	taskRegister(runQue, MS_TO_TICKS(100), "Run Que");
}

void driveGetVW(float *v, float *w) {
	float l, r, c;
	//get left and right speed in rev/sec
    motGetVel(&l, &r);
	//circumference 
    c = 2*M_PI*WHEEL_RADIUS;
    //robot forward velocity in mm/sec (rev/s * circumference)
    *v = c * 0.5 * (l + r);
    *w = (WHEEL_RADIUS / BASELINE) * (r - l) * 2 * M_PI;
}

void driveSetVW(float v, float w) {
  float l, r;
  //calulate l and r wheel velocities from v and w
  r = (2 * v + BASELINE * w) / (2 * WHEEL_RADIUS) / (2*M_PI);
  l = (2 * v - BASELINE * w) / (2 * WHEEL_RADIUS) / (2*M_PI);
  motSetVelCmd(l, r);
}

void driveSetVL(float v, float l) {
  float w = v / l;
  driveSetVW(v, w);
}

void incOd() {
	float l, r;
    motGetPos(&l, &r);
    float diff_l, diff_r;

    //Difference since last update in r and l wheel
    diff_l = (l - rev_last[L]) * 2 * M_PI;
    diff_r = (r - rev_last[R]) * 2 * M_PI;

    //Difference in orientation since last update
    float diff_orientation = (WHEEL_RADIUS * (diff_r - diff_l)) / BASELINE;

    float average_forward = (WHEEL_RADIUS / 2) * (diff_r + diff_l);

    float diff_x = average_forward * cos(diff_orientation + r_pose[OMEGA]);

    float diff_y = average_forward * sin(diff_orientation + r_pose[OMEGA]);

    //Update pose 
    r_pose[X] = r_pose[X] + diff_x;
	r_pose[Y] = r_pose[Y] + diff_y;

	//Radians in circle
	float radians = 2 * M_PI;
	//Handle wrap around
	r_pose[OMEGA] = fmodf((r_pose[OMEGA] + diff_orientation), radians);

	//Set current l and r for next update
	rev_last[L] = l;
	rev_last[R] = r;
}

void driveGetPose(float *x, float *y, float *omega) {
	*x = r_pose[X];
	*y = r_pose[Y];
	*omega = r_pose[OMEGA];
}

void driveResetPose() {
	r_pose[X] = 0;
	r_pose[Y] = 0;
	r_pose[OMEGA] = 0;
}

void driveForward(float d) {
	//Calculate goal pose
	float x = r_pose[X] + (d * cos(r_pose[OMEGA]));
	float y = r_pose[Y] + (d * sin(r_pose[OMEGA]));

	//Set goal pose
	g_pose[X] = x;
	g_pose[Y] = y;
	g_pose[OMEGA] = r_pose[OMEGA];

	if (d >= 0) { 
		driveSetVW(DRIVE_VEL, 0); 
		direction = 1;
	}
	else {
		driveSetVW(( -1 * DRIVE_VEL), 0);
		direction = -1;
	}

	cmd_live = 1;
	taskRegister(bangBangForward, MS_TO_TICKS(50), "bang-bang forward control");
}

void readOut() {
	//Easy acces to various global variables for debuggin
	//HLP_DBG(" ", some_var);
}

void bangBangForward() {
	//Error - difference in current pose and goal pose
	float diff_x = g_pose[X] - r_pose[X];
	float diff_y = g_pose[Y] - r_pose[Y];
	float e = fabs(diff_x) + fabs(diff_y);

	//If within 100mm or goal implement proportional control, turn off bang-bang
	if (e < 100) {
		taskCancel(bangBangForward);
		taskRegister(proportionalForward, MS_TO_TICKS(50), "proportional forward control");
	}

	/**
 	 * Originally implemented a simple bang-bang control based on a time estimation of the 
 	 * command. Was fairly accurate and much simpler than when I added the proportional 
 	 * conroller to the impementation. Proportional control introduced some problems I
 	 * had not considered
 	**/
}

void proportionalForward() {
	float v; //velocity  (or a)
	float k = 0.8; //constant
	float e; //error

	float diff_x = g_pose[X] - r_pose[X];
	float diff_y = g_pose[Y] - r_pose[Y];

	//Distance formula
	e = sqrtf((diff_x * diff_x) + (diff_y * diff_y));

	//If within 20mm and error become larger than last (meaning driving past point)
	if ((e <= 20) && (e > fabs(e_last))) {
		driveSetVW(0, 0);
		taskCancel(proportionalForward);
		e_last = 0;
		v_last = 0;
		queRemove();
		cmd_live = 0;
		return;
	}

	e = e * direction; //e times directoin, changes to negative if in reverse
	v = v_last + k * (e - e_last);
	driveSetVW(v, 0);

	//Set current error and velocity for next update
	e_last = e;
	v_last = v;

    /**
 	 * After thinking about this more, I should have set the drive distance f as the goal
 	 * rather than the pose. Then on each update the error could have been the difference
 	 * between current calculated distance from start point and goal distance. This would
 	 * have given feedback to the p controller whether the robot was before the goal (positive
 	 * error) or after the goal (negative erro). My implementation does not give that feedback
 	 * since caluclating the distance between a robot post (x, y) and goal pose (x, y) always 
 	 * results in a positive. This is what I had difficulty with, it made termination more 
 	 * difficult, as well as implementing reverse driving. (current implementation has these
 	 * issues with termination sometimes and reverse driving)
	 *
	 * Note - I wanted more accuracy than simple stopping at the deadband setting, in this case 
	 * within 20mm each time. So I tried setting termination at the point that the robot just
	 * passes its goal (when current error is greater than last). A better implementation maybe
	 * would have been simply ending once the difference between current and last error was 
	 * below a certain threshold (as e was getting really close to 0). I implementation had
	 * some issues with termination.

	 * Note - Should have added an estimated time to completion to gurantee termination. 
	 * My current implementation does not gurantee termination if for some circumstance 
	 * the robot never came within 20mm of goal.
 	**/
}

void driveTurn(float t) {
	//Wrap around, for example 7.28 radians is equal to orientation of 1 radian
	float radians = 2 * M_PI;
	g_pose[OMEGA] = fmodf((r_pose[OMEGA] + t), radians);

	if (t >= 0) { 
		driveSetVW(0, TURN_VEL);
		direction = 1;
	}
	else {
		driveSetVW(0, ( -1 * TURN_VEL));
		direction = -1;
	}

	cmd_live = 1;
	taskRegister(bangBangTurn, MS_TO_TICKS(50), "bang-bang turn control");
}

void bangBangTurn() {
	float e = fabs(g_pose[OMEGA] - r_pose[OMEGA]);

	//When error is less thand 0.7, turn off bang-bang and turn on proportional control
	if (e < 0.7) {
		taskCancel(bangBangTurn);
		taskRegister(proportionalTurn, MS_TO_TICKS(50), "proportional turn control");
	}

	/**
 	 * Same note as in bangBangForward
 	**/
}

void proportionalTurn() {
	float v; //velocity  (or a)
	float k = 1; //constant
	float e; //error - orientation difference

	e = fabs(g_pose[OMEGA] - r_pose[OMEGA]);
	
	//If within 20mm and error become larger than last (meaning driving past point)
	if ((e <= 0.2) && (e > fabs(e_last))) {
		driveSetVW(0, 0);
		taskCancel(proportionalTurn);
		e_last = 0;
		v_last = 0;
		queRemove();
		cmd_live = 0;
		return;
	}

	e = e * direction; 
	v = v_last + k * (e - e_last);
	driveSetVW(0, v);

	e_last = e;
	v_last = v;

	/**
 	 * Same note as in proportionalForward
 	**/
}

void runQue() {
	if (cmd_paused != 1 && cmd_live != 1) queNext();
}

int8_t driveGetStatus() {
	return queCount() + cmd_live;
}

void drivePause() {
	cmd_paused = 1;
}

void driveUnPause() {
	cmd_paused = 0;
}

void driveReInit(){
	queReInit();
	//NOTE should have stopped current, but does not
}

void dgvwCmdHandler() {
  float v, w;
  driveGetVW(&v, &w);
  cmdFmtFloat(&v);
  cmdFmtFloat(&w);
}

void dsvwCmdHandler() {
  float v, w;
  cmdParseFloat(&v);
  cmdParseFloat(&w);
  driveSetVW(v, w);
}

void dsvlCmdHandler() {
  float v, l;
  cmdParseFloat(&v);
  cmdParseFloat(&l);
  driveSetVL(v, l);
}

void dgpCmdHandler() {
	float x, y, omega;
	driveGetPose(&x, &y, &omega);
  	cmdFmtFloat(&x);
  	cmdFmtFloat(&y);
  	cmdFmtFloat(&omega);
}

void drpCmdHandler() {
	driveResetPose();
}

void dfCmdHandler() {
	float d;
  	cmdParseFloat(&d);
  	queAdd(driveForward, d);
}

void dtCmdHandler() {
	float t;
  	cmdParseFloat(&t);
  	queAdd(driveTurn, t);
}

void dgsCmdHandler() {
	int8_t i = driveGetStatus();
  	cmdFmtInt(&i, -4);
}

void dpCmdHandler() {
	drivePause();
}

void dupCmdHandler() {
	driveUnPause();
}

void driCmdHandler() {
	driveReInit();
}

void roCmdHandler() {
	readOut();
}

void controlRegisterCmds() {

  cmdRegister_P(110, PSTR("dgvw"), PSTR("get v in mm/s and w in rad/s"),
                dgvwCmdHandler, 0, 0); 
  cmdRegister_P(111, PSTR("dsvw"), PSTR("set v in mm/s and w in rad/s"),
                dsvwCmdHandler, 2, 8); 
  cmdRegister_P(121, PSTR("dsvl"), PSTR("set v in mm/s and l turning radius in mm"),
                dsvlCmdHandler, 2, 8); 
  cmdRegister_P(112, PSTR("dgp"), PSTR("Get robot pose (x, y, omega)"),
                dgpCmdHandler, 0, 0); 
  cmdRegister_P(113, PSTR("drp"), PSTR("Reset robot pose"),
                drpCmdHandler, 0, 0); 
  cmdRegister_P(114, PSTR("df"), PSTR("Drive forward d mmm"),
                dfCmdHandler, 1, 4); 
  cmdRegister_P(115, PSTR("dt"), PSTR("Drive turn t radians"),
                dtCmdHandler, 1, 4); 
  cmdRegister_P(116, PSTR("dgs"), PSTR("Get number of commands in que"),
                dgsCmdHandler, 0, 0); 
  cmdRegister_P(117, PSTR("dp"), PSTR("Pause drive"),
                dpCmdHandler, 0, 0); 
  cmdRegister_P(118, PSTR("dup"), PSTR("Unpause drive"),
                dupCmdHandler, 0, 0); 
  cmdRegister_P(119, PSTR("dri"), PSTR("ReInitialize que"),
                dupCmdHandler, 0, 0); 
  cmdRegister_P(120, PSTR("ro"), PSTR("read out"),
                roCmdHandler, 0, 0); 
}