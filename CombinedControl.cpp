#include "CombinedControl.h"


/* ======================================================================
	Initializes the control object to control both the motor and the
	joystick.
 ====================================================================== */

CombinedControl :: CombinedControl() {

}

void CombinedControl :: begin() {

	// byte csPin, byte enablePin, int ID
	motor.set(CSPIN, ENABLEPIN, 1);
	// double yrange, double ythreshold, int ypin, double xrange, double xthreshold, int xpin
	joystick.set(200000.0, 0.005 * 200000.0, YAXIS, 1, 0.005 * 1, XAXIS);

	_seekStep = 0;
	_stepResolution = 256;
	_lastVel = 0;
	_lastRead = 0;
	_resolutionNum = 1;

	// Initializing the motor objects and start it at home position
	joystick.begin();
	motor.begin();

	// Print out motor data to confirm proper results
	Serial.print(motor.getMotorID());
	Serial.print(F(" : Motor Data: "));
	motor.getMotorData();
	Serial.flush();
}

//====================================================================================
//====================== JOYSTCIK FUNCTIONS ==========================================
//====================================================================================

/* ======================================================================
	Function stops the joystick from controlling the motor and stops the 
	motor from running.
 ====================================================================== */

void CombinedControl :: disableJoystick() {
	motor.stop();
}

/* ======================================================================
	Function allows joystick to take over speed and direction controls.
		Up 		-> Increase speed, forward direction
		Down 	-> Decrease speed, backward direction
	Left and right are not currently configured.
 ====================================================================== */

void CombinedControl :: enableJoystick() {

	if (CombinedControl :: _timer(_lastRead)) {
		_lastRead = millis();

		double velocity = joystick.yAxisControl();

		#ifdef MOTOR_DEBUG
			Serial.print("velocity: ");
			Serial.println(velocity);
		#endif

		// check the direction of the velocity and past velocity
		if ( (_lastVel >= 0) ^ (velocity < 0) ) {
			// if it exceeds a certain range, update the driving
			if( (abs(velocity) > abs(_lastVel) * 1.10) || (abs(velocity) < abs(_lastVel) * 0.90) ) {
				_lastVel = velocity;
				CombinedControl :: _setJS(velocity);
			}
		}

		// always update motor if velocity and last velocity are in different directions
		else {
			_lastVel = velocity;
			CombinedControl :: _setJS(velocity);
		}
	}
}

/* ======================================================================
 	Sets the direction and speed of the motor as read from the joystick.
====================================================================== */

void CombinedControl :: _setJS(double velocity) {

	if ( velocity < 0 ) {
		motor.constReverse(abs(velocity));
	}

	else {
		motor.constForward(velocity);
	}
}

/* ======================================================================
 	Simple non-blocking timer to limit the amount of updates for reading
 	values from the joystick.
====================================================================== */

bool CombinedControl :: _timer(unsigned long lastReadTime) {
	bool done = false;
	unsigned long now = millis();
	if(now - lastReadTime > 200 ) {
		done = true;
	}
	return done;
}

//====================================================================================
//====================== MOVEMENT FUNCTIONS ==========================================
//====================================================================================

/* ======================================================================
	Function sends commands to the motor to move it to any state. Going to 
	position '0' will send it back to the home state.
 ====================================================================== */

void CombinedControl :: goPos(unsigned long position) {
	motor.goPos(position);
}

/* ======================================================================
	Function sends commands to the motor to return it to the "home" state.
	Function checks homing device in order and will not complete until all
	steps have been completed successfully.
 ====================================================================== */

void CombinedControl :: setHome() {
	bool setHome = false;
	while (!setHome) {
		setHome = motor.setHome();
	}
}

/* ======================================================================
	Function sends commands to rotate the motor the number of half-steps
	specified in the forward direction. Returns true if the commands are 
	successfully sent.
 ====================================================================== */

void CombinedControl :: forward(unsigned long stepsForward, unsigned long velocity) {
	motor.forward(stepsForward, velocity);
}

/* ======================================================================
	Function sends commands to rotate the motor the number of half-steps
	specified in the reverse direction. Returns true if the commands are 
	successfully sent.
 ====================================================================== */

void CombinedControl :: reverse(unsigned long stepsBackward, unsigned long velocity) {
	motor.reverse(stepsBackward, velocity);
}

/* ======================================================================
	Function sends commands to rotate the motor continuously forwards at a
	constant velocity
 ====================================================================== */

void CombinedControl :: constForward(unsigned long velocity) {
	motor.constForward(velocity);
}

/* ======================================================================
	Function sends commands to rotate the motor continuously backwards at a
	constant velocity
 ====================================================================== */

void CombinedControl :: constReverse(unsigned long velocity) {
	motor.constReverse(velocity);
}

/* ======================================================================
	Function sends commands to stop the motor when it is in a constant reverse
	or in a constant forward movement.
 ====================================================================== */

void CombinedControl :: stop() {
	motor.stop();
}

/* ======================================================================
	Seeks a stop event such that a button on the left or right is pressed
	to indicate the stopping of the motor.
 ====================================================================== */

bool CombinedControl :: seek(bool goForward) {

	bool done = false;

	switch(_seekStep) {

		// Set the direction of seeking
		case(0): {
			if (goForward == true) {
				motor.constForward(STANDVELOCITY / _resolutionNum);
				_seekStep = 1;
			}
			else {
				motor.constReverse(STANDVELOCITY / _resolutionNum);
				_seekStep = 2;
			}
		} break;

		// check if right button (goForward = true) is pressed
		case(1): {
			motor.buttonStatus();
			if (motor.forwardSwitch == true) {
				_seekStep = 3;
			}
		} break;

		// check if left button (goForward = false) is pressed
		case(2): {
			motor.buttonStatus();
			if (motor.backwardSwitch == true) {
				_seekStep = 3;
			}
		} break;

		// stop the motor and finish seeking
		default: {
			motor.stop();
			_seekStep = 0;
			done = true;
		}
	}
	return done;
}

//====================================================================================
//==================== INFORMATION FUNCTIONS =========================================
//====================================================================================

/* ======================================================================
	Gets the current status of the motor and sends it back as an integer
	array of 1's and 0's.
 ====================================================================== */

void CombinedControl :: status(int * statusBits) {

	motor.readStatus();

	statusBits[0] = motor.status_sg2;
	statusBits[1] = motor.status_sg2_event;
	statusBits[2] = motor.status_standstill;

	statusBits[3] = motor.status_velocity_reached;
	statusBits[4] = motor.status_position_reached;
	statusBits[5] = motor.status_position_reached_event;

	statusBits[6] = motor.status_stop_l;
	statusBits[7] = motor.status_stop_r;
	statusBits[8] = motor.status_stop_l_event;
	statusBits[9] = motor.status_stop_r_event;
	statusBits[10] = motor.status_latch_l;
	statusBits[11] = motor.status_latch_r;

	statusBits[12] = motor.status_openLoad_A;
	statusBits[13] = motor.status_openLoad_B;
	statusBits[14] = motor.status_shortToGround_A;
	statusBits[15] = motor.status_shortToGround_B;

	statusBits[16] = motor.status_overtemperatureWarning;
	statusBits[17] = motor.status_overtemperatureShutdown;

	statusBits[18] = motor.status_isReverse;
	statusBits[19] = motor.status_resetDetected;
	statusBits[20] = motor.status_driverError;
	statusBits[21] = motor.status_underVoltage;

	statusBits[22] = motor.getIsForward();
	statusBits[23] = motor.getIsPositionMode();
	statusBits[24] = motor.getIsHomed();
}

/* ======================================================================
	Gets the current status of the drive status register, looking at only the
	stall guard bits and sends it back as an integer array of 1's and 0's.
 ====================================================================== */

unsigned long  CombinedControl :: sgStatus() {
	motor.sgStatus();
	return motor.sgStatusBits;
}

/* ======================================================================
	Checks and confirms if the motor comes to a standstill
 ====================================================================== */

bool CombinedControl :: standstill() {
	return motor.status_standstill;
}

/* ======================================================================
	Returns the actual position of the motor.
 ====================================================================== */

double CombinedControl :: getXactual() {
	double xPos = motor.getXactual();
	if (xPos > 2147483648.0) {
		Serial.println(xPos);
		xPos -= 4294967296.0;
		Serial.println(xPos);
	}
	return (xPos);
}

/* ======================================================================
	Gets the maximum velocity to the given value
 ====================================================================== */

unsigned long CombinedControl :: getVelocity() {
	return motor.getVelocity();
}

/* ======================================================================
	Gets the acceleration from maximum velocity to the stop velocity
	to the given value.
 ====================================================================== */

unsigned long CombinedControl :: getAcceleration() {
	return motor.getAcceleration();
}

/* ======================================================================
	Gets the deceleration from maximum velocity to the stop velocity
	to the given value.
 ====================================================================== */

unsigned long CombinedControl :: getDeceleration() {
	return motor.getDeceleration();
}

/* ======================================================================
	Gets the deceleration from maximum velocity to the stop velocity
	to the given value.
 ====================================================================== */

unsigned long CombinedControl :: getPower() {
	return motor.getPowerLevel();
}

//====================================================================================
//====================== SETTER FUNCTIONS ============================================
//====================================================================================

/* ======================================================================
	Changes the maximum velocity to the given value
 ====================================================================== */

void CombinedControl :: setVelocity(unsigned long velocity) {
	motor.setVelocity(velocity);
}

/* ======================================================================
	Changes the acceleration from maximum velocity to the stop velocity
	to the given value.
 ====================================================================== */

void CombinedControl :: setAcceleration(unsigned long acceleration) {
	motor.setAcceleration(acceleration);
}

/* ======================================================================
	Changes the deceleration from maximum velocity to the stop velocity
	to the given value.
 ====================================================================== */

void CombinedControl :: setDeceleration(unsigned long deceleration) {
	motor.setDeceleration(deceleration);
}

/* ======================================================================
	Changes the deceleration from maximum velocity to the stop velocity
	to the given value.
 ====================================================================== */

void CombinedControl :: setPower(unsigned long holdPower, unsigned long runPower) {
	motor.setPowerLevel(holdPower, runPower);
}

/* ======================================================================
	Changes xtarget (where the motor will go, this starts a motion if xtarget
	is not the same as xactual in the positioning mode ie: ramp mode = 0) value 
	to the given position.
 ====================================================================== */

void CombinedControl :: setXtarget(unsigned long position) {
	motor.setXtarget(position);
}

/* ======================================================================
	Sets the CHOPCONF register to set the resolution of the motor. The
	default value is 256.
 ====================================================================== */

void CombinedControl :: setResolution(int resolution) {
	motor.setChopConf(resolution);
	_resolutionNum = motor.getResolution();
}

/* ======================================================================
	Changes the position of the motor without moving the motor by resetting
	the current position to the specified position.
 ====================================================================== */

void CombinedControl :: changePosNoMove(unsigned long position) {
	motor.stop();

	motor.setRampMode(1);

	motor.setXactual(position);
	motor.setXtarget(position);
}

/* ======================================================================
	Sets the direction of the switches and the motor relative to each other.
	When the values are set (shown below) the given value is the switch set forward
	and the direction set forward
	DIREC. MODE 	F_SWITCH	B_SWITCH	FOR_MOTOR	BACK_MOTOR
		1			right		left 		cw 			ccw
		2			right 		left 		ccw 		cw
		3			left 		right 		cw 			ccw
		4			left 		right 		ccw 		cw
====================================================================== */

void CombinedControl :: setDirections(bool forwardDirection, bool forwardSwitch) {
	motor.swapDirection(!forwardDirection, !forwardSwitch);
}

/* ======================================================================
	Sets the active state of the left and right switches. Setting to 1 
	(true) is active low and setting to 0 is active high.
====================================================================== */

void CombinedControl :: switchActiveEnable(bool fw, bool bw) {
	motor.switchActiveEnable(fw, bw);
}







