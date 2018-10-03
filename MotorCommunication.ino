#include "CmdMessenger.h"
#include "Definitions.h"
#include "Enum.h"
#include "debugutils.h"
#include "CombinedControl.h"
#include "Flags.h"

#define DEBUG 1

// Arduino Information
bool 			analogRefInternal 	= false;
int 			ADCResolution 		= 10; 			// Arduino Mega is 10-bit resolution
float 			ADCRefVolt 			= 5; 			// Arduino Mega has a max input voltage of 5V

// Timing bit for loop
int b = 0;

// Build extra objects
CmdMessenger cmdMessenger = CmdMessenger(Serial);
String outputStr;
CombinedControl control;

int status[25];


// =============== Arduino Functions and Calls ===============


void setup() {

	// =============== Setup Motor Control ===============

	// turn on general pins
	pinMode(CLOCKPIN,OUTPUT);
	pinMode(CSPIN, OUTPUT);
	pinMode(ENABLEPIN, OUTPUT);
	pinMode(STOP, OUTPUT);

	//set up Timer1
  	TCCR1A = bit (COM1A0);                //toggle OC1A on Compare Match
  	TCCR1B = bit (WGM12) | bit (CS10);    //CTC, no prescaling
  	OCR1A = 0;                            //output every cycle

	// initialize the SPI interface with TMC5130 settings
	SPI.setBitOrder(MSBFIRST);
  	SPI.setClockDivider(SPI_CLOCK_DIV8);
  	SPI.setDataMode(SPI_MODE3);
  	SPI.begin();

  	// =============== Setup Board Configuration and Sensors ===============

	Serial.begin(57600);
	Serial.println("starting");

	// Macro for Arduino AVR Boards (setting voltages and bit resolution)
	// Configures the reference voltage with built-in 2.56V reference (Arduino Mega only)
	#ifdef __AVR_ATmega2560__

		analogRefInternal = false;
		ADCRefVolt = 5; 								
		analogReference(DEFAULT);
		
		if(DEBUG) {
			DEBUG_PRINT("INFO: Analog Reference 5V Default");
		}
	#endif

	// Macro for Arduino SAMD Boards (setting voltages and bit resolution)
	#ifdef __SAM3X8E__ 
		ADCResolution = 12;
		ADCRefVolt = 3.3;
		analogReadResolution(ADCResolution); //changes the default resolution from 10 - 12 bits
		
		for ( int i = 0; i < numSensors; i++ ) {
			sensorList[i] -> setSaturatedVal(3690);
			sensorList[i] -> setMinimumVal(250);
		}
	#endif

	// =============== Initialize Objects ===============

	// Initialize the motor and Sensor objects
	control.begin();

	outputStr.reserve(128);
	cmdMessenger.printLfCr();
	attachCommandCallbacks();

	pinMode(12, OUTPUT);
}

void loop() {

	// 20 us
	cmdMessenger.feedinSerialData();

	// 100 us
	if ( motorFlags.isJSEnable ) {
		control.enableJoystick();
	}

	// 12 us
	if ( motorFlags.isSeeking ) {
		motorFlags.isSeeking = !control.seek(motorFlags.direction);
	}

	// 50 us
	if ( motorFlags.isPositioning ) {
		motorFlags.isPositioning = !control.standstill();
	}
}


// =============== Command Callbacks ===============


void attachCommandCallbacks() {
  cmdMessenger.attach(OnUnknownCommand);                                	//Reply: e,

  cmdMessenger.attach(REQUEST_MOTOR_STATUS, 	onRequestMotorStatus);		//Reply: m,
  cmdMessenger.attach(REQUEST_SG_STATUS,		onRequestStallStatus);		//Reply: g,
  cmdMessenger.attach(REQUEST_POS_NO_MOVE,		onRequestSetPosNoMove);		//Reply: d,
 
  cmdMessenger.attach(GET_ADCBITS,              onGetADCBits);          	//Reply: A,
  cmdMessenger.attach(GET_ADCREFVOLT,           onGetADCRefVolt);       	//Reply: V,
  cmdMessenger.attach(GET_XACTUAL, 				onGetXactual);				//Reply: x,
  cmdMessenger.attach(GET_VELOCITY,				onGetVelocity);				//Reply: v,
  cmdMessenger.attach(GET_ACCELERATION, 		onGetAcceleration);			//Reply: a,
  cmdMessenger.attach(GET_DECELERATION, 		onGetDeceleration);			//Reply: d,
  cmdMessenger.attach(GET_POWER,				onGetPower);				//Reply: P,

  cmdMessenger.attach(SET_CONST_FW, 			onConstantForward);			//Reply: S,1;
  cmdMessenger.attach(SET_CONST_BW,				onConstantBackward);		//Reply: S,1;
  cmdMessenger.attach(SET_MOVE_POS,				onMovePosition);			//Reply: S,1;
  cmdMessenger.attach(SET_MOVE_FW,				onMoveForward);				//Reply: S,1;
  cmdMessenger.attach(SET_MOVE_BW,				onMoveBackward);			//Reply: S,1;
  cmdMessenger.attach(SET_VELOCITY,				onVelocity);				//Reply: S,1;
  cmdMessenger.attach(SET_ACCELERATION, 		onAcceleration);			//Reply: S,1;
  cmdMessenger.attach(SET_DECELERATION, 		onDeceleration);			//Reply: S,1;
  cmdMessenger.attach(SET_POWER,				onPower);					//Reply: S,1;
  cmdMessenger.attach(SET_DIRECTION,			onDirection);				//Reply: S,0;

  cmdMessenger.attach(JS_ENABLE,				onJSEnable);				//Reply: S,1;
  cmdMessenger.attach(JS_DISABLE,				onJSDisable);				//Reply: S,1;
  cmdMessenger.attach(MOTOR_STOP,				onMotorStop);				//Reply: S,1;
  cmdMessenger.attach(MOTOR_HOME, 				onMotorHome);				//Reply: S,1;
  cmdMessenger.attach(SEEK,						onSeek);					//Reply: S,1;
  cmdMessenger.attach(RESOLUTION, 				onResolution);				//Reply: S,1;
  cmdMessenger.attach(ACTIVESETTINGS, 			onActiveSettings);			//Reply: S,1;
  cmdMessenger.attach(PCPING,                   onPing);               		//Reply: p,PONG;
}

// ADD IF SWITCHES ARE HIT DURING GO TO SPECIFIC POSITION, THEN STOP RATHER THAN CONTINUEING ON



// =============== Additional Helper Functions ===============

bool hasExpired( unsigned long &prevTime, unsigned long interval) {
	if( millis() - prevTime > interval ) {
		prevTime = millis();
		return true;
	}
	return false;
}

void onSuccess() {
	outputStr.remove(0);
	outputStr.concat(F("S,1;"));
	Serial.println(outputStr);
}

void onFail() {
	outputStr.remove(0);
	outputStr.concat(F("S,0;"));
	Serial.println(outputStr);
}

bool _checkFlags() {
	bool done = false;
	_checkJS();
	if ( !motorFlags.isSeeking && !motorFlags.isPositioning) {
		done = true;
	}
	return done;
}

void _checkJS() {
	if (motorFlags.isJSEnable) {
		onJSDisable();
	}
}

void _binaryDisplay(unsigned long status) {
	unsigned long statusBit = 0;

	for (int i = 9; i > -1; i--) {
		statusBit = status >> i;
		statusBit = statusBit & 0x00000001;
		if (statusBit == 0) {
			outputStr.concat(F("0"));
		}
		else {
			outputStr.concat(F("1"));
		}
	}
}


// =============== Callback Functions ===============


// Format : outputStr = "e, unknown command;"
void OnUnknownCommand() {
	outputStr.remove(0);
	outputStr.concat(F("e, unknown command;"));
	Serial.println(outputStr);
}

// Format : outputStr = "m,time,bit0,bit1,...,bit24;"
void onRequestMotorStatus() {
	outputStr.remove(0);
	outputStr.concat(F("m,"));
	outputStr.concat(millis());

	control.status(&status[0]);

	for ( int i = 0; i < STATUS_SIZE; i++ ) {
		outputStr.concat(F(","));
		outputStr.concat(status[i]);
	}

	outputStr.concat(F(";"));
	Serial.println(outputStr);
}

// Format : outputStr = "g,time,status;"
void onRequestStallStatus() {
	outputStr.remove(0);
	outputStr.concat(F("g,"));
	outputStr.concat(millis());
	outputStr.concat(F(","));

	_binaryDisplay(control.sgStatus());

	outputStr.concat(F(";"));
	Serial.println(outputStr);
}

// Format : outputStr = "m,time,oldpos,newpos;"
void onRequestSetPosNoMove() {
	outputStr.remove(0);
	outputStr.concat(F("d,"));
	outputStr.concat(millis());
	outputStr.concat(F(","));

	outputStr.concat(control.getXactual());
	outputStr.concat(F(","));
	control.changePosNoMove(cmdMessenger.readDoubleArg());
	outputStr.concat(control.getXactual());

	outputStr.concat(F(";"));
	Serial.println(outputStr);
}

// Format : outputStr = "A,ADCBits;"
void onGetADCBits() {
    outputStr.remove(0);
    outputStr.concat(F("a,"));
    outputStr.concat(ADCResolution);
    outputStr.concat(F(";"));
    Serial.println(outputStr);
}

// Format : outputStr = "V,ADCVolts;"
void onGetADCRefVolt() {
    outputStr.remove(0);
    outputStr.concat(F("v,"));
    outputStr.concat(ADCRefVolt);
    outputStr.concat(F(";"));
    Serial.println(outputStr);
}

// Format : outputStr = "x,time,xposition;" (Note that it is a 200 stepper motor)
void onGetXactual() {
	outputStr.remove(0);
  	outputStr.concat(F("x,"));
  	outputStr.concat(millis());
  	outputStr.concat(F(","));

  	outputStr.concat(control.getXactual());

  	outputStr.concat(F(";"));
  	Serial.println(outputStr);
}

// Format : outputStr = "v,time,velocity;"
void onGetVelocity() {
	outputStr.remove(0);
  	outputStr.concat(F("v,"));
  	outputStr.concat(millis());
  	outputStr.concat(F(","));

  	outputStr.concat(control.getVelocity());

  	outputStr.concat(F(";"));
  	Serial.println(outputStr);
}

// Format : outputStr = "a,time,acceleration;"
void onGetAcceleration() {
	outputStr.remove(0);
  	outputStr.concat(F("a,"));
  	outputStr.concat(millis());
  	outputStr.concat(F(","));

  	outputStr.concat(control.getAcceleration());

  	outputStr.concat(F(";"));
  	Serial.println(outputStr);
}

// Format : outputStr = "d,time,deceleration;"
void onGetDeceleration() {
	outputStr.remove(0);
  	outputStr.concat(F("d,"));
  	outputStr.concat(millis());
  	outputStr.concat(F(","));

  	outputStr.concat(control.getDeceleration());

  	outputStr.concat(F(";"));
  	Serial.println(outputStr);
}

// Format : outputStr = "P,time,power;"
void onGetPower() {
	outputStr.remove(0);
  	outputStr.concat(F("P,"));
  	outputStr.concat(millis());
  	outputStr.concat(F(","));

  	outputStr.concat(control.getPower());

  	outputStr.concat(F(";"));
  	Serial.println(outputStr);
}

// Format : not changes to outputStr
void onConstantForward() {
	if ( _checkFlags() ) {
		double velocity = cmdMessenger.readDoubleArg();
		if (velocity == 0) {
			onFail();
		}
		else {
			control.constForward(velocity);
			#ifdef DEBUG_COM
				Serial.print("Velocity: ");
				Serial.println(velocity);
			#endif
			onSuccess();
		}
	}
	else {
		onFail();
	}
}

// Format : not changes to outputStr
void onConstantBackward() {
	if ( _checkFlags() ) {
		double velocity = cmdMessenger.readDoubleArg();
		if ( velocity == 0 ) {
			onFail();
		}
		else {
			control.constReverse(velocity);
			#ifdef DEBUG_COM
				Serial.print("Velocity: ");
				Serial.println(velocity);
			#endif
			onSuccess();
		}
	}
	else {
		onFail();
	}
}

// Format : not changes to outputStr
void onMovePosition() {
	_checkJS();
	if ( !motorFlags.isSeeking ) {
		double pos = cmdMessenger.readDoubleArg();
		control.goPos(pos); // Note that the default if no argument read is to go home
		#ifdef DEBUG_COM
			Serial.print("Position: ");
			Serial.println(pos);
		#endif
		onSuccess();
	}
	else {
		onFail();
	}
}

// Format : not changes to outputStr
void onMoveForward() {
	_checkJS();
	if ( !motorFlags.isSeeking ) {
		double stepsForward = cmdMessenger.readDoubleArg();
		double velocity = cmdMessenger.readDoubleArg();
		if(stepsForward == 0 || velocity == 0) {
			onFail();
		}
		else { 
			control.forward(stepsForward, velocity);

			#ifdef DEBUG_CO
				Serial.print("Velocity: ");
				Serial.println(velocity);
				Serial.print("Steps Forward: ");
				Serial.println(stepsForward);
			#endif
			onSuccess();
		}
	}
	else {
		onFail();
	}
}

// Format : not changes to outputStr
void onMoveBackward() {
	_checkJS();
	if ( !motorFlags.isSeeking ) {
		double stepsForward = cmdMessenger.readDoubleArg();
		double velocity = cmdMessenger.readDoubleArg();
		if(stepsForward == 0 || velocity == 0) {
			onFail();
		}
		else {
			control.reverse(stepsForward, velocity);

			#ifdef DEBUG_COM
				Serial.print("Velocity: ");
				Serial.println(velocity);
				Serial.print("Steps Forward: ");
				Serial.println(stepsForward);
			#endif
			onSuccess();
		}
	}
	else {
		onFail();
	}
}

// Format : not changes to outputStr
void onVelocity() {
	double velocity = cmdMessenger.readDoubleArg();

	if (velocity == 0) {
		onFail();
	}
	else {
		control.setVelocity(velocity);
		onSuccess();
	}
}

// Format : not changes to outputStr
void onAcceleration() {
	double acceleration = cmdMessenger.readDoubleArg();

	if(acceleration == 0) {
		onFail();
	}
	else {
		control.setAcceleration(acceleration);
		onSuccess();
	}
}

// Format : not changes to outputStr
void onDeceleration() {
	double deceleration = cmdMessenger.readDoubleArg();

	if(deceleration == 0) {
		onFail();
	}
	else {
		control.setDeceleration(deceleration);
		onSuccess();
	}
}

// Format : not changes to outputStr
void onPower() {
	double holdPower = cmdMessenger.readDoubleArg();
	double runPower = cmdMessenger.readDoubleArg();

	if(runPower == 0 || holdPower == 0) {
		onFail();
	}
	else {
		control.setPower(holdPower, runPower);
		onSuccess();
	}
}

// Format : not changes to outputStr
void onDirection() {

	double direction = cmdMessenger.readDoubleArg();

	switch((int)direction) {
		case(2): {
			control.setDirections(true, false);
		} break;

		case(3): {
			control.setDirections(false, true);
		} break;

		case(4): {
			control.setDirections(false, false);
		} break;

		default: {
			control.setDirections(true, true);
		}
	}

	onSuccess();
}

// Format : not changes to outputStr
void onJSEnable() {
	motorFlags.isJSEnable = true;
	control.enableJoystick();
	onSuccess();
}

// Format : not changes to outputStr
void onJSDisable() {
	motorFlags.isJSEnable = false;
	control.disableJoystick();
	onSuccess();
}

// Format : not changes to outputStr
void onMotorStop() {
	control.stop();
	motorFlags.isJSEnable = false;
	motorFlags.isSeeking = false;
	motorFlags.isPositioning = false;
	onSuccess();
}

// Format : not changes to outputStr
void onMotorHome() {
	if ( _checkFlags() ) {
		control.setHome();
		onSuccess();
	}
	else {
		onFail();
	}
}

// Format : not changes to outputStr
void onSeek() {
	_checkJS();
	if (motorFlags.isPositioning) {
		onFail();
	}
	else {
		motorFlags.direction = cmdMessenger.readBoolArg();
		motorFlags.isSeeking = true;
		onSuccess();
	}
}

// Format : not changes to outputStr
void onResolution() {
	int resolution = cmdMessenger.readInt16Arg();
	Serial.println(resolution);
	control.setResolution(resolution);
	onSuccess();
}

/* =======================================
	Note that on stop switches and on active
	settings are not guaranteed to work if
	called during a movement (ie: constForward,
	homing), but are guaranteed to work in
	the following call.
======================================= */

// Format : not changes to outputStr
void onActiveSettings() {
	bool fw = cmdMessenger.readBoolArg();
	bool bw = cmdMessenger.readBoolArg();
	control.switchActiveEnable(fw, bw);
	onSuccess();
}

// Format : not changes to outputStr
void onPing(){
    Serial.println(F("p,PONG;"));
}

