#include "MotorControl.h"

byte WRITE = 0x01;
byte READ = 0x00; 

/* ======================================================================
	Constructor functiona to make a motor object. Requires the Chip Select (cs)
	pin, the enable output pin, and the motor ID as inputs. Note that the pins
	are active low. The empty constructor cannot take the pins and so
	they must be set after construction by the MotorControl :: set function.
====================================================================== */

MotorControl :: MotorControl() {

	// Initialize typical data for motor
	_microSteps = 256;
	_homeCaseNum = 0;
	_isHomed = false;
	_resolutionNum = 1;

	IsPositionMode = true;
	IsForward = true;

	// change so there's only one const and step function instead and deal with in structure

	// Initialize directional structures
	forwardDirection.activeEnableNum 	= 3;
	forwardDirection.buttonStatusNum 	= 1;
	forwardDirection.dirMultiplier 		= 1;
	forwardDirection.address 			= ADDRESS_MODE_VELPOS;

	backwardDirection.activeEnableNum 	= 2;
	backwardDirection.buttonStatusNum 	= 0;
	backwardDirection.dirMultiplier 	= -1;
	backwardDirection.address 			= ADDRESS_MODE_VELNEG;

	// Writing to Registers
	GCONF.rw 					= WRITE;
	GCONF.address 				= ADDRESS_GCONF;
	GCONF.data 					= 0x00000000;

	CHOPCONF.rw 				= WRITE;
	CHOPCONF.address 			= ADDRESS_CHOPCONF;
	CHOPCONF.data 				= 0x000101D5;

	IHOLD_IRUN.rw 				= WRITE;
	IHOLD_IRUN.address			= ADDRESS_IHOLD_IRUN;
	IHOLD_IRUN.data 			= 0x0000190A;

	TPOWERDOWN.rw 				= WRITE;
	TPOWERDOWN.address 			= ADDRESS_TZEROWAIT;
	TPOWERDOWN.data 			= 0x0000000A;

	PWMCONF.rw 					= WRITE;
	PWMCONF.address 			= ADDRESS_PWMCONF;
	PWMCONF.data 				= 0x00150480;

	A1.rw 						= WRITE;
	A1.address 					= ADDRESS_A1;
	A1.data 					= 0x000003E8;

	V1.rw 						= WRITE;
	V1.address 					= ADDRESS_V1;
	V1.data 					= 0x000186A0;

	AMAX.rw 					= WRITE;
	AMAX.address 				= ADDRESS_AMAX;
	AMAX.data	 				= 0x0000C350;

	VMAX.rw 					= WRITE;
	VMAX.address 				= ADDRESS_VMAX;
	VMAX.data 					= 0x000186A0;

	DMAX.rw 					= WRITE;
	DMAX.address 				= ADDRESS_DMAX;
	DMAX.data 					= 0x0000C350;

	D1.rw 						= WRITE;
	D1.address 					= ADDRESS_D1;
	D1.data 					= 0x00000578;

	VSTOP.rw					= WRITE;
	VSTOP.address 				= ADDRESS_VSTOP;
	VSTOP.data					= 0x0000000A;

	RAMPMODE.rw 				= WRITE;
	RAMPMODE.address 			= ADDRESS_RAMPMODE;
	RAMPMODE.data 				= 0x00000000;

	XACTUAL.rw 					= WRITE;
	XACTUAL.address 			= ADDRESS_XACTUAL;
	XACTUAL.data 				= 0x00000000;

	XTARGET.rw 					= WRITE;
	XTARGET.address 			= ADDRESS_XTARGET;
	XTARGET.data 				= 0x00000000;

	HOME_XTARGET.rw				= WRITE;
	HOME_XTARGET.address 		= ADDRESS_XTARGET;
	HOME_XTARGET.data 			= 0x00000000;

	SW_MODE.rw 					= WRITE;
	SW_MODE.address 			= ADDRESS_SWMODE;
	SW_MODE.data 				= 0x00000002;

	// Reading to Registers
	DRV_STATUS_READ.rw 			= READ;
	DRV_STATUS_READ.address 	= ADDRESS_DRVSTATUS;
	DRV_STATUS_READ.data 		= 0x00000000;

	GCONF_READ.rw 				= READ;
	GCONF_READ.address 			= ADDRESS_GCONF;
	GCONF_READ.data 			= 0x00000000;

	GSTAT_READ.rw 				= READ;
	GSTAT_READ.address 			= ADDRESS_GSTAT;
	GSTAT_READ.data 			= 0x00000000;

	RAMP_STAT_READ.rw 			= READ;
	RAMP_STAT_READ.address 		= ADDRESS_RAMPSTAT;
	RAMP_STAT_READ.data 		= 0x00000000;

	XACTUAL_READ.rw 			= READ;
	XACTUAL_READ.address 		= ADDRESS_XACTUAL;
	XACTUAL_READ.data 			= 0x00000000;

}

/* ======================================================================
	Constructor functiona to make a motor object. Requires the Chip Select (cs)
	pin, the enable output pin, and the motor ID as inputs. Note that the pins
	are active low. 
====================================================================== */

MotorControl :: MotorControl(byte csPin, byte enablePin, int ID) {

	// MotorMoveStartTime = 0;
	motorID = ID;

	// Initialize typical data for motor
	_microSteps = 256;
	_homeCaseNum = 0;
	_isHomed = false;
	_resolutionNum = 1;

	IsPositionMode = true;
	IsForward = true;

	// Initialize directional structures
	forwardDirection.activeEnableNum 	= 3;
	forwardDirection.buttonStatusNum 	= 1;
	forwardDirection.dirMultiplier 		= 1;
	forwardDirection.address 			= ADDRESS_MODE_VELPOS;

	backwardDirection.activeEnableNum 	= 2;
	backwardDirection.buttonStatusNum 	= 0;
	backwardDirection.dirMultiplier 	= -1;
	backwardDirection.address 			= ADDRESS_MODE_VELNEG;

	// Pass CS and enable pin assignments to private variables
	//start automatically with pin not connected (active LOW)
	_csPin = csPin;
	_enablePin = enablePin;

	digitalWrite(csPin, HIGH);
	digitalWrite(enablePin, HIGH);

	// Writing to Registers
	GCONF.rw 					= WRITE;
	GCONF.address 				= ADDRESS_GCONF;
	GCONF.data 					= 0x00000000;

	CHOPCONF.rw 				= WRITE;
	CHOPCONF.address 			= ADDRESS_CHOPCONF;
	CHOPCONF.data 				= 0x000101D5;

	IHOLD_IRUN.rw 				= WRITE;
	IHOLD_IRUN.address			= ADDRESS_IHOLD_IRUN;
	IHOLD_IRUN.data 			= 0x0000190A;

	TPOWERDOWN.rw 				= WRITE;
	TPOWERDOWN.address 			= ADDRESS_TZEROWAIT;
	TPOWERDOWN.data 			= 0x0000000A;

	PWMCONF.rw 					= WRITE;
	PWMCONF.address 			= ADDRESS_PWMCONF;
	PWMCONF.data 				= 0x00150480;

	A1.rw 						= WRITE;
	A1.address 					= ADDRESS_A1;
	A1.data 					= 0x000003E8;

	V1.rw 						= WRITE;
	V1.address 					= ADDRESS_V1;
	V1.data 					= 0x000186A0;

	AMAX.rw 					= WRITE;
	AMAX.address 				= ADDRESS_AMAX;
	AMAX.data	 				= 0x0000C350;

	VMAX.rw 					= WRITE;
	VMAX.address 				= ADDRESS_VMAX;
	VMAX.data 					= 0x000186A0;

	DMAX.rw 					= WRITE;
	DMAX.address 				= ADDRESS_DMAX;
	DMAX.data 					= 0x0000C350;

	D1.rw 						= WRITE;
	D1.address 					= ADDRESS_D1;
	D1.data 					= 0x00000578;

	VSTOP.rw					= WRITE;
	VSTOP.address 				= ADDRESS_VSTOP;
	VSTOP.data					= 0x0000000A;

	RAMPMODE.rw 				= WRITE;
	RAMPMODE.address 			= ADDRESS_RAMPMODE;
	RAMPMODE.data 				= 0x00000000;

	XACTUAL.rw 					= WRITE;
	XACTUAL.address 			= ADDRESS_XACTUAL;
	XACTUAL.data 				= 0x00000000;

	XTARGET.rw 					= WRITE;
	XTARGET.address 			= ADDRESS_XTARGET;
	XTARGET.data 				= 0x00000000;

	HOME_XTARGET.rw				= WRITE;
	HOME_XTARGET.address 		= ADDRESS_XTARGET;
	HOME_XTARGET.data 			= 0x00000000;

	SW_MODE.rw 					= WRITE;
	SW_MODE.address 			= ADDRESS_SWMODE;
	SW_MODE.data 				= 0x00000002;

	// Reading to Registers
	DRV_STATUS_READ.rw 			= READ;
	DRV_STATUS_READ.address 	= ADDRESS_DRVSTATUS;
	DRV_STATUS_READ.data 		= 0x00000000;

	GCONF_READ.rw 				= READ;
	GCONF_READ.address 			= ADDRESS_GCONF;
	GCONF_READ.data 			= 0x00000000;

	GSTAT_READ.rw 				= READ;
	GSTAT_READ.address 			= ADDRESS_GSTAT;
	GSTAT_READ.data 			= 0x00000000;

	RAMP_STAT_READ.rw 			= READ;
	RAMP_STAT_READ.address 		= ADDRESS_RAMPSTAT;
	RAMP_STAT_READ.data 		= 0x00000000;

	XACTUAL_READ.rw 			= READ;
	XACTUAL_READ.address 		= ADDRESS_XACTUAL;
	XACTUAL_READ.data 			= 0x00000000;
}

/* ======================================================================
	Sets the motor pins, required when using the empty constructor. Not 
	needed when using the non-empty constructor.
 ====================================================================== */

void MotorControl :: set(byte csPin, byte enablePin, int ID) {

	motorID = ID;

	// Pass CS and enable pin assignments to private variables
	// start automatically with pin not connected (active LOW)
	_csPin = csPin;
	_enablePin = enablePin;
}

/* ======================================================================
	Function initializes motor values by sending in the data to the motor.
	Must be done before movement operations can be completed, ncesssary
	for both the empty and non-empty constructor.
 ====================================================================== */

void MotorControl :: begin() {

	digitalWrite(_csPin, HIGH);
	digitalWrite(_enablePin, HIGH);
	delay(3);

	// Turn on the enable pin to talk to the motor
	MotorControl :: powerEnable();

	MotorControl::sendData(&GCONF);
	MotorControl::sendData(&IHOLD_IRUN);
	MotorControl::sendData(&TPOWERDOWN);
	
	MotorControl::sendData(&CHOPCONF);
	MotorControl::sendData(&PWMCONF);
	MotorControl::sendData(&RAMPMODE);
	
	MotorControl::sendData(&A1);
	MotorControl::sendData(&V1);
	MotorControl::sendData(&D1);
	
	MotorControl::sendData(&AMAX);
	MotorControl::sendData(&VMAX);

	MotorControl::sendData(&VSTOP);
	
	MotorControl::sendData(&XACTUAL);
	MotorControl::sendData(&XTARGET);
	
	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.println(F(" : Motor initialised."));
	Serial.flush();
	#endif
}

/* ======================================================================
	Function sends the datagrams to the TMC5130 in order of MSB first. Note
 	the format of the datagram as the first byte is the address and its MSB
 	is either read(0) or write(1). The remaining four bytes are reserved for 
 	data.

	SPI MODE:		CLOCK POLARITY:		CLOCK PHASE:		CLOCK EDGE:
		SPI_MODE0			0				0					1
		SPI_MODE1			0				1					0		
		SPI_MODE2			1				0					1
		SPI_MODE3			1				1					0
	
	Note that the TMC5130 uses MODE 3 as expressed in the data sheet (Chp.4)
====================================================================== */

void MotorControl :: sendData(datagram * out_datagram) {
	//TMC5130 takes 40 bit data: 8 address and 32 data, first bit determines read(0) or write(1)

	// Re-write this function 
	delayMicroseconds(3);
	MotorControl::csEnable();

	i_datagram.responseFlags = SPI.transfer(((out_datagram->rw << 7) | out_datagram->address) & 0xff);

	i_datagram.data = SPI.transfer((out_datagram->data >> 24) & 0xff);
	i_datagram.data <<= 8;
	i_datagram.data |= SPI.transfer((out_datagram->data >> 16) & 0xff);
	i_datagram.data <<= 8;
	i_datagram.data |= SPI.transfer((out_datagram->data >> 8) & 0xff);
	i_datagram.data <<= 8;
	i_datagram.data |= SPI.transfer((out_datagram->data) & 0xff);
	
	MotorControl::csDisable();

	_outputDatagram = i_datagram;

	#ifdef DEBUG_MOTOR
	Serial.print("Sent: ");
	Serial.println(out_datagram->data, HEX);
	Serial.print("Received: ");
	Serial.println(i_datagram.data, HEX);
	Serial.print(" from register: ");
	Serial.println(((out_datagram->rw << 7) | out_datagram->address) & 0xff, HEX);
	#endif
}

/* ======================================================================
	Function gets and prints the main motor data. Used for debugging.
 ====================================================================== */

void MotorControl :: getMotorData() {

	Serial.println("");

	Serial.print("Ramp Mode: ");
	Serial.println(RAMPMODE.data, HEX);

	Serial.print("Acceleration 1: ");
	Serial.println(A1.data, HEX);

	Serial.print("Velocity 1: ");
	Serial.println(V1.data, HEX);

	Serial.print("Position 1: ");
	Serial.println(D1.data, HEX);

	Serial.print("Maximum Acceleration: ");
	Serial.println(AMAX.data, HEX);

	Serial.print("Maximum Velocity: ");
	Serial.println(VMAX.data, HEX);

	Serial.print("Stop Velocity: ");
	Serial.println(VSTOP.data, HEX);

	Serial.print("Actual Position: ");
	Serial.println(XACTUAL.data, HEX);

	Serial.print("Target Position: ");
	Serial.println(XTARGET.data, HEX);

	Serial.println("");
}


/* ======================================================================
	Function finds the status of all the registers of the motor. See chapter
	6 of the datasheet for specific register and bit numbers.
 ====================================================================== */

void MotorControl :: readStatus() {
    status_sg2 = false;
    status_standstill = false;
    status_velocity_reached = false;
    status_position_reached = false;
    status_stop_l = false;
    status_stop_r = false;   

    //pipeline flush
    MotorControl::sendData( & RAMP_STAT_READ);
    MotorControl::sendData( & RAMP_STAT_READ);
	MotorControl::sendData( & RAMP_STAT_READ);
	
    MotorControl::sendData( & DRV_STATUS_READ);
	// Flags for the RAMP_STAT register
    status_stop_l = MotorControl::_checkBit( & _outputDatagram,0);
    status_stop_r = MotorControl::_checkBit( & _outputDatagram,1);
	status_stop_l_event = MotorControl::_checkBit( & _outputDatagram,4);
	status_stop_r_event = MotorControl::_checkBit( & _outputDatagram,5);
	status_latch_l = MotorControl::_checkBit( & _outputDatagram,2);
	status_latch_r = MotorControl::_checkBit( & _outputDatagram,3);
    status_position_reached = MotorControl::_checkBit( & _outputDatagram,9);
    status_velocity_reached = MotorControl::_checkBit( & _outputDatagram,8);
	status_position_reached_event = MotorControl::_checkBit( & _outputDatagram,7);
	status_sg2 = MotorControl::_checkBit( & _outputDatagram,13);
	status_sg2_event = MotorControl::_checkBit( & _outputDatagram,6);
	
	MotorControl::sendData(&GCONF_READ);
	// Flags for the DRV_STATUS register
	status_openLoad_A = MotorControl::_checkBit( & _outputDatagram,29);
	status_openLoad_B = MotorControl::_checkBit( & _outputDatagram,30);
	status_standstill = MotorControl::_checkBit( & _outputDatagram,31);
	status_shortToGround_A = MotorControl::_checkBit( & _outputDatagram,28);
	status_shortToGround_B = MotorControl::_checkBit( & _outputDatagram,27);
	status_overtemperatureWarning = MotorControl::_checkBit( & _outputDatagram,26);
	status_overtemperatureShutdown = MotorControl::_checkBit( & _outputDatagram,25);
	
	MotorControl::sendData(&GSTAT_READ);
	// Flags for the GCONF register
	status_isReverse = MotorControl::_checkBit( & i_datagram,4);
	
	MotorControl::sendData(&GSTAT_READ);
	// Flags for the GSTAT register
	status_resetDetected = MotorControl::_checkBit( & i_datagram,0);
	status_driverError = MotorControl::_checkBit( & i_datagram,1);
	status_underVoltage = MotorControl::_checkBit( & i_datagram,2);

	if (false){
		Serial.print(motorID);
		Serial.print(F(" : "));
		Serial.print(F("Response Flags: "));
		Serial.println(i_datagram.responseFlags,HEX);
		Serial.println(i_datagram.responseFlags,BIN);
		Serial.print(F("Stop L: "));
		Serial.println(status_stop_l);
		Serial.print(F("Stop R:"));
		Serial.println(status_stop_r);
		Serial.print(F("Latch L: "));
		Serial.println(status_latch_l);
		Serial.print(F("Latch R:"));
		Serial.println(status_latch_r);
		Serial.print(F("Stop L Event: "));
		Serial.println(status_stop_l_event);
		Serial.print(F("Stop R Event:"));
		Serial.println(status_stop_r_event);
		Serial.print(F("Position reached: "));
		Serial.println(status_position_reached);
		Serial.print(F("Velocity Reached: "));
		Serial.println(status_velocity_reached);
		Serial.print(F("Position Reached Event: "));
		Serial.println(status_position_reached_event);
		Serial.print(F("Stall Gaurd On:"));
		Serial.println(status_sg2);
		Serial.print(F("Stall gaurd event: "));
		Serial.println(status_sg2_event);
		Serial.print(F("Motor in standstill: "));
		Serial.println(status_standstill);
		Serial.print(F("Open load A: "));
		Serial.println(status_openLoad_A);
		Serial.print(F("Open load B: "));
		Serial.println(status_openLoad_B);
		Serial.print(F("Short to ground A: "));
		Serial.println(status_shortToGround_A);
		Serial.print(F("Short to ground B: "));
		Serial.println(status_shortToGround_B);
		Serial.print(F("Overtemperature warning: "));
		Serial.println(status_overtemperatureWarning);
		Serial.print(F("Overtemperature Shutdown: "));
		Serial.println(status_overtemperatureShutdown);
		Serial.print(F("Running Reverse: "));
		Serial.println(status_isReverse);
		Serial.print(F("Chip had been reset: "));
		Serial.println(status_resetDetected);
		Serial.print(F("Driver Error: "));
		Serial.println(status_driverError);
		Serial.print(F("Undervoltage: "));
		Serial.println(status_underVoltage);
		Serial.print(F("Current Position: "));

		Serial.flush();
	}
}

/* ======================================================================
	Function finds the status of all bits in the drive status register and 
	records the values (0-9) that reference the stall guard.
====================================================================== */

void MotorControl :: sgStatus() {
	
    MotorControl::sendData( & DRV_STATUS_READ);
    MotorControl::sendData( & DRV_STATUS_READ);

    // sgStatusBits = _outputDatagram.data & 0x000003FF;
    sgStatusBits = _outputDatagram.data & 0xFFFFFFFF;
}

/* ======================================================================
	Function finds the status of the push buttons connected to the motor
	controller to signal the switch has been engaged.
====================================================================== */

void MotorControl :: buttonStatus() {

	//pipeline flush
    MotorControl::sendData( & RAMP_STAT_READ);
    MotorControl::sendData( & RAMP_STAT_READ);

    // Bit 1 = right switch, Bit 0 = left switch
    forwardSwitch = MotorControl::_checkBit( & _outputDatagram,(forwardDirection.buttonStatusNum));
	backwardSwitch = MotorControl::_checkBit( & _outputDatagram,(backwardDirection.buttonStatusNum));
}

/* ======================================================================
	Checks and compares the specific bit within the datagram dg. The bit 
	is only in the data part of the datagram and not in the address (note 
	the data ranges from bit 0-31).
====================================================================== */

bool MotorControl::_checkBit(datagram * dg, int targetBit) {

    if (dg->data & (0x80000000 >> 31-targetBit)) {
        return true;
    }

    return false;
}

/* ======================================================================
	Swicthes the input reference of the right and left switches.
====================================================================== */

void MotorControl :: switchReference(bool rightIsREFR) {

	if (rightIsREFR) {
		// Keep old SW_MODE.data except for bit-4, replace bit with 0
		SW_MODE.data = (SW_MODE.data & 0xFFFFFFEF) | 0x00000000;
		MotorControl :: sendData( &SW_MODE );
	}
	else {
		// Keep old SW_MODE.data except for bit-4, replace bit with 1
		SW_MODE.data = (SW_MODE.data & 0xFFFFFFEF) | 0x00000010;
		MotorControl :: sendData( &SW_MODE );
	}
}

//==============================================================
//================= ENABLE AND DISABLE FUNCTIONS ===============
//==============================================================

bool MotorControl :: powerEnable() {

    digitalWrite(_enablePin,LOW);
    return true;
}

bool MotorControl :: powerDisable() {

    digitalWrite(_enablePin, HIGH);
    return true;
}

bool MotorControl :: csEnable() {

    digitalWrite(_csPin,LOW);
    delay(1);
    return true;
}

bool MotorControl :: csDisable() {

	delay(1);
    digitalWrite(_csPin,HIGH);
	//digitalWrite(DIO13,LOW);
    delay(1);
    return true;
}

/* ======================================================================
	checks the user input and changes the bits for the polarity of the
	switches. Forward and backward switch are determined by the user's
	setting of directions by using MotorControl :: swapDirection
 ====================================================================== */

bool MotorControl :: switchActiveEnable(bool fw, bool bw) {

	// declare 32-bit long value
	unsigned long forward  = 0x00000000;
	unsigned long backward = 0x00000000;

	// bit 2 is left switch, bit 3 is right switch
	forward = ((long) fw) << (forwardDirection.activeEnableNum);
	backward = ((long) bw) << (backwardDirection.activeEnableNum);

	// Keep old SW_MODE.data except for the first two bits, replace them with new bits
	SW_MODE.data = (SW_MODE.data & 0xFFFFFFF3) | (forward | backward);
	MotorControl :: sendData( &SW_MODE );

	#ifdef DEBUG_DIR
		Serial.print("SW Mode: ");
		Serial.println("SW_MODE.data");
	#endif

	return true;
}

//==============================================================
//================== READ AND  WRITE FUNCTIONS =================
//==============================================================

void MotorControl :: setPowerLevel(unsigned long holdPower, unsigned long runPower) {

	IHOLD_IRUN.data = holdPower + ( runPower << 8 );
	MotorControl :: sendData(&IHOLD_IRUN);
}

void MotorControl :: setVelocity(unsigned long velocity) {

	VMAX.data = velocity;
	MotorControl :: sendData(&VMAX);

	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.print(F(" : SetVelocity: New Velocity: "));
	Serial.println(VMAX.data, HEX);
	Serial.flush();
	#endif
}

void MotorControl :: setAcceleration(unsigned long acceleration) {

	AMAX.data = acceleration;
	MotorControl :: sendData(&AMAX);

	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.print(F(" : SetAcceleration: New acceleration: "));
	Serial.println(AMAX.data,HEX);
	Serial.flush();
	#endif
}

void MotorControl :: setDeceleration(unsigned long deceleration) {

	DMAX.data = deceleration;
	MotorControl :: sendData(&DMAX);

	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.print(F(" : SetDeceleration: New deceleration: "));
	Serial.println(DMAX.data,HEX);
	Serial.flush();
	#endif
}

void MotorControl :: setXtarget(unsigned long xtarget) {

	XTARGET.data = xtarget;
	MotorControl :: sendData(&XTARGET);

	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.print(F(" : SetXtarget: New XTARGET: "));
	Serial.println(XTARGET.data,HEX);
	Serial.flush();
	#endif
}

void MotorControl :: setXactual(unsigned long xactual) {
	XACTUAL.data = xactual;
	MotorControl :: sendData(&XACTUAL);

	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.print(F(" : SetXactual: New XACTUAL: "));
	Serial.println(XACTUAL.data,HEX);
	Serial.flush();
	#endif
}

/* ======================================================================
	Function sends sets the ramp mode to the desired type of motor
	movement.

	RAMP MODE:			POSITION MODE:		FORWARD DIRECTION:
		RAMP_MODE0			TRUE				-
		RAMP_MODE1			FALSE				TRUE	
		RAMP_MODE2			FALSE				FALSE
		RAMP_MODE3			-					-
	
	Note that detailed descriptions can be found on page 35 in (chp.6)
====================================================================== */

void MotorControl :: setRampMode(unsigned long rampMode) {

	switch(rampMode) {

		case(0):
		{
			IsPositionMode = true;
			IsForward = true;
			break;
		}

		case(1):
		{
			IsPositionMode = false;
			IsForward = true;
			break;
		}

		case(2):
		{
			IsPositionMode = false;
			IsForward = false;
			break;
		}

		default:
		{
			IsPositionMode = true;
			IsForward = true;
			rampMode = ADDRESS_MODE_POSITION;
			break;
		}
	}
	RAMPMODE.data = rampMode;
	MotorControl :: sendData(&RAMPMODE);

	#ifdef DEBUG_MOTOR
	Serial.print(motorID);
	Serial.print(F(" : Set Ramp Mode: New Ramp Mode: "));
	Serial.println(RAMPMODE.data,HEX);
	Serial.flush();
	#endif
}

// See datasheet pg.42, it 27-24 reserved for motor step resolution
void MotorControl :: setChopConf(int resolution) {
	// 256 => B0000, 128 => B0001, 64 => B0010,..., 1 => B1000
	int res[] = {256, 128, 64, 32, 16, 8, 4, 2, 1};

	unsigned long binaryNum = 0b0000;

	for (int i = 0; i < 9; i++) {
		if ( res[i] == resolution ) {
			_resolutionNum = i+1;
			binaryNum = i & (0b000000000000000000001111);
			Serial.println(binaryNum, HEX);
			break;
		}
	}

	binaryNum <<= 24;

	// Serial.print("Binary Number: ");
	// Serial.println(binaryNum,HEX);

	CHOPCONF.data = binaryNum | 0x000101D5;

	// Serial.print("CHOPCONF data: ");
	// Serial.println(CHOPCONF.data, HEX);

	MotorControl :: sendData(&CHOPCONF);
}

int MotorControl :: getMotorID() {
	return motorID;
}

unsigned long MotorControl :: getPowerLevel() {
	return IHOLD_IRUN.data;
}

unsigned long MotorControl :: getVelocity() {
	return VMAX.data;
}

unsigned long MotorControl :: getAcceleration() {
	return AMAX.data;
}

unsigned long MotorControl :: getDeceleration() {
	return DMAX.data;
}

unsigned long MotorControl :: getXtarget() {
	return XTARGET.data;
}

unsigned long MotorControl :: getXactual() {
	MotorControl :: sendData(&XACTUAL_READ);
	MotorControl :: sendData(&XACTUAL_READ);

	return i_datagram.data;
}

unsigned long MotorControl :: getHomeXtarget() {
	return HOME_XTARGET.data;
}

unsigned long MotorControl :: getRampMode() {
	return RAMPMODE.data;
}

bool MotorControl :: getIsForward() {
	return IsForward;
}

bool MotorControl :: getIsPositionMode() {
	return IsPositionMode;
}

bool MotorControl :: getIsHomed() {
	return _isHomed;
}

int MotorControl :: getResolution() {
	return _resolutionNum;
}

//==============================================================
//====================== MOVEMENT FUNCTIONS ====================
//==============================================================

/* ======================================================================
	Function sends commands to the motor to move it to any state. Going to 
	position '0' will send it back to the home state.
 ====================================================================== */

bool MotorControl :: goPos(unsigned long position) {

	MotorControl :: setVelocity(0);

	// Check if the motor is in position mode, if not, change to position mode
	if (MotorControl :: getRampMode() != ADDRESS_MODE_POSITION) {
		MotorControl :: setRampMode(ADDRESS_MODE_POSITION);
	}

	MotorControl :: setVelocity(STANDVELOCITY / _resolutionNum);
 	MotorControl :: setXtarget(position);
 	
 	return true;
}

/* ======================================================================
	Function sends commands to the motor to return it to the "home" state.
	Function checks homing device in order:
		case(0): 	checks the register to see if the right (home) switch is active.
					If active, finishes homing procedure. If home position has not 
					been set, sets home position.
		case(1):	Set switch mode for right active switch.
		case(2):	Monitors register for right switch active status.
		case(3):	Once the right switch is active, sets the home position to the
					current position.	
		default:	
 ====================================================================== */

bool MotorControl :: setHome() {

	bool done = false;

	switch(_homeCaseNum) {

		case(0): {

			MotorControl::buttonStatus();

			if (forwardSwitch) {

				#ifdef DEBUG_HOME
					Serial.print(motorID);
					Serial.print(F(" : "));
					Serial.println(F("Homing: Stage at home. "));
				#endif

				_homeCaseNum = 3;
			}

			else {

				#ifdef DEBUG_HOME
					Serial.print(motorID);
					Serial.print(F(" : "));
					Serial.println(F("Homing: Stage not homed, continuing with homing."));
				#endif 

				_homeCaseNum++;
			}
		} break;
		
		case(1): {

			MotorControl :: sendData(&RAMPMODE);
			MotorControl :: sendData(&XACTUAL);
			MotorControl :: sendData(&VMAX);
			_homeCaseNum++;
			
		} break;

		case(2): {

			#ifdef DEBUG_HOME
				Serial.print(motorID);
				Serial.print(F(" : "));
				Serial.println(F("Homing: Set home target."));
			#endif 

			MotorControl::sendData(&HOME_XTARGET);
			MotorControl::sendData(&SW_MODE);
			_homeCaseNum++;
		} break;
		
		case(3): {

			MotorControl :: constForward(STANDVELOCITY / _resolutionNum);

			while(forwardSwitch == false) {
				MotorControl :: buttonStatus();
			}

			MotorControl :: stop();

			#ifdef DEBUG_HOME
				Serial.print(motorID);
				Serial.print(F(" : "));
				Serial.println(F("Homing: Motor Homed."));
			#endif

			MotorControl::sendData(&XACTUAL);

			HOME_XTARGET.data = MotorControl :: getXactual();
			MotorControl :: setXtarget(HOME_XTARGET.data);


			_homeCaseNum++;
		} break;
		
		default: {

			#ifdef DEBUG_HOME
				Serial.print(motorID);
				Serial.print(F(" : "));
				Serial.println(F("Homing: Operation complete."));
			#endif 

			_homeCaseNum = 0;
			done = true;
		}	
	}

    return done;
}

/* ======================================================================
	Function sends commands to rotate the motor the number of half-steps
	specified in the forward direction. Returns true if the commands are 
	successfully sent. The number of steps require for 1 revolution is the
	value of the resolution (ie: resolution 0 => 256 half steps => 256
	steps/revolution).
 ====================================================================== */

void MotorControl :: forward(unsigned long stepsForward, unsigned long velocity) {

	// see TMC5130 datasheet, recommened 10 for stop velocity instead of 0
	MotorControl :: setVelocity(10);

	if (MotorControl :: getRampMode() != ADDRESS_MODE_POSITION) {
		MotorControl :: setRampMode(ADDRESS_MODE_POSITION);
	}

	unsigned long amountForward = stepsForward * MOTOR_STEPS;

 	unsigned long currentPosition = MotorControl :: getXtarget();
 	unsigned long xTarget = currentPosition + (forwardDirection.dirMultiplier)*amountForward;
 	MotorControl :: setXtarget(xTarget);

 	#ifdef DEBUG_DIR
	 	Serial.print("Forward Multiplier: ");
	 	Serial.println((forwardDirection.dirMultiplier));
	 	Serial.print("Amount: ");
	 	Serial.println(amountForward);
 	#endif

 	MotorControl :: setVelocity(velocity);
}

/* ======================================================================
	Function sends commands to rotate the motor the number of half-steps
	specified in the reverse direction. Returns true if the commands are 
	successfully sent.
 ====================================================================== */

void MotorControl :: reverse(unsigned long stepsBackward, unsigned long velocity) {

	// see TMC5130 datasheet, recommened 10 for stop velocity instead of 0

	// get rid of, stop velocity
	MotorControl :: setVelocity(10);

	if (MotorControl :: getRampMode() != ADDRESS_MODE_POSITION) {
		MotorControl :: setRampMode(ADDRESS_MODE_POSITION);
	}

 	unsigned long amountBackward = stepsBackward * MOTOR_STEPS;

 	unsigned long currentPosition = MotorControl :: getXtarget();
 	unsigned long xTarget = currentPosition + (backwardDirection.dirMultiplier)*amountBackward;
 	MotorControl :: setXtarget(xTarget);

 	#ifdef DEBUG_DIR
	 	Serial.print("Backward Multiplier: ");
	 	Serial.println((backwardDirection.dirMultiplier));
	 	Serial.print("Amount: ");
	 	Serial.println(amountBackward);
 	#endif

 	MotorControl :: setVelocity(velocity);
}

/* ======================================================================
	Function sends commands to rotate the motor continuously forward at a
	constant velocity
 ====================================================================== */

void MotorControl :: constForward(unsigned long velocity) {
 	
 	// Check if the motor is in mode 1, if not, change
	if (MotorControl :: getRampMode() != forwardDirection.address) {
		MotorControl :: setRampMode(forwardDirection.address);
	}

	MotorControl :: setVelocity(velocity);

	#ifdef DEBUG_DIR
		Serial.print("Ramp Mode: ");
		Serial.println(MotorControl :: getRampMode());
	#endif

 	return true;
}

/* ======================================================================
	Function sends commands to rotate the motor continuously backwards at a
	constant velocity
 ====================================================================== */

void MotorControl :: constReverse(unsigned long velocity) {
 	
 	// Check if the motor is in mode 2, if not, change
	if (MotorControl :: getRampMode() != backwardDirection.address) {
		MotorControl :: setRampMode(backwardDirection.address);
	}

	// You must set velocity after ramp mode otherwise will go in the positive direction
	MotorControl :: setVelocity(velocity);

	#ifdef DEBUG_DIR
		Serial.print("Ramp Mode: ");
		Serial.println(MotorControl :: getRampMode());
	#endif 

 	return true;
}


/* ======================================================================
	Function sends commands to stop the motor when it is in a constant reverse
	or in a constant forward movement.
 ====================================================================== */

bool MotorControl :: stop() {

	MotorControl :: constForward(0);
	MotorControl :: setVelocity(0);
	setXtarget(getXactual());

 	return true;
}

/* ======================================================================
	Changes the direction of the switches. If swapDirection is false then 
	forward is clockwise, otherwise if its true then forward is counter 
	clockwise. If swapSwitch is false the right switch is the switch to be
	hit going in the forward direction, otherwise if it is true the left switch
	is the switch hit when going in the forward direction.
	
						Right Switch	Left Switch
	Active Enable 			3 				2
	Button Status 			1 				0

	Note that the switch direction swap will not effect the reading of the registers,
	which will print out the absolute left and right switch bits rather than 
	forward and backward.
 ====================================================================== */

void MotorControl :: swapDirection(bool swapDirection, bool swapSwitch) {

	// If we want to switch the conventional direction, forward = ccw backward = cw
	if (swapDirection) {
		#ifdef DEBUG_DIR
		Serial.println("Swap motor");
		#endif
		forwardDirection.dirMultiplier 		= -1;
		forwardDirection.address 			= ADDRESS_MODE_VELNEG;
		backwardDirection.dirMultiplier 	= 1;
		backwardDirection.address 			= ADDRESS_MODE_VELPOS;
	}

	// If we want to keep the conventional direction, forward = cw backward = ccw
	else {
		#ifdef DEBUG_DIR
		Serial.println("Don't swap motor");
		#endif
		forwardDirection.dirMultiplier 		= 1;
		forwardDirection.address 			= ADDRESS_MODE_VELPOS;
		backwardDirection.dirMultiplier 	= -1;
		backwardDirection.address 			= ADDRESS_MODE_VELNEG;
	}

	// If we want to switch the conventional switch direction, right = backward left = forward
	if(swapSwitch) {
		#ifdef DEBUG_DIR
		Serial.println("Swap switches");
		#endif
		forwardDirection.activeEnableNum 	= 2;
		forwardDirection.buttonStatusNum 	= 0;
		backwardDirection.activeEnableNum 	= 3;
		backwardDirection.buttonStatusNum 	= 1;
		MotorControl :: switchReference(false);
	}

	// If we want to keep the conventional switch direction, right = forward left = backward
	else {
		#ifdef DEBUG_DIR
		Serial.println("Don't swap switches");
		#endif
		forwardDirection.activeEnableNum 	= 3;
		forwardDirection.buttonStatusNum 	= 1;
		backwardDirection.activeEnableNum 	= 2;
		backwardDirection.buttonStatusNum 	= 0;
		MotorControl :: switchReference(true);
	}

	#ifdef DEBUG_DIR
		Serial.println("=== DIRECTION DATA ===");
		Serial.println(" ");
		Serial.print("FW : Active: ");
		Serial.println(forwardDirection.activeEnableNum);
		Serial.print("BW : Active: ");
		Serial.println(backwardDirection.activeEnableNum);
		Serial.println(" ");

		Serial.print("FW : Button: ");
		Serial.println(forwardDirection.buttonStatusNum);
		Serial.print("BW : Button: ");
		Serial.println(backwardDirection.buttonStatusNum);
		Serial.println(" ");

		Serial.print("FW : Multiplier: ");
		Serial.println(forwardDirection.dirMultiplier);
		Serial.print("BW : Multiplier: ");
		Serial.println(backwardDirection.dirMultiplier);
		Serial.println(" ");

		Serial.print("FW : Address: ");
		Serial.println(forwardDirection.address);
		Serial.print("BW : Address: ");
		Serial.println(backwardDirection.address);
	#endif
}