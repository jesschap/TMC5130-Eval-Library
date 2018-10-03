#ifndef MotorControl_H

/* ========================================================================
   $File: MotorControl.h$
   $Date: $
   $Revision: $
   $Creator:  $
   $Email:  $
   $Notice: $
   ======================================================================== */

#define MotorControl_H
#include "Definitions.h"
#include <SPI.h>
#include "Arduino.h"
#include "Joystick.h"

// Debug options
// #define MOTOR_DEBUG 1

struct datagram {
	public:
		byte rw = 0x0;
		byte address = 0x0;
		unsigned long data = 0x0;
		byte responseFlags = 0x0;
};

struct directionControl {
	public: 
		int activeEnableNum;
		int buttonStatusNum;
		int dirMultiplier;

		unsigned long address;
};

class MotorControl {

	public:

		// Class functions
		MotorControl(byte csPin, byte enablePin, int ID); //Constructor
		MotorControl();

		void set(byte csPin, byte enablePin, int ID);
		void begin(); //initialise motor settings

		// Settings
		bool IsPositionMode;
		bool IsForward;
		int motorID;

		// Build direction storing objects
		directionControl forwardDirection;
		directionControl backwardDirection;

		// Motor datagrams to send in for 
		// read and write direction
		datagram GCONF;
		datagram IHOLD_IRUN;
		datagram TPOWERDOWN;
		datagram CHOPCONF;
		datagram RAMPMODE;
		datagram PWMCONF;

		datagram XACTUAL;
		datagram XTARGET;
		datagram HOME_XTARGET;

		datagram A1;
		datagram V1;
		datagram D1;
		datagram AMAX;
		datagram VMAX;
		datagram DMAX;
		datagram VSTOP;
		datagram SW_MODE;

		datagram XACTUAL_READ;
		datagram DRV_STATUS_READ;
		datagram GCONF_READ;
		datagram GSTAT_READ;
		datagram RAMP_STAT_READ;

	    // SPI Reads
	    datagram i_datagram;

	    // Status Bits read in from registers
	    // to check condition of the motor
	    bool status_sg2;
	    bool status_sg2_event;
	    bool status_standstill;

	    bool status_velocity_reached;
	    bool status_position_reached;
	    bool status_position_reached_event;

	    bool status_stop_l;
	    bool status_stop_r;
	    bool status_stop_l_event;
	    bool status_stop_r_event;
	    bool status_latch_l;
	    bool status_latch_r;

	    bool status_openLoad_A;
	    bool status_openLoad_B;
	    bool status_shortToGround_A;
	    bool status_shortToGround_B;

	    bool status_overtemperatureWarning;
	    bool status_overtemperatureShutdown;

	    bool status_isReverse;
	    bool status_resetDetected;
	    bool status_driverError;
	    bool status_underVoltage;

	    unsigned long sgStatusBits;

	    bool forwardSwitch;
	    bool backwardSwitch;


		//======================= HELPER FUNCTIONS =====================

		void sendData(datagram * datagram);
		void getMotorData();
		void readStatus();
		void sgStatus();
		void buttonStatus();
		void switchReference(bool rightIsREFR);


		//================= ENABLE AND DISABLE FUNCTIONS ===============

		bool csEnable(); 								// enable chip select pin, call before sending datagram
		bool csDisable(); 								// disable chip select pin, latch
		bool powerEnable(); 							// enable power to this motor
		bool powerDisable(); 							// disable power to this motor
		bool switchActiveEnable(bool fw, bool bw);		// enables active high or low for the switches


		//=================== READ AND WRITE FUNCTIONS ================

		void setPowerLevel(unsigned long holdPower, unsigned long runPower); 	//32bit binary
		void setVelocity(unsigned long velocity); 								//32bit binary
		void setAcceleration(unsigned long acceleration); 						//32bit binary
		void setDeceleration(unsigned long acceleration); 						//32bit binary
		void setXtarget(unsigned long xtarget); 								//32bit binary
		void setXactual(unsigned long xactual); 								//32bit binary
		void setRampMode(unsigned long rampMode); 								//32bit binary
		void setChopConf(int resolution);

		int getMotorID();
		int getResolution();
		unsigned long getPowerLevel();
		unsigned long getVelocity();
		unsigned long getAcceleration();
		unsigned long getDeceleration();
		unsigned long getXtarget();
		unsigned long getXactual();
		unsigned long getHomeXtarget();
		unsigned long getRampMode();
		bool getIsForward();
		bool getIsPositionMode();
		bool getIsHomed();


		//===================== MOVEMENT FUNCTIONS ==================

		bool goPos(unsigned long position); 				// brings the motor back to its home position
		bool setHome(); 									// sets the home position using the right hand switch
		bool stop();
		// bool movement(bool direction, bool type, unsigned long speed, unsigned long steps);
		void swapDirection(bool swapDirection, bool swapSwitch);
		void constForward(unsigned long velocity);							// moves the motor forward constantly
		void constReverse(unsigned long velocity);							// moves the motor backwards constantly
		void forward(unsigned long stepsForward, unsigned long velocity); 	// push forward at the specified velocity
		void reverse(unsigned long stepsBackward, unsigned long velocity);	// moves motor in reverse direction

	private:

		int _csPin;
		int _enablePin;
		int _microSteps;
		int _homeCaseNum;
		int _resolutionNum;

		datagram _outputDatagram;

		bool _isHomed;
		bool _checkBit(datagram * dg, int targetBit);
};

#endif