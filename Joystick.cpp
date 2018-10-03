/* ========================================================================
   $File: Joystick.cpp$
   $Date: $
   $Revision: $
   $Creator:  $
   $Email:  $
   $Notice: $
   ======================================================================== */


#include "Joystick.h"

Joystick :: Joystick() {
	// controlled by input pin STOP
	_joystickStop = true;
}

Joystick :: Joystick(double yrange, double ythreshold, int ypin, double xrange, double xthreshold, int xpin) {

	pinMode(ypin, INPUT);
	pinMode(xpin, INPUT);

	_ypin 			= ypin;
	_xpin 			= xpin;

	_yrange 		= yrange;
	_xrange			= xrange;
	_ythreshold		= ythreshold;
	_xthreshold		= xthreshold;

	// controlled by input pin STOP
	_joystickStop	= true;
	
	_beta = - yrange;
	_alpha = (yrange - _beta) / 1023.0;
}

void Joystick :: set(double yrange, double ythreshold, int ypin, double xrange, double xthreshold, int xpin) {

	_ypin 			= ypin;
	_xpin 			= xpin;

	_yrange 		= yrange;
	_xrange			= xrange;
	_ythreshold		= ythreshold;
	_xthreshold		= xthreshold;
	
	_beta 			= - yrange;
	_alpha 			= (yrange - _beta) / 1023.0;
}

void Joystick :: begin() {
	pinMode(_ypin, INPUT);
	pinMode(_xpin, INPUT);
}

double Joystick :: xAxisControl() {
	double xVal = Joystick :: _readAxis(XAXIS, _xthreshold);
	return xVal;
}

double Joystick :: yAxisControl() {
	double yVal = Joystick :: _readAxis(YAXIS, _ythreshold);
	return yVal;
}

bool Joystick :: getJoystickStop() {
	return _joystickStop;
}

/* ======================================================================
 	Read axis will return the function converted to the measurement given
 	by the y_range or x_range. Note that for a yrange value, the function
 	will return values within (-y_range < val < +y_range).
====================================================================== */

double Joystick :: _readAxis(int pin, double threshold) {
	
	double reading = analogRead(pin);
	double speed = _alpha * reading + _beta;

	if (abs(speed) < threshold) {
		speed = 0.0;
	}

	#ifdef DEBUG_MOTOR
		Serial.print("Raw Reading: ");
		Serial.println(reading);
		Serial.print("Calibrated Reading: ");
		Serial.println(speed);
	#endif

	// delay(200);
	
	return speed;
}



