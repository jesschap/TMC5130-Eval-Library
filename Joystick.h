#ifndef Joystick_H

/* ========================================================================
   $File: MotorControl.h$
   $Date: $
   $Revision: $
   $Creator:  $
   $Email:  $
   $Notice: $
   ======================================================================== */

#define Joystick_H
#include "Definitions.h"
#include "Arduino.h"

class Joystick {

	public:

		// Object constructor 
		Joystick(double yrange, double ythreshold, int ypin, double xrange, double xthreshold, int xpin);
		Joystick();

		void set(double yrange, double ythreshold, int ypin, double xrange, double xthreshold, int xpin);
		void begin();

		double xAxisControl();
		double yAxisControl();

		bool getJoystickStop();

	private:
		double _yrange;
		double _xrange;
		double _alpha;
		double _beta;

		int _xpin;
		int _ypin;

		double _xthreshold;
		double _ythreshold;

		bool _joystickStop;

		double _readAxis(int pin, double threshold);
};
#endif
