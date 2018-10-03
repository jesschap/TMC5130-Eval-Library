#ifndef CONTROL_H

/* ========================================================================
   $File: Control.h$
   $Date: $
   $Revision: $
   $Creator:  $
   $Email:  $
   $Notice: $
   ======================================================================== */

#define CONTROL_H
#include "Definitions.h"
#include "Joystick.h"
#include "MotorControl.h"

class CombinedControl {
	public:

		// Class Function
      CombinedControl();
      void begin();

      //================= JOYSTICK CONTROL FUNCTIONS ===============

		void enableJoystick();
      void disableJoystick();

      //=================== MOTOR CONTROL FUNCTIONS =================

      //===== MOVE FUNCTIONS =====

      void goPos(unsigned long position);                                  // brings the motor back to its home position
      void setHome();                                                      // sets the home position using the right hand switch
      void forward(unsigned long stepsForward, unsigned long velocity);    // push forward at the specified velocity
      void reverse(unsigned long stepsBackward, unsigned long velocity);   // moves motor in reverse direction
      void constForward(unsigned long velocity);                           // moves the motor forward constantly
      void constReverse(unsigned long velocity);                           // moves the motor backwards constantly
      void stop();                                                         // stops the motor when it is in continuous movement
      bool seek(bool goForward);                                           // goes until a switch as defined by goForward is pressed

      //===== INFO FUNCTIONS =====

      void status(int * statusBits);                                       // returns the status bits of the motor
      unsigned long sgStatus();                                            // returns the stallguard status info

      bool standstill();                                                   // checks if the motor is at a standstill

      double getXactual();                                                 // returns the position of the motor
      unsigned long getVelocity();                                         // returns the vmax speed
      unsigned long getAcceleration();                                     // returns the amax acceleration
      unsigned long getDeceleration();                                     // returns the dmax deceleration
      unsigned long getPower();                                            // returns the running power

      //===== SET FUNCTIONS =====

      void setVelocity(unsigned long velocity);                            // sets the vmax velocity
      void setAcceleration(unsigned long acceleration);                    // sets the amax acceleration
      void setDeceleration(unsigned long deceleration);                    // sets the dmax deceleration
      void setPower(unsigned long holdPower, unsigned long runPower);      // sets the hold and run power
      void setXtarget(unsigned long position);                             // sets the target position (will move in mode 0)
      void setResolution(int resolution);                                  // sets the step resolution of the motor

      void changePosNoMove(unsigned long position);                        // changes the actual position value without moving
      void setDirections(bool forwardDirection, bool forwardSwitch);       // sets the dir of switches and which is the forward dir
      void switchActiveEnable(bool fw, bool bw);                           // allows the user to change switches active high or low

   private:
      MotorControl motor;
      Joystick joystick;

      int _seekStep;
      int _stepResolution;
      int _resolutionNum;
      double _lastVel;
      unsigned long _lastRead;

      void _setJS(double velocity);
      bool _timer(unsigned long lastReadTime);
};

#endif