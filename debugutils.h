/*
* Utility functions to help debugging running code.
*/

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#include "arduino.h"

#define DEBUG_PRINT(str) \
   Serial.print(millis()); \
   Serial.print(": "); \
   Serial.print(__FUNCTION__); \
   Serial.print("() :        "); \
   Serial.println(str);

#define DEBUG_PRINT_FULL(str) \
   Serial.print(millis()); \
   Serial.print(": "); \
   Serial.print(__FUNCTION__); \
   Serial.print("() in "); \
   Serial.print(__FILE__); \
   Serial.print(':'); \
   Serial.print(__LINE__); \
   Serial.print(' '); \
   Serial.println(str);


#endif