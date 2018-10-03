#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// Debug Options (set to #ifdef)
// #define DEBUG_MOTOR		0
// #define DEBUG_HOME		0
// #define DEBUG_DIR		1
// #define DEBUG_COM		0


// Define Pins (Set for Ardunio Mega2560)
#define STOP 			3
#define CLOCKPIN		11
#define ENABLEPIN		17
#define MISOPIN 		50
#define MOSIPIN 		51
#define SCKPIN 			52
#define CSPIN			25

#define XAXIS			A0
#define YAXIS			A1
#define SENSOR1PIN 		A2

// Additional constants
#define STANDVELOCITY 	100000
#define STATUS_SIZE		25
#define MOTOR_STEPS		200

// Define Registers for TMC5130
#define ADDRESS_GCONF      	0x00
#define ADDRESS_GSTAT      	0x01
#define ADDRESS_IFCNT      	0x02
#define ADDRESS_SLAVECONF  	0x03
#define ADDRESS_INP_OUT    	0x04
#define ADDRESS_X_COMPARE  	0x05
#define ADDRESS_IHOLD_IRUN 	0x10
#define ADDRESS_TZEROWAIT  	0x11
#define ADDRESS_TSTEP  		0x12
#define ADDRESS_TPWMTHRS  	0x13
#define ADDRESS_TCOOLTHRS  	0x14
#define ADDRESS_THIGH      	0x15

#define ADDRESS_RAMPMODE   	0x20
#define ADDRESS_XACTUAL    	0x21
#define ADDRESS_VACTUAL    	0x22
#define ADDRESS_VSTART     	0x23
#define ADDRESS_A1         	0x24
#define ADDRESS_V1         	0x25
#define ADDRESS_AMAX       	0x26
#define ADDRESS_VMAX       	0x27
#define ADDRESS_DMAX       	0x28
#define ADDRESS_D1         	0x2A
#define ADDRESS_VSTOP      	0x2B
#define ADDRESS_TZEROCROSS 	0x2C
#define ADDRESS_XTARGET    	0x2D

#define ADDRESS_VDCMIN     	0x33
#define ADDRESS_SWMODE     	0x34
#define ADDRESS_RAMPSTAT   	0x35
#define ADDRESS_XLATCH     	0x36
#define ADDRESS_ENCMODE    	0x38
#define ADDRESS_XENC       	0x39
#define ADDRESS_ENC_CONST  	0x3A
#define ADDRESS_ENC_STATUS 	0x3B
#define ADDRESS_ENC_LATCH  	0x3C

#define ADDRESS_MSLUT0     	0x60
#define ADDRESS_MSLUT1     	0x61
#define ADDRESS_MSLUT2     	0x62
#define ADDRESS_MSLUT3     	0x63
#define ADDRESS_MSLUT4     	0x64
#define ADDRESS_MSLUT5     	0x65
#define ADDRESS_MSLUT6     	0x66
#define ADDRESS_MSLUT7     	0x67
#define ADDRESS_MSLUTSEL   	0x68
#define ADDRESS_MSLUTSTART 	0x69
#define ADDRESS_MSCNT      	0x6A
#define ADDRESS_MSCURACT   	0x6B
#define ADDRESS_CHOPCONF   	0x6C
#define ADDRESS_COOLCONF   	0x6D
#define ADDRESS_DCCTRL     	0x6E
#define ADDRESS_DRVSTATUS  	0x6F
#define ADDRESS_PWMCONF  	0x70
#define ADDRESS_PWMSTATUS 	0x71
#define ADDRESS_EN_CTRL 	0x72
#define ADDRESS_LOST_STEPS 	0x73

// Register ADDRESS_RAMPMODE
#define ADDRESS_MODE_POSITION   0
#define ADDRESS_MODE_VELPOS     1
#define ADDRESS_MODE_VELNEG     2
#define ADDRESS_MODE_HOLD       3

// Register ADDRESS_SWMODE
#define ADDRESS_SW_STOPL_ENABLE   0x0001
#define ADDRESS_SW_STOPR_ENABLE   0x0002
#define ADDRESS_SW STOPL_POLARITY 0x0004
#define ADDRESS_SW_STOPR_POLARITY 0x0008
#define ADDRESS_SW_SWAP_LR        0x0010
#define ADDRESS_SW_LATCH_L_ACT    0x0020
#define ADDRESS_SW_LATCH_L_INACT  0x0040
#define ADDRESS_SW_LATCH_R_ACT    0x0080
#define ADDRESS_SW_LATCH_R_INACT  0x0100
#define ADDRESS_SW_LATCH_ENC      0x0200
#define ADDRESS_SW_SG_STOP        0x0400
#define ADDRESS_SW_SOFTSTOP       0x0800


// Register ADDRESS_RAMPSTAT
#define ADDRESS_RS_STOPL          0x0001
#define ADDRESS_RS_STOPR          0x0002
#define ADDRESS_RS_LATCHL         0x0004
#define ADDRESS_RS_LATCHR         0x0008
#define ADDRESS_RS_EV_STOPL       0x0010
#define ADDRESS_RS_EV_STOPR       0x0020
#define ADDRESS_RS_EV_STOP_SG     0x0040
#define ADDRESS_RS_EV_POSREACHED  0x0080
#define ADDRESS_RS_VELREACHED     0x0100
#define ADDRESS_RS_POSREACHED     0x0200
#define ADDRESS_RS_VZERO          0x0400
#define ADDRESS_RS_ZEROWAIT       0x0800
#define ADDRESS_RS_SECONDMOVE     0x1000
#define ADDRESS_RS_SG             0x2000

#endif



