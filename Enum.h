enum : uint8_t {
    // 1-9 reserved for variables
    ack                     = 1,
    err                     = 2,
    // 10-19 reserved for requesting values/proccesses
    REQUEST_MOTOR_STATUS    = 10,
    REQUEST_SG_STATUS       = 11,
    REQUEST_POS_NO_MOVE     = 12,
    // 20-29 reserved for getting values
    GET_ADCBITS             = 20,
    GET_ADCREFVOLT          = 21,
    GET_XACTUAL             = 22,
    GET_VELOCITY            = 23,
    GET_ACCELERATION        = 24,
    GET_DECELERATION        = 25,
    GET_POWER               = 26,
    // 30-39 reserved for setting values
    SET_CONST_FW            = 30, //move forever -careful. 
    SET_CONST_BW            = 31, //move forever -careful
    SET_MOVE_POS            = 32, //move to absolute pos
    SET_MOVE_FW             = 33, //move x steps (1 rotation ~ 51000) (command, speed, # steps)
    SET_MOVE_BW             = 34, //move x steps
    SET_VELOCITY            = 35,
    SET_ACCELERATION        = 36,
    SET_DECELERATION        = 37,
    SET_POWER               = 38,
    SET_DIRECTION           = 39,
    // 40-49 is reserved for additional commands
    JS_ENABLE               = 40, //joystick
    JS_DISABLE              = 41, //joystick
    MOTOR_STOP              = 42, //emergency stop
    MOTOR_HOME              = 43, //DONT USE THIS
    SEEK                    = 44, //DONT USE THIS
    RESOLUTION              = 45,
    ACTIVESETTINGS          = 46,
    PCPING                  = 49,
};