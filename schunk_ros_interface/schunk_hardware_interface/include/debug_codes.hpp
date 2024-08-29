#ifndef SCHUNK_CODES_H
#define SCHUNK_CODES_H

#include <iostream>
#include <map>

// ---------------------------------------------------------------------------- //
// ------------------------------- WARNING CODES ------------------------------ //
// ---------------------------------------------------------------------------- //
const uint8_t NO_WARNINGS = 0x0;
const uint8_t WRN_LGC_TEMP_LO = 0x90;
const uint8_t WRN_LGC_TEMP_HI = 0x91;
const uint8_t WRN_MOT_TEMP_LO = 0x92;
const uint8_t WRN_MOT_TEMP_HI = 0x93;
const uint8_t WRN_NOT_FEASIBLE = 0x94;
const uint8_t WRN_POS_LIMIT = 0x95;
const uint8_t WRN_LGC_VOLT_LO = 0x96;
const uint8_t WRN_LGC_VOLT_HI = 0x97;
const uint8_t WRN_MOT_VOLT_LO = 0x98;
const uint8_t WRN_MOT_VOLT_HI = 0x99;

const std::map<uint8_t, std::string> mapper_warning = {
    {NO_WARNINGS, ""},
    {WRN_LGC_TEMP_LO, "The logic temperature measured is too low"},
    {WRN_LGC_TEMP_HI, "The logic temperature measured is too high"},
    {WRN_MOT_TEMP_LO, "The motor temperature measured is too low"},
    {WRN_MOT_TEMP_HI, "The motor temperature measured is too high"},
    {WRN_NOT_FEASIBLE, "The control command sent to the module is not feasible"},
    {WRN_POS_LIMIT, "The movement during jog mode was automatically terminated by reaching the minimum or maximum position"},
    {WRN_LGC_VOLT_LO, "The logic supply voltage measured is too low"},
    {WRN_LGC_VOLT_HI, "The logic supply voltage measured is too high"},
    {WRN_MOT_VOLT_LO, "The motor supply voltage measured is too low"},
    {WRN_MOT_VOLT_HI, "The motor supply voltage measured is too high"}
};

// ---------------------------------------------------------------------------- //
// -------------------------------- ERROR CODES ------------------------------- //
// ---------------------------------------------------------------------------- //

const uint8_t NO_ERRORS = 0x0;
const uint8_t ERR_BT_FAILED = 0x28;
const uint8_t ERR_MOT_TEMP_LO = 0x6C;
const uint8_t ERR_MOT_TEMP_HI = 0x6D;
const uint8_t ERR_LGC_TEMP_LO = 0x70;
const uint8_t ERR_LGC_TEMP_HI = 0x71;
const uint8_t ERR_LGC_VOLT_LO = 0x72;
const uint8_t ERR_LGC_VOLT_HI = 0x73;
const uint8_t ERR_MOT_VOLT_LO = 0x74;
const uint8_t ERR_MOT_VOLT_HI = 0x75;
const uint8_t ERROR_SOFT_LOW = 0xD5;
const uint8_t ERR_SOFT_HIGH = 0xD6;
const uint8_t ERR_FAST_STOP = 0xD9;
const uint8_t ERR_TOO_FAST = 0xE4;
const uint8_t ERR_COMM_LOST = 0xEF;
const uint8_t ERR_MOV_ABORT_TO = 0xF1;
const uint8_t ERR_MOVE_BLOCKED = 0xF4;

const std::map<uint8_t, std::string> mapper_error = {
    {NO_ERRORS, ""},
    {ERR_BT_FAILED, "The brake test was performed unsuccessfully"},
    {ERR_BT_FAILED, "The brake test was performed unsuccessfully"},
    {ERR_MOT_TEMP_LO, "The motor temperature measured is too low"},
    {ERR_MOT_TEMP_HI, "The motor temperature measured is too high"},
    {ERR_LGC_TEMP_LO, "The logic temperature measured is too low"},
    {ERR_LGC_TEMP_HI, "The logic temperature measured is too high"},
    {ERR_LGC_VOLT_LO, "The logic supply voltage measured is too low"},
    {ERR_LGC_VOLT_HI, "The logic supply voltage measured is too high"},
    {ERR_MOT_VOLT_LO, "The motor supply voltage measured is too low"},
    {ERR_MOT_VOLT_HI, "The motor supply voltage measured is too high"},
    {ERROR_SOFT_LOW, "The lower software limit has been reached or exceeded"},
    {ERR_SOFT_HIGH, "The upper software limit has been reached or exceeded"},
    {ERR_FAST_STOP, "A fast stop was triggered"},
    {ERR_TOO_FAST, "The maximum permissible speed was exceeded by a factor of 1.2"},
    {ERR_COMM_LOST, "The communication link between the module and the receiver has been interrupted"},
    {ERR_MOV_ABORT_TO, "Positioning could not be performed within the expected period of time"},
    {ERR_MOVE_BLOCKED, "The drive was blocked"}
};
#endif