# This message represents the state of the Omnicore Controller

uint8 state_machine_state

uint8 IDLE          = 0
uint8 INITIALISE    = 1
uint8 EGM_ROUTINE   = 2
uint8 RAPID_ROUTINE = 3
uint8 UNKNOWN       = 4

uint8 controller_state

uint8 EGM_MODE        = 0
uint8 FREE_DRIVE_MODE = 1

byte digital_inputs_state
int8[4] robot_configuration  # This data represents the robot configuration (confdata). See ABB RAPID manual for further information.