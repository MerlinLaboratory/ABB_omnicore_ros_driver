# This message represents the state of the Omnicore Controller

uint8 controller_state

uint8 EGM_MODE        = 0
uint8 FREE_DRIVE_MODE = 1

byte digital_inputs_state
int8[4] robot_configuration  # This data represents the robot configuration (confdata). See ABB RAPID manual for further information.