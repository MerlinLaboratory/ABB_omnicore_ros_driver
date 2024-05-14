# This message represents the state of the Omnicore Controller

string state_machine_state

string IDLE          = "IDLE"
string INITIALISE    = "INITIALISE"
string EGM_ROUTINE   = "EGM_ROUTINE"
string RAPID_ROUTINE = "RAPID_ROUTINE"
string UNKNOWN       = "UNKNOWN"

string egm_action_state

string EGM_ACTION_UNKNOWN = "EGM_ACTION_UNKNOWN"
string EGM_ACTION_RUN_JOINT = "EGM_ACTION_RUN_JOINT"
string EGM_ACTION_RUN_POSE = "EGM_ACTION_RUN_POSE"
string EGM_ACTION_STOP = "EGM_ACTION_STOP"
string EGM_ACTION_START_STREAMING = "EGM_ACTION_START_STREAMING"
string EGM_ACTION_STOP_STREAMING = "EGM_ACTION_STOP_STREAMING"

string controller_state

string EGM_MODE        = "EGM_MODE"
string FREE_DRIVE_MODE = "FREE_DRIVE_MODE"

byte digital_inputs_state
int8[4] robot_configuration  # This data represents the robot configuration (confdata). See ABB RAPID manual for further information.