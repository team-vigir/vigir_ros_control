# Define status codes for footstep execution
uint32 NO_ERROR                         =    0
uint32 MODE_ACCEPTED                    =    1
uint32 MODE_REJECTED                    =    2
uint32 MODE_PREEMPTED                   =    4
uint32 MODE_PROCESSING                  =    8
uint32 MODE_DUPLICATE                   =   16
uint32 EXTERNAL_CHANGE                  =   32

# Error codes
uint32 ERR_UNKNOWN                      =  256
uint32 ERR_CONTROLLER_NOT_READY         =  512
uint32 ERR_INVALID_BEHAVIOR_MODE        = 1024
uint32 ERR_INVALID_TRANSITION           = 2048
uint32 ERR_PRIOR_COMMAND_IN_PROGRESS    = 4096

# Message format
Header header
string requested_control_mode
string current_control_mode
uint32 status



