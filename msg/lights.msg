## Used to command an actuation in the lights, which is mapped to a specific output in the control allocation module

uint64 timestamp

int8 command		# Commanded state for the lights
float32 on
float32 off
int8 COMMAND_ON = 1
int8 COMMAND_OFF = 0
