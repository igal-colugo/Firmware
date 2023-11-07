uint64 timestamp # time since system start (microseconds)
uint32 error
float32 pitch_offset
float32 roll_offset
float32 reserved_2
uint32 version
int16 temperature
uint8 nav_state
uint8 nav_output
uint8 gps_jamming_state
uint8 active_sensor
uint8 active_rec
uint8 active_db_rec
uint8 active_nav
uint8 active_slam
uint8 battery
uint8 sys_state
uint8 reserved
uint8 map_warning
uint8 calibration_state

uint8 ASIO_ERROR_GENERAL_CRITICAL = 1
uint8 ASIO_ERROR_GENERAL_WARNNING = 2
uint8 ASIO_ERROR_TELEM = 4
uint8 ASIO_ERROR_DAY_SENSOR = 8
uint8 ASIO_ERROR_NIGHT_SENSOR = 16
uint8 ASIO_ERROR_ANCHOR = 32
uint8 ASIO_ERROR_TEMPERATURE = 64
uint8 ASIO_ERROR_LOGGING = 128
