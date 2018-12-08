uint32 drone_id

GPS position
GPS next_waypoint
float32 ground_speed
float32 ground_speed_setpoint
float32 heading
float32 battery_SOC
float32 battery_voltage
float32 relative_alt
float32 absolute_alt

uint64 GPS_timestamp    # timestamp of when GPS info was read on the drone
uint8 status            # one of the constants below
uint16 path_id
uint16 mission_index    # index of the current complete mission (dock to dock)
uint16 mission_length   # length of the complete mission
bool armed              # is this necessary?



uint8 Run = 1
uint8 Land = 2
uint8 holding = 3

