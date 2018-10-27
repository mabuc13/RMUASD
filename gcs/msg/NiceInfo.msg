uint32 drone_id
time last_heard             # last heard from drone x seconds ago
uint32 up_time              # time since boot of drone

string drone_handler_state  # what is the drone handler doing?

string autopilot
string mav_state
string mav_type
string main_flightmode
string sub_flightmode

uint32 msg_sent_gcs
uint32 msg_received_gcs
uint32 msg_dropped_gcs
uint32 msg_dropped_uas

#string status_text         # should be in a message on its own
#string severity_level      # runs completely asynchronously to the rest

uint16 active_waypoint_idx	# On drone index
uint16 active_mission_len	# On drone mission length

# bool armed
bool manual_input
bool hil_simulation
bool stabilized_mode
bool guided_mode
bool auto_mode
bool test_mode
bool custom_mode

float32[3] RPY
float32 climb_rate
uint16 throttle

GPS home
# GPS origin_local_NED      # ignore offboard stuff for now
# GPS offboard_setpoint     # not immediately necessary 
