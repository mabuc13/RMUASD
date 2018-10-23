float32[3] RPY
string main_flightmode
string sub_flightmode

uint32 msg_sent_gcs
uint32 msg_received_gcs
uint32 msg_dropped_gcs
uint32 msg_dropped_uas

string status_text
string severity_level

uint16 active_waypoint_index	#On drone index
uint16 mission_length		#On drone mission length

bool manual_input
bool HIL_simulation
bool stabilized_mode
bool guided_mode
bool auto_mode
bool test_mode
bool custom_mode

string autopilot
string mav_state
string mav_type

GPS origin_local_NED
GPS home
GPS offboard_setpoint

time last_heard #last heard from drone x seconds ago
uint64 up_time #Time since boot of drone

uint32 drone_id
