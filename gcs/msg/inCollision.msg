uint32 drone_id

int32 plan_index1
int32 plan_index2

GPS start
GPS end

uint8 zone_type

uint8 normal_zone = 1 #Future problem
uint8 inside_zone = 2 #We are currently inside a zone
uint8 landing_zone = 3 #There is a zone on out landing spot

uint64 valid_to 
