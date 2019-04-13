# waypoint following modes
uint8 MODE_LINE = 0
uint8 MODE_FILLET = 1
uint8 MODE_DUBINS = 2

bool use_default_waypoints
uint8 mode
bool map_changed
float64 city_width
int32 num_blocks
float64 street_width
Building[] buildings
