# waypoint following modes
uint8 MODE_LINE = 0
uint8 MODE_FILLET = 1
uint8 MODE_DUBINS = 2

Header header
bool waypoints_changed
bool manager_requests_waypoints
bool wrap_waypoints
uint8 mode
int32 num_waypoints
Waypoint[] waypoints
