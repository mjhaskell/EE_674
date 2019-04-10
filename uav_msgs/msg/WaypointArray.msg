# waypoint following modes
uint8 MODE_LINE
uint8 MODE_FILLET
uint8 MODE_DUBINS

Header header
bool waypoints_changed
bool manager_requests_waypoints
bool wrap_waypoints
uint8 mode
int32 num_waypoints
Waypoint[] waypoints
