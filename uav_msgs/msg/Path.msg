# path modes
uint8 MODE_ORBIT = 0
uint8 MODE_LINE = 1

# orbit directions
uint8 DIR_CW = 0
uint8 DIR_CCW = 1

Header header
uint8 mode
bool path_changed
float64 airspeed
# for lines
geometry_msgs/Point line_origin
geometry_msgs/Point line_direction
# for orbits
geometry_msgs/Point orbit_center
float32 orbit_radius
uint8 orbit_direction
