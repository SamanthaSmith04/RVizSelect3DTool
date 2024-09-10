# Selection3D.msg

# The points of the selection region polygon
geometry_msgs/Point[] points

# True if the tool was used to select, false if used to deselect
bool is_selecting

# Camera pose at the time of selection
geometry_msgs/Point camera_pos