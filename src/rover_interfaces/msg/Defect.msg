# Defect detection message
# Published by perception node, consumed by defect_logger

std_msgs/Header header

# Detection metadata
string defect_type          # "rust", "leak", "aruco_XX"
float32 confidence          # 0.0 to 1.0

# Pixel coordinates (bounding box center)
int32 pixel_u
int32 pixel_v

# Map-frame coordinates (from pose_projector)
geometry_msgs/Point map_position

# Image crop file path (filled by defect_logger)
string image_path
