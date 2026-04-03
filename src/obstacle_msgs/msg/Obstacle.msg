# Single obstacle detected by camera/LiDAR fusion
std_msgs/Header header

float32 x          # Position in robot frame (meters)
float32 y
float32 z

float32 width      # Bounding box dimensions (meters)
float32 height
float32 depth

float32 bbox_x     # Image bounding box (pixels)
float32 bbox_y
float32 bbox_w
float32 bbox_h

string  label      # Color label / class from HSV segmentation
float32 confidence # Detection confidence [0.0, 1.0]
bool    is_dynamic # True if identified as moving obstacle
