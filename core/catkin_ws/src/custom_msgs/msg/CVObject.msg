std_msgs/Header header

# Max/min x- and y-coordinates for the bounding box
float64 xmin
float64 ymin
float64 xmax
float64 ymax

int32 height
int32 width

string label
float64 score

# Stereo data (not present for all frames)
bool stereo_enabled

float64 depth
float64 distance

float64 angle_horiz
float64 angle_vert