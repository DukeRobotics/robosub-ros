std_msgs/Header header

# Max/min x- and y-coordinates for the bounding box
float64 xmin
float64 ymin
float64 xmax
float64 ymax

# Object coordinates
geometry_msgs/Point coords

int32 height
int32 width

float64 distance

string label
float64 score
bool sonar # True = using sonar