# One result per successfully decoded camera frame.
# detected is true only when the selected marker has a valid pose.
std_msgs/Header header
bool detected
# Camera optical frame pose. Ignore when detected is false.
geometry_msgs/Pose pose
