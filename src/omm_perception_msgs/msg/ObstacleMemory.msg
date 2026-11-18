# Complete camera-only occupancy snapshot, not semantic object tracks.
# The transport stamp is refreshed; observation_stamp retains sensor freshness.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
bool sensor_healthy
bool geometry_healthy
bool memory_healthy
string status
float32 voxel_size
geometry_msgs/Point32[] points
geometry_msgs/Point32[] archived_points
float32 archive_resolution
uint32 archived_voxels
uint32 dynamic_tracks
uint32 visible_voxels
uint32 occluded_voxels
uint32 outside_fov_voxels
uint32 uncertain_voxels
uint64 cleared_voxels_total
