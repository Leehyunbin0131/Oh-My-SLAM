# Bounded metadata from the same completed snapshot as ObstacleMemory.
# Python control/UI consumers must not deserialize the full point arrays.
std_msgs/Header header
builtin_interfaces/Time observation_stamp
bool sensor_healthy
bool geometry_healthy
bool memory_healthy
string status
float32 voxel_size
float32 archive_resolution
uint32 active_points
uint32 archived_points
uint32 archived_voxels
uint32 dynamic_tracks
uint32 visible_voxels
uint32 occluded_voxels
uint32 outside_fov_voxels
uint32 uncertain_voxels
uint64 cleared_voxels_total
