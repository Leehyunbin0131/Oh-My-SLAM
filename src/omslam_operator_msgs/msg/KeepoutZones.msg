# Authoritative rectangles in the current map frame. A new map gets a new session.
std_msgs/Header header
string session_id
uint64 revision
geometry_msgs/Polygon[] polygons
