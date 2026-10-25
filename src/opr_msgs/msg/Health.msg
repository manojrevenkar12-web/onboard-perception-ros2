# One summary of the perception chain for the operator and the autonomy stack.
uint8 OK = 0
uint8 WARN = 1
uint8 ERROR = 2
uint8 STALE = 3
std_msgs/Header header
uint8 level
string worst          # component responsible for the level
string message
float32 track_age_ms  # now minus the exposure stamp of the newest TrackArray
