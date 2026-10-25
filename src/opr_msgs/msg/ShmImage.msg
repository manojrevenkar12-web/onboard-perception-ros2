# A handle to an image in a shared-memory segment (opr_shm). Published on <image topic>/shm.
# The pixels stay in the segment; a reader maps it, checks `seq` and reads in place.
std_msgs/Header header
string segment          # POSIX shm name, e.g. /opr_camera_image_raw_1a2b
uint64 epoch            # random per segment creation: a restarted publisher never matches an old handle
string boot_id          # /proc/sys/kernel/random/boot_id of the publisher's kernel
uint32 slot
uint64 seq              # slot sequence when committed (even); changes when the slot is reused
uint32 height
uint32 width
string encoding
uint32 step
uint32 data_bytes
