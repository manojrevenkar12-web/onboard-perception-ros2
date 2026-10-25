# Published by the camera for every delivered frame, before any processing.
std_msgs/Header header        # stamp = exposure start on CLOCK_MONOTONIC
uint32 sequence               # continuous across reconnects: gaps are frames that never arrived
uint32 driver_sequence        # V4L2 buffer sequence (restarts at every STREAMON)
uint32 dropped_before         # frames the sensor exposed but the driver had no buffer for
uint8 STAMP_START_OF_EXPOSURE = 0
uint8 STAMP_END_OF_FRAME = 1
uint8 STAMP_DEQUEUE = 2
uint8 stamp_source
uint32 queue_depth            # buffers the driver held when this one was dequeued
