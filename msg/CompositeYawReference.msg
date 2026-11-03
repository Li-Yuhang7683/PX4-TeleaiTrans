uint64 timestamp
uint64 timestamp_sample       # low-passed composite yaw generation time

float32 yaw_rad
float32 yaw_sigma_rad
uint8 reset_counter

uint8 QUALITY_INVALID = 0
uint8 QUALITY_DEGRADED = 1
uint8 QUALITY_NORMAL = 2
uint8 quality

uint16 FLAG_VALID = 1
uint16 FLAG_DEGRADED = 2
uint16 flags
