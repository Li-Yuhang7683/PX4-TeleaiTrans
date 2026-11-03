uint64 timestamp
uint64 timestamp_sample

float32[4] q                  # Hamilton w,x,y,z; C FRD to common NED
float32[3] angular_velocity   # C FRD, rad/s
float32[3] angular_acceleration
float32[3] specific_force     # Virtual IMU specific force in C FRD, m/s^2
float32 yaw_sigma_rad

uint8 reset_counter
uint32 source_age_a_us
uint32 source_age_b_us

uint16 HEALTH_DEGRADED = 1
uint16 HEALTH_CLOCK_READY = 2
uint16 HEALTH_SOURCE_A_FRESH = 4
uint16 HEALTH_SOURCE_B_FRESH = 8
uint16 HEALTH_ATTITUDE_CONSISTENT = 16
uint16 HEALTH_EKF_VALID = 32
uint16 health_flags
bool valid
