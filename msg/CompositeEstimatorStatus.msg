uint64 timestamp

uint8 STATE_INITIALIZING = 0
uint8 STATE_READY = 1
uint8 STATE_DEGRADED = 2
uint8 STATE_FAULT = 3
uint8 state

uint16 FAULT_NONE = 0
uint16 FAULT_SESSION = 1
uint16 FAULT_SENSOR_STALE = 2
uint16 FAULT_CLOCK = 3
uint16 FAULT_INTERPOLATION = 4
uint16 FAULT_NUMERICAL = 5
uint16 FAULT_ATTITUDE_CONSISTENCY = 6
uint16 FAULT_EKF = 7
uint16 fault_reason
uint32 fault_epoch

int32 clock_offset_us
int32 clock_residual_us
uint32 source_gap_us
uint32 frame_count
uint32 rejected_frames

float32 attitude_residual_a_deg
float32 attitude_residual_b_deg
float32 ab_consistency_deg
float32 tilt_residual_a_deg
float32 tilt_residual_b_deg
float32 tilt_consistency_deg
float32 yaw_consistency_deg
float32 lever_correction_a
float32 lever_correction_b
float32 yaw_innovation_a
float32 yaw_innovation_b

float32[4] q_ab
float32 yaw_reference_rad
uint32 yaw_reference_count
bool clock_ready

uint8 REJECT_NONE = 0
uint8 REJECT_SENSOR_FLAGS = 1
uint8 REJECT_SOURCE_STALE = 2
uint8 REJECT_QUATERNION = 3
uint8 REJECT_SEQUENCE = 4
uint8 REJECT_CLOCK = 5
uint8 REJECT_ATTITUDE_TIME = 6
uint8 REJECT_LOCAL_IMU = 7
uint8 REJECT_ATTITUDE_INTERPOLATION = 8
uint8 REJECT_COMPOSITE_SAMPLE = 9
uint8 reject_reason

uint32 clock_samples
uint32 last_raw_sample_us
uint64 last_mapped_sample_us
uint32 last_attitude_raw_sample_us
uint64 last_mapped_attitude_us
int32 last_attitude_delta_us
uint8 selected_imu_instance
uint8 local_imu_samples
uint8 local_attitude_samples
uint64 local_imu_oldest_us
uint64 local_imu_newest_us
uint64 local_attitude_oldest_us
uint64 local_attitude_newest_us

# Current comp_attitude mode; independent of the aircraft EKF2 instances.
bool enabled
bool attitude_initialized
bool peer_connected
float32 weight_a
float32 weight_b
