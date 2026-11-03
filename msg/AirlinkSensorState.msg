# Decoded AIRLINK v2 SENSOR_STATE sample.
#
# This preserves the slave's 32-bit sample time and the local frame arrival
# time so the composite preprocessor can perform clock recovery itself.

uint64 timestamp               # host arrival, or zero when injected by CLI
uint64 timestamp_sample        # decoded slave 32-bit time, widened for logging
uint32 source_session
uint16 sequence
uint16 status_flags
uint32 imu_sample_time_us
uint32 attitude_sample_time_us
int16[3] accelerometer_raw
int16[3] gyro_raw
int16[4] attitude_q_raw
int16[3] angular_velocity_raw
int16[2] esc_rpm
uint16 battery_voltage_mv

uint8 ORB_QUEUE_LENGTH = 32
