uint64 timestamp
uint64 timestamp_sample

uint32 session_id
uint32 peer_session_id

uint8 ROLE_DISABLED = 0
uint8 ROLE_MASTER = 1
uint8 ROLE_SLAVE = 2
uint8 role

uint8 STATE_DOWN = 0
uint8 STATE_DISCOVERED = 1
uint8 STATE_HANDSHAKE_PENDING = 2
uint8 STATE_READY = 3
uint8 STATE_PAIR_ENABLE_PENDING = 4
uint8 STATE_PAIR_CONTROL_READY = 5
uint8 STATE_PAIR_ARM_PENDING = 6
uint8 STATE_PAIR_ARMED_GROUND = 7
uint8 STATE_PAIR_ACTIVE = 8
uint8 STATE_PAIR_LOST = 9
uint8 link_state

uint16 peer_health_flags
uint16 peer_sensor_status_flags

bool peer_heartbeat_fresh
bool peer_sensor_fresh
bool peer_control_fresh

bool follower_active
bool pair_active
bool pair_lost
bool pair_reattach_latched
bool pair_mode_selected
bool local_airborne_latched
bool local_arm_authorized
bool control_tx_allowed
bool link_test_active

uint32 rx_frames
uint32 tx_frames
uint32 crc_errors
uint32 length_errors
uint32 version_errors
uint32 eof_errors
uint32 resyncs
uint32 partial_timeouts
uint32 session_errors
uint32 unknown_messages
uint32 reliable_retries
uint32 duplicate_events
uint32 write_failures
uint32 buffer_overflows
