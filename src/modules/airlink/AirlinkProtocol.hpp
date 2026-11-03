/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

/**
 * AIRLINK v2 wire contract. Version 1 frames remain parser-compatible for
 * diagnostics, but v2 handshakes require CompositeYawReference capability.
 *
 * This file intentionally uses explicit little-endian field serialization.
 * Do not cast it onto the wire or rely on structure padding.
 */
namespace airlink
{

static constexpr uint8_t kSof0{0xA5};
static constexpr uint8_t kSof1{0x5A};
static constexpr uint8_t kEof0{0x5A};
static constexpr uint8_t kEof1{0xA5};
static constexpr uint8_t kMinSupportedVersion{1};
static constexpr uint8_t kVersion{2};
static constexpr size_t kHeaderSize{16};
static constexpr size_t kCrcSize{4};
static constexpr size_t kEofSize{2};
static constexpr size_t kOverhead{kHeaderSize + kCrcSize + kEofSize};
static constexpr size_t kMaxPayload{512};
static constexpr size_t kMaxFrameSize{kOverhead + kMaxPayload};
static constexpr uint32_t kCrc32cPolynomial{0x82F63B78u};
static constexpr uint32_t kCrc32cCheck{0xE3069283u};
static constexpr uint64_t kSensorFreshTimeoutUs{10'000};
static constexpr uint64_t kPrecontrolHeartbeatTimeoutUs{100'000};
static constexpr uint64_t kReliableAckTimeoutUs{5'000};
static constexpr uint64_t kPairArmAckTimeoutUs{100'000};
static constexpr uint64_t kPairArmCommitTimeoutUs{100'000};
static constexpr uint64_t kPartialFrameTimeoutUs{5'000};

enum class FrameType : uint8_t {
	Data = 0x01,
	Command = 0x02,
};

enum class Role : uint8_t {
	Disabled = 0,
	Master = 0x01,
	Slave = 0x02,
};

enum class MessageId : uint8_t {
	SensorState = 0x01,
	YawReference = 0x02,
	ActuatorCommand = 0x01,
	Heartbeat = 0x02,
	ControlEnable = 0x03,
	ControlRelease = 0x04,
	EmergencyWithdraw = 0x05,
	Ack = 0x06,
	PairArm = 0x07,
	PairDisarm = 0x08,
	LinkHandshake = 0x09,
	LinkTestControl = 0x0A,
	LinkTestData = 0x0B,
};

enum FrameFlags : uint8_t {
	AckRequired = 1u << 0,
	Retry = 1u << 1,
	Response = 1u << 2,
	Urgent = 1u << 3,
	ReservedFrameFlags = 0xF0u,
};

enum SensorStatusFlags : uint16_t {
	AccelValid = 1u << 0,
	GyroValid = 1u << 1,
	AttitudeValid = 1u << 2,
	AngularVelocityValid = 1u << 3,
	EscUpperValid = 1u << 4,
	EscLowerValid = 1u << 5,
	BatteryValid = 1u << 6,
	SampleFresh = 1u << 7,
	LocalRcValid = 1u << 8,
	SafetyGateAllow = 1u << 9,
	AirlinkControlActive = 1u << 10,
	AccelClipped = 1u << 11,
	GyroClipped = 1u << 12,
	AngularVelocityClipped = 1u << 13,
	ReservedSensorStatusFlags = 0xC000u,
};

enum HealthFlags : uint16_t {
	LocalReady = 1u << 0,
	HealthLocalRcValid = 1u << 1,
	HealthSafetyGateAllow = 1u << 2,
	HealthAirlinkControlActive = 1u << 3,
	InboundPeriodicFresh = 1u << 4,
	Armed = 1u << 5,
	AirborneLatched = 1u << 6,
	PairModeSelected = 1u << 7,
	GsLinkValid = 1u << 8,
	GsPositionActive = 1u << 9,
	SingleLandingActive = 1u << 10,
	PairReattachLatched = 1u << 11,
	LinkTestActive = 1u << 12,
	ReservedHealthFlags = 0xE000u,
};

enum CapabilityFlags : uint32_t {
	Single = 1u << 0,
	Pair = 1u << 1,
	PairArmCapability = 1u << 2,
	GsPosition = 1u << 3,
	SingleLanding = 1u << 4,
	LinkTest = 1u << 5,
	SensorStateV1 = 1u << 6,
	Output2M3s = 1u << 7,
	CompositeYawReference = 1u << 8,
	ReservedCapabilityFlags = 0xFFFFFE00u,
};

enum HandshakeStateFlags : uint32_t {
	HsLocalReady = 1u << 0,
	HsArmed = 1u << 1,
	HsAirborneLatched = 1u << 2,
	HsPairModeSelected = 1u << 3,
	HsPairReattachLatched = 1u << 4,
	HsGsLinkValid = 1u << 5,
	HsLocalRcValid = 1u << 6,
	HsSafetyGateAllow = 1u << 7,
	HsLinkTestActive = 1u << 8,
	HsPairActive = 1u << 9,
	ReservedHandshakeStateFlags = 0xFFFFFC00u,
};

enum class ReasonCode : uint8_t {
	None = 0,
	OperatorRequest = 1,
	PeerStreamStale = 2,
	SensorStale = 3,
	ControlStale = 4,
	HeartbeatTimeout = 5,
	LocalRcOrSafety = 6,
	SessionChanged = 7,
	InvalidCommand = 8,
	EmergencyTrigger = 9,
	Other = 10,
};

enum class AckResult : uint8_t {
	Success = 0,
	Unsupported = 1,
	BadLength = 2,
	SessionMismatch = 3,
	InvalidState = 4,
	RejectedSafety = 5,
	Busy = 6,
	Duplicate = 7,
	BadArgument = 8,
	InternalError = 9,
};

enum class HandshakePhase : uint8_t {
	Request = 1,
	Report = 2,
	Confirm = 3,
};

enum class LinkTestOpcode : uint8_t {
	Start = 1,
	Abort = 2,
};

enum class LinkTestEchoMode : uint8_t {
	Exact = 1,
};

struct Header {
	uint8_t version{kVersion};
	FrameType frame_type{};
	uint8_t flags{0};
	uint8_t message_id{0};
	Role source_role{Role::Disabled};
	Role dest_role{Role::Disabled};
	uint32_t source_session{0};
	uint16_t sequence{0};
	uint16_t payload_len{0};
};

struct Frame {
	Header header{};
	uint8_t payload[kMaxPayload] {};
};

struct ParseStats {
	uint32_t crc_errors{0};
	uint32_t length_errors{0};
	uint32_t version_errors{0};
	uint32_t resyncs{0};
	uint32_t buffer_overflows{0};
	uint32_t partial_timeouts{0};
	uint32_t header_errors{0};
	uint32_t eof_errors{0};
};

struct SensorInput {
	bool accel_valid{false};
	bool gyro_valid{false};
	bool attitude_valid{false};
	bool angular_velocity_valid{false};
	bool esc_upper_valid{false};
	bool esc_lower_valid{false};
	bool battery_valid{false};
	bool core_sources_fresh{false};
	bool local_rc_valid{false};
	bool safety_gate_allow{false};
	bool airlink_control_active{false};
	uint32_t imu_sample_time_us{0};
	uint32_t attitude_sample_time_us{0};
	float accelerometer[3] {};
	float gyro[3] {};
	float attitude_q[4] {};
	float angular_velocity[3] {};
	int32_t esc_rpm[2] {};
	float battery_voltage_v{0.f};
};

struct SensorState {
	uint16_t status_flags{0};
	uint32_t imu_sample_time_us{0};
	uint32_t attitude_sample_time_us{0};
	int16_t accelerometer[3] {};
	int16_t gyro[3] {};
	int16_t attitude_q[4] {};
	int16_t angular_velocity[3] {};
	int16_t esc_rpm[2] {};
	uint16_t battery_voltage_mv{0};
};

// A received packet can be fresh while its IMU payload is unavailable.
inline bool sensor_imu_valid(uint16_t flags, uint32_t sample_time_us)
{
	constexpr uint16_t required = AccelValid | GyroValid | SampleFresh;
	return sample_time_us != 0 && (flags & required) == required;
}

enum YawReferenceFlags : uint16_t {
	YawValid = 1u << 0,
	YawDegraded = 1u << 1,
	ReservedYawReferenceFlags = 0xFFFCu,
};

struct YawReference {
	uint32_t source_sample_time_us{0};
	float yaw_rad{0.f};
	float yaw_sigma_rad{0.f};
	uint8_t reset_counter{0};
	uint8_t quality{0};
	uint16_t flags{0};
};

struct ActuatorCommand {
	float upper_motor{0.f};
	float lower_motor{0.f};
	float servos[3] {};
};

struct Heartbeat {
	uint16_t health_flags{0};
	uint16_t reserved{0};
};

struct ReliableEvent {
	uint32_t target_peer_session{0};
	uint32_t event_id{0};
	ReasonCode reason_code{ReasonCode::None};
	uint8_t event_flags{0};
	uint16_t reserved{0};
};

struct Ack {
	uint32_t target_peer_session{0};
	uint32_t event_id{0};
	uint8_t acked_message_id{0};
	AckResult result{AckResult::Success};
	uint8_t detail{0};
	uint8_t reserved0{0};
	uint32_t reserved1{0};
};

struct Handshake {
	uint32_t target_peer_session{0};
	uint32_t transaction_id{0};
	uint16_t protocol_version{kVersion};
	HandshakePhase phase{HandshakePhase::Request};
	uint8_t reserved{0};
	uint32_t capabilities{0};
	uint32_t state_flags{0};
	uint32_t config_crc32c{0};
};

struct LinkTestControl {
	uint32_t target_peer_session{0};
	uint32_t test_id{0};
	LinkTestOpcode opcode{LinkTestOpcode::Start};
	LinkTestEchoMode echo_mode{LinkTestEchoMode::Exact};
	uint16_t payload_len{0};
	uint16_t rate_hz{0};
	uint16_t duration_ms{0};
};

struct LinkTestDataHeader {
	uint32_t test_id{0};
	uint32_t test_sequence{0};
	uint32_t host_send_time_us{0};
	uint16_t pattern_len{0};
	uint16_t pattern_seed{0};
};

static constexpr size_t kSensorPayloadSize{42};
static constexpr size_t kYawReferencePayloadSize{12};
static constexpr size_t kActuatorPayloadSize{6};
static constexpr size_t kHeartbeatPayloadSize{4};
static constexpr size_t kReliableEventPayloadSize{12};
static constexpr size_t kAckPayloadSize{16};
static constexpr size_t kHandshakePayloadSize{24};
static constexpr size_t kLinkTestControlPayloadSize{16};
static constexpr size_t kLinkTestDataHeaderSize{16};
static constexpr size_t kMinLinkTestDataPayloadSize{32};

inline uint32_t crc32c(const uint8_t *data, size_t length)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < length; ++i) {
		crc ^= data[i];

		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc & 1u) ? (crc >> 1) ^ kCrc32cPolynomial : crc >> 1;
		}
	}

	return crc ^ 0xFFFFFFFFu;
}

inline uint16_t read_u16(const uint8_t *data)
{
	return static_cast<uint16_t>(data[0])
	       | static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

inline uint32_t read_u32(const uint8_t *data)
{
	return static_cast<uint32_t>(data[0])
	       | (static_cast<uint32_t>(data[1]) << 8)
	       | (static_cast<uint32_t>(data[2]) << 16)
	       | (static_cast<uint32_t>(data[3]) << 24);
}

inline void write_u16(uint8_t *data, uint16_t value)
{
	data[0] = static_cast<uint8_t>(value);
	data[1] = static_cast<uint8_t>(value >> 8);
}

inline void write_u32(uint8_t *data, uint32_t value)
{
	data[0] = static_cast<uint8_t>(value);
	data[1] = static_cast<uint8_t>(value >> 8);
	data[2] = static_cast<uint8_t>(value >> 16);
	data[3] = static_cast<uint8_t>(value >> 24);
}

inline int32_t round_half_away(float value)
{
	if (!std::isfinite(value)) {
		return 0;
	}

	return static_cast<int32_t>(value >= 0.f ? std::floor(value + 0.5f) : std::ceil(value - 0.5f));
}

template<typename T>
constexpr T minimum(T a, T b)
{
	return a < b ? a : b;
}

template<typename T>
constexpr T maximum(T a, T b)
{
	return a > b ? a : b;
}

template<typename T>
constexpr T constrain(T value, T lower, T upper)
{
	return value < lower ? lower : (value > upper ? upper : value);
}

inline bool roles_are_opposite(Role source, Role destination)
{
	return (source == Role::Master && destination == Role::Slave)
	       || (source == Role::Slave && destination == Role::Master);
}

inline bool valid_message_for_frame_type(FrameType frame_type, uint8_t message_id)
{
	(void)message_id;
	return frame_type == FrameType::Data || frame_type == FrameType::Command;
}

inline bool valid_frame_flags(FrameType frame_type, uint8_t message_id, uint8_t flags)
{
	if ((flags & ReservedFrameFlags) != 0) {
		return false;
	}

	if (frame_type == FrameType::Data) {
		if (message_id == static_cast<uint8_t>(MessageId::SensorState)) {
			return flags == 0;
		}

		if (message_id == static_cast<uint8_t>(MessageId::YawReference)) {
			return flags == 0;
		}

		if (message_id == static_cast<uint8_t>(MessageId::LinkTestData)) {
			return flags == 0 || flags == Response;
		}

		return true;
	}

	switch (static_cast<MessageId>(message_id)) {
	case MessageId::ActuatorCommand:
	case MessageId::Heartbeat:
		return flags == 0;

	case MessageId::ControlEnable:
	case MessageId::ControlRelease:
	case MessageId::PairArm:
	case MessageId::PairDisarm:
		return (flags & AckRequired) != 0
		       && (flags & ~(AckRequired | Retry)) == 0;

	case MessageId::EmergencyWithdraw:
		return (flags & AckRequired) != 0
		       && (flags & ~(AckRequired | Retry | Urgent)) == 0;

	case MessageId::Ack:
		return flags == Response;

	case MessageId::LinkHandshake:
		return flags == 0 || flags == Retry || flags == Response
		       || flags == (Response | Retry) || flags == AckRequired
		       || flags == (AckRequired | Retry);

	case MessageId::LinkTestControl:
		return (flags & AckRequired) != 0
		       && (flags & ~(AckRequired | Retry)) == 0;

	case MessageId::LinkTestData:
		return flags == 0 || flags == Response;

	default:
		return true;
	}
}

inline bool expected_payload_length(FrameType frame_type, uint8_t message_id, uint16_t &minimum,
				    uint16_t &maximum)
{
	minimum = 0;
	maximum = kMaxPayload;

	if (frame_type == FrameType::Data) {
		switch (static_cast<MessageId>(message_id)) {
		case MessageId::SensorState:
			minimum = maximum = kSensorPayloadSize;
			return true;

		case MessageId::YawReference:
			minimum = maximum = kYawReferencePayloadSize;
			return true;

		case MessageId::LinkTestData:
			minimum = kMinLinkTestDataPayloadSize;
			return true;

		default:
			return false;
		}
	}

	switch (static_cast<MessageId>(message_id)) {
	case MessageId::ActuatorCommand:
		minimum = maximum = kActuatorPayloadSize;
		break;

	case MessageId::Heartbeat:
		minimum = maximum = kHeartbeatPayloadSize;
		break;

	case MessageId::ControlEnable:
	case MessageId::ControlRelease:
	case MessageId::EmergencyWithdraw:
	case MessageId::PairArm:
	case MessageId::PairDisarm:
		minimum = maximum = kReliableEventPayloadSize;
		break;

	case MessageId::Ack:
		minimum = maximum = kAckPayloadSize;
		break;

	case MessageId::LinkHandshake:
		minimum = maximum = kHandshakePayloadSize;
		break;

	case MessageId::LinkTestControl:
		minimum = maximum = kLinkTestControlPayloadSize;
		break;

	default:
		return false;
	}

	return true;
}

inline uint16_t u16_mod_delta(uint16_t newer, uint16_t older)
{
	return static_cast<uint16_t>(newer - older);
}

inline bool u32_forward_delta(uint32_t newer, uint32_t older, uint32_t &delta)
{
	delta = newer - older;
	return delta < 0x80000000u;
}

inline bool build_frame(const Header &header, const uint8_t *payload, uint16_t payload_len,
			uint8_t *output, size_t output_capacity, size_t &frame_len)
{
	frame_len = 0;

	if (payload == nullptr || output == nullptr || payload_len > kMaxPayload
	    || header.version < kMinSupportedVersion || header.version > kVersion
	    || !valid_message_for_frame_type(header.frame_type, header.message_id)
	    || !roles_are_opposite(header.source_role, header.dest_role)
	    || header.source_session == 0
	    || !valid_frame_flags(header.frame_type, header.message_id, header.flags)) {
		return false;
	}

	uint16_t minimum = 0;
	uint16_t maximum = kMaxPayload;

	if (expected_payload_length(header.frame_type, header.message_id, minimum, maximum)
	    && (payload_len < minimum || payload_len > maximum)) {
		return false;
	}

	const size_t total = kOverhead + payload_len;

	if (output_capacity < total) {
		return false;
	}

	output[0] = kSof0;
	output[1] = kSof1;
	output[2] = header.version;
	output[3] = static_cast<uint8_t>(header.frame_type);
	output[4] = header.flags;
	output[5] = header.message_id;
	output[6] = static_cast<uint8_t>(header.source_role);
	output[7] = static_cast<uint8_t>(header.dest_role);
	write_u32(output + 8, header.source_session);
	write_u16(output + 12, header.sequence);
	write_u16(output + 14, payload_len);

	if (payload_len > 0) {
		memcpy(output + kHeaderSize, payload, payload_len);
	}

	const uint32_t crc = crc32c(output + 2, kHeaderSize - 2 + payload_len);
	write_u32(output + kHeaderSize + payload_len, crc);
	output[kHeaderSize + payload_len + kCrcSize] = kEof0;
	output[kHeaderSize + payload_len + kCrcSize + 1] = kEof1;
	frame_len = total;
	return true;
}

class StreamParser
{
public:
	void reset()
	{
		_length = 0;
		_partial_since = 0;
	}

	void feed(const uint8_t *data, size_t length, uint64_t now_us)
	{
		if (data == nullptr || length == 0) {
			return;
		}

		if (_partial_since == 0) {
			_partial_since = now_us;
		}

		for (size_t i = 0; i < length; ++i) {
			if (_length >= sizeof(_buffer)) {
				++_stats.buffer_overflows;
				drop_prefix(1, false);
			}

			_buffer[_length++] = data[i];
		}
	}

	void check_timeout(uint64_t now_us)
	{
		if (_length > 0 && _partial_since != 0 && now_us >= _partial_since
		    && now_us - _partial_since > kPartialFrameTimeoutUs) {
			++_stats.partial_timeouts;
			reset();
		}
	}

	bool next(Frame &frame)
	{
		while (true) {
			if (_length < 2) {
				return false;
			}

			if (_buffer[0] != kSof0 || _buffer[1] != kSof1) {
				resync_to_sof();

				if (_length < 2) {
					return false;
				}
			}

			if (_length < kHeaderSize) {
				return false;
			}

			Header header{};
			header.version = _buffer[2];
			header.frame_type = static_cast<FrameType>(_buffer[3]);
			header.flags = _buffer[4];
			header.message_id = _buffer[5];
			header.source_role = static_cast<Role>(_buffer[6]);
			header.dest_role = static_cast<Role>(_buffer[7]);
			header.source_session = read_u32(_buffer + 8);
			header.sequence = read_u16(_buffer + 12);
			header.payload_len = read_u16(_buffer + 14);

			if (header.version < kMinSupportedVersion || header.version > kVersion) {
				++_stats.version_errors;
				drop_prefix(1);
				continue;
			}

			if (!valid_message_for_frame_type(header.frame_type, header.message_id)
			    || !valid_frame_flags(header.frame_type, header.message_id, header.flags)
			    || !roles_are_opposite(header.source_role, header.dest_role)
			    || header.source_session == 0) {
				++_stats.header_errors;
				drop_prefix(1);
				continue;
			}

			if (header.payload_len > kMaxPayload) {
				++_stats.length_errors;
				drop_prefix(1);
				continue;
			}

			uint16_t minimum = 0;
			uint16_t maximum = kMaxPayload;

			if (expected_payload_length(header.frame_type, header.message_id, minimum, maximum)
			    && (header.payload_len < minimum || header.payload_len > maximum)) {
				++_stats.length_errors;
				drop_prefix(1);
				continue;
			}

			const size_t total = kOverhead + header.payload_len;

			if (_length < total) {
				return false;
			}

			const size_t crc_offset = kHeaderSize + header.payload_len;
			const uint32_t expected_crc = read_u32(_buffer + crc_offset);
			const uint32_t actual_crc = crc32c(_buffer + 2, kHeaderSize - 2 + header.payload_len);

			if (actual_crc != expected_crc) {
				++_stats.crc_errors;
				drop_prefix(1);
				continue;
			}

			if (_buffer[crc_offset + kCrcSize] != kEof0
			    || _buffer[crc_offset + kCrcSize + 1] != kEof1) {
				++_stats.eof_errors;
				drop_prefix(1);
				continue;
			}

			frame.header = header;

			if (header.payload_len > 0) {
				memcpy(frame.payload, _buffer + kHeaderSize, header.payload_len);
			}

			consume(total);
			return true;
		}
	}

	const ParseStats &stats() const { return _stats; }
	size_t buffered() const { return _length; }
	size_t space_remaining() const { return sizeof(_buffer) - _length; }

private:
	void consume(size_t count)
	{
		if (count >= _length) {
			_length = 0;
			_partial_since = 0;

		} else {
			memmove(_buffer, _buffer + count, _length - count);
			_length -= count;
		}
	}

	void drop_prefix(size_t count, bool count_resync = true)
	{
		consume(count);

		if (count_resync) {
			++_stats.resyncs;
		}
	}

	void resync_to_sof()
	{
		size_t offset = 1;

		while (offset + 1 < _length
		       && !(_buffer[offset] == kSof0 && _buffer[offset + 1] == kSof1)) {
			++offset;
		}

		if (offset + 1 >= _length) {
			offset = _length;
		}

		if (offset > 0) {
			consume(offset);
			++_stats.resyncs;
		}
	}

	uint8_t _buffer[kMaxFrameSize] {};
	size_t _length{0};
	uint64_t _partial_since{0};
	ParseStats _stats{};
};

inline bool encode_sensor_state(const SensorInput &input, SensorState &state)
{
	state = {};
	uint16_t status = 0;

	for (int axis = 0; axis < 3; ++axis) {
		if (input.accel_valid && std::isfinite(input.accelerometer[axis])) {
			const float scaled = input.accelerometer[axis] * 128.f;
			status |= AccelValid;

			if (scaled < -32768.f) {
				state.accelerometer[axis] = INT16_MIN;
				status |= AccelClipped;

			} else if (scaled > 32767.f) {
				state.accelerometer[axis] = INT16_MAX;
				status |= AccelClipped;

			} else {
				state.accelerometer[axis] = static_cast<int16_t>(round_half_away(scaled));
			}
		}

		if (input.gyro_valid && std::isfinite(input.gyro[axis])) {
			const float scaled = input.gyro[axis] * 800.f;
			status |= GyroValid;

			if (scaled < -32768.f) {
				state.gyro[axis] = INT16_MIN;
				status |= GyroClipped;

			} else if (scaled > 32767.f) {
				state.gyro[axis] = INT16_MAX;
				status |= GyroClipped;

			} else {
				state.gyro[axis] = static_cast<int16_t>(round_half_away(scaled));
			}
		}

		if (input.angular_velocity_valid && std::isfinite(input.angular_velocity[axis])) {
			const float scaled = input.angular_velocity[axis] * 800.f;
			status |= AngularVelocityValid;

			if (scaled < -32768.f) {
				state.angular_velocity[axis] = INT16_MIN;
				status |= AngularVelocityClipped;

			} else if (scaled > 32767.f) {
				state.angular_velocity[axis] = INT16_MAX;
				status |= AngularVelocityClipped;

			} else {
				state.angular_velocity[axis] = static_cast<int16_t>(round_half_away(scaled));
			}
		}
	}

	bool quaternion_valid = input.attitude_valid;
	float norm_squared = 0.f;

	for (int i = 0; i < 4; ++i) {
		quaternion_valid = quaternion_valid && std::isfinite(input.attitude_q[i])
				   && std::abs(input.attitude_q[i]) <= 1.f;
		norm_squared += input.attitude_q[i] * input.attitude_q[i];
	}

	quaternion_valid = quaternion_valid && std::isfinite(norm_squared)
			   && std::abs(std::sqrt(norm_squared) - 1.f) <= 0.01f;

	if (quaternion_valid) {
		status |= AttitudeValid;

		for (int i = 0; i < 4; ++i) {
			state.attitude_q[i] = static_cast<int16_t>(
						      constrain<int32_t>(round_half_away(
								      input.attitude_q[i] * 32767.f),
								      -32767, 32767));
		}
	}

	for (int motor = 0; motor < 2; ++motor) {
		const bool valid = motor == 0 ? input.esc_upper_valid : input.esc_lower_valid;

		if (valid && input.esc_rpm[motor] >= -32767 && input.esc_rpm[motor] <= 32767) {
			state.esc_rpm[motor] = static_cast<int16_t>(input.esc_rpm[motor]);
			status |= motor == 0 ? EscUpperValid : EscLowerValid;
		}
	}

	if (input.battery_valid && std::isfinite(input.battery_voltage_v)
	    && input.battery_voltage_v >= 0.f && input.battery_voltage_v <= 65.535f) {
		state.battery_voltage_mv = static_cast<uint16_t>(
						   round_half_away(input.battery_voltage_v * 1000.f));
		status |= BatteryValid;
	}

	const uint16_t core_valid_mask = AccelValid | GyroValid | AttitudeValid
					 | AngularVelocityValid;

	if (input.core_sources_fresh && (status & core_valid_mask) == core_valid_mask) {
		status |= SampleFresh;
	}

	if (input.local_rc_valid) {
		status |= LocalRcValid;
	}

	if (input.safety_gate_allow) {
		status |= SafetyGateAllow;
	}

	if (input.airlink_control_active) {
		status |= AirlinkControlActive;
	}

	state.status_flags = status;
	state.imu_sample_time_us = input.imu_sample_time_us;
	state.attitude_sample_time_us = input.attitude_sample_time_us;
	return true;
}

inline bool decode_sensor_state(const uint8_t *payload, uint16_t payload_len, SensorState &state)
{
	if (payload == nullptr || payload_len != kSensorPayloadSize) {
		return false;
	}

	state = {};
	state.status_flags = read_u16(payload);

	if ((state.status_flags & ReservedSensorStatusFlags) != 0) {
		return false;
	}

	state.imu_sample_time_us = read_u32(payload + 2);
	state.attitude_sample_time_us = read_u32(payload + 6);

	for (int axis = 0; axis < 3; ++axis) {
		const int offset = 10 + axis * 2;
		state.accelerometer[axis] = static_cast<int16_t>(read_u16(payload + offset));
		const int gyro_offset = 16 + axis * 2;
		state.gyro[axis] = static_cast<int16_t>(read_u16(payload + gyro_offset));
		const int angular_offset = 30 + axis * 2;
		state.angular_velocity[axis] = static_cast<int16_t>(read_u16(payload + angular_offset));
	}

	float norm_squared = 0.f;

	for (int i = 0; i < 4; ++i) {
		const int16_t raw = static_cast<int16_t>(read_u16(payload + 22 + i * 2));
		state.attitude_q[i] = raw;
		const float value = raw == INT16_MIN ? -1.f
				    : constrain(raw / 32767.f, -1.f, 1.f);
		norm_squared += value * value;
	}

	if ((state.status_flags & AttitudeValid) != 0) {
		const float norm = std::sqrt(norm_squared);

		if (!std::isfinite(norm) || std::abs(norm - 1.f) > 0.0101f) {
			state.status_flags &= static_cast<uint16_t>(~AttitudeValid);
		}
	}

	state.esc_rpm[0] = static_cast<int16_t>(read_u16(payload + 36));
	state.esc_rpm[1] = static_cast<int16_t>(read_u16(payload + 38));
	state.battery_voltage_mv = read_u16(payload + 40);

	if (state.esc_rpm[0] == INT16_MIN) {
		state.status_flags &= static_cast<uint16_t>(~EscUpperValid);
	}

	if (state.esc_rpm[1] == INT16_MIN) {
		state.status_flags &= static_cast<uint16_t>(~EscLowerValid);
	}

	if ((state.status_flags & AccelValid) == 0) {
		memset(state.accelerometer, 0, sizeof(state.accelerometer));
	}

	if ((state.status_flags & GyroValid) == 0) {
		memset(state.gyro, 0, sizeof(state.gyro));
	}

	if ((state.status_flags & AttitudeValid) == 0) {
		memset(state.attitude_q, 0, sizeof(state.attitude_q));
	}

	if ((state.status_flags & AngularVelocityValid) == 0) {
		memset(state.angular_velocity, 0, sizeof(state.angular_velocity));
	}

	if ((state.status_flags & EscUpperValid) == 0) {
		state.esc_rpm[0] = 0;
	}

	if ((state.status_flags & EscLowerValid) == 0) {
		state.esc_rpm[1] = 0;
	}

	if ((state.status_flags & BatteryValid) == 0) {
		state.battery_voltage_mv = 0;
	}

	return true;
}

inline bool encode_sensor_payload(const SensorState &state, uint8_t payload[kSensorPayloadSize])
{
	if ((state.status_flags & ReservedSensorStatusFlags) != 0) {
		return false;
	}

	write_u16(payload, state.status_flags);
	write_u32(payload + 2, state.imu_sample_time_us);
	write_u32(payload + 6, state.attitude_sample_time_us);

	for (int axis = 0; axis < 3; ++axis) {
		write_u16(payload + 10 + axis * 2, static_cast<uint16_t>(state.accelerometer[axis]));
		write_u16(payload + 16 + axis * 2, static_cast<uint16_t>(state.gyro[axis]));
		write_u16(payload + 30 + axis * 2, static_cast<uint16_t>(state.angular_velocity[axis]));
	}

	for (int i = 0; i < 4; ++i) {
		write_u16(payload + 22 + i * 2, static_cast<uint16_t>(state.attitude_q[i]));
	}

	write_u16(payload + 36, static_cast<uint16_t>(state.esc_rpm[0]));
	write_u16(payload + 38, static_cast<uint16_t>(state.esc_rpm[1]));
	write_u16(payload + 40, state.battery_voltage_mv);
	return true;
}

inline bool yaw_reference_is_valid(const YawReference &reference)
{
	if ((reference.flags & ReservedYawReferenceFlags) != 0 || reference.quality > 2
	    || !std::isfinite(reference.yaw_rad)
	    || !std::isfinite(reference.yaw_sigma_rad)
	    || reference.yaw_rad < -3.1415927f || reference.yaw_rad > 3.1415927f
	    || reference.yaw_sigma_rad < 0.f || reference.yaw_sigma_rad > 6.5535f) {
		return false;
	}

	if ((reference.flags & YawValid) == 0) {
		return false;
	}

	if ((reference.flags & YawDegraded) != 0 && reference.quality < 1) {
		return false;
	}

	return true;
}

inline bool encode_yaw_reference(const YawReference &reference,
				 uint8_t payload[kYawReferencePayloadSize])
{
	if (!yaw_reference_is_valid(reference)) {
		return false;
	}

	const int32_t yaw_scaled = constrain(round_half_away(reference.yaw_rad * 10'000.f),
					     int32_t{-31'416}, int32_t{31'416});
	const int32_t sigma_scaled = constrain(round_half_away(reference.yaw_sigma_rad * 10'000.f),
					      int32_t{0}, int32_t{65'535});
	write_u32(payload, reference.source_sample_time_us);
	write_u16(payload + 4, static_cast<uint16_t>(static_cast<int16_t>(yaw_scaled)));
	write_u16(payload + 6, static_cast<uint16_t>(sigma_scaled));
	payload[8] = reference.reset_counter;
	payload[9] = reference.quality;
	write_u16(payload + 10, reference.flags);
	return true;
}

inline bool decode_yaw_reference(const uint8_t *payload, uint16_t payload_len,
				 YawReference &reference)
{
	if (payload == nullptr || payload_len != kYawReferencePayloadSize) {
		return false;
	}

	reference = {};
	reference.source_sample_time_us = read_u32(payload);
	reference.yaw_rad = static_cast<int16_t>(read_u16(payload + 4)) / 10'000.f;
	reference.yaw_sigma_rad = read_u16(payload + 6) / 10'000.f;
	reference.reset_counter = payload[8];
	reference.quality = payload[9];
	reference.flags = read_u16(payload + 10);
	return yaw_reference_is_valid(reference);
}

inline bool actuator_command_is_valid(const ActuatorCommand &command)
{
	if (!std::isfinite(command.upper_motor) || command.upper_motor < 0.f || command.upper_motor > 1.f
	    || !std::isfinite(command.lower_motor) || command.lower_motor < 0.f || command.lower_motor > 1.f) {
		return false;
	}

	for (float servo : command.servos) {
		if (!std::isfinite(servo) || servo < -1.f || servo > 1.f) {
			return false;
		}
	}

	return true;
}

inline bool encode_actuator_command(const ActuatorCommand &command,
				    uint8_t payload[kActuatorPayloadSize])
{
	if (!actuator_command_is_valid(command)) {
		return false;
	}

	const uint64_t upper_code = static_cast<uint64_t>(
					    constrain(round_half_away(command.upper_motor * 500.f),
						      int32_t{0}, int32_t{500}));
	const uint64_t lower_code = static_cast<uint64_t>(
					    constrain(round_half_away(command.lower_motor * 500.f),
						      int32_t{0}, int32_t{500}));
	const uint64_t packed = upper_code
				| (lower_code << 9)
				| (static_cast<uint64_t>(constrain(round_half_away(
						500.f * (command.servos[0] + 1.f)),
						int32_t{0}, int32_t{1000})) << 18)
				| (static_cast<uint64_t>(constrain(round_half_away(
						500.f * (command.servos[1] + 1.f)),
						int32_t{0}, int32_t{1000})) << 28)
				| (static_cast<uint64_t>(constrain(round_half_away(
						500.f * (command.servos[2] + 1.f)),
						int32_t{0}, int32_t{1000})) << 38);

	for (int i = 0; i < 6; ++i) {
		payload[i] = static_cast<uint8_t>(packed >> (i * 8));
	}

	return true;
}

inline bool decode_actuator_command(const uint8_t *payload, uint16_t payload_len,
				    ActuatorCommand &command)
{
	if (payload == nullptr || payload_len != kActuatorPayloadSize) {
		return false;
	}

	uint64_t packed = 0;

	for (int i = 0; i < 6; ++i) {
		packed |= static_cast<uint64_t>(payload[i]) << (i * 8);
	}

	const uint16_t upper_code = static_cast<uint16_t>(packed & 0x1FFu);
	const uint16_t lower_code = static_cast<uint16_t>((packed >> 9) & 0x1FFu);
	const uint16_t servo_codes[3] {
		static_cast<uint16_t>((packed >> 18) & 0x3FFu),
		static_cast<uint16_t>((packed >> 28) & 0x3FFu),
		static_cast<uint16_t>((packed >> 38) & 0x3FFu),
	};

	if (upper_code > 500 || lower_code > 500) {
		return false;
	}

	for (uint16_t code : servo_codes) {
		if (code > 1000) {
			return false;
		}
	}

	command.upper_motor = upper_code / 500.f;
	command.lower_motor = lower_code / 500.f;

	for (int i = 0; i < 3; ++i) {
		command.servos[i] = servo_codes[i] / 500.f - 1.f;
	}

	return true;
}

inline bool encode_heartbeat(const Heartbeat &heartbeat, uint8_t payload[kHeartbeatPayloadSize])
{
	if (heartbeat.reserved != 0 || (heartbeat.health_flags & ReservedHealthFlags) != 0) {
		return false;
	}

	write_u16(payload, heartbeat.health_flags);
	write_u16(payload + 2, heartbeat.reserved);
	return true;
}

inline bool decode_heartbeat(const uint8_t *payload, uint16_t payload_len, Heartbeat &heartbeat)
{
	if (payload == nullptr || payload_len != kHeartbeatPayloadSize) {
		return false;
	}

	heartbeat.health_flags = read_u16(payload);
	heartbeat.reserved = read_u16(payload + 2);
	return heartbeat.reserved == 0 && (heartbeat.health_flags & ReservedHealthFlags) == 0;
}

inline bool encode_reliable_event(const ReliableEvent &event,
				  uint8_t payload[kReliableEventPayloadSize])
{
	if (event.target_peer_session == 0 || event.event_id == 0 || event.event_flags != 0
	    || event.reserved != 0 || static_cast<uint8_t>(event.reason_code) > 10) {
		return false;
	}

	write_u32(payload, event.target_peer_session);
	write_u32(payload + 4, event.event_id);
	payload[8] = static_cast<uint8_t>(event.reason_code);
	payload[9] = event.event_flags;
	write_u16(payload + 10, event.reserved);
	return true;
}

inline bool decode_reliable_event(const uint8_t *payload, uint16_t payload_len,
				  ReliableEvent &event)
{
	if (payload == nullptr || payload_len != kReliableEventPayloadSize) {
		return false;
	}

	event.target_peer_session = read_u32(payload);
	event.event_id = read_u32(payload + 4);
	event.reason_code = static_cast<ReasonCode>(payload[8]);
	event.event_flags = payload[9];
	event.reserved = read_u16(payload + 10);
	return event.target_peer_session != 0 && event.event_id != 0
	       && static_cast<uint8_t>(event.reason_code) <= 10
	       && event.event_flags == 0 && event.reserved == 0;
}

inline bool encode_ack(const Ack &ack, uint8_t payload[kAckPayloadSize])
{
	if (ack.target_peer_session == 0 || ack.event_id == 0
	    || static_cast<uint8_t>(ack.result) > 9 || ack.reserved0 != 0
	    || ack.reserved1 != 0) {
		return false;
	}

	write_u32(payload, ack.target_peer_session);
	write_u32(payload + 4, ack.event_id);
	payload[8] = ack.acked_message_id;
	payload[9] = static_cast<uint8_t>(ack.result);
	payload[10] = ack.detail;
	payload[11] = ack.reserved0;
	write_u32(payload + 12, ack.reserved1);
	return true;
}

inline bool decode_ack(const uint8_t *payload, uint16_t payload_len, Ack &ack)
{
	if (payload == nullptr || payload_len != kAckPayloadSize) {
		return false;
	}

	ack.target_peer_session = read_u32(payload);
	ack.event_id = read_u32(payload + 4);
	ack.acked_message_id = payload[8];
	ack.result = static_cast<AckResult>(payload[9]);
	ack.detail = payload[10];
	ack.reserved0 = payload[11];
	ack.reserved1 = read_u32(payload + 12);
	return ack.target_peer_session != 0 && ack.event_id != 0
	       && static_cast<uint8_t>(ack.result) <= 9
	       && ack.reserved0 == 0 && ack.reserved1 == 0;
}

inline bool encode_handshake(const Handshake &handshake,
			     uint8_t payload[kHandshakePayloadSize])
{
	if (handshake.target_peer_session == 0 || handshake.transaction_id == 0
	    || handshake.protocol_version != kVersion || handshake.reserved != 0
	    || static_cast<uint8_t>(handshake.phase) < 1
	    || static_cast<uint8_t>(handshake.phase) > 3
	    || (handshake.capabilities & ReservedCapabilityFlags) != 0
	    || (handshake.state_flags & ReservedHandshakeStateFlags) != 0) {
		return false;
	}

	write_u32(payload, handshake.target_peer_session);
	write_u32(payload + 4, handshake.transaction_id);
	write_u16(payload + 8, handshake.protocol_version);
	payload[10] = static_cast<uint8_t>(handshake.phase);
	payload[11] = handshake.reserved;
	write_u32(payload + 12, handshake.capabilities);
	write_u32(payload + 16, handshake.state_flags);
	write_u32(payload + 20, handshake.config_crc32c);
	return true;
}

inline bool decode_handshake(const uint8_t *payload, uint16_t payload_len,
			     Handshake &handshake)
{
	if (payload == nullptr || payload_len != kHandshakePayloadSize) {
		return false;
	}

	handshake.target_peer_session = read_u32(payload);
	handshake.transaction_id = read_u32(payload + 4);
	handshake.protocol_version = read_u16(payload + 8);
	handshake.phase = static_cast<HandshakePhase>(payload[10]);
	handshake.reserved = payload[11];
	handshake.capabilities = read_u32(payload + 12);
	handshake.state_flags = read_u32(payload + 16);
	handshake.config_crc32c = read_u32(payload + 20);
	return handshake.target_peer_session != 0 && handshake.transaction_id != 0
	       && handshake.protocol_version == kVersion && handshake.reserved == 0
	       && static_cast<uint8_t>(handshake.phase) >= 1
	       && static_cast<uint8_t>(handshake.phase) <= 3
	       && (handshake.capabilities & ReservedCapabilityFlags) == 0
	       && (handshake.state_flags & ReservedHandshakeStateFlags) == 0;
}

inline bool encode_link_test_control(const LinkTestControl &control,
				     uint8_t payload[kLinkTestControlPayloadSize])
{
	if (control.target_peer_session == 0 || control.test_id == 0
	    || (control.opcode != LinkTestOpcode::Start && control.opcode != LinkTestOpcode::Abort)
	    || control.echo_mode != LinkTestEchoMode::Exact
	    || control.payload_len < kMinLinkTestDataPayloadSize || control.payload_len > kMaxPayload
	    || control.rate_hz < 1 || control.rate_hz > 500
	    || control.duration_ms < 1000 || control.duration_ms > 10000) {
		return false;
	}

	write_u32(payload, control.target_peer_session);
	write_u32(payload + 4, control.test_id);
	payload[8] = static_cast<uint8_t>(control.opcode);
	payload[9] = static_cast<uint8_t>(control.echo_mode);
	write_u16(payload + 10, control.payload_len);
	write_u16(payload + 12, control.rate_hz);
	write_u16(payload + 14, control.duration_ms);
	return true;
}

inline bool decode_link_test_control(const uint8_t *payload, uint16_t payload_len,
				     LinkTestControl &control)
{
	if (payload == nullptr || payload_len != kLinkTestControlPayloadSize) {
		return false;
	}

	control.target_peer_session = read_u32(payload);
	control.test_id = read_u32(payload + 4);
	control.opcode = static_cast<LinkTestOpcode>(payload[8]);
	control.echo_mode = static_cast<LinkTestEchoMode>(payload[9]);
	control.payload_len = read_u16(payload + 10);
	control.rate_hz = read_u16(payload + 12);
	control.duration_ms = read_u16(payload + 14);
	return control.target_peer_session != 0 && control.test_id != 0
	       && (control.opcode == LinkTestOpcode::Start || control.opcode == LinkTestOpcode::Abort)
	       && control.echo_mode == LinkTestEchoMode::Exact
	       && control.payload_len >= kMinLinkTestDataPayloadSize
	       && control.payload_len <= kMaxPayload
	       && control.rate_hz >= 1 && control.rate_hz <= 500
	       && control.duration_ms >= 1000 && control.duration_ms <= 10000;
}

inline bool decode_link_test_data_header(const uint8_t *payload, uint16_t payload_len,
		LinkTestDataHeader &header)
{
	if (payload == nullptr || payload_len < kMinLinkTestDataPayloadSize) {
		return false;
	}

	header.test_id = read_u32(payload);
	header.test_sequence = read_u32(payload + 4);
	header.host_send_time_us = read_u32(payload + 8);
	header.pattern_len = read_u16(payload + 12);
	header.pattern_seed = read_u16(payload + 14);
	return header.test_id != 0
	       && header.pattern_len == static_cast<uint16_t>(payload_len - kLinkTestDataHeaderSize);
}

} // namespace airlink
