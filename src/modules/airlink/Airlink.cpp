/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "Airlink.hpp"

#include <px4_platform_common/board_common.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>

namespace
{

constexpr uint8_t kMotor1Function{101};
constexpr uint8_t kMotor2Function{102};
constexpr uint8_t kConfigFormatVersion{2};
constexpr uint8_t kOutputLayout2M3s{1};
constexpr uint32_t kCollectiveNeutralMilli{500};
constexpr uint32_t kCollectiveDescentMilli{250};
constexpr uint32_t kPairArmCommitTimeoutUs{
	static_cast<uint32_t>(airlink::kPairArmCommitTimeoutUs)
};
constexpr float kDefaultImpactAcceleration{35.f};
constexpr int32_t kDefaultImpactConfirmMs{20};
constexpr float kDefaultFlipAngleDegrees{70.f};
constexpr int32_t kDefaultFlipConfirmMs{150};
constexpr int32_t kDefaultLandingTimeoutMs{15000};

bool finite_xyz(const float values[3])
{
	return PX4_ISFINITE(values[0]) && PX4_ISFINITE(values[1]) && PX4_ISFINITE(values[2]);
}

bool fresh_sample(uint64_t now_us, uint64_t timestamp_us, uint64_t timeout_us)
{
	return timestamp_us != 0 && now_us >= timestamp_us && now_us - timestamp_us <= timeout_us;
}

bool period_elapsed(uint64_t now_us, uint64_t last_us, uint64_t period_us)
{
	return last_us == 0 || now_us < last_us || now_us - last_us >= period_us;
}

bool parse_u16_in_range(const char *text, uint16_t minimum, uint16_t maximum,
			uint16_t &value)
{
	if (text == nullptr || text[0] == '\0' || text[0] == '-') {
		return false;
	}

	char *end = nullptr;
	const unsigned long parsed = strtoul(text, &end, 10);

	if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) {
		return false;
	}

	value = static_cast<uint16_t>(parsed);
	return true;
}

} // namespace

Airlink::Airlink(const char *device, uint32_t baudrate) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::airlink),
	_serial(device, baudrate)
{
}

Airlink::~Airlink()
{
	_serial.close();
}

bool Airlink::init()
{
	updateParams();
	uint32_t id{0};
#ifndef BOARD_HAS_NO_UUID
	uuid_uint32_t uuid_words{};
	board_get_uuid32(uuid_words);
	id = uuid_words[0] ^ uuid_words[1] ^ uuid_words[2];
#endif
	_board_id = id ^ 0xA18C2D4Fu;
	_config_crc32c = computeConfigCrc32c();
	int32_t role = _param_role.get();

	if (role < static_cast<int32_t>(airlink::Role::Disabled)
	    || role > static_cast<int32_t>(airlink::Role::Slave)) {
		role = static_cast<int32_t>(airlink::Role::Disabled);
		param_set(param_find("AIRLINK_ROLE"), &role);
	}

	_applied_role = role;
	_applied_pair_enable = _param_pair_enable.get() != 0 ? 1 : 0;
	initializeSession(hrt_absolute_time());
	ScheduleOnInterval(kRunInterval);
	return true;
}

void Airlink::initializeSession(uint64_t now_us)
{
	_session_id = generateSessionId(now_us);
	const airlink::Role role = static_cast<airlink::Role>(_applied_role);
	airlink::LocalStatus local{};
	local.pair_mode_selected = _applied_pair_enable != 0;
	local.armed = _actuator_armed.armed;
	local.airborne_latched = _airborne_latched;
	local.pair_reattach_latched = _pair_lost_latched;
	_session.start(role, _session_id, local, now_us);
	_parser.reset();
}

uint32_t Airlink::generateSessionId(uint64_t now_us)
{
	static uint32_t counter{0};
	++counter;
	uint32_t value = _board_id ^ static_cast<uint32_t>(now_us) ^ (counter * 0x9E3779B9u);
	value ^= value >> 16;
	value *= 0x7FEB352Du;
	value ^= value >> 15;
	return value == 0 ? 0xA5A50001u : value;
}

uint32_t Airlink::computeConfigCrc32c()
{
	uint8_t config[40] {};
	size_t offset = 0;
	config[offset++] = kConfigFormatVersion;
	config[offset++] = static_cast<uint8_t>(airlink::kVersion);
	config[offset++] = kOutputLayout2M3s;
	config[offset++] = static_cast<uint8_t>(airlink::SensorStateV1
					       | airlink::Output2M3s);
	airlink::write_u16(config + offset, kCollectiveNeutralMilli);
	offset += 2;
	airlink::write_u16(config + offset, kCollectiveDescentMilli);
	offset += 2;
	airlink::write_u32(config + offset, kPairArmCommitTimeoutUs);
	offset += 4;
	int32_t model_index{-1};
	int32_t airframe_id{-1};
	const param_t model_handle = param_find("NN_LQI_MODEL");
	const param_t airframe_handle = param_find("NN_AIRFRAME_ID");

	if (model_handle != PARAM_INVALID) {
		param_get(model_handle, &model_index);
	}

	if (airframe_handle != PARAM_INVALID) {
		param_get(airframe_handle, &airframe_id);
	}

	airlink::write_u32(config + offset, static_cast<uint32_t>(model_index));
	offset += 4;
	airlink::write_u32(config + offset, static_cast<uint32_t>(airframe_id));
	offset += 4;
	float impact_acceleration{kDefaultImpactAcceleration};
	float flip_angle_degrees{kDefaultFlipAngleDegrees};
	int32_t impact_confirm_ms{kDefaultImpactConfirmMs};
	int32_t flip_confirm_ms{kDefaultFlipConfirmMs};
	int32_t landing_timeout_ms{kDefaultLandingTimeoutMs};
	param_get(param_find("AIRLINK_IMPACT"), &impact_acceleration);
	param_get(param_find("AIRLINK_IMP_MS"), &impact_confirm_ms);
	param_get(param_find("AIRLINK_FLIP_DEG"), &flip_angle_degrees);
	param_get(param_find("AIRLINK_FLIP_MS"), &flip_confirm_ms);
	param_get(param_find("AIRLINK_LAND_MS"), &landing_timeout_ms);
	impact_acceleration = PX4_ISFINITE(impact_acceleration)
			      ? impact_acceleration : kDefaultImpactAcceleration;
	flip_angle_degrees = PX4_ISFINITE(flip_angle_degrees)
			     ? flip_angle_degrees : kDefaultFlipAngleDegrees;
	impact_acceleration = airlink::constrain(impact_acceleration, 10.f, 200.f);
	impact_confirm_ms = airlink::constrain(impact_confirm_ms, int32_t{5}, int32_t{1000});
	flip_angle_degrees = airlink::constrain(flip_angle_degrees, 20.f, 180.f);
	flip_confirm_ms = airlink::constrain(flip_confirm_ms, int32_t{10}, int32_t{2000});
	landing_timeout_ms = airlink::constrain(landing_timeout_ms,
						int32_t{1000}, int32_t{60000});
	airlink::write_u32(config + offset, static_cast<uint32_t>(
				   airlink::round_half_away(impact_acceleration * 1000.f)));
	offset += 4;
	airlink::write_u32(config + offset, static_cast<uint32_t>(impact_confirm_ms));
	offset += 4;
	airlink::write_u32(config + offset, static_cast<uint32_t>(
				   airlink::round_half_away(flip_angle_degrees * 1000.f)));
	offset += 4;
	airlink::write_u32(config + offset, static_cast<uint32_t>(flip_confirm_ms));
	offset += 4;
	airlink::write_u32(config + offset, static_cast<uint32_t>(landing_timeout_ms));
	offset += 4;
	return airlink::crc32c(config, offset);
}

airlink::LocalStatus Airlink::makeLocalStatus(uint64_t now_us)
{
	airlink::LocalStatus local{};
	const bool rc_valid = fresh_sample(now_us, _input_rc.timestamp_last_signal, kRcTimeout)
			      && !_input_rc.rc_lost && !_input_rc.rc_failsafe;
	const bool actuator_locked = _actuator_armed.lockdown
				     || _actuator_armed.manual_lockdown
				     || _actuator_armed.force_failsafe
				     || _actuator_armed.in_esc_calibration_mode;
	const bool power_failure = _failsafe_flags.battery_unhealthy
				   || _failsafe_flags.battery_warning >= battery_status_s::WARNING_CRITICAL;
	const bool hard_failure = _failsafe_flags.fd_critical_failure
				  || _failsafe_flags.fd_esc_arming_failure
				  || _failsafe_flags.fd_motor_failure;
	// LOCAL_READY is the endpoint's protocol/service readiness. Arm readiness is
	// independently enforced by RC, safety, Commander and config checks.
	local.local_ready = _applied_role != static_cast<int32_t>(airlink::Role::Disabled);
	local.local_rc_valid = rc_valid;
	local.safety_gate_allow = rc_valid && !actuator_locked && !power_failure && !hard_failure
				  && !_failsafe_flags.manual_control_signal_lost;
	local.armed = _actuator_armed.armed;
	local.airborne_latched = _airborne_latched;
	local.pair_mode_selected = _applied_pair_enable != 0;
	const bool gs_status_fresh = fresh_sample(now_us, _gs_link_status.timestamp,
				    kGsLinkTimeout);
	local.gs_link_valid = gs_status_fresh && !_gs_link_status.failsafe;
	local.gs_position_active = local.gs_link_valid && _gs_link_status.gs_mode_active;
	local.single_landing_active = gs_status_fresh && _gs_link_status.landing_active;
	local.pair_reattach_latched = _pair_lost_latched;
	return local;
}

airlink::SensorState Airlink::makeSensorState(uint64_t now_us)
{
	airlink::SensorInput input{};
	const uint64_t accel_sample = _sensor_accel.timestamp_sample != 0
				      ? _sensor_accel.timestamp_sample : _sensor_accel.timestamp;
	const uint64_t gyro_sample = _sensor_gyro.timestamp_sample != 0
				     ? _sensor_gyro.timestamp_sample : _sensor_gyro.timestamp;
	const uint64_t attitude_sample = _vehicle_attitude.timestamp_sample != 0
					 ? _vehicle_attitude.timestamp_sample : _vehicle_attitude.timestamp;
	const uint64_t angular_sample =
		_vehicle_angular_velocity.timestamp_sample != 0
		? _vehicle_angular_velocity.timestamp_sample
		: _vehicle_angular_velocity.timestamp;

	const float accelerometer[3] {_sensor_accel.x, _sensor_accel.y, _sensor_accel.z};
	const float gyro[3] {_sensor_gyro.x, _sensor_gyro.y, _sensor_gyro.z};
	input.accel_valid = finite_xyz(accelerometer);
	input.gyro_valid = finite_xyz(gyro);
	input.attitude_valid = PX4_ISFINITE(_vehicle_attitude.q[0])
			       && PX4_ISFINITE(_vehicle_attitude.q[1])
			       && PX4_ISFINITE(_vehicle_attitude.q[2])
			       && PX4_ISFINITE(_vehicle_attitude.q[3]);
	input.angular_velocity_valid = finite_xyz(_vehicle_angular_velocity.xyz);
	input.core_sources_fresh = input.accel_valid && input.gyro_valid && input.attitude_valid
				   && input.angular_velocity_valid
				   && fresh_sample(now_us, accel_sample, kPeerFreshTimeout)
				   && fresh_sample(now_us, gyro_sample, kPeerFreshTimeout)
				   && fresh_sample(now_us, attitude_sample, kPeerFreshTimeout)
				   && fresh_sample(now_us, angular_sample, kPeerFreshTimeout);
	input.imu_sample_time_us = static_cast<uint32_t>(gyro_sample);
	input.attitude_sample_time_us = static_cast<uint32_t>(attitude_sample);
	memcpy(input.accelerometer, accelerometer, sizeof(input.accelerometer));
	memcpy(input.gyro, gyro, sizeof(input.gyro));
	memcpy(input.attitude_q, _vehicle_attitude.q, sizeof(input.attitude_q));
	memcpy(input.angular_velocity, _vehicle_angular_velocity.xyz,
	       sizeof(input.angular_velocity));

	bool esc_found[2] {false, false};

	if (fresh_sample(now_us, _esc_status.timestamp, kEscTimeout)) {
		for (uint8_t index = 0; index < _esc_status.esc_count
		     && index < esc_status_s::CONNECTED_ESC_MAX; ++index) {
			if ((_esc_status.esc_online_flags & (1u << index)) == 0) {
				continue;
			}

			const uint8_t function = _esc_status.esc[index].actuator_function;
			const int motor = function == kMotor1Function ? 0
					  : function == kMotor2Function ? 1 : -1;

			if (motor < 0) {
				continue;
			}

			const int32_t rpm = _esc_status.esc[index].esc_rpm;
			input.esc_rpm[motor] = rpm;
			input.esc_upper_valid |= motor == 0 && rpm >= -32767 && rpm <= 32767;
			input.esc_lower_valid |= motor == 1 && rpm >= -32767 && rpm <= 32767;
			esc_found[motor] = true;
		}
	}

	input.esc_upper_valid = input.esc_upper_valid && esc_found[0];
	input.esc_lower_valid = input.esc_lower_valid && esc_found[1];
	input.battery_valid = _battery_status.connected
			      && fresh_sample(now_us, _battery_status.timestamp, kBatteryTimeout)
			      && PX4_ISFINITE(_battery_status.voltage_v)
			      && _battery_status.voltage_v >= 0.f
			      && _battery_status.voltage_v <= 65.535f;
	input.battery_voltage_v = _battery_status.voltage_v;
	const airlink::LocalStatus local = makeLocalStatus(now_us);
	input.local_rc_valid = local.local_rc_valid;
	input.safety_gate_allow = local.safety_gate_allow;
	input.airlink_control_active = _applied_role == static_cast<int32_t>(airlink::Role::Slave)
				       && _session.state() == airlink::LinkState::PairActive;
	airlink::SensorState state{};
	airlink::encode_sensor_state(input, state);
	return state;
}

void Airlink::updateSubscriptions()
{
	_actuator_armed_sub.update(&_actuator_armed);
	_battery_status_sub.update(&_battery_status);
	_esc_status_sub.update(&_esc_status);
	_failsafe_flags_sub.update(&_failsafe_flags);
	_gs_link_status_sub.update(&_gs_link_status);
	_input_rc_sub.update(&_input_rc);
	_sensor_accel_sub.update(&_sensor_accel);
	_sensor_gyro_sub.update(&_sensor_gyro);
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	_vehicle_command_ack_sub.update(&_vehicle_command_ack);
	_vehicle_status_sub.update(&_vehicle_status);
	_pair_command_sub.update(&_pair_command);
}

void Airlink::updateParameters(bool force)
{
	if (!_parameter_update_sub.updated() && !force) {
		return;
	}

	parameter_update_s update{};
	_parameter_update_sub.copy(&update);
	updateParams();
	applyRequestedRole();
	applyRequestedPairMode();
	const uint32_t config_crc = computeConfigCrc32c();

	if (config_crc != _config_crc32c && !_actuator_armed.armed
	    && _session.state() != airlink::LinkState::PairActive
	    && !_session.link_test_active()) {
		_config_crc32c = config_crc;
		initializeSession(hrt_absolute_time());
	}
}

void Airlink::applyRequestedRole()
{
	int32_t requested = _param_role.get();

	if (requested < static_cast<int32_t>(airlink::Role::Disabled)
	    || requested > static_cast<int32_t>(airlink::Role::Slave)) {
		requested = static_cast<int32_t>(airlink::Role::Disabled);
		param_set(param_find("AIRLINK_ROLE"), &requested);
	}

	if (requested == _applied_role) {
		return;
	}

	const airlink::Role new_role = static_cast<airlink::Role>(requested);
	const airlink::LocalStatus local = makeLocalStatus(hrt_absolute_time());

	if (_actuator_armed.armed || _session.state() == airlink::LinkState::PairActive
	    || _session.state() == airlink::LinkState::PairArmPending
	    || _session.link_test_active()) {
		int32_t old = _applied_role;
		param_set(param_find("AIRLINK_ROLE"), &old);
		PX4_WARN("AIRLINK_ROLE change rejected while armed/active/testing");
		return;
	}

	_applied_role = requested;
	_session_id = generateSessionId(hrt_absolute_time());
	_session.start(new_role, _session_id, local, hrt_absolute_time());
	_parser.reset();
	closeSerial();
	PX4_INFO("AIRLINK role set to %ld, session %08lx",
		 static_cast<long>(requested), static_cast<unsigned long>(_session_id));
}

void Airlink::applyRequestedPairMode()
{
	const int32_t requested = _param_pair_enable.get() != 0 ? 1 : 0;

	if (requested == _applied_pair_enable) {
		return;
	}

	if (_actuator_armed.armed || _session.state() == airlink::LinkState::PairActive
	    || _session.state() == airlink::LinkState::PairArmPending) {
		int32_t old = _applied_pair_enable;
		param_set(param_find("AIRLINK_PAIR_EN"), &old);
		PX4_WARN("AIRLINK_PAIR_EN change rejected while armed/active");
		return;
	}

	_applied_pair_enable = requested;
}

void Airlink::handleExternalArming(uint64_t now_us)
{
	bool request_arm = false;
	bool request_disarm = false;

	if (_vehicle_command_sub.update(&_vehicle_command)
	    && _vehicle_command.from_external
	    && _vehicle_command.command == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM
	    && PX4_ISFINITE(_vehicle_command.param1)) {
		request_arm = _vehicle_command.param1 > 0.5f;
		request_disarm = !request_arm;
	}

	if (_action_request_sub.update(&_action_request)) {
		switch (_action_request.action) {
		case action_request_s::ACTION_ARM:
			request_arm = true;
			break;

		case action_request_s::ACTION_DISARM:
			request_disarm = true;
			break;

		case action_request_s::ACTION_TOGGLE_ARMING:
			request_arm = !_actuator_armed.armed;
			request_disarm = _actuator_armed.armed;
			break;

		default:
			break;
		}
	}

	if (request_arm) {
		if (_applied_pair_enable == 0
		    || _applied_role != static_cast<int32_t>(airlink::Role::Master)
		    || _actuator_armed.armed) {
			return;
		}

		if (_session.link_test_active()) {
			_commands.fetch_or(CommandLinkTestAbort);
			return;
		}

		_operator_arm_requested = true;

		if (_session.state() == airlink::LinkState::Ready) {
			(void)_session.request_control_enable(now_us);
		}

	} else if (request_disarm) {
		_operator_arm_requested = false;
		_commands.fetch_or(CommandDisarm);
	}
}

bool Airlink::openSerial()
{
	if (_applied_role == static_cast<int32_t>(airlink::Role::Disabled)) {
		closeSerial();
		return false;
	}

	if (_serial.isOpen()) {
		return true;
	}

	if (!_serial.open()) {
		++_open_failures;
		PX4_ERR("failed to open %s at %lu baud", _serial.getPort(),
			static_cast<unsigned long>(_serial.getBaudrate()));
		return false;
	}

	_serial.flush();
	_parser.reset();
	PX4_INFO("AIRLINK v1 on %s at %lu baud, session %08lx",
		 _serial.getPort(), static_cast<unsigned long>(_serial.getBaudrate()),
		 static_cast<unsigned long>(_session_id));
	return true;
}

void Airlink::closeSerial()
{
	if (_serial.isOpen()) {
		_serial.close();
	}

	_parser.reset();
}

void Airlink::readSerial(uint64_t now_us, airlink::SessionActions &actions)
{
	uint8_t buffer[512];

	while (true) {
		// Parse before requesting more bytes so the parser never has to queue
		// more than its single maximum-sized frame.
		parseFrames(now_us, actions);
		const size_t space = _parser.space_remaining();

		if (space == 0) {
			break;
		}

		const size_t request = airlink::minimum(sizeof(buffer), space);
		const ssize_t received = _serial.read(buffer, request);

		if (received <= 0) {
			break;
		}

		_parser.feed(buffer, static_cast<size_t>(received), now_us);
		parseFrames(now_us, actions);
	}
}

void Airlink::mergeActions(airlink::SessionActions &destination,
			   const airlink::SessionActions &source)
{
	destination.request_arm |= source.request_arm;
	destination.request_disarm |= source.request_disarm;
	destination.clear_follower_control |= source.clear_follower_control;
	destination.control_enable_acked |= source.control_enable_acked;
	destination.pair_active |= source.pair_active;
	destination.pair_lost |= source.pair_lost;
	destination.link_test_finished |= source.link_test_finished;

	if (source.request_disarm) {
		destination.disarm_reason = source.disarm_reason;
	}
}

void Airlink::parseFrames(uint64_t now_us, airlink::SessionActions &actions)
{
	_parser.check_timeout(now_us);
	airlink::Frame frame{};

	while (_parser.next(frame)) {
		airlink::ActuatorCommand command{};
		const bool actuator_candidate =
			frame.header.frame_type == airlink::FrameType::Command
			&& frame.header.message_id
			== static_cast<uint8_t>(airlink::MessageId::ActuatorCommand)
			&& decode_actuator_command(frame.payload, frame.header.payload_len, command);
		const airlink::SessionActions received = _session.receive(frame, now_us);
		mergeActions(actions, received);

		if (actuator_candidate
		    && _applied_role == static_cast<int32_t>(airlink::Role::Slave)
		    && frame.header.source_session == _session.peer_session_id()
		    && !(_last_received_control_us != 0 && now_us >= _last_received_control_us
			 && now_us - _last_received_control_us <= kPeerFreshTimeout
			 && frame.header.sequence == _last_received_control_sequence)
		    && (_session.state() == airlink::LinkState::PairActive
			|| _session.state() == airlink::LinkState::PairArmPending)) {
			_last_received_control_us = now_us;
			_last_received_control_sequence = frame.header.sequence;
			_last_received_control_session = frame.header.source_session;

			if (airlink::maximum(command.upper_motor, command.lower_motor) > 0.f) {
				_airborne_latched = true;
			}
		}
	}
}

bool Airlink::writeFrame(const airlink::Frame &frame)
{
	uint8_t encoded[airlink::kMaxFrameSize] {};
	size_t frame_len = 0;

	if (!airlink::build_frame(frame.header, frame.payload, frame.header.payload_len,
				  encoded, sizeof(encoded), frame_len)) {
		return false;
	}

	size_t sent = 0;
	int retries = 0;
	int error = 0;

	while (sent < frame_len && retries < 4) {
		const ssize_t written = _serial.write(encoded + sent, frame_len - sent);

		if (written > 0) {
			sent += static_cast<size_t>(written);
			retries = 0;

		} else if (written < 0 && errno == EAGAIN) {
			error = errno;
			++retries;
			px4_usleep(20);

		} else {
			error = written < 0 ? errno : EIO;
			break;
		}
	}

	if (sent == frame_len) {
		_last_write_errno = 0;
		return true;
	}

	++_write_failures;
	_last_write_errno = error;
	return false;
}

void Airlink::flushTx()
{
	airlink::Frame frame{};

	while (_session.pop_tx(frame)) {
		if (!writeFrame(frame)) {
			break;
		}
	}
}

void Airlink::issueVehicleArm(bool arm, uint64_t now_us)
{
	if (!arm && now_us - _last_disarm_request_us < kDisarmRepeatInterval
	    && _last_disarm_request_us != 0) {
		return;
	}

	if (arm && _arm_command_active) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now_us;
	command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	command.param1 = arm ? vehicle_command_s::ARMING_ACTION_ARM
			 : vehicle_command_s::ARMING_ACTION_DISARM;
	command.target_system = _vehicle_status.system_id;
	command.target_component = _vehicle_status.component_id;
	command.source_system = _vehicle_status.system_id;
	command.source_component = _vehicle_status.component_id;

	if (arm) {
		_arm_command_active = true;
		_arm_command_sent_us = now_us;
		// Publish the matching arming authorization before asking Commander to
		// evaluate the arm command. This prevents the request from racing the
		// 500 Hz status publication in the other work queue.
		publishStatus(now_us, false);
		PX4_INFO("AIRLINK requested local PX4 arm");

	} else {
		_last_disarm_request_us = now_us;
		_arm_command_active = false;
		PX4_WARN("AIRLINK requested local PX4 disarm");
	}

	_vehicle_command_pub.publish(command);
}

void Airlink::updateArmResult(uint64_t now_us)
{
	if (!_arm_command_active) {
		return;
	}

	if (_actuator_armed.armed) {
		_arm_command_active = false;

		if (!_session.notify_local_arm_result(true, now_us)) {
			PX4_WARN("AIRLINK observed armed without a matching arm transaction");
		}

		return;
	}

	if (_vehicle_command_ack.timestamp >= _arm_command_sent_us
	    && _vehicle_command_ack.command
	    == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM
	    && _vehicle_command_ack.result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
	    && _vehicle_command_ack.result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
		_arm_command_active = false;
		(void)_session.notify_local_arm_result(false, now_us);

		if (_applied_role == static_cast<int32_t>(airlink::Role::Master)) {
			(void)_session.request_pair_disarm(airlink::ReasonCode::Other, now_us);
		}

		issueVehicleArm(false, now_us);
		PX4_WARN("AIRLINK arm rejected by Commander (%u)",
			 static_cast<unsigned>(_vehicle_command_ack.result));
		return;
	}

	if (now_us - _arm_command_sent_us > kArmCommandTimeout) {
		_arm_command_active = false;
		(void)_session.notify_local_arm_result(false, now_us);

		if (_applied_role == static_cast<int32_t>(airlink::Role::Master)) {
			(void)_session.request_pair_disarm(airlink::ReasonCode::Other, now_us);
		}

		issueVehicleArm(false, now_us);
		PX4_WARN("AIRLINK local arm timed out");
	}
}

airlink::ActuatorCommand Airlink::toActuatorCommand(const float motors[2],
		const float servos[3]) const
{
	airlink::ActuatorCommand command{};
	command.upper_motor = motors[0];
	command.lower_motor = motors[1];
	command.servos[0] = servos[0];
	command.servos[1] = servos[1];
	command.servos[2] = servos[2];
	return command;
}

bool Airlink::pairCommandValid(const airlink_pair_command_s &command,
			       uint64_t now_us) const
{
	return command.valid
	       && fresh_sample(now_us, command.timestamp, airlink::kSensorFreshTimeoutUs)
	       && airlink::actuator_command_is_valid(
		       toActuatorCommand(command.follower_motors, command.follower_servos))
	       && airlink::actuator_command_is_valid(
		       toActuatorCommand(command.master_motors, command.master_servos));
}

void Airlink::handleCommands(uint64_t now_us)
{
	const uint32_t commands = _commands.fetch_and(0);

	if (commands == 0 && !_operator_arm_requested) {
		return;
	}

	if ((commands & CommandArm) != 0) {
		if (_applied_role != static_cast<int32_t>(airlink::Role::Master)) {
			PX4_WARN("AIRLINK pair arm requires the master role");

		} else if (_session.state() == airlink::LinkState::Ready) {
			_operator_arm_requested = true;

			if (!_session.request_control_enable(now_us)) {
				PX4_WARN("AIRLINK pair arm requires fresh CONTROL_ENABLE");
			}

		} else if (_session.state() == airlink::LinkState::PairControlReady) {
			if (!pairCommandValid(_pair_command, now_us)) {
				PX4_WARN("AIRLINK pair arm refused: fresh pair command unavailable");

			} else if (!_session.request_pair_arm(now_us)) {
				PX4_WARN("AIRLINK pair arm preconditions failed");
			} else {
				_operator_arm_requested = false;
			}

		} else {
			PX4_WARN("AIRLINK pair arm requires AIRLINK_READY/PAIR_CONTROL_READY");
		}
	}

	if ((commands & CommandDisarm) != 0) {
		if (_session.peer_session_id() != 0) {
			(void)_session.request_pair_disarm(airlink::ReasonCode::OperatorRequest, now_us);
		}

		issueVehicleArm(false, now_us);
	}

	if ((commands & CommandEnable) != 0
	    && !_session.request_control_enable(now_us)) {
		PX4_WARN("AIRLINK CONTROL_ENABLE preconditions failed");
	}

	if ((commands & CommandRelease) != 0) {
		if (!_session.request_control_release(airlink::ReasonCode::OperatorRequest, now_us)) {
			PX4_WARN("AIRLINK CONTROL_RELEASE was not sent");
		}

		if (!_airborne_latched) {
			issueVehicleArm(false, now_us);
		}
	}

	if ((commands & CommandEmergency) != 0) {
		if (!_session.request_emergency_withdraw(airlink::ReasonCode::EmergencyTrigger, now_us)) {
			PX4_WARN("AIRLINK EMERGENCY_WITHDRAW was not sent");
		}

		if (!_airborne_latched) {
			issueVehicleArm(false, now_us);
		}
	}

	if ((commands & CommandLinkTest) != 0) {
		airlink::LinkTestRequest request{};
		request.payload_len = _requested_test_payload_len;
		request.rate_hz = _requested_test_rate_hz;
		request.duration_ms = _requested_test_duration_ms;
		_test_baseline_crc_errors = _parser.stats().crc_errors;
		_test_baseline_length_errors = _parser.stats().length_errors;
		_test_baseline_session_errors = _session.session_errors();
		_test_baseline_resyncs = _parser.stats().resyncs;
		_test_baseline_buffer_overflows = _parser.stats().buffer_overflows;
		_test_baseline_partial_timeouts = _parser.stats().partial_timeouts;

		if (!_session.start_link_test(request, now_us)) {
			PX4_WARN("AIRLINK link test refused");
		}
	}

	if ((commands & CommandLinkTestAbort) != 0
	    && !_session.abort_link_test(now_us)) {
		PX4_WARN("AIRLINK link test abort refused");
	}

	if (_applied_role == static_cast<int32_t>(airlink::Role::Master)
	    && _applied_pair_enable != 0 && _session.state() == airlink::LinkState::Ready
	    && pairCommandValid(_pair_command, now_us)
	    && (_last_enable_attempt_us == 0
		|| now_us - _last_enable_attempt_us > 100'000)) {
		_last_enable_attempt_us = now_us;
		(void)_session.request_control_enable(now_us);
	}

	if (_operator_arm_requested
	    && _applied_role == static_cast<int32_t>(airlink::Role::Master)
	    && _session.state() == airlink::LinkState::PairControlReady) {
		if (pairCommandValid(_pair_command, now_us)
		    && _session.request_pair_arm(now_us)) {
			_operator_arm_requested = false;

		} else {
			_operator_arm_requested = false;
			PX4_WARN("AIRLINK operator arm request rejected");
		}
	}
}

void Airlink::handleActions(const airlink::SessionActions &actions, uint64_t now_us)
{
	if (actions.request_arm) {
		issueVehicleArm(true, now_us);
	}

	if (actions.request_disarm) {
		issueVehicleArm(false, now_us);
	}

	if (actions.clear_follower_control || actions.pair_lost) {
		airlink_actuator_command_s command{};
		command.timestamp = now_us;
		_actuator_command_pub.publish(command);
		_follower_control_published = false;
	}

	if (actions.pair_lost) {
		_pair_lost_latched = true;
	}

	if (actions.link_test_finished) {
		publishTestResult(now_us, true, _session.link_test_was_aborted());
		publishTestJson(now_us);
	}
}

void Airlink::publishFollowerCommand(uint64_t now_us)
{
	airlink_actuator_command_s output{};
	output.timestamp = now_us;
	output.timestamp_sample = _last_received_control_us;
	output.valid = _applied_role == static_cast<int32_t>(airlink::Role::Slave)
		       && _session.state() == airlink::LinkState::PairActive
		       && _last_received_control_us != 0
		       && now_us >= _last_received_control_us
		       && now_us - _last_received_control_us <= kPeerFreshTimeout;
	output.source_session = _last_received_control_session;
	output.sequence = _last_received_control_sequence;
	const airlink::ActuatorCommand &command = _session.peer_actuator_command();
	output.motors[0] = command.upper_motor;
	output.motors[1] = command.lower_motor;
	output.servos[0] = command.servos[0];
	output.servos[1] = command.servos[1];
	output.servos[2] = command.servos[2];
	_actuator_command_pub.publish(output);
	_follower_control_published = output.valid;
}

void Airlink::publishStatus(uint64_t now_us, bool pair_lost)
{
	airlink_status_s status{};
	status.timestamp = now_us;
	status.timestamp_sample = now_us;
	status.session_id = _session_id;
	status.peer_session_id = _session.peer_session_id();
	status.role = static_cast<uint8_t>(_applied_role);
	status.link_state = static_cast<uint8_t>(_session.state());
	status.peer_health_flags = _session.remote().health_flags;
	status.peer_sensor_status_flags = _session.remote().sensor_status_flags;
	status.peer_heartbeat_fresh = _session.last_peer_age_us(now_us) <= 100'000
				      && _session.last_peer_age_us(now_us) != UINT32_MAX;
	status.peer_sensor_fresh = _session.last_peer_sensor_age_us(now_us) <= kPeerFreshTimeout;
	status.peer_control_fresh = _session.last_peer_control_age_us(now_us) <= kPeerFreshTimeout;
	status.follower_active = _applied_role == static_cast<int32_t>(airlink::Role::Slave)
				 && _session.state() == airlink::LinkState::PairActive;
	status.pair_active = _session.state() == airlink::LinkState::PairArmedGround
			     || _session.state() == airlink::LinkState::PairActive;
	status.pair_lost = _pair_lost_latched || pair_lost
			   || _session.state() == airlink::LinkState::PairLost;
	status.pair_reattach_latched = _session.pair_reattach_latched();
	status.pair_mode_selected = _applied_pair_enable != 0;
	status.local_airborne_latched = _airborne_latched;
	status.local_arm_authorized = _arm_command_active
				      && _session.state() == airlink::LinkState::PairArmPending;
	status.control_tx_allowed = _session.query_control_tx_allowed(now_us);
	status.link_test_active = _session.link_test_active();
	status.rx_frames = _session.rx_frames();
	status.tx_frames = _session.tx_frames();
	status.crc_errors = _parser.stats().crc_errors;
	status.length_errors = _parser.stats().length_errors;
	status.version_errors = _parser.stats().version_errors;
	status.eof_errors = _parser.stats().eof_errors;
	status.resyncs = _parser.stats().resyncs;
	status.buffer_overflows = _parser.stats().buffer_overflows;
	status.partial_timeouts = _parser.stats().partial_timeouts;
	status.session_errors = _session.session_errors();
	status.unknown_messages = _session.unknown_messages();
	status.reliable_retries = _session.reliable_retries();
	status.duplicate_events = _session.duplicate_events();
	status.write_failures = _write_failures;
	_status_pub.publish(status);
}

void Airlink::publishTestResult(uint64_t now_us, bool complete, bool aborted)
{
	const airlink::LinkTestStats &source = _session.link_test_stats();
	airlink_test_result_s result{};
	result.timestamp = now_us;
	result.test_id = _session.link_test_id();
	result.valid = complete;
	result.complete = complete;
	result.aborted = aborted;
	result.sent = source.sent;
	result.echoed = source.echoed;
	result.duplicate = source.duplicate;
	result.reordered = source.reordered;
	result.missing = source.missing;
	result.bad_echo = source.bad_echo;
	result.crc_errors = _parser.stats().crc_errors - _test_baseline_crc_errors;
	result.length_errors = _parser.stats().length_errors - _test_baseline_length_errors;
	result.session_errors = _session.session_errors() - _test_baseline_session_errors;
	result.resyncs = _parser.stats().resyncs - _test_baseline_resyncs;
	result.buffer_overflows = _parser.stats().buffer_overflows
				  - _test_baseline_buffer_overflows;
	result.partial_timeouts = _parser.stats().partial_timeouts
				  - _test_baseline_partial_timeouts;
	result.rtt_min_us = source.rtt_min_us;
	result.rtt_mean_us = source.rtt_mean_us;
	result.rtt_p50_us = source.rtt_p50_us;
	result.rtt_p95_us = source.rtt_p95_us;
	result.rtt_p99_us = source.rtt_p99_us;
	result.rtt_max_us = source.rtt_max_us;
	result.rtt_jitter_us = source.rtt_jitter_us;
	result.max_receive_gap_us = source.max_receive_gap_us;
	result.link_interruptions = source.link_interruptions;
	result.offered_bytes_per_second = source.offered_bytes_per_second;
	result.echo_bytes_per_second = source.echo_bytes_per_second;
	result.bidirectional_bytes_per_second = source.bidirectional_bytes_per_second;
	_test_result_pub.publish(result);
}

void Airlink::publishTestJson(uint64_t now_us)
{
	if (now_us - _last_test_result_us < 1'000'000 && _last_test_result_us != 0) {
		return;
	}

	_last_test_result_us = now_us;
	const airlink::LinkTestStats &stats = _session.link_test_stats();
	PX4_INFO("AIRLINK_TEST_JSON={\"test_id\":%lu,\"sent\":%lu,\"echoed\":%lu,"
		 "\"missing\":%lu,\"duplicate\":%lu,\"reordered\":%lu,"
		 "\"rtt_min_us\":%lu,\"rtt_mean_us\":%lu,\"rtt_p50_us\":%lu,"
		 "\"rtt_p95_us\":%lu,\"rtt_p99_us\":%lu,\"rtt_max_us\":%lu,"
		 "\"rtt_jitter_us\":%lu,\"offered_bps\":%lu,\"echo_bps\":%lu,"
		 "\"bidirectional_bps\":%lu,\"buffer_overflows\":%lu}",
		 static_cast<unsigned long>(_session.link_test_id()),
		 static_cast<unsigned long>(stats.sent),
		 static_cast<unsigned long>(stats.echoed),
		 static_cast<unsigned long>(stats.missing),
		 static_cast<unsigned long>(stats.duplicate),
		 static_cast<unsigned long>(stats.reordered),
		 static_cast<unsigned long>(stats.rtt_min_us),
		 static_cast<unsigned long>(stats.rtt_mean_us),
		 static_cast<unsigned long>(stats.rtt_p50_us),
		 static_cast<unsigned long>(stats.rtt_p95_us),
		 static_cast<unsigned long>(stats.rtt_p99_us),
		 static_cast<unsigned long>(stats.rtt_max_us),
		 static_cast<unsigned long>(stats.rtt_jitter_us),
		 static_cast<unsigned long>(stats.offered_bytes_per_second),
		 static_cast<unsigned long>(stats.echo_bytes_per_second),
		 static_cast<unsigned long>(stats.bidirectional_bytes_per_second),
		 static_cast<unsigned long>(_parser.stats().buffer_overflows
					    - _test_baseline_buffer_overflows));
	mavlink_log_info(&_mavlink_log_pub,
			 "AIRLINK_TEST sent=%lu echo=%lu miss=%lu p95=%lu",
			 static_cast<unsigned long>(stats.sent),
			 static_cast<unsigned long>(stats.echoed),
			 static_cast<unsigned long>(stats.missing),
			 static_cast<unsigned long>(stats.rtt_p95_us));
}

void Airlink::Run()
{
	if (should_exit()) {
		ScheduleClear();
		closeSerial();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	updateSubscriptions();

	if (_previous_armed && !_actuator_armed.armed) {
		_airborne_latched = false;
	}

	_previous_armed = _actuator_armed.armed;
	updateParameters();
	handleExternalArming(now);
	updateArmResult(now);
	bool serial_open = openSerial();
	airlink::SessionActions actions{};

	if (serial_open) {
		_session.set_local_status(makeLocalStatus(now));
		readSerial(now, actions);
		handleCommands(now);

		if (_applied_role == static_cast<int32_t>(airlink::Role::Slave)
		    && _session.state() == airlink::LinkState::PairActive) {
			const airlink::ActuatorCommand &command = _session.peer_actuator_command();

			if (airlink::maximum(command.upper_motor, command.lower_motor) > 0.f) {
				_airborne_latched = true;
			}

			if (period_elapsed(now, _last_sensor_tx_us, kProtocolFramePeriod)
			    && _session.queue_sensor_state(makeSensorState(now), now)) {
				_last_sensor_tx_us = now;
			}
		}

		if (_applied_role == static_cast<int32_t>(airlink::Role::Master)
		    && _session.query_control_tx_allowed(now)
		    && pairCommandValid(_pair_command, now)
		    && period_elapsed(now, _last_control_tx_us, kProtocolFramePeriod)) {
			const airlink::ActuatorCommand follower =
				toActuatorCommand(_pair_command.follower_motors,
						  _pair_command.follower_servos);

			if (_session.queue_actuator_command(follower, _pair_command.collective, now)) {
				_last_control_tx_us = now;

				if (_pair_command.collective > 0.f) {
					_airborne_latched = true;
				}
			}
		}

		if (period_elapsed(now, _last_session_tick_us, kProtocolFramePeriod)) {
			mergeActions(actions, _session.tick(now));
			_last_session_tick_us = now;
		}

		handleActions(actions, now);
		publishFollowerCommand(now);

		if (period_elapsed(now, _last_status_pub_us, kProtocolFramePeriod)) {
			publishStatus(now, actions.pair_lost);
			_last_status_pub_us = now;
		}

		flushTx();

	} else {
		if (period_elapsed(now, _last_status_pub_us, kProtocolFramePeriod)) {
			publishStatus(now, false);
			_last_status_pub_us = now;
		}
	}
}

void Airlink::requestArm()
{
	_commands.fetch_or(CommandArm);
}

void Airlink::requestDisarm()
{
	_operator_arm_requested = false;
	_commands.fetch_or(CommandDisarm);
}

void Airlink::requestControlEnable()
{
	_commands.fetch_or(CommandEnable);
}

void Airlink::requestControlRelease()
{
	_commands.fetch_or(CommandRelease);
}

void Airlink::requestEmergencyWithdraw()
{
	_commands.fetch_or(CommandEmergency);
}

void Airlink::requestLinkTest(uint16_t payload_len, uint16_t rate_hz,
			      uint16_t duration_ms)
{
	_requested_test_payload_len = payload_len;
	_requested_test_rate_hz = rate_hz;
	_requested_test_duration_ms = duration_ms;
	_commands.fetch_or(CommandLinkTest);
}

void Airlink::requestLinkTestAbort()
{
	_commands.fetch_or(CommandLinkTestAbort);
}

int Airlink::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("port: %s, baud: %lu, open: %d, role: %ld",
		 _serial.getPort(), static_cast<unsigned long>(_serial.getBaudrate()),
		 _serial.isOpen() ? 1 : 0, static_cast<long>(_applied_role));
	PX4_INFO("state: %s, session: %08lx, peer: %08lx, config CRC: %08lx",
		 airlink::link_state_name(_session.state()),
		 static_cast<unsigned long>(_session_id),
		 static_cast<unsigned long>(_session.peer_session_id()),
		 static_cast<unsigned long>(_config_crc32c));
	PX4_INFO("fresh: peer %lu us, sensor %lu us, control %lu us",
		 static_cast<unsigned long>(_session.last_peer_age_us(now)),
		 static_cast<unsigned long>(_session.last_peer_sensor_age_us(now)),
		 static_cast<unsigned long>(_session.last_peer_control_age_us(now)));
	PX4_INFO("rx/tx frames: %lu/%lu, retries: %lu, duplicates: %lu",
		 static_cast<unsigned long>(_session.rx_frames()),
		 static_cast<unsigned long>(_session.tx_frames()),
		 static_cast<unsigned long>(_session.reliable_retries()),
		 static_cast<unsigned long>(_session.duplicate_events()));
	PX4_INFO("errors: crc %lu, length %lu, version %lu, eof %lu, resync %lu, buffer overflow %lu, partial timeout %lu",
		 static_cast<unsigned long>(_parser.stats().crc_errors),
		 static_cast<unsigned long>(_parser.stats().length_errors),
		 static_cast<unsigned long>(_parser.stats().version_errors),
		 static_cast<unsigned long>(_parser.stats().eof_errors),
		 static_cast<unsigned long>(_parser.stats().resyncs),
		 static_cast<unsigned long>(_parser.stats().buffer_overflows),
		 static_cast<unsigned long>(_parser.stats().partial_timeouts));
	PX4_INFO("session errors: %lu, unknown messages: %lu, write failures: %lu (errno %d), open failures: %lu",
		 static_cast<unsigned long>(_session.session_errors()),
		 static_cast<unsigned long>(_session.unknown_messages()),
		 static_cast<unsigned long>(_write_failures), _last_write_errno,
		 static_cast<unsigned long>(_open_failures));
	PX4_INFO("pair: active %d, follower %d, reattach latched %d, local arm authorized %d",
		 _session.state() == airlink::LinkState::PairActive ? 1 : 0,
		 _applied_role == static_cast<int32_t>(airlink::Role::Slave)
		 && _session.state() == airlink::LinkState::PairActive ? 1 : 0,
		 _session.pair_reattach_latched() ? 1 : 0,
		 _session.state() == airlink::LinkState::PairArmPending ? 1 : 0);
	PX4_INFO("local: armed %d, airborne latched %d, pair mode %ld, RC valid %d, safety allow %d",
		 _actuator_armed.armed ? 1 : 0, _airborne_latched ? 1 : 0,
		 static_cast<long>(_applied_pair_enable), _input_rc.rc_lost ? 0 : 1,
		 makeLocalStatus(now).safety_gate_allow ? 1 : 0);
	return 0;
}

int Airlink::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS1";
	uint32_t baudrate = 6'000'000;

	for (int i = 1; i < argc; ++i) {
		if (!strcmp(argv[i], "-d") && i + 1 < argc) {
			device = argv[++i];

		} else if (!strcmp(argv[i], "-b") && i + 1 < argc) {
			baudrate = static_cast<uint32_t>(strtoul(argv[++i], nullptr, 10));

		} else {
			return print_usage("unexpected argument");
		}
	}

	Airlink *instance = new Airlink(device, baudrate);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	PX4_ERR("alloc/init failed");
	return PX4_ERROR;
}

int Airlink::custom_command(int argc, char *argv[])
{
	if (argc == 0 || get_instance() == nullptr) {
		return print_usage("module not running");
	}

	Airlink *instance = get_instance();

	if (!strcmp(argv[0], "arm")) {
		instance->requestArm();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "disarm")) {
		instance->requestDisarm();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "enable")) {
		instance->requestControlEnable();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "release")) {
		instance->requestControlRelease();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "emergency")) {
		instance->requestEmergencyWithdraw();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "role") && argc == 2) {
		int32_t role = -1;

		if (!strcmp(argv[1], "off")) {
			role = 0;

		} else if (!strcmp(argv[1], "master")) {
			role = 1;

		} else if (!strcmp(argv[1], "slave")) {
			role = 2;
		}

		if (role < 0) {
			return print_usage("role must be off, master or slave");
		}

		return param_set(param_find("AIRLINK_ROLE"), &role) == PX4_OK ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "pair") && argc == 2) {
		int32_t enabled = -1;

		if (!strcmp(argv[1], "on") || !strcmp(argv[1], "1")) {
			enabled = 1;

		} else if (!strcmp(argv[1], "off") || !strcmp(argv[1], "0")) {
			enabled = 0;
		}

		if (enabled < 0) {
			return print_usage("pair must be on or off");
		}

		return param_set(param_find("AIRLINK_PAIR_EN"), &enabled) == PX4_OK
		       ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "test")) {
		uint16_t duration_ms = 5000;
		uint16_t rate_hz = 500;
		uint16_t payload_len = 256;

		for (int i = 1; i < argc; ++i) {
			if (!strcmp(argv[i], "--duration-ms") && i + 1 < argc) {
				if (!parse_u16_in_range(argv[++i], 1000, 10000, duration_ms)) {
					return print_usage("duration must be 1000..10000 ms");
				}

			} else if (!strcmp(argv[i], "--rate-hz") && i + 1 < argc) {
				if (!parse_u16_in_range(argv[++i], 1, 500, rate_hz)) {
					return print_usage("rate must be 1..500 Hz");
				}

			} else if (!strcmp(argv[i], "--payload-bytes") && i + 1 < argc) {
				if (!parse_u16_in_range(argv[++i], 32, 512, payload_len)) {
					return print_usage("payload must be 32..512 bytes");
				}

			} else {
				return print_usage("invalid test argument");
			}
		}

		instance->requestLinkTest(payload_len, rate_hz, duration_ms);
		PX4_INFO("AIRLINK link test queued: duration=%u ms, rate=%u Hz, payload=%u B",
			 static_cast<unsigned>(duration_ms), static_cast<unsigned>(rate_hz),
			 static_cast<unsigned>(payload_len));
		return PX4_OK;
	}

	if (!strcmp(argv[0], "aborttest")) {
		instance->requestLinkTestAbort();
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int Airlink::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
AIRLINK v1 dual-aircraft 6 Mbaud full-duplex serial transport.

The module exchanges 500 Hz sensor state and five-output actuator commands,
implements the three-phase handshake, reliable events, explicit PAIR_ARM and
PAIR_DISARM transactions, and a loopback link test. AIRLINK_READY alone never
grants actuator authority. PX4 Commander, RC, failsafe and actuator gates keep
their veto.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airlink", "communication");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start on /dev/ttyS1 at 6000000 8N1");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", nullptr,
					"Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 6000000, 1, 6000000, "Baudrate", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("role", "Write AIRLINK_ROLE: off|master|slave");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pair", "Select paired mode: on|off");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable", "Request fresh CONTROL_ENABLE");
	PRINT_MODULE_USAGE_COMMAND_DESCR("arm", "Start explicit pair arm transaction");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disarm", "Request paired local disarm");
	PRINT_MODULE_USAGE_COMMAND_DESCR("release", "Release remote control ownership");
	PRINT_MODULE_USAGE_COMMAND_DESCR("emergency", "Send sticky emergency withdraw");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Loopback link test");
	PRINT_MODULE_USAGE_ARG("--duration-ms", "1000..10000 (default 5000)", false);
	PRINT_MODULE_USAGE_ARG("--rate-hz", "1..500 (default 500)", false);
	PRINT_MODULE_USAGE_ARG("--payload-bytes", "32..512 (default 256)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("aborttest", "Abort an active link test");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int airlink_main(int argc, char *argv[])
{
	return Airlink::main(argc, argv);
}
