/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "AirlinkSession.hpp"
#include "AirlinkImu.hpp"

#include <drivers/drv_hrt.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/airlink_actuator_command.h>
#include <uORB/topics/airlink_pair_command.h>
#include <uORB/topics/airlink_sensor_state.h>
#include <uORB/topics/airlink_status.h>
#include <uORB/topics/airlink_test_result.h>
#include <uORB/topics/airlink_yaw_reference.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/gs_link_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/composite_yaw_reference.h>

#include <cstddef>
#include <cstdint>

using namespace time_literals;

class Airlink : public ModuleBase<Airlink>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Airlink(const char *device, uint32_t baudrate);
	~Airlink() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	void requestArm();
	void requestDisarm();
	void requestControlEnable();
	void requestControlRelease();
	void requestEmergencyWithdraw();
	void requestLinkTest(uint16_t payload_len, uint16_t rate_hz, uint16_t duration_ms);
	void requestLinkTestAbort();
	void requestSensorTest(uint16_t duration_ms);

private:
	enum CommandRequest : uint32_t {
		CommandArm = 1u << 0,
		CommandDisarm = 1u << 1,
		CommandEnable = 1u << 2,
		CommandRelease = 1u << 3,
		CommandEmergency = 1u << 4,
		CommandLinkTest = 1u << 5,
		CommandLinkTestAbort = 1u << 6,
		CommandSensorTest = 1u << 7,
	};

	void Run() override;
	void updateSubscriptions();
	void updateParameters(bool force = false);
	void handleExternalArming(uint64_t now_us);
	void applyRequestedRole();
	void applyRequestedPairMode();
	void initializeSession(uint64_t now_us);
	uint32_t generateSessionId(uint64_t now_us);
	uint32_t computeConfigCrc32c();
	airlink::LocalStatus makeLocalStatus(uint64_t now_us);
	airlink::SensorState makeSensorState(uint64_t now_us);
	void mergeActions(airlink::SessionActions &destination,
			  const airlink::SessionActions &source);
	void handleActions(const airlink::SessionActions &actions, uint64_t now_us);
	void handleCommands(uint64_t now_us);
	bool openSerial();
	void closeSerial();
	void readSerial(uint64_t now_us, airlink::SessionActions &actions);
	void parseFrames(uint64_t now_us, airlink::SessionActions &actions);
	bool writeFrame(const airlink::Frame &frame);
	void flushTx();
	void issueVehicleArm(bool arm, uint64_t now_us);
	void updateArmResult(uint64_t now_us);
	void publishFollowerCommand(uint64_t now_us);
	void publishStatus(uint64_t now_us, bool pair_lost);
	void publishTestResult(uint64_t now_us, bool complete, bool aborted);
	void publishTestJson(uint64_t now_us);
	void publishPeerSensor(const airlink::Frame &frame, const airlink::SensorState &sensor,
			       uint64_t now_us);
	void publishPeerYaw(const airlink::Frame &frame, const airlink::YawReference &reference,
			    uint64_t now_us);
	void finishSensorTest(uint64_t now_us);
	bool pairCommandValid(const airlink_pair_command_s &command, uint64_t now_us) const;
	airlink::ActuatorCommand toActuatorCommand(const float motors[2], const float servos[3]) const;

	// Receive at 1 kHz for lower event latency, but keep protocol data streams
	// at their contracted 500 Hz rate.
	static constexpr hrt_abstime kRunInterval{1_ms};
	static constexpr hrt_abstime kProtocolFramePeriod{2_ms};
	static constexpr hrt_abstime kPeerFreshTimeout{10_ms};
	static constexpr hrt_abstime kRcTimeout{200_ms};
	static constexpr hrt_abstime kEscTimeout{100_ms};
	static constexpr hrt_abstime kBatteryTimeout{1_s};
	static constexpr hrt_abstime kGsLinkTimeout{100_ms};
	static constexpr hrt_abstime kArmCommandTimeout{100_ms};
	static constexpr hrt_abstime kDisarmRepeatInterval{50_ms};

	device::Serial _serial;
	airlink::StreamParser _parser{};
	airlink::Session _session{0};
	uint32_t _config_crc32c{0};
	int32_t _applied_role{-1};
	int32_t _applied_pair_enable{-1};
	uint32_t _session_id{0};
	uint32_t _board_id{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _action_request_sub{ORB_ID(action_request)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _gs_link_status_sub{ORB_ID(gs_link_status)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	airlink::ImuSource _imu_source{};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _pair_command_sub{ORB_ID(airlink_pair_command)};
	uORB::Subscription _yaw_reference_sub{ORB_ID(composite_yaw_reference)};

	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<airlink_actuator_command_s>
	_actuator_command_pub{ORB_ID(airlink_actuator_command)};
	uORB::Publication<airlink_status_s> _status_pub{ORB_ID(airlink_status)};
	uORB::Publication<airlink_test_result_s> _test_result_pub{ORB_ID(airlink_test_result)};
	uORB::Publication<airlink_sensor_state_s> _sensor_state_pub{ORB_ID(airlink_sensor_state)};
	uORB::Publication<airlink_yaw_reference_s> _yaw_reference_pub{ORB_ID(airlink_yaw_reference)};

	actuator_armed_s _actuator_armed{};
	action_request_s _action_request{};
	battery_status_s _battery_status{};
	esc_status_s _esc_status{};
	failsafe_flags_s _failsafe_flags{};
	gs_link_status_s _gs_link_status{};
	input_rc_s _input_rc{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_status_s _vehicle_status{};
	vehicle_command_ack_s _vehicle_command_ack{};
	vehicle_command_s _vehicle_command{};
	airlink_pair_command_s _pair_command{};
	composite_yaw_reference_s _yaw_reference{};

	px4::atomic<uint32_t> _commands{0};
	uint16_t _requested_test_payload_len{256};
	uint16_t _requested_test_rate_hz{500};
	uint16_t _requested_test_duration_ms{5000};
	uint16_t _requested_sensor_test_duration_ms{5000};

	uint64_t _last_received_control_us{0};
	uint16_t _last_received_control_sequence{0};
	uint32_t _last_received_control_session{0};
	uint64_t _last_disarm_request_us{0};
	uint64_t _arm_command_sent_us{0};
	uint64_t _last_enable_attempt_us{0};
	uint64_t _last_session_tick_us{0};
	uint64_t _last_sensor_tx_us{0};
	uint64_t _last_control_tx_us{0};
	uint64_t _last_yaw_tx_us{0};
	uint64_t _last_status_pub_us{0};
	uint64_t _last_test_result_us{0};
	uint64_t _sensor_test_until_us{0};
	uint32_t _test_baseline_crc_errors{0};
	uint32_t _test_baseline_length_errors{0};
	uint32_t _test_baseline_session_errors{0};
	uint32_t _test_baseline_resyncs{0};
	uint32_t _test_baseline_buffer_overflows{0};
	uint32_t _test_baseline_partial_timeouts{0};
	uint32_t _write_failures{0};
	uint32_t _sensor_rx_frames{0};
	uint32_t _sensor_bad_frames{0};
	uint32_t _sensor_duplicate_frames{0};
	uint32_t _sensor_gap_frames{0};
	uint32_t _sensor_test_start_rx{0};
	uint32_t _sensor_test_start_bad{0};
	uint32_t _sensor_test_start_duplicate{0};
	uint32_t _sensor_test_start_gap{0};
	uint16_t _sensor_last_sequence{0};
	bool _sensor_has_last_sequence{false};
	int _last_write_errno{0};
	uint32_t _open_failures{0};
	bool _arm_command_active{false};
	bool _airborne_latched{false};
	bool _previous_armed{false};
	bool _pair_lost_latched{false};
	bool _follower_control_published{false};
	bool _operator_arm_requested{false};
	bool _sensor_test_active{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AIRLINK_ROLE>) _param_role,
		(ParamInt<px4::params::AIRLINK_PAIR_EN>) _param_pair_enable
	)
};

extern "C" __EXPORT int airlink_main(int argc, char *argv[]);
