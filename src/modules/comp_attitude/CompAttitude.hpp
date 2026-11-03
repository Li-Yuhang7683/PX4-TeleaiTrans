/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "CompositeMath.hpp"
#include "../airlink/AirlinkImu.hpp"

#include <modules/ekf2/EKF/ekf.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/airlink_sensor_state.h>
#include <uORB/topics/airlink_status.h>
#include <uORB/topics/composite_attitude_estimate.h>
#include <uORB/topics/composite_estimator_status.h>
#include <uORB/topics/composite_yaw_reference.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>


#include <cstddef>
#include <cstdint>

class CompAttitude final : public ModuleBase<CompAttitude>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	CompAttitude();
	~CompAttitude() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	void requestReset();


private:
	enum class EstimatorState : uint8_t {
		Initializing = composite_estimator_status_s::STATE_INITIALIZING,
		Ready = composite_estimator_status_s::STATE_READY,
		Degraded = composite_estimator_status_s::STATE_DEGRADED,
	};

	struct TimedImu {
		uint64_t timestamp_us{0};
		composite::Vec3 angular_velocity{};
		composite::Vec3 specific_force{};
		composite::Vec3 angular_acceleration{};
	};

	struct VirtualSample {
		uint64_t timestamp_us{0};
		float dt_s{0.f};
		composite::Vec3 angular_velocity{};
		composite::Vec3 specific_force{};
		composite::Vec3 angular_acceleration{};
		uint64_t age_a_us{0};
		uint64_t age_b_us{0};
	};

	struct AppliedParameters {
		float master_yaw_rad{0.f};
		float slave_yaw_rad{0.f};
		float yaw_tau_s{0.f};
		float yaw_sigma_rad{0.f};
		bool lever_enabled{false};
		composite::Vec3 position_a{};
		composite::Vec3 position_b{};
	};

	void Run() override;
	void updateParameters(bool force = false);
	void applyEkfParameters();
	void resetEstimator(uint64_t now_us, const char *reason);
	bool updateLocalSource(uint64_t now_us);
	bool initializeAttitudeFromLocalSources(uint64_t sample_time_us);
	void processVirtualSample(const VirtualSample &sample, uint64_t now_us);
	void publishEstimate(uint64_t now_us, const VirtualSample &sample, const composite::Quat &q_c);
	void publishStatus(uint64_t now_us);
	void publishYawReference(uint64_t now_us);
	static composite::Quat quatFromMessage(const float q[4]);
	static composite::Quat quatFromMatrix(const matrix::Quatf &q);

	static constexpr hrt_abstime kRunInterval{1'000};
	static constexpr uint64_t kStatusPeriodUs{100'000};
	static constexpr uint64_t kYawPeriodUs{1'000'000};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	airlink::ImuSource _imu_source{};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _airlink_sensor_sub{ORB_ID(airlink_sensor_state)};
	uORB::Subscription _airlink_status_sub{ORB_ID(airlink_status)};
	uORB::Publication<composite_attitude_estimate_s> _estimate_pub{ORB_ID(composite_attitude_estimate)};
	uORB::Publication<composite_estimator_status_s> _status_pub{ORB_ID(composite_estimator_status)};
	uORB::Publication<composite_yaw_reference_s> _yaw_reference_pub{ORB_ID(composite_yaw_reference)};
	uORB::Publication<mount_orientation_s> _mount_orientation_pub{ORB_ID(mount_orientation)};
	actuator_armed_s _actuator_armed{};
	vehicle_attitude_s _attitude_a{};
	vehicle_land_detected_s _vehicle_land_detected{};
	airlink_sensor_state_s _peer_sensor{};
	airlink_status_s _link{};
	Ekf *_ekf{nullptr};
	composite::Quat _q_ca{}, _q_cb{}, _q_ab{}, _latest_q_c{};
	composite::YawAnchor _yaw_anchor{};
	TimedImu _local_imu{};
	float _local_dt_s{0.001f};
	composite::Vec3 _previous_omega{}, _filtered_alpha{};
	bool _has_previous_omega{false}, _peer_available{false}, _external_attitude_initialized{false};
	EstimatorState _state{EstimatorState::Initializing};
	uint64_t _last_common_us{0}, _last_status_pub_us{0}, _last_yaw_pub_us{0};
	uint32_t _peer_session{0}, _frame_count{0}, _yaw_reference_count{0}, _selected_imu_instance{0};
	uint8_t _reset_counter{0};
	px4::atomic<uint32_t> _commands{0};
	AppliedParameters _applied{};

	DEFINE_PARAMETERS(
	    (ParamBool<px4::params::CEKF_EN>) _param_enabled,
	    (ParamFloat<px4::params::CEKF_MSTR_YAW>) _param_master_yaw_deg,
	    (ParamFloat<px4::params::CEKF_SLV_YAW>) _param_slave_yaw_deg,
	    (ParamInt<px4::params::CEKF_PREDICT_US>) _param_predict_us,
	    (ParamFloat<px4::params::CEKF_GYR_NOISE>) _param_gyro_noise,
	    (ParamFloat<px4::params::CEKF_ACC_NOISE>) _param_accel_noise,
	    (ParamFloat<px4::params::CEKF_ANGERR>) _param_angle_error,
	    (ParamFloat<px4::params::CEKF_GBIAS>) _param_gyro_bias,
	    (ParamFloat<px4::params::CEKF_ABIAS>) _param_accel_bias,
	    (ParamFloat<px4::params::CEKF_GB_NOISE>) _param_gyro_bias_noise,
	    (ParamFloat<px4::params::CEKF_AB_NOISE>) _param_accel_bias_noise,
	    (ParamInt<px4::params::CEKF_IMU_CTRL>) _param_imu_ctrl,
	    (ParamFloat<px4::params::CEKF_YAW_TAU>) _param_yaw_tau,
	    (ParamFloat<px4::params::CEKF_YAW_NSD>) _param_yaw_noise,
	    (ParamBool<px4::params::CEKF_LEVER_EN>) _param_lever_enabled,
	    (ParamFloat<px4::params::CEKF_POS_AX>) _param_pos_a_x,
	    (ParamFloat<px4::params::CEKF_POS_AY>) _param_pos_a_y,
	    (ParamFloat<px4::params::CEKF_POS_AZ>) _param_pos_a_z,
	    (ParamFloat<px4::params::CEKF_POS_BX>) _param_pos_b_x,
	    (ParamFloat<px4::params::CEKF_POS_BY>) _param_pos_b_y,
	    (ParamFloat<px4::params::CEKF_POS_BZ>) _param_pos_b_z
	)
};

extern "C" __EXPORT int comp_attitude_main(int argc, char *argv[]);
