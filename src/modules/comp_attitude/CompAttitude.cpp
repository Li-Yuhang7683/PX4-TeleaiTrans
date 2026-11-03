/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "CompAttitude.hpp"

#include "../airlink/AirlinkProtocol.hpp"

#include <px4_platform_common/log.h>

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace time_literals;

namespace
{

constexpr float kLeverFilterTau{0.05f};
// Ekf::update() needs the same stack budget as the stock EKF2 instance.
// Keep the composite estimator on its own queue so it cannot overflow the
// smaller nav_and_controllers stack and cannot delay the vehicle controllers.
constexpr px4::wq_config_t kCompositeEkfWorkQueue{
	"wq:composite_ekf", 6144, -14
};
} // namespace

CompAttitude::CompAttitude() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, kCompositeEkfWorkQueue)
{
}

CompAttitude::~CompAttitude()
{
	delete _ekf;
}

bool CompAttitude::init()
{
	updateParameters(true);
	ScheduleOnInterval(kRunInterval);
	return true;
}

void CompAttitude::updateParameters(bool force)
{
	if (!force && !_parameter_update_sub.updated()) {
		return;
	}
	parameter_update_s update{};
	_parameter_update_sub.copy(&update);
	updateParams();
	_applied.master_yaw_rad = math::radians(_param_master_yaw_deg.get());
	_applied.slave_yaw_rad = math::radians(_param_slave_yaw_deg.get());
	_applied.yaw_tau_s = _param_yaw_tau.get();
	_applied.yaw_sigma_rad = _param_yaw_noise.get();
	_applied.lever_enabled = _param_lever_enabled.get();
	_applied.position_a = {_param_pos_a_x.get(), _param_pos_a_y.get(), _param_pos_a_z.get()};
	_applied.position_b = {_param_pos_b_x.get(), _param_pos_b_y.get(), _param_pos_b_z.get()};
	_q_ca = composite::vehicleToCompositeQuatFromMountYaw(_applied.master_yaw_rad);
	_q_cb = composite::vehicleToCompositeQuatFromMountYaw(_applied.slave_yaw_rad);
	_q_ab = composite::inverse_unit(_q_ca) * _q_cb;
	if (_param_enabled.get() && !_ekf) {
		resetEstimator(hrt_absolute_time(), "enabled");
	}
	if (!_param_enabled.get()) {
		delete _ekf;
		_ekf = nullptr;
		_external_attitude_initialized = false;
		_state = EstimatorState::Initializing;
	}
	applyEkfParameters();
}

void CompAttitude::applyEkfParameters()
{
	if (_ekf == nullptr) {
		return;
	}

	parameters *const params = _ekf->getParamHandle();
	params->filter_update_interval_us = _param_predict_us.get();
	params->delay_max_ms = 50.f;
	params->imu_ctrl = _param_imu_ctrl.get();
	params->gyro_noise = _param_gyro_noise.get();
	params->accel_noise = _param_accel_noise.get();
	params->gyro_bias_p_noise = _param_gyro_bias_noise.get();
	params->accel_bias_p_noise = _param_accel_bias_noise.get();
	params->initial_tilt_err = _param_angle_error.get();
	params->switch_on_gyro_bias = _param_gyro_bias.get();
	params->switch_on_accel_bias = _param_accel_bias.get();
	params->imu_pos_body.zero();
	params->valid_timeout_max = 10'000'000;

#if defined(CONFIG_EKF2_BAROMETER)
	params->baro_ctrl = 0;
#endif
#if defined(CONFIG_EKF2_GNSS)
	params->gnss_ctrl = 0;
#endif
#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	params->ev_ctrl = 0;
#endif
#if defined(CONFIG_EKF2_RANGE_FINDER)
	params->rng_ctrl = 0;
#endif
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	params->flow_ctrl = 0;
#endif
#if defined(CONFIG_EKF2_AIRSPEED)
	params->arsp_thr = 0.f;
#endif
#if defined(CONFIG_EKF2_SIDESLIP)
	params->beta_fusion_enabled = 0;
#endif

	_ekf->updateParameters();
}

void CompAttitude::resetEstimator(uint64_t now_us, const char *reason)
{
	delete _ekf;
	_ekf = new Ekf();
	_external_attitude_initialized = false;
	_state = EstimatorState::Initializing;
	_last_common_us = 0;
	_has_previous_omega = false;
	_filtered_alpha = {};
	if (!_ekf) {
		return;
	}
	applyEkfParameters();
	if (!_ekf->init(now_us)) {
		delete _ekf;
		_ekf = nullptr;
		return;
	}
	++_reset_counter;
	PX4_INFO("CEKF reset: %s", reason);
	initializeAttitudeFromLocalSources(now_us);
}

bool CompAttitude::initializeAttitudeFromLocalSources(uint64_t)
{
	if (!_ekf || _external_attitude_initialized || !_attitude_a.timestamp) {
		return false;
	}
	const composite::Quat q = composite::attitudeInCompositeFrame(
	                              quatFromMessage(_attitude_a.q), _applied.master_yaw_rad);
	if (!q.normalized()) {
		return false;
	}
	const float values[4] {q.w, q.x, q.y, q.z};
	const float tilt = _param_angle_error.get() * _param_angle_error.get();
	const float yaw = _applied.yaw_sigma_rad * _applied.yaw_sigma_rad;
	_ekf->initializeAttitudeFromExternalObservation(matrix::Quatf(values), {tilt, tilt, yaw});
	_external_attitude_initialized = true;
	_latest_q_c = q;
	_yaw_anchor.reset(composite::yaw(q));
	return true;
}

bool CompAttitude::updateLocalSource(uint64_t now_us)
{
	_imu_source.update();
	float acceleration[3] {}, angular_velocity[3] {};
	if (!_imu_source.acceleration(acceleration) || !_imu_source.angularVelocity(angular_velocity)) {
		return false;
	}
	const uint64_t timestamp = _imu_source.timestampSample();
	if (timestamp <= _local_imu.timestamp_us) {
		return false;
	}
	_local_dt_s = _imu_source.angularDtSeconds();
	_selected_imu_instance = _imu_source.gyroInstance();
	_local_imu = TimedImu{timestamp,
		{angular_velocity[0], angular_velocity[1], angular_velocity[2]},
		{acceleration[0], acceleration[1], acceleration[2]}, {}};
	return true;
}

void CompAttitude::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	const uint64_t now_us = hrt_absolute_time();
	updateParameters();
	_actuator_armed_sub.update(&_actuator_armed);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_attitude_sub.update(&_attitude_a);
	const bool local_updated = updateLocalSource(now_us);
	const bool peer_updated = _airlink_sensor_sub.update(&_peer_sensor);
	_airlink_status_sub.update(&_link);
	// Link freshness belongs to Airlink. A scheduler tick without a packet is not a disconnect.
	const bool peer_imu_valid = airlink::sensor_imu_valid(_peer_sensor.status_flags,
				   _peer_sensor.imu_sample_time_us);
	const bool connected = peer_imu_valid && _link.role == airlink_status_s::ROLE_MASTER
	                       && _link.peer_heartbeat_fresh && _link.peer_sensor_fresh
	                       && now_us >= _link.timestamp && now_us - _link.timestamp <= 100'000;
	const bool new_connection = connected && (!_peer_available || _peer_session != _link.peer_session_id);
	_peer_available = connected;
	_peer_session = _link.peer_session_id;
	if (!_ekf) {
		publishStatus(now_us);
		return;
	}
	if (new_connection || (_commands.fetch_and(0) & 1u)) {
		resetEstimator(now_us, new_connection ? "AIRLINK connected" : "CLI reset");
	}
	if (!initializeAttitudeFromLocalSources(now_us) && !_external_attitude_initialized) {
		publishStatus(now_us);
		return;
	}
	if (_local_imu.timestamp_us && (local_updated || (connected && (peer_updated || new_connection)))) {
		const TimedImu &local = _local_imu;
		// Use the local sample clock for both modes to keep integration time continuous.
		if (local.timestamp_us > _last_common_us) {
			VirtualSample sample{};
			sample.timestamp_us = local.timestamp_us;
			sample.age_a_us = now_us >= local.timestamp_us ? now_us - local.timestamp_us : 0;
			sample.dt_s = _last_common_us ? (local.timestamp_us - _last_common_us) * 1e-6f : _local_dt_s;
			sample.angular_velocity = composite::rotate(_q_ca, local.angular_velocity);
			sample.specific_force = composite::rotate(_q_ca, local.specific_force);
			if (connected) {
				sample.age_b_us = now_us >= _peer_sensor.timestamp ? now_us - _peer_sensor.timestamp : 0;
				sample.angular_velocity = (sample.angular_velocity + composite::rotate(_q_cb,
				{_peer_sensor.gyro_raw[0] / 800.f, _peer_sensor.gyro_raw[1] / 800.f, _peer_sensor.gyro_raw[2] / 800.f})) * 0.5f;
				sample.specific_force = (sample.specific_force + composite::rotate(_q_cb, {
					_peer_sensor.accelerometer_raw[0] / 128.f, _peer_sensor.accelerometer_raw[1] / 128.f,
					_peer_sensor.accelerometer_raw[2] / 128.f
				})) * 0.5f;
				++_frame_count;
			}
			if (_has_previous_omega) {
				_filtered_alpha += ((sample.angular_velocity - _previous_omega) * (1.f / sample.dt_s)
				                    - _filtered_alpha) * (sample.dt_s / (kLeverFilterTau + sample.dt_s));
			}
			_previous_omega = sample.angular_velocity;
			_has_previous_omega = true;
			sample.angular_acceleration = _filtered_alpha;
			if (_applied.lever_enabled) {
				const auto correction = [&](const composite::Vec3 &position) {
					return composite::cross(_filtered_alpha, position)
					       + composite::cross(sample.angular_velocity, composite::cross(sample.angular_velocity, position));
				};
				sample.specific_force = sample.specific_force - correction(_applied.position_a) * (connected ? 0.5f : 1.f);
				if (connected) {
					sample.specific_force = sample.specific_force - correction(_applied.position_b) * 0.5f;
				}
			}
			_last_common_us = sample.timestamp_us;
			processVirtualSample(sample, now_us);
		}
	}
	publishStatus(now_us);
	publishYawReference(now_us);
}

void CompAttitude::processVirtualSample(const VirtualSample &sample, uint64_t now_us)
{
	imuSample imu{};
	imu.time_us = sample.timestamp_us;
	imu.delta_ang_dt = sample.dt_s;
	imu.delta_vel_dt = sample.dt_s;
	imu.delta_ang = {sample.angular_velocity.x * sample.dt_s, sample.angular_velocity.y * sample.dt_s,
	                 sample.angular_velocity.z * sample.dt_s
	                };
	imu.delta_vel = {sample.specific_force.x * sample.dt_s, sample.specific_force.y * sample.dt_s,
	                 sample.specific_force.z * sample.dt_s
	                };
	systemFlagUpdate flags{};
	flags.time_us = sample.timestamp_us;
	flags.in_air = _vehicle_land_detected.timestamp ? !_vehicle_land_detected.landed : _actuator_armed.armed;
	flags.at_rest = _vehicle_land_detected.timestamp && _vehicle_land_detected.at_rest;
	_ekf->setSystemFlagData(flags);
	_ekf->setIMUData(imu);
	_ekf->update();
	_latest_q_c = quatFromMatrix(_ekf->getQuaternion());
	_state = _peer_available ? EstimatorState::Ready : EstimatorState::Degraded;
	_yaw_anchor.update(composite::yaw(_latest_q_c), sample.dt_s, _applied.yaw_tau_s);
	publishEstimate(now_us, sample, _latest_q_c);
}

void CompAttitude::publishEstimate(uint64_t now_us, const VirtualSample &sample,
                                   const composite::Quat &q_c)
{
	const float q_values[4] {q_c.w, q_c.x, q_c.y, q_c.z};
	const matrix::Eulerf euler{matrix::Quatf(q_values)};
	composite_attitude_estimate_s estimate{};
	estimate.timestamp = now_us;
	estimate.timestamp_sample = sample.timestamp_us;
	estimate.q[0] = q_c.w;
	estimate.q[1] = q_c.x;
	estimate.q[2] = q_c.y;
	estimate.q[3] = q_c.z;
	estimate.angular_velocity[0] = sample.angular_velocity.x;
	estimate.angular_velocity[1] = sample.angular_velocity.y;
	estimate.angular_velocity[2] = sample.angular_velocity.z;
	estimate.angular_acceleration[0] = sample.angular_acceleration.x;
	estimate.angular_acceleration[1] = sample.angular_acceleration.y;
	estimate.angular_acceleration[2] = sample.angular_acceleration.z;
	estimate.specific_force[0] = sample.specific_force.x;
	estimate.specific_force[1] = sample.specific_force.y;
	estimate.specific_force[2] = sample.specific_force.z;
	estimate.yaw_sigma_rad = _applied.yaw_sigma_rad;
	estimate.reset_counter = _reset_counter;
	estimate.source_age_a_us = static_cast<uint32_t>(
	                               sample.age_a_us > UINT32_MAX ? UINT32_MAX : sample.age_a_us);
	estimate.source_age_b_us = static_cast<uint32_t>(
	                               sample.age_b_us > UINT32_MAX ? UINT32_MAX : sample.age_b_us);
	estimate.health_flags = composite_attitude_estimate_s::HEALTH_SOURCE_A_FRESH
	                        | composite_attitude_estimate_s::HEALTH_EKF_VALID;
	if (_peer_available) {
		estimate.health_flags |= composite_attitude_estimate_s::HEALTH_SOURCE_B_FRESH;
	}

	if (_state == EstimatorState::Degraded) {
		estimate.health_flags |= composite_attitude_estimate_s::HEALTH_DEGRADED;
	}

	estimate.valid = _state == EstimatorState::Ready || _state == EstimatorState::Degraded;
	_estimate_pub.publish(estimate);

	mount_orientation_s mount{};
	mount.timestamp = now_us;
	mount.attitude_euler_angle[0] = euler.phi();
	mount.attitude_euler_angle[1] = euler.theta();
	mount.attitude_euler_angle[2] = euler.psi();
	_mount_orientation_pub.publish(mount);
}

void CompAttitude::publishStatus(uint64_t now_us)
{
	if (now_us - _last_status_pub_us < kStatusPeriodUs) {
		return;
	}
	_last_status_pub_us = now_us;
	composite_estimator_status_s status{};
	status.timestamp = now_us;
	status.state = static_cast<uint8_t>(_state);
	status.enabled = _param_enabled.get();
	status.attitude_initialized = _external_attitude_initialized;
	status.peer_connected = _peer_available;
	status.weight_a = _peer_available ? 0.5f : 1.f;
	status.weight_b = _peer_available ? 0.5f : 0.f;
	status.frame_count = _frame_count;
	status.selected_imu_instance = _selected_imu_instance;
	status.local_imu_samples = _local_imu.timestamp_us ? 1 : 0;
	status.yaw_reference_rad = _yaw_anchor.initialized() ? _yaw_anchor.yaw() : NAN;
	status.yaw_reference_count = _yaw_reference_count;
	_status_pub.publish(status);
}

void CompAttitude::publishYawReference(uint64_t now_us)
{
	if ((_state != EstimatorState::Ready && _state != EstimatorState::Degraded)
	    || !_yaw_anchor.initialized()
	    || (_last_yaw_pub_us != 0 && now_us - _last_yaw_pub_us < kYawPeriodUs)) {
		return;
	}

	_last_yaw_pub_us = now_us;
	composite_yaw_reference_s reference{};
	reference.timestamp = now_us;
	reference.timestamp_sample = now_us;
	reference.yaw_rad = _yaw_anchor.yaw();
	reference.yaw_sigma_rad = _applied.yaw_sigma_rad;
	reference.reset_counter = _reset_counter;
	reference.quality = _state == EstimatorState::Ready
	                    ? composite_yaw_reference_s::QUALITY_NORMAL
	                    : composite_yaw_reference_s::QUALITY_DEGRADED;
	reference.flags = composite_yaw_reference_s::FLAG_VALID;

	if (_state == EstimatorState::Degraded) {
		reference.flags |= composite_yaw_reference_s::FLAG_DEGRADED;
	}

	_yaw_reference_pub.publish(reference);
	++_yaw_reference_count;
}

composite::Quat CompAttitude::quatFromMessage(const float q[4])
{
	const composite::Quat result{q[0], q[1], q[2], q[3]};
	return result;
}

composite::Quat CompAttitude::quatFromMatrix(const matrix::Quatf &q)
{
	float values[4] {};
	q.copyTo(values);
	return {values[0], values[1], values[2], values[3]};
}

void CompAttitude::requestReset()
{
	PX4_INFO("composite reset requested");
	_commands.fetch_or(1u);
	ScheduleNow();
}

int CompAttitude::task_spawn(int argc, char *argv[])
{
	for (int i = 1; i < argc; ++i) {
		return print_usage("unexpected start argument");
	}

	CompAttitude *instance = new CompAttitude();

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
	PX4_ERR("composite attitude alloc/init failed");
	return PX4_ERROR;
}

int CompAttitude::print_status()
{
	PX4_INFO("CEKF initialized %d, AIRLINK %d, weights %.1f/%.1f", _external_attitude_initialized,
	         _peer_available, _peer_available ? 0.5 : 1.0, _peer_available ? 0.5 : 0.0);
	return PX4_OK;
}
int CompAttitude::custom_command(int argc, char *argv[])
{
	if (argc && !strcmp(argv[0], "reset") && get_instance()) {
		get_instance()->requestReset();
		return PX4_OK;
	}
	return print_usage("unknown command");
}
int CompAttitude::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}
	PRINT_MODULE_USAGE_NAME("comp_attitude", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}
extern "C" __EXPORT int comp_attitude_main(int argc, char *argv[])
{
	return CompAttitude::main(argc, argv);
}
