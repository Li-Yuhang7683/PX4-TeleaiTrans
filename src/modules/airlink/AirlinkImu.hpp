/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu.h>

#include <cmath>

namespace airlink
{

// Read the calibrated body-FRD IMU selected for the local EKF. No EKF bias
// subtraction: CEKF estimates its own remaining bias after combining the IMUs.
class ImuSource
{
public:
	void update()
	{
		_selection_sub.update(&_selection);
		updateSensor(_accel_sub, _accel, _selection.accel_device_id, false, _accel_instance);
		updateSensor(_gyro_sub, _gyro, _selection.gyro_device_id, true, _gyro_instance);
	}

	uint64_t timestampSample() const
	{
		return _gyro.timestamp_sample;
	}
	uint64_t accelerationTimestamp() const
	{
		return _accel.timestamp_sample;
	}
	float angularDtSeconds() const
	{
		return _gyro.delta_angle_dt * 1e-6f;
	}
	uint8_t gyroInstance() const
	{
		return _gyro_instance;
	}

	bool acceleration(float xyz[3]) const
	{
		return rates(_accel.timestamp_sample, _accel.delta_velocity, _accel.delta_velocity_dt, xyz);
	}

	bool angularVelocity(float xyz[3]) const
	{
		return rates(_gyro.timestamp_sample, _gyro.delta_angle, _gyro.delta_angle_dt, xyz);
	}

private:
	// PX4 votes accelerometers and gyroscopes independently. Their selected
	// device IDs need not occur together in any one vehicle_imu instance.
	static void updateSensor(uORB::Subscription &sub, vehicle_imu_s &sample,
	                         uint32_t device_id, bool gyro, uint8_t &selected_instance)
	{
		const auto matches = [device_id, gyro](const vehicle_imu_s &imu) {
			return device_id != 0 && (gyro ? imu.gyro_device_id : imu.accel_device_id) == device_id;
		};
		if (matches(sample)) {
			sub.update(&sample);
			if (matches(sample)) {
				return;
			}
		}
		sample = {};
		if (device_id == 0) {
			return;
		}
		for (uint8_t instance = 0; instance < 4; ++instance) {
			uORB::Subscription candidate{ORB_ID(vehicle_imu), instance};
			vehicle_imu_s imu{};
			if (candidate.copy(&imu) && matches(imu)) {
				sub.ChangeInstance(instance);
				sample = imu;
				selected_instance = instance;
				return;
			}
		}
	}

	static bool rates(uint64_t timestamp, const float delta[3], uint32_t dt_us, float xyz[3])
	{
		if (timestamp == 0 || dt_us == 0) {
			return false;
		}

		const float dt_s = dt_us * 1e-6f;

		for (unsigned axis = 0; axis < 3; ++axis) {
			xyz[axis] = delta[axis] / dt_s;
		}

		return std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]);
	}

	uORB::Subscription _selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _accel_sub{ORB_ID(vehicle_imu)};
	uORB::Subscription _gyro_sub{ORB_ID(vehicle_imu)};
	sensor_selection_s _selection{};
	vehicle_imu_s _accel{}, _gyro{};
	uint8_t _accel_instance{0}, _gyro_instance{0};
};

} // namespace airlink
