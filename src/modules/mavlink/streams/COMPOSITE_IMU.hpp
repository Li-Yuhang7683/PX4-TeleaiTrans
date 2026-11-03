/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef COMPOSITE_IMU_HPP
#define COMPOSITE_IMU_HPP

#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/airlink_sensor_state.h>
#include <uORB/topics/composite_attitude_estimate.h>
#include <uORB/topics/composite_estimator_status.h>
#include <uORB/topics/composite_yaw_reference.h>
#include <parameters/param.h>
#include <cmath>

// Each axis is a separate named scalar for QGC telemetry plots.
class MavlinkStreamCompositeImu : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCompositeImu(mavlink);
	}
	static constexpr const char *get_name_static()
	{
		return "COMPOSITE_IMU";
	}
	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}
	const char *get_name() const override
	{
		return get_name_static();
	}
	uint16_t get_id() override
	{
		return get_id_static();
	}
	unsigned get_size() override
	{
		return _accel_sub.advertised() || _gyro_sub.advertised() || _peer_sub.advertised()
		       || _estimate_sub.advertised() || _status_sub.advertised()
		       ? kChannelCount * (MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	static constexpr unsigned kChannelCount{51};
	static const char *channelName(unsigned channel)
	{
		static const char *const names[kChannelCount] {
			"A_ACC_R_X", "A_ACC_R_Y", "A_ACC_R_Z",
			"B_ACC_R_X", "B_ACC_R_Y", "B_ACC_R_Z",
			"A_ACC_C_X", "A_ACC_C_Y", "A_ACC_C_Z",
			"B_ACC_C_X", "B_ACC_C_Y", "B_ACC_C_Z",
			"A_GYR_R_X", "A_GYR_R_Y", "A_GYR_R_Z",
			"B_GYR_R_X", "B_GYR_R_Y", "B_GYR_R_Z",
			"A_GYR_C_X", "A_GYR_C_Y", "A_GYR_C_Z",
			"B_GYR_C_X", "B_GYR_C_Y", "B_GYR_C_Z",
			"C_ROLL", "C_PITCH", "C_YAW",
			"C_ACC_X", "C_ACC_Y", "C_ACC_Z",
			"C_GYR_X", "C_GYR_Y", "C_GYR_Z",
			"C_ALP_X", "C_ALP_Y", "C_ALP_Z",
			"CE_RUN", "CE_EN", "CE_INIT", "CE_STATE", "CE_LINK", "CE_W_A", "CE_W_B",
			"CE_VALID", "CE_RESET", "CE_FLAGS", "CE_AGE_A", "CE_AGE_B", "CE_FRAMES",
			"CE_YAWREF", "CE_YSIG"
		};
		return names[channel];
	}
	explicit MavlinkStreamCompositeImu(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	uORB::Subscription _accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription _peer_sub{ORB_ID(airlink_sensor_state)};
	uORB::Subscription _estimate_sub{ORB_ID(composite_attitude_estimate)};
	uORB::Subscription _status_sub{ORB_ID(composite_estimator_status)};
	uORB::Subscription _yaw_sub{ORB_ID(composite_yaw_reference)};
	composite_attitude_estimate_s _estimate{};
	composite_estimator_status_s _status{};
	composite_yaw_reference_s _yaw{};
	sensor_accel_s _accel{};
	sensor_gyro_s _gyro{};
	airlink_sensor_state_s _peer{};
	param_t _role{param_find("AIRLINK_ROLE")};
	param_t _yaw_a{param_find("CEKF_MSTR_YAW")};
	param_t _yaw_b{param_find("CEKF_SLV_YAW")};

	static void fill(float *raw, float *mapped, float x, float y, float z, float yaw_deg)
	{
		if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(yaw_deg)) {
			return;
		}

		raw[0] = x;
		raw[1] = y;
		raw[2] = z;
		// Calibration is the FRD vector mapping X -> C: v_C = Rz(yaw) v_X.
		const float angle = yaw_deg * 0.017453292519943295f;
		mapped[0] = cosf(angle) * x - sinf(angle) * y;
		mapped[1] = sinf(angle) * x + cosf(angle) * y;
		mapped[2] = z;
	}

	bool send() override
	{
		int32_t role = 0;
		float yaw_a = -30.f;
		float yaw_b = 30.f;
		param_get(_role, &role);
		param_get(_yaw_a, &yaw_a);
		param_get(_yaw_b, &yaw_b);

		_accel_sub.update(&_accel);
		_gyro_sub.update(&_gyro);
		_peer_sub.update(&_peer);
		_estimate_sub.update(&_estimate);
		_status_sub.update(&_status);
		_yaw_sub.update(&_yaw);
		const uint64_t now = hrt_absolute_time();
		const auto fresh = [now](uint64_t timestamp) {
			return timestamp != 0 && timestamp <= now && now - timestamp < 200'000;
		};
		float values[kChannelCount];

		for (float &value : values) {
			value = NAN;
		}

		// XYZ groups: A accel raw, B accel raw, A accel C, B accel C,
		// A gyro raw, B gyro raw, A gyro C, B gyro C. SI units.
		const bool master = role == 1;
		const unsigned offset = master ? 0 : 3;
		const float yaw = master ? yaw_a : yaw_b;

		if ((role == 1 || role == 2) && fresh(_accel.timestamp_sample)) {
			fill(&values[offset], &values[6 + offset], _accel.x, _accel.y, _accel.z, yaw);
		}

		if ((role == 1 || role == 2) && fresh(_gyro.timestamp_sample)) {
			fill(&values[12 + offset], &values[18 + offset], _gyro.x, _gyro.y, _gyro.z, yaw);
		}

		// SENSOR_STATE travels slave -> master only. Use local arrival for freshness;
		// the slave sample timestamp belongs to a different clock.
		if (master && fresh(_peer.timestamp)) {
			if (_peer.status_flags & (1u << 0)) {
				fill(&values[3], &values[9], _peer.accelerometer_raw[0] / 128.f,
				     _peer.accelerometer_raw[1] / 128.f, _peer.accelerometer_raw[2] / 128.f, yaw_b);
			}

			if (_peer.status_flags & (1u << 1)) {
				fill(&values[15], &values[21], _peer.gyro_raw[0] / 800.f,
				     _peer.gyro_raw[1] / 800.f, _peer.gyro_raw[2] / 800.f, yaw_b);
			}
		}

		// Status is a 10 Hz heartbeat from the module, including CEKF_EN=0.
		// Report stopped/stale explicitly; retain gaps rather than replaying old fusion values.
		const bool running = fresh(_status.timestamp);
		const bool valid = running && _status.enabled && _status.attitude_initialized
		                   && fresh(_estimate.timestamp) && _estimate.valid;
		values[36] = running ? 1.f : 0.f;
		values[43] = valid ? 1.f : 0.f;
		if (running) {
			values[37] = _status.enabled;
			values[38] = _status.attitude_initialized;
			values[39] = _status.state;
			values[40] = _status.peer_connected;
			values[41] = _status.weight_a;
			values[42] = _status.weight_b;
			values[48] = _status.frame_count;
		}
		if (valid) {
			// Hamilton quaternion maps C FRD -> NED; Euler curves are in degrees.
			const float w = _estimate.q[0], x = _estimate.q[1], y = _estimate.q[2], z = _estimate.q[3];
			constexpr float rad_to_deg = 57.29577951308232f;
			values[24] = atan2f(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y)) * rad_to_deg;
			values[25] = asinf(fmaxf(-1.f, fminf(1.f, 2.f * (w * y - z * x)))) * rad_to_deg;
			values[26] = atan2f(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z)) * rad_to_deg;
			for (unsigned axis = 0; axis < 3; ++axis) {
				values[27 + axis] = _estimate.specific_force[axis];
				values[30 + axis] = _estimate.angular_velocity[axis];
				values[33 + axis] = _estimate.angular_acceleration[axis];
			}
			values[44] = _estimate.reset_counter;
			values[45] = _estimate.health_flags;
			values[46] = _estimate.source_age_a_us * 0.001f;
			values[47] = _estimate.source_age_b_us * 0.001f;
			values[50] = _estimate.yaw_sigma_rad;
		}
		if (running && _status.enabled && _status.attitude_initialized && _yaw.timestamp
		    && now >= _yaw.timestamp && now - _yaw.timestamp < 1'500'000
		    && (_yaw.flags & composite_yaw_reference_s::FLAG_VALID)) {
			values[49] = _yaw.yaw_rad * 57.29577951308232f;
		}

		// Send every channel each cycle, including NaN for stale/missing sources.
		for (unsigned channel = 0; channel < kChannelCount; ++channel) {
			mavlink_named_value_float_t msg{};
			msg.time_boot_ms = now / 1000ULL;
			strncpy(msg.name, channelName(channel), sizeof(msg.name) - 1);
			msg.value = values[channel];
			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);
		}
		return true;
	}
};
#endif // COMPOSITE_IMU_HPP
