/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace composite
{

constexpr float kPi{3.14159265358979323846f};
constexpr float kTwoPi{2.f * kPi};
constexpr uint64_t kU32PeriodUs{1ull << 32};

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
constexpr T clampValue(T value, T lower, T upper)
{
	return value < lower ? lower : (value > upper ? upper : value);
}

inline float wrapPi(float angle)
{
	while (angle >= kPi) {
		angle -= kTwoPi;
	}

	while (angle < -kPi) {
		angle += kTwoPi;
	}

	return angle;
}

inline bool finite(float value)
{
	return std::isfinite(value);
}

struct Vec3 {
	float x{0.f};
	float y{0.f};
	float z{0.f};

	Vec3 operator+(const Vec3 &other) const { return {x + other.x, y + other.y, z + other.z}; }
	Vec3 operator-(const Vec3 &other) const { return {x - other.x, y - other.y, z - other.z}; }
	Vec3 operator*(float scale) const { return {x * scale, y * scale, z * scale}; }
	Vec3 &operator+=(const Vec3 &other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	bool finite() const { return composite::finite(x) && composite::finite(y) && composite::finite(z); }
	float norm() const { return std::sqrt(x * x + y * y + z * z); }
};

inline Vec3 cross(const Vec3 &a, const Vec3 &b)
{
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

struct Quat {
	float w{1.f};
	float x{0.f};
	float y{0.f};
	float z{0.f};

	bool finite() const
	{
		return composite::finite(w) && composite::finite(x) && composite::finite(y)
		       && composite::finite(z);
	}

	float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

	bool normalized(float tolerance = 1e-4f) const
	{
		return finite() && std::fabs(norm() - 1.f) < tolerance;
	}

	Quat canonical() const
	{
		return w < 0.f ? Quat{-w, -x, -y, -z} : *this;
	}

	Quat normalized_or_identity(float tolerance = 1e-4f) const
	{
		const float length = norm();

		if (!finite() || !std::isfinite(length) || std::fabs(length - 1.f) >= tolerance) {
			return {};
		}

		return {w / length, x / length, y / length, z / length};
	}
};

inline Quat operator*(const Quat &a, const Quat &b)
{
	return {
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
	};
}

inline Quat conjugate(const Quat &q)
{
	return {q.w, -q.x, -q.y, -q.z};
}

inline Quat inverse_unit(const Quat &q)
{
	const Quat unit = q.normalized_or_identity();
	return conjugate(unit);
}

inline Vec3 rotate(const Quat &q_unchecked, const Vec3 &v)
{
	const Quat q = q_unchecked.normalized_or_identity();
	const Quat vector{0.f, v.x, v.y, v.z};
	const Quat result = q * vector * conjugate(q);
	return {result.x, result.y, result.z};
}

inline Quat fromAxisAngle(const Vec3 &axis, float angle)
{
	const float length = axis.norm();

	if (!std::isfinite(length) || length < 1e-9f || !std::isfinite(angle)) {
		return {};
	}

	const float half = 0.5f * angle;
	const float scale = std::sin(half) / length;
	return Quat{std::cos(half), axis.x * scale, axis.y * scale, axis.z * scale}.normalized_or_identity();
}

inline Quat fromYaw(float yaw_rad)
{
	return fromAxisAngle({0.f, 0.f, 1.f}, yaw_rad);
}

inline Quat fromEuler(float roll, float pitch, float yaw)
{
	const Quat q_roll = fromAxisAngle({1.f, 0.f, 0.f}, roll);
	const Quat q_pitch = fromAxisAngle({0.f, 1.f, 0.f}, pitch);
	const Quat q_yaw = fromAxisAngle({0.f, 0.f, 1.f}, yaw);
	return q_yaw * q_pitch * q_roll;
}

inline void eulerAngles(const Quat &q_unchecked, float &roll, float &pitch, float &yaw_angle)
{
	const Quat q = q_unchecked.normalized_or_identity();
	roll = std::atan2(2.f * (q.w * q.x + q.y * q.z),
			  1.f - 2.f * (q.x * q.x + q.y * q.y));
	const float sin_pitch = clampValue(2.f * (q.w * q.y - q.z * q.x),
					   -1.f, 1.f);
	pitch = std::asin(sin_pitch);
	yaw_angle = std::atan2(2.f * (q.w * q.z + q.x * q.y),
			       1.f - 2.f * (q.y * q.y + q.z * q.z));
}

inline Quat slerp(const Quat &a_unchecked, const Quat &b_in, float t)
{
	const Quat a = a_unchecked.normalized_or_identity();
	Quat b = b_in.normalized_or_identity();
	float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

	if (dot < 0.f) {
		b = {-b.w, -b.x, -b.y, -b.z};
		dot = -dot;
	}

	t = clampValue(t, 0.f, 1.f);

	if (dot > 0.9995f) {
		return Quat{
			a.w + t * (b.w - a.w),
			a.x + t * (b.x - a.x),
			a.y + t * (b.y - a.y),
			a.z + t * (b.z - a.z)
		}.normalized_or_identity();
	}

	const float theta = std::acos(clampValue(dot, -1.f, 1.f));
	const float sin_theta = std::sin(theta);
	const float first = std::sin((1.f - t) * theta) / sin_theta;
	const float second = std::sin(t * theta) / sin_theta;
	return Quat{
		first * a.w + second * b.w,
		first * a.x + second * b.x,
		first * a.y + second * b.y,
		first * a.z + second * b.z
	}.normalized_or_identity();
}

inline float yaw(const Quat &q)
{
	const Quat n = q.normalized_or_identity();
	return std::atan2(2.f * (n.w * n.z + n.x * n.y),
			  1.f - 2.f * (n.y * n.y + n.z * n.z));
}

inline float angleBetween(const Quat &a, const Quat &b)
{
	const Quat delta = inverse_unit(a) * b.normalized_or_identity();
	const float w = clampValue(std::fabs(delta.w), 0.f, 1.f);
	return 2.f * std::acos(w);
}

inline float tiltDifferenceBetween(const Quat &a_unchecked, const Quat &b_unchecked)
{
	const Quat a = a_unchecked.normalized_or_identity();
	const Quat b = b_unchecked.normalized_or_identity();
	const Vec3 down_a = rotate(inverse_unit(a), {0.f, 0.f, -1.f});
	const Vec3 down_b = rotate(inverse_unit(b), {0.f, 0.f, -1.f});
	const float dot = clampValue(down_a.x * down_b.x + down_a.y * down_b.y
				    + down_a.z * down_b.z, -1.f, 1.f);

	if (!std::isfinite(dot)) {
		return composite::kPi;
	}

	return std::acos(dot);
}

inline float yawDifference(float reference, float measured)
{
	return wrapPi(measured - reference);
}

/**
 * Mount calibration angles are FRD vector mappings: v_C = Rz(yaw_X) v_X.
 * q_AB maps B to A, so q_AB = inverse(q_CA) * q_CB.
 * Defaults A->C=-30 deg, B->C=+30 deg give q_AB=Rz(+60 deg).
 * Geometrically C's forward axis is 30 deg to the right of A's forward axis.
 */
inline Quat assemblyQuatFromMountYaw(float master_mount_yaw_rad,
				     float slave_mount_yaw_rad)
{
	return fromYaw(wrapPi(slave_mount_yaw_rad - master_mount_yaw_rad));
}

/** Rotation of vector components from aircraft FRD to independent C FRD. */
inline Quat vehicleToCompositeQuatFromMountYaw(float mount_yaw_rad)
{
	return fromYaw(mount_yaw_rad);
}

inline Quat attitudeInCompositeFrame(const Quat &q_nx, float mount_yaw_rad)
{
	return q_nx.normalized_or_identity()
	       * inverse_unit(vehicleToCompositeQuatFromMountYaw(mount_yaw_rad));
}

inline Quat assemblyQuatFromLocalAttitudes(const Quat &q_na, const Quat &q_nb)
{
	return inverse_unit(q_na) * q_nb.normalized_or_identity();
}

inline Quat slaveAttitudeInCompositeFrame(const Quat &q_nb, const Quat &q_ab)
{
	return q_nb.normalized_or_identity() * inverse_unit(q_ab);
}

inline float circularMeanYaw(float first, float second)
{
	if (!std::isfinite(first) || !std::isfinite(second)) {
		return NAN;
	}

	return std::atan2(0.5f * (std::sin(first) + std::sin(second)),
			  0.5f * (std::cos(first) + std::cos(second)));
}

/**
 * Convert a slave 32-bit timestamp into a monotonic 64-bit host time base.
 * The high bits are seeded from the host arrival time when no history exists.
 */
class U32Unwrapper
{
public:
	void reset(uint32_t raw, uint64_t host_time_us)
	{
		const uint64_t high = host_time_us & ~(kU32PeriodUs - 1);
		_unwrapped_us = high | static_cast<uint64_t>(raw);
		_raw = raw;
		_valid = true;
	}

	bool update(uint32_t raw, uint64_t host_time_us, uint64_t &unwrapped_us)
	{
		if (!_valid) {
			reset(raw, host_time_us);
			unwrapped_us = _unwrapped_us;
			return true;
		}

		if (raw == _raw) {
			return false;
		}

		uint64_t candidate = (_unwrapped_us & ~(kU32PeriodUs - 1))
				     | static_cast<uint64_t>(raw);

		if (candidate + kU32PeriodUs / 2 < _unwrapped_us) {
			candidate += kU32PeriodUs;

		} else if (candidate > _unwrapped_us + kU32PeriodUs / 2) {
			candidate -= kU32PeriodUs;
		}

		if (candidate <= _unwrapped_us || candidate == 0) {
			return false;
		}

		const uint64_t delta = candidate - _unwrapped_us;

		if (delta > 10'000) {
			return false;
		}

		_unwrapped_us = candidate;
		_raw = raw;
		unwrapped_us = candidate;
		return true;
	}

	uint64_t unwrapped() const { return _unwrapped_us; }

private:
	uint64_t _unwrapped_us{0};
	uint32_t _raw{0};
	bool _valid{false};
};

/**
 * AIRLINK arrival-time to host-time mapper from the estimator design.
 *
 * d_k = arrival_k - slave_unwrapped_k. D_hat is the 10th percentile over
 * 5 seconds and at least 50 samples. Mapping is:
 *
 * t_slave_on_host = slave_unwrapped + D_hat - one_way_latency.
 */
class ClockMapper
{
public:
	static constexpr size_t kMaxSamples{251};
	static constexpr size_t kMinSamples{50};
	static constexpr uint64_t kWindowUs{5'000'000};
	static constexpr uint64_t kCollectIntervalUs{20'000};

	void reset(uint64_t host_time_us)
	{
		_unwrapper.reset(0, host_time_us);
		_count = 0;
		_last_collect_us = 0;
		_offset_us = 0;
		_offset_ready = false;
		_last_arrival_us = 0;
		_last_raw = 0;
		last_residual_us = 0;
		samples = 0;
		_first_sample = true;
	}

	bool add(uint32_t raw, uint64_t arrival_us, int64_t one_way_latency_us,
		 uint64_t &mapped_us, uint64_t &age_us)
	{
		if (arrival_us == 0 || !std::isfinite(static_cast<float>(one_way_latency_us))) {
			return false;
		}

		if (_first_sample) {
			_unwrapper.reset(raw, arrival_us);
			_last_slave_us = _unwrapper.unwrapped();
			_first_sample = false;

		} else if (!_unwrapper.update(raw, arrival_us, _last_slave_us)) {
			return false;
		}

		if (_last_arrival_us != 0 && arrival_us <= _last_arrival_us) {
			return false;
		}

		_last_arrival_us = arrival_us;
		_last_raw = raw;
		const int64_t delta = static_cast<int64_t>(arrival_us)
				      - static_cast<int64_t>(_last_slave_us);

		if (_last_collect_us == 0 || arrival_us - _last_collect_us >= kCollectIntervalUs) {
			push(delta, arrival_us);
			_last_collect_us = arrival_us;
		}

		if (!_offset_ready) {
			mapped_us = 0;
			age_us = 0;
			return false;
		}

		const int64_t mapped_signed = static_cast<int64_t>(_last_slave_us)
					      + _offset_us - one_way_latency_us;
		mapped_us = mapped_signed > 0 ? static_cast<uint64_t>(mapped_signed) : 0;

		if (mapped_us == 0 || mapped_us > arrival_us) {
			return false;
		}

		last_residual_us = static_cast<int64_t>(arrival_us)
				   - static_cast<int64_t>(mapped_us);
		age_us = arrival_us - mapped_us;
		return true;
	}

	bool offsetReady() const { return _offset_ready; }
	int64_t offsetUs() const { return _offset_us; }
	uint64_t lastSlaveUnwrappedUs() const { return _last_slave_us; }
	size_t sampleCount() const { return _count; }
	uint32_t lastRaw() const { return _last_raw; }
	int64_t last_residual_us{0};
	size_t samples{0};

private:
	struct Entry {
		int64_t delta_us{0};
		uint64_t arrival_us{0};
	};

	void push(int64_t delta_us, uint64_t arrival_us)
	{
		size_t first = 0;

		while (first < _count && arrival_us - _entries[first].arrival_us > kWindowUs) {
			++first;
		}

		if (first > 0) {
			for (size_t i = first; i < _count; ++i) {
				_entries[i - first] = _entries[i];
			}

			_count -= first;
		}

		if (_count == kMaxSamples) {
			for (size_t i = 1; i < _count; ++i) {
				_entries[i - 1] = _entries[i];
			}

			--_count;
		}

		_entries[_count++] = Entry{delta_us, arrival_us};
		_samples = _count;
		samples = _count;

		for (size_t i = 0; i < _count; ++i) {
			_sorted[i] = _entries[i].delta_us;
		}

		if (_count < kMinSamples) {
			return;
		}

		for (size_t i = 1; i < _count; ++i) {
			const int64_t value = _sorted[i];
			size_t position = i;

			while (position > 0 && _sorted[position - 1] > value) {
				_sorted[position] = _sorted[position - 1];
				--position;
			}

			_sorted[position] = value;
		}

		_offset_us = _sorted[(_count - 1) / 10];
		_offset_ready = true;
	}

	U32Unwrapper _unwrapper{};
	Entry _entries[kMaxSamples] {};
	int64_t _sorted[kMaxSamples] {};
	size_t _count{0};
	size_t _samples{0};
	uint64_t _last_collect_us{0};
	uint64_t _last_arrival_us{0};
	uint64_t _last_slave_us{0};
	int64_t _offset_us{0};
	bool _offset_ready{false};
	bool _first_sample{true};
	uint32_t _last_raw{0};
};

class YawAnchor
{
public:
	void reset()
	{
		_yaw_rad = 0.f;
		_initialized = false;
	}

	void reset(float yaw_rad)
	{
		_yaw_rad = wrapPi(yaw_rad);
		_initialized = std::isfinite(_yaw_rad);
	}

	bool update(float measured_yaw_rad, float dt_s, float time_constant_s)
	{
		if (!std::isfinite(measured_yaw_rad) || !std::isfinite(dt_s) || dt_s <= 0.f
		    || !std::isfinite(time_constant_s) || time_constant_s <= 0.f) {
			return false;
		}

		if (!_initialized) {
			_yaw_rad = wrapPi(measured_yaw_rad);
			_initialized = true;
			return true;
		}

		const float alpha = dt_s / (time_constant_s + dt_s);
		_yaw_rad = wrapPi(_yaw_rad + alpha * wrapPi(measured_yaw_rad - _yaw_rad));
		return true;
	}

	bool initialized() const { return _initialized; }
	float yaw() const { return _yaw_rad; }

private:
	float _yaw_rad{0.f};
	bool _initialized{false};
};

} // namespace composite
