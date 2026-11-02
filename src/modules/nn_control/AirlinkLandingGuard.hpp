/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

namespace airlink_landing
{

enum class Termination : uint8_t {
	None = 0,
	Impact = 1,
	Flip = 2,
	Timeout = 3,
};

class Guard
{
public:
	void reset()
	{
		_started_us = 0;
		_impact_since_us = 0;
		_flip_since_us = 0;
	}

	void start(uint64_t now_us)
	{
		reset();
		_started_us = now_us;
	}

	Termination update(uint64_t now_us, bool impact, bool flipped,
			   uint64_t impact_confirm_us, uint64_t flip_confirm_us,
			   uint64_t landing_timeout_us)
	{
		if (_started_us == 0 || now_us < _started_us) {
			return Termination::None;
		}

		_impact_since_us = update_condition(impact, now_us, _impact_since_us);
		_flip_since_us = update_condition(flipped, now_us, _flip_since_us);

		if (_impact_since_us != 0 && now_us - _impact_since_us >= impact_confirm_us) {
			return Termination::Impact;
		}

		if (_flip_since_us != 0 && now_us - _flip_since_us >= flip_confirm_us) {
			return Termination::Flip;
		}

		if (now_us - _started_us > landing_timeout_us) {
			return Termination::Timeout;
		}

		return Termination::None;
	}

private:
	static uint64_t update_condition(bool condition, uint64_t now_us, uint64_t since_us)
	{
		if (!condition) {
			return 0;
		}

		return since_us == 0 ? now_us : since_us;
	}

	uint64_t _started_us{0};
	uint64_t _impact_since_us{0};
	uint64_t _flip_since_us{0};
};

} // namespace airlink_landing
