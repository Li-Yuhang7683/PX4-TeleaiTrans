/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "AirlinkSession.hpp"

#include <cmath>
#include <cstring>

namespace airlink
{

namespace
{

constexpr uint32_t kUnknownAgeUs{UINT32_MAX};

uint32_t age_us(uint64_t now_us, uint64_t then_us)
{
	if (then_us == 0 || now_us < then_us) {
		return kUnknownAgeUs;
	}

	return static_cast<uint32_t>(minimum<uint64_t>(now_us - then_us, kUnknownAgeUs));
}

bool handshake_flags_match(uint8_t flags, uint8_t phase_flags)
{
	return flags == phase_flags || flags == static_cast<uint8_t>(phase_flags | Retry);
}

bool link_test_pattern_matches(const uint8_t *payload, uint16_t payload_len,
			       uint32_t test_sequence, uint16_t pattern_seed)
{
	if (pattern_seed != static_cast<uint16_t>(0xA55Au ^ test_sequence)) {
		return false;
	}

	uint32_t state = pattern_seed | 1u;
	uint16_t offset = kLinkTestDataHeaderSize;

	while (offset < payload_len) {
		state ^= state << 13;
		state ^= state >> 17;
		state ^= state << 5;

		if (payload[offset] != static_cast<uint8_t>(state)) {
			return false;
		}

		++offset;
	}

	return offset == payload_len;
}

} // namespace

Session::Session(uint32_t config_crc32c) :
	_config_crc32c(config_crc32c)
{
}

void Session::start(Role role, uint32_t session_id, const LocalStatus &local, uint64_t now_us)
{
	_role = role == Role::Disabled ? Role::Disabled : role;
	_session_id = session_id == 0 ? 1 : session_id;
	_local = local;
	_pair_reattach_latched = local.pair_reattach_latched;
	_emergency_latched = false;
	_peer_session = 0;
	_remote = {};
	_peer_command = {};
	_peer_sensor = {};
	_state = _role == Role::Disabled ? LinkState::Down : LinkState::Down;
	_tx_sequence = 0;
	_last_peer_sensor_sequence = 0;
	_last_peer_control_sequence = 0;
	_tx_frames = 0;
	_rx_frames = 0;
	_session_errors = 0;
	_unknown_messages = 0;
	_reliable_retries = 0;
	_duplicate_events = 0;
	_tx_head = 0;
	_tx_count = 0;
	_pending = {};
	_handshake = {};
	for (DuplicateEntry &entry : _duplicates) {
		entry = {};
	}
	_master_arm_pending = false;
	_slave_arm_pending = false;
	_slave_arm_event_id = 0;
	_slave_arm_request = {};
	_control_enable_acked_us = 0;
	_pair_arm_commit_deadline_us = 0;
	_last_peer_frame_us = 0;
	_last_peer_heartbeat_us = 0;
	_last_peer_sensor_us = 0;
	_last_peer_control_us = 0;
	_last_local_sensor_sent_us = 0;
	_last_local_control_sent_us = 0;
	_last_control_sent_us = 0;
	_last_heartbeat_sent_us = 0;
	_handshake_retry_after_us = now_us;
	_pair_control_ready_us = 0;
	_next_transaction_id = 1;
	_completed_handshake_transaction = 0;
	_peer_sensor_consecutive = 0;
	_peer_control_consecutive = 0;
	_local_sensor_consecutive = 0;
	_local_control_consecutive = 0;
	_has_peer_sensor_sequence = false;
	_has_peer_control_sequence = false;
	_link_test = {};
}

void Session::set_local_status(const LocalStatus &local)
{
	_local = local;
	_local.pair_reattach_latched = _pair_reattach_latched;
}

void Session::set_armed(bool armed)
{
	_local.armed = armed;
}

bool Session::set_role(Role role, uint32_t new_session_id, const LocalStatus &local,
		       uint64_t now_us)
{
	if (_local.armed || _state == LinkState::PairActive || _state == LinkState::PairArmedGround
	    || _state == LinkState::PairArmPending || _link_test.active) {
		return false;
	}

	const bool latch = _pair_reattach_latched || local.pair_reattach_latched;
	LocalStatus updated = local;
	updated.pair_reattach_latched = latch;
	start(role, new_session_id, updated, now_us);
	_pair_reattach_latched = latch;
	return true;
}

uint32_t Session::last_peer_age_us(uint64_t now_us) const
{
	return age_us(now_us, _last_peer_frame_us);
}

uint32_t Session::last_peer_sensor_age_us(uint64_t now_us) const
{
	return age_us(now_us, _last_peer_sensor_us);
}

uint32_t Session::last_peer_control_age_us(uint64_t now_us) const
{
	return age_us(now_us, _last_peer_control_us);
}

bool Session::enqueue_existing(const Frame &frame)
{
	if (_tx_count >= kTxQueueSize) {
		return false;
	}

	const size_t tail = (_tx_head + _tx_count) % kTxQueueSize;
	_tx_queue[tail] = frame;
	++_tx_count;
	return true;
}

bool Session::queue_frame(FrameType frame_type, uint8_t message_id, uint8_t flags,
			  const uint8_t *payload, uint16_t payload_len, Frame *saved)
{
	if (_role == Role::Disabled || payload_len > kMaxPayload
	    || (_tx_count >= kTxQueueSize)) {
		return false;
	}

	Frame frame{};
	frame.header.frame_type = frame_type;
	frame.header.flags = flags;
	frame.header.message_id = message_id;
	frame.header.source_role = _role;
	frame.header.dest_role = _role == Role::Master ? Role::Slave : Role::Master;
	frame.header.source_session = _session_id;
	frame.header.sequence = _tx_sequence;
	frame.header.payload_len = payload_len;

	if (payload_len > 0) {
		memcpy(frame.payload, payload, payload_len);
	}

	uint8_t encoded[kMaxFrameSize] {};
	size_t encoded_length = 0;

	if (!build_frame(frame.header, frame.payload, payload_len, encoded, sizeof(encoded),
			 encoded_length)) {
		return false;
	}

	if (!enqueue_existing(frame)) {
		return false;
	}

	++_tx_sequence;

	if (saved != nullptr) {
		*saved = frame;
	}

	return true;
}

bool Session::build_and_enqueue(PendingTemplate &pending, bool retry)
{
	if (!pending.active) {
		return false;
	}

	Frame frame = pending.frame;

	if (retry) {
		frame.header.flags |= Retry;
	}

	uint8_t encoded[kMaxFrameSize] {};
	size_t encoded_length = 0;

	if (!build_frame(frame.header, frame.payload, frame.header.payload_len, encoded, sizeof(encoded),
			 encoded_length) || !enqueue_existing(frame)) {
		return false;
	}

	if (retry) {
		++_reliable_retries;
	}

	return true;
}

bool Session::pop_tx(Frame &frame)
{
	if (_tx_count == 0) {
		return false;
	}

	frame = _tx_queue[_tx_head];
	_tx_head = (_tx_head + 1) % kTxQueueSize;
	--_tx_count;
	++_tx_frames;
	return true;
}

uint32_t Session::allocate_transaction()
{
	const uint32_t result = _next_transaction_id++;

	if (_next_transaction_id == 0) {
		_next_transaction_id = 1;
	}

	return result == 0 ? allocate_transaction() : result;
}

uint16_t Session::local_health_flags(uint64_t now_us) const
{
	uint16_t flags = 0;

	if (_local.local_ready) {
		flags |= LocalReady;
	}

	if (_local.local_rc_valid) {
		flags |= HealthLocalRcValid;
	}

	if (_local.safety_gate_allow) {
		flags |= HealthSafetyGateAllow;
	}

	if (_state == LinkState::PairActive) {
		flags |= HealthAirlinkControlActive;
	}

	if (_peer_session != 0 && now_us >= _last_peer_frame_us
	    && now_us - _last_peer_frame_us <= kSensorFreshTimeoutUs) {
		flags |= InboundPeriodicFresh;
	}

	if (_local.armed) {
		flags |= Armed;
	}

	if (_local.airborne_latched) {
		flags |= AirborneLatched;
	}

	if (_local.pair_mode_selected) {
		flags |= PairModeSelected;
	}

	if (_local.gs_link_valid) {
		flags |= GsLinkValid;
	}

	if (_local.gs_position_active) {
		flags |= GsPositionActive;
	}

	if (_local.single_landing_active) {
		flags |= SingleLandingActive;
	}

	if (_pair_reattach_latched) {
		flags |= PairReattachLatched;
	}

	if (_link_test.active) {
		flags |= LinkTestActive;
	}

	return flags;
}

uint32_t Session::local_capabilities() const
{
	return Single | Pair | PairArmCapability | GsPosition | SingleLanding
	       | LinkTest | SensorStateV1 | Output2M3s;
}

uint32_t Session::local_handshake_flags() const
{
	uint32_t flags = 0;

	if (_local.local_ready) {
		flags |= HsLocalReady;
	}

	if (_local.armed) {
		flags |= HsArmed;
	}

	if (_local.airborne_latched) {
		flags |= HsAirborneLatched;
	}

	if (_local.pair_mode_selected) {
		flags |= HsPairModeSelected;
	}

	if (_pair_reattach_latched) {
		flags |= HsPairReattachLatched;
	}

	if (_local.gs_link_valid) {
		flags |= HsGsLinkValid;
	}

	if (_local.local_rc_valid) {
		flags |= HsLocalRcValid;
	}

	if (_local.safety_gate_allow) {
		flags |= HsSafetyGateAllow;
	}

	if (_link_test.active) {
		flags |= HsLinkTestActive;
	}

	if (_state == LinkState::PairActive || _state == LinkState::PairArmedGround) {
		flags |= HsPairActive;
	}

	return flags;
}

bool Session::peer_is_fresh(uint64_t now_us) const
{
	return _peer_session != 0 && _last_peer_heartbeat_us != 0
	       && now_us >= _last_peer_heartbeat_us
	       && now_us - _last_peer_heartbeat_us <= kPeerHeartbeatTimeoutUs;
}

bool Session::active_data_streams_confirmed() const
{
	if (_state != LinkState::PairActive) {
		return false;
	}

	if (_role == Role::Master) {
		return _local_control_consecutive >= 2 && _peer_sensor_consecutive >= 2;
	}

	return _local_sensor_consecutive >= 2 && _peer_control_consecutive >= 2;
}

void Session::queue_periodic_heartbeat(uint64_t now_us)
{
	if (active_data_streams_confirmed() || _link_test.active
	    || _link_test.pending_start || _link_test.pending_abort) {
		return;
	}

	if (_last_heartbeat_sent_us != 0 && now_us >= _last_heartbeat_sent_us
	    && now_us - _last_heartbeat_sent_us < kHeartbeatPeriodUs) {
		return;
	}

	Heartbeat heartbeat{};
	heartbeat.health_flags = local_health_flags(now_us);
	uint8_t payload[kHeartbeatPayloadSize] {};

	if (encode_heartbeat(heartbeat, payload)
	    && queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::Heartbeat), 0,
			   payload, sizeof(payload))) {
		_last_heartbeat_sent_us = now_us;
	}
}

void Session::start_handshake(uint64_t now_us)
{
	if (_role != Role::Master || _peer_session == 0 || _handshake.active
	    || _state == LinkState::PairActive || _state == LinkState::PairArmedGround) {
		return;
	}

	Handshake handshake{};
	handshake.target_peer_session = _peer_session;
	handshake.transaction_id = allocate_transaction();
	handshake.phase = HandshakePhase::Request;
	handshake.capabilities = local_capabilities();
	handshake.state_flags = local_handshake_flags();
	handshake.config_crc32c = _config_crc32c;
	uint8_t payload[kHandshakePayloadSize] {};

	if (!encode_handshake(handshake, payload)) {
		return;
	}

	Frame frame{};

	if (queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::LinkHandshake), 0,
			payload, sizeof(payload), &frame)) {
		_handshake.active = true;
		_handshake.frame = frame;
		_handshake.sends = 1;
		_handshake.max_sends = 4;
		_handshake.last_send_us = now_us;
		_handshake.timeout_us = kHandshakeRetryDelayUs;
		_handshake.transaction_id = handshake.transaction_id;
		_handshake.message_id = static_cast<uint8_t>(MessageId::LinkHandshake);
		_state = LinkState::HandshakePending;
	}
}

void Session::restart_handshake(uint64_t now_us)
{
	_handshake = {};
	_pending = {};
	_state = LinkState::Discovered;
	_handshake_retry_after_us = now_us + kHandshakeRestartDelayUs;
}

bool Session::handshake_report_valid(const Frame &frame, const Handshake &handshake) const
{
	return frame.header.source_role == Role::Slave
	       && frame.header.source_session == _peer_session
	       && handshake.target_peer_session == _session_id
	       && handshake.phase == HandshakePhase::Report
	       && handshake.protocol_version == kVersion
	       && (handshake.capabilities & kRequiredCapabilities) == kRequiredCapabilities
	       && handshake.config_crc32c == _config_crc32c
	       && (handshake.state_flags & HsLocalReady) != 0
	       && (handshake.state_flags & ReservedHandshakeStateFlags) == 0;
}

void Session::send_ack(const Frame &request, AckResult result, uint8_t detail)
{
	if (request.header.source_session == 0 || request.header.message_id == 0) {
		return;
	}

	Ack ack{};
	ack.target_peer_session = request.header.source_session;
	ack.event_id = request.header.payload_len >= 8 ? read_u32(request.payload + 4) : 0;
	ack.acked_message_id = request.header.message_id;
	ack.result = result;
	ack.detail = detail;
	uint8_t payload[kAckPayloadSize] {};

	if (encode_ack(ack, payload)) {
		queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::Ack), Response,
			    payload, sizeof(payload));
	}
}

Session::DuplicateEntry *Session::find_duplicate(uint32_t source_session, uint32_t event_id,
		uint8_t message_id, uint64_t now_us)
{
	DuplicateEntry *oldest = &_duplicates[0];

	for (DuplicateEntry &entry : _duplicates) {
		if (entry.valid && now_us >= entry.timestamp_us
		    && now_us - entry.timestamp_us > kDuplicateRetentionUs) {
			entry.valid = false;
		}

		if (entry.valid && entry.source_session == source_session && entry.event_id == event_id
		    && entry.message_id == message_id) {
			return &entry;
		}

		if (!entry.valid) {
			oldest = &entry;

		} else if (entry.timestamp_us < oldest->timestamp_us) {
			oldest = &entry;
		}
	}

	return oldest;
}

Session::DuplicateEntry *Session::find_event_duplicate(uint32_t source_session,
		uint32_t event_id, uint64_t now_us)
{
	DuplicateEntry *oldest = &_duplicates[0];

	for (DuplicateEntry &entry : _duplicates) {
		if (entry.valid && now_us >= entry.timestamp_us
		    && now_us - entry.timestamp_us > kDuplicateRetentionUs) {
			entry.valid = false;
		}

		if (entry.valid && entry.source_session == source_session
		    && entry.event_id == event_id) {
			return &entry;
		}

		if (!entry.valid) {
			oldest = &entry;

		} else if (entry.timestamp_us < oldest->timestamp_us) {
			oldest = &entry;
		}
	}

	return oldest;
}

void Session::cache_event(const Frame &frame, const ReliableEvent &event,
			  AckResult result, uint8_t detail, uint64_t now_us)
{
	DuplicateEntry *entry = find_event_duplicate(frame.header.source_session, event.event_id,
			       now_us);

	if (entry == nullptr) {
		return;
	}

	*entry = {};
	entry->valid = true;
	entry->source_session = frame.header.source_session;
	entry->event_id = event.event_id;
	entry->message_id = frame.header.message_id;
	entry->result = result;
	entry->detail = detail;
	entry->timestamp_us = now_us;
}

bool Session::acknowledge_duplicate(const Frame &frame, const ReliableEvent &event)
{
	DuplicateEntry *entry = find_event_duplicate(frame.header.source_session, event.event_id,
			       _last_peer_frame_us);

	if (entry == nullptr || !entry->valid) {
		return false;
	}

	const uint64_t now = _last_peer_frame_us;
	entry->timestamp_us = now;
	Frame original_request = frame;
	original_request.header.message_id = entry->message_id;
	send_ack(original_request, entry->result, entry->detail);
	++_duplicate_events;
	return true;
}

void Session::clear_control(bool disarm, ReasonCode reason, bool mark_reattach,
			    SessionActions &actions)
{
	if (disarm && _local.armed) {
		actions.request_disarm = true;
		actions.disarm_reason = reason;
	}

	actions.clear_follower_control = _role == Role::Slave;

	if (mark_reattach) {
		_pair_reattach_latched = true;
		_local.pair_reattach_latched = true;
	}

	_pair_arm_commit_deadline_us = 0;
	_master_arm_pending = false;
	_slave_arm_pending = false;
	_peer_sensor_consecutive = 0;
	_peer_control_consecutive = 0;
	_local_sensor_consecutive = 0;
	_local_control_consecutive = 0;
}

void Session::enter_link_down(SessionActions &actions, ReasonCode reason)
{
	const bool was_pair_transaction = _state == LinkState::PairActive
					  || _state == LinkState::PairArmedGround
					  || _state == LinkState::PairArmPending;
	const bool was_pair_active = _state == LinkState::PairActive
				     || _state == LinkState::PairArmedGround;
	clear_control(_local.armed && !_local.airborne_latched && was_pair_transaction,
		      reason, was_pair_active, actions);
	_state = LinkState::Down;
	_peer_session = 0;
	_remote = {};
	_handshake = {};
	_pending = {};
	actions.pair_lost = was_pair_active;
}

void Session::expire_pending(uint64_t now_us, SessionActions &actions)
{
	if (_handshake.active && now_us >= _handshake.last_send_us
	    && now_us - _handshake.last_send_us >= _handshake.timeout_us) {
		if (_handshake.sends < _handshake.max_sends) {
			if (build_and_enqueue(_handshake, true)) {
				++_handshake.sends;
				_handshake.last_send_us = now_us;
			}

		} else {
			restart_handshake(now_us);
		}
	}

	if (!_pending.active || now_us < _pending.last_send_us
	    || now_us - _pending.last_send_us < _pending.timeout_us) {
		return;
	}

	if (_pending.sends < _pending.max_sends) {
		if (build_and_enqueue(_pending, true)) {
			++_pending.sends;
			_pending.last_send_us = now_us;
		}

		return;
	}

	const uint8_t failed_message = _pending.message_id;
	_pending = {};

	if (failed_message == static_cast<uint8_t>(MessageId::PairArm)) {
		clear_control(true, ReasonCode::Other, _local.armed, actions);
		_state = LinkState::PairControlReady;
		_master_arm_pending = false;

		if (_role == Role::Master) {
			(void)request_event(MessageId::PairDisarm, ReasonCode::Other, now_us);
		}

	} else if (failed_message == static_cast<uint8_t>(MessageId::LinkTestControl)) {
		_link_test.pending_start = false;
		_link_test.pending_abort = false;
		finish_link_test(now_us, actions);

	} else {
		_state = LinkState::Ready;
	}
}

void Session::update_link_test(uint64_t now_us, SessionActions &actions)
{
	if (!_link_test.active) {
		return;
	}

	if (_link_test.master) {
		while (_link_test.next_send_us != 0 && now_us >= _link_test.next_send_us
		       && now_us <= _link_test.end_us
		       && _link_test.send_count < 5000) {
			uint8_t payload[kMaxPayload] {};
			const uint32_t sequence = _link_test.next_sequence++;
			write_u32(payload, _link_test.test_id);
			write_u32(payload + 4, sequence);
			write_u32(payload + 8, static_cast<uint32_t>(now_us));
			write_u16(payload + 12, _link_test.payload_len - kLinkTestDataHeaderSize);
			const uint16_t seed = static_cast<uint16_t>(0xA55Au ^ sequence);
			write_u16(payload + 14, seed);
			uint32_t state = seed | 1u;

			for (uint16_t index = kLinkTestDataHeaderSize; index < _link_test.payload_len; ++index) {
				state ^= state << 13;
				state ^= state >> 17;
				state ^= state << 5;
				payload[index] = static_cast<uint8_t>(state);
			}

			if (!queue_frame(FrameType::Data, static_cast<uint8_t>(MessageId::LinkTestData), 0,
					 payload, _link_test.payload_len)) {
				break;
			}

			_link_test.send_times_us[sequence % 5000] = now_us;
			++_link_test.send_count;
			const uint64_t period_us = 1'000'000ull / _link_test.rate_hz;
			_link_test.next_send_us += period_us;
		}

		if (now_us > _link_test.end_us) {
			finish_link_test(now_us, actions);
		}

	} else if (now_us > _link_test.end_us + kPairArmCommitTimeoutUs) {
		finish_link_test(now_us, actions);
	}
}

void Session::publish_link_test_stats(uint64_t now_us)
{
	LinkTestStats &stats = _link_test.stats;
	const uint32_t echo_count = minimum<uint32_t>(_link_test.echo_count, 5000);
	stats.sent = _link_test.send_count;
	stats.echoed = echo_count;
	stats.missing = stats.sent > stats.echoed ? stats.sent - stats.echoed : 0;

	if (echo_count == 0) {
		stats.rtt_min_us = 0;
		stats.rtt_mean_us = 0;
		stats.rtt_p50_us = 0;
		stats.rtt_p95_us = 0;
		stats.rtt_p99_us = 0;
		stats.rtt_max_us = 0;
		stats.rtt_jitter_us = 0;

	} else {
		uint64_t sum = 0;
		uint64_t jitter_sum = 0;
		uint32_t previous = 0;

		for (uint32_t i = 0; i < echo_count; ++i) {
			const uint32_t rtt = _link_test.rtt_samples_us[i];
			sum += rtt;

			if (i > 0) {
				jitter_sum += rtt > previous ? rtt - previous : previous - rtt;
			}

			previous = rtt;
		}

		for (uint32_t gap = echo_count / 2; gap > 0; gap /= 2) {
			for (uint32_t i = gap; i < echo_count; ++i) {
				const uint32_t value = _link_test.rtt_samples_us[i];
				uint32_t j = i;

				while (j >= gap && _link_test.rtt_samples_us[j - gap] > value) {
					_link_test.rtt_samples_us[j] = _link_test.rtt_samples_us[j - gap];
					j -= gap;
				}

				_link_test.rtt_samples_us[j] = value;
			}
		}

		stats.rtt_min_us = _link_test.rtt_samples_us[0];
		stats.rtt_max_us = _link_test.rtt_samples_us[echo_count - 1];
		stats.rtt_mean_us = static_cast<uint32_t>(sum / echo_count);
		stats.rtt_p50_us = _link_test.rtt_samples_us[(echo_count * 50) / 100];
		stats.rtt_p95_us = _link_test.rtt_samples_us[minimum<uint32_t>(
							     echo_count - 1, (echo_count * 95) / 100)];
		stats.rtt_p99_us = _link_test.rtt_samples_us[minimum<uint32_t>(
							     echo_count - 1, (echo_count * 99) / 100)];
		stats.rtt_jitter_us = echo_count > 1 ? static_cast<uint32_t>(jitter_sum / (echo_count - 1)) : 0;
	}

	const double duration_s = maximum(0.001,
					  static_cast<double>(now_us - _link_test.started_us) * 1e-6);
	stats.offered_bytes_per_second = static_cast<uint32_t>(
			(_link_test.send_count * (kOverhead + _link_test.payload_len)) / duration_s);
	stats.echo_bytes_per_second = static_cast<uint32_t>(
			(stats.echoed * (kOverhead + _link_test.payload_len)) / duration_s);
	stats.bidirectional_bytes_per_second =
		stats.offered_bytes_per_second + stats.echo_bytes_per_second;
}

void Session::finish_link_test(uint64_t now_us, SessionActions &actions)
{
	if (!_link_test.active && !_link_test.pending_start && !_link_test.pending_abort) {
		return;
	}

	if (_link_test.master) {
		publish_link_test_stats(now_us);
		actions.link_test_finished = true;
	}

	_link_test.active = false;
	_link_test.pending_start = false;
	_link_test.pending_abort = false;
	_link_test.next_send_us = 0;

	if (_role == Role::Slave) {
		_handshake_retry_after_us = now_us;
	}
}

bool Session::process_link_test_data(const Frame &frame, uint64_t now_us)
{
	if (!_link_test.active) {
		return false;
	}

	if (frame.header.payload_len != _link_test.payload_len) {
		++_link_test.stats.bad_echo;
		return false;
	}

	LinkTestDataHeader header{};

	if (!decode_link_test_data_header(frame.payload, frame.header.payload_len, header)
	    || header.test_id != _link_test.test_id) {
		++_link_test.stats.bad_echo;
		return false;
	}

	if (!link_test_pattern_matches(frame.payload, frame.header.payload_len,
				       header.test_sequence, header.pattern_seed)) {
		++_link_test.stats.bad_echo;
		return false;
	}

	if (!_link_test.master) {
		if (frame.header.flags != 0 || frame.header.source_role != Role::Master) {
			return false;
		}

		return queue_frame(FrameType::Data, static_cast<uint8_t>(MessageId::LinkTestData),
				   Response, frame.payload, frame.header.payload_len);
	}

	if (frame.header.source_role != Role::Slave || frame.header.flags != Response
	    || header.test_sequence >= _link_test.send_count) {
		++_link_test.stats.bad_echo;
		return false;
	}

	const size_t word = (header.test_sequence % 5000) / 8;
	const uint8_t mask = static_cast<uint8_t>(1u << (header.test_sequence % 8));

	if ((_link_test.echoed_bits[word] & mask) != 0) {
		++_link_test.stats.duplicate;
		return true;
	}

	if (header.test_sequence < _link_test.highest_echoed) {
		++_link_test.stats.reordered;
	}

	_link_test.highest_echoed = maximum(_link_test.highest_echoed, header.test_sequence);
	_link_test.echoed_bits[word] |= mask;
	const uint64_t send_time = _link_test.send_times_us[header.test_sequence % 5000];
	const uint32_t rtt_us = send_time != 0 && now_us >= send_time
				? static_cast<uint32_t>(minimum<uint64_t>(now_us - send_time,
						UINT32_MAX))
				: 0;
	_link_test.rtt_samples_us[_link_test.echo_count++] = rtt_us;
	++_link_test.stats.echoed;

	if (_link_test.last_echo_arrival_us != 0 && now_us >= _link_test.last_echo_arrival_us) {
		const uint32_t gap = static_cast<uint32_t>(minimum<uint64_t>(
					     now_us - _link_test.last_echo_arrival_us,
					     UINT32_MAX));
		_link_test.stats.max_receive_gap_us =
			maximum(_link_test.stats.max_receive_gap_us, gap);

		if (gap > kSensorFreshTimeoutUs) {
			++_link_test.stats.link_interruptions;
		}
	}

	_link_test.last_echo_arrival_us = now_us;
	return true;
}

bool Session::start_link_test(const LinkTestRequest &request, uint64_t now_us)
{
	if (_role != Role::Master || _state != LinkState::Ready || _local.armed
	    || _pair_reattach_latched || _emergency_latched || !peer_is_fresh(now_us)
	    || (_remote.health_flags & Armed) != 0
	    || (_remote.health_flags & AirborneLatched) != 0
	    || _link_test.active || _pending.active) {
		return false;
	}

	LinkTestControl control{};
	control.target_peer_session = _peer_session;
	control.test_id = allocate_transaction();
	control.opcode = LinkTestOpcode::Start;
	control.payload_len = request.payload_len;
	control.rate_hz = request.rate_hz;
	control.duration_ms = request.duration_ms;
	uint8_t payload[kLinkTestControlPayloadSize] {};

	if (!encode_link_test_control(control, payload)) {
		return false;
	}

	Frame frame{};

	if (!queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::LinkTestControl),
			 AckRequired, payload, sizeof(payload), &frame)) {
		return false;
	}

	_pending.active = true;
	_pending.frame = frame;
	_pending.sends = 1;
	_pending.max_sends = 4;
	_pending.last_send_us = now_us;
	_pending.timeout_us = kReliableAckTimeoutUs;
	_pending.transaction_id = control.test_id;
	_pending.message_id = static_cast<uint8_t>(MessageId::LinkTestControl);
	_link_test = {};
	_link_test.pending_start = true;
	_link_test.master = true;
	_link_test.test_id = control.test_id;
	_link_test.payload_len = request.payload_len;
	_link_test.rate_hz = request.rate_hz;
	_link_test.duration_ms = request.duration_ms;
	return true;
}

bool Session::abort_link_test(uint64_t now_us)
{
	if (_role != Role::Master || !_link_test.active || _link_test.pending_abort) {
		return false;
	}

	LinkTestControl control{};
	control.target_peer_session = _peer_session;
	control.test_id = _link_test.test_id;
	control.opcode = LinkTestOpcode::Abort;
	control.payload_len = _link_test.payload_len;
	control.rate_hz = _link_test.rate_hz;
	control.duration_ms = _link_test.duration_ms;
	uint8_t payload[kLinkTestControlPayloadSize] {};

	if (!encode_link_test_control(control, payload)) {
		return false;
	}

	Frame frame{};

	if (!queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::LinkTestControl),
			 AckRequired, payload, sizeof(payload), &frame)) {
		return false;
	}

	_pending.active = true;
	_pending.frame = frame;
	_pending.sends = 1;
	_pending.max_sends = 4;
	_pending.last_send_us = now_us;
	_pending.timeout_us = kReliableAckTimeoutUs;
	_pending.transaction_id = control.test_id;
	_pending.message_id = static_cast<uint8_t>(MessageId::LinkTestControl);
	_link_test.pending_abort = true;
	_link_test.aborted = true;
	return true;
}

bool Session::request_control_enable(uint64_t now_us)
{
	if (_role != Role::Master || _state != LinkState::Ready || _pending.active
	    || _pair_reattach_latched || _emergency_latched || _local.armed
	    || _link_test.active || _link_test.pending_start || _link_test.pending_abort
	    || !_local.pair_mode_selected || !_local.local_ready || !_local.local_rc_valid
	    || !_local.safety_gate_allow || !peer_is_fresh(now_us)
	    || (_remote.health_flags & InboundPeriodicFresh) == 0
	    || (_remote.health_flags & HealthLocalRcValid) == 0
	    || (_remote.health_flags & HealthSafetyGateAllow) == 0
	    || (_remote.health_flags & PairReattachLatched) != 0) {
		return false;
	}

	ReliableEvent event{};
	event.target_peer_session = _peer_session;
	event.event_id = allocate_transaction();
	event.reason_code = ReasonCode::None;
	uint8_t payload[kReliableEventPayloadSize] {};

	if (!encode_reliable_event(event, payload)) {
		return false;
	}

	Frame frame{};

	if (!queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::ControlEnable),
			 AckRequired, payload, sizeof(payload), &frame)) {
		return false;
	}

	_pending.active = true;
	_pending.frame = frame;
	_pending.sends = 1;
	_pending.max_sends = 4;
	_pending.last_send_us = now_us;
	_pending.timeout_us = kReliableAckTimeoutUs;
	_pending.transaction_id = event.event_id;
	_pending.message_id = static_cast<uint8_t>(MessageId::ControlEnable);
	_state = LinkState::PairEnablePending;
	return true;
}

bool Session::request_pair_arm(uint64_t now_us)
{
	if (_role != Role::Master || _state != LinkState::PairControlReady || _pending.active
	    || _control_enable_acked_us == 0 || now_us < _control_enable_acked_us
	    || now_us - _control_enable_acked_us > kSensorFreshTimeoutUs
	    || _pair_reattach_latched || _emergency_latched
	    || _local.armed || _local.airborne_latched || !_local.pair_mode_selected
	    || !_local.local_rc_valid || !_local.safety_gate_allow || !peer_is_fresh(now_us)
	    || (_remote.health_flags & HealthLocalRcValid) == 0
	    || (_remote.health_flags & HealthSafetyGateAllow) == 0
	    || (_remote.health_flags & InboundPeriodicFresh) == 0
	    || (_remote.health_flags & PairModeSelected) == 0
	    || (_remote.health_flags & Armed) != 0
	    || (_remote.health_flags & AirborneLatched) != 0
	    || (_remote.health_flags & PairReattachLatched) != 0) {
		return false;
	}

	ReliableEvent event{};
	event.target_peer_session = _peer_session;
	event.event_id = allocate_transaction();
	event.reason_code = ReasonCode::OperatorRequest;
	uint8_t payload[kReliableEventPayloadSize] {};

	if (!encode_reliable_event(event, payload)) {
		return false;
	}

	Frame frame{};

	if (!queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::PairArm),
			 AckRequired, payload, sizeof(payload), &frame)) {
		return false;
	}

	_pending.active = true;
	_pending.frame = frame;
	_pending.sends = 1;
	_pending.max_sends = 2;
	_pending.last_send_us = now_us;
	_pending.timeout_us = kPairArmAckTimeoutUs;
	_pending.transaction_id = event.event_id;
	_pending.message_id = static_cast<uint8_t>(MessageId::PairArm);
	_state = LinkState::PairArmPending;
	return true;
}

bool Session::request_event(MessageId message_id, ReasonCode reason, uint64_t now_us)
{
	if (_peer_session == 0 || _pending.active || (message_id != MessageId::EmergencyWithdraw
	    && message_id != MessageId::PairDisarm && message_id != MessageId::ControlRelease
	    && _state != LinkState::PairControlReady && _state != LinkState::PairArmPending
	    && _state != LinkState::PairArmedGround && _state != LinkState::PairActive)) {
		return false;
	}

	ReliableEvent event{};
	event.target_peer_session = _peer_session;
	event.event_id = allocate_transaction();
	event.reason_code = reason;
	uint8_t payload[kReliableEventPayloadSize] {};

	if (!encode_reliable_event(event, payload)) {
		return false;
	}

	const uint8_t flags = AckRequired | (message_id == MessageId::EmergencyWithdraw ? Urgent : 0);
	Frame frame{};

	if (!queue_frame(FrameType::Command, static_cast<uint8_t>(message_id), flags,
			 payload, sizeof(payload), &frame)) {
		return false;
	}

	_pending.active = true;
	_pending.frame = frame;
	_pending.sends = 1;
	_pending.max_sends = message_id == MessageId::PairDisarm ? 1 : 4;
	_pending.last_send_us = now_us;
	_pending.timeout_us = kReliableAckTimeoutUs;
	_pending.transaction_id = event.event_id;
	_pending.message_id = static_cast<uint8_t>(message_id);
	return true;
}

bool Session::request_pair_disarm(ReasonCode reason, uint64_t now_us)
{
	SessionActions ignored{};
	clear_control(true, reason, false, ignored);
	_state = LinkState::Ready;
	return request_event(MessageId::PairDisarm, reason, now_us);
}

bool Session::request_control_release(ReasonCode reason, uint64_t now_us)
{
	SessionActions ignored{};
	clear_control(false, reason, false, ignored);
	_state = LinkState::Ready;
	return request_event(MessageId::ControlRelease, reason, now_us);
}

bool Session::request_emergency_withdraw(ReasonCode reason, uint64_t now_us)
{
	SessionActions ignored{};
	clear_control(_local.armed && !_local.airborne_latched, reason, false, ignored);
	_emergency_latched = true;
	_state = LinkState::Discovered;
	return request_event(MessageId::EmergencyWithdraw, reason, now_us);
}

bool Session::notify_local_arm_result(bool success, uint64_t now_us)
{
	if (_role == Role::Master && _master_arm_pending && _state == LinkState::PairArmPending) {
		_master_arm_pending = false;
		_local.armed = success;

		if (success) {
			_state = LinkState::PairArmedGround;
			_pair_arm_commit_deadline_us = now_us + kPairArmCommitTimeoutUs;
			return true;
		}

		_state = LinkState::PairControlReady;
		return false;
	}

	if (_role == Role::Slave && _slave_arm_pending && _slave_arm_event_id != 0) {
		_slave_arm_pending = false;
		_local.armed = success;
		const AckResult result = success ? AckResult::Success : AckResult::RejectedSafety;
		send_ack(_slave_arm_request, result);
		ReliableEvent event{};

		if (decode_reliable_event(_slave_arm_request.payload,
					  _slave_arm_request.header.payload_len, event)) {
			cache_event(_slave_arm_request, event, result, 0, now_us);
		}

		_slave_arm_event_id = 0;

		if (success) {
			_state = LinkState::PairArmPending;
			_pair_arm_commit_deadline_us = now_us + kPairArmCommitTimeoutUs;
			return true;
		}

		_state = LinkState::Ready;
		return false;
	}

	return false;
}

bool Session::queue_sensor_state(const SensorState &sensor, uint64_t now_us)
{
	if (_role != Role::Slave || _state != LinkState::PairActive
	    || _last_peer_control_us == 0
	    || now_us - _last_peer_control_us > kSensorFreshTimeoutUs) {
		return false;
	}

	uint8_t payload[kSensorPayloadSize] {};

	if (!encode_sensor_payload(sensor, payload)) {
		return false;
	}

	if (!queue_frame(FrameType::Data, static_cast<uint8_t>(MessageId::SensorState), 0,
			 payload, sizeof(payload))) {
		return false;
	}

	_local_sensor_consecutive =
		_last_local_sensor_sent_us != 0 && now_us >= _last_local_sensor_sent_us
		&& now_us - _last_local_sensor_sent_us <= kSensorFreshTimeoutUs
		? static_cast<uint8_t>(minimum<int>(255, _local_sensor_consecutive + 1)) : 1;
	_last_local_sensor_sent_us = now_us;
	return true;
}

bool Session::query_control_tx_allowed(uint64_t now_us) const
{
	if (_role != Role::Master || !_local.armed
	    || (_state != LinkState::PairArmedGround && _state != LinkState::PairActive)) {
		return false;
	}

	if (_pair_arm_commit_deadline_us != 0 && now_us > _pair_arm_commit_deadline_us) {
		return false;
	}

	if (_state == LinkState::PairActive && last_peer_sensor_age_us(now_us) > kSensorFreshTimeoutUs) {
		return false;
	}

	return true;
}

bool Session::queue_actuator_command(const ActuatorCommand &command, float collective,
				     uint64_t now_us)
{
	if (!query_control_tx_allowed(now_us)) {
		return false;
	}

	uint8_t payload[kActuatorPayloadSize] {};

	if (!encode_actuator_command(command, payload)
	    || !queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::ActuatorCommand), 0,
			    payload, sizeof(payload))) {
		return false;
	}

	_last_control_sent_us = now_us;
	_pair_arm_commit_deadline_us = 0;
	_local_control_consecutive =
		_last_local_control_sent_us != 0 && now_us >= _last_local_control_sent_us
		&& now_us - _last_local_control_sent_us <= kSensorFreshTimeoutUs
		? static_cast<uint8_t>(minimum<int>(255, _local_control_consecutive + 1)) : 1;
	_last_local_control_sent_us = now_us;

	if (collective > 0.f) {
		_local.airborne_latched = true;
	}

	return true;
}

void Session::handle_ack(const Ack &ack, SessionActions &actions, uint64_t now_us)
{
	if (ack.target_peer_session != _session_id || !_pending.active
	    || ack.event_id != _pending.transaction_id || ack.acked_message_id != _pending.message_id) {
		return;
	}

	const uint8_t completed_message = _pending.message_id;
	const AckResult result = ack.result;
	_pending = {};

	if (completed_message == static_cast<uint8_t>(MessageId::LinkHandshake)) {
		if (result == AckResult::Success) {
			_state = LinkState::Ready;
			_handshake = {};
			_emergency_latched = false;

		} else {
			restart_handshake(now_us);
		}

		return;
	}

	if (completed_message == static_cast<uint8_t>(MessageId::ControlEnable)) {
		if (result == AckResult::Success) {
			_control_enable_acked_us = now_us;
			_state = LinkState::PairControlReady;
			_pair_control_ready_us = now_us;
			actions.control_enable_acked = true;

		} else {
			_state = LinkState::Ready;
		}

		return;
	}

	if (completed_message == static_cast<uint8_t>(MessageId::PairArm)) {
		if (result == AckResult::Success) {
			_master_arm_pending = true;
			_pair_arm_commit_deadline_us = now_us + kPairArmCommitTimeoutUs;
			actions.request_arm = true;

		} else {
			_state = LinkState::PairControlReady;
			clear_control(true, ReasonCode::Other, _local.armed, actions);
		}

		return;
	}

	if (completed_message == static_cast<uint8_t>(MessageId::LinkTestControl)) {
		const bool expected_detail =
			(_link_test.pending_start
			 && ack.detail == static_cast<uint8_t>(LinkTestOpcode::Start))
			|| (_link_test.pending_abort
			    && ack.detail == static_cast<uint8_t>(LinkTestOpcode::Abort));

		if (result != AckResult::Success || !expected_detail) {
			_link_test.pending_start = false;
			_link_test.pending_abort = false;
			finish_link_test(now_us, actions);
			return;
		}

		if (_link_test.pending_start) {
			_link_test.pending_start = false;
			_link_test.active = true;
			_link_test.master = true;
			_link_test.started_us = now_us;
			_link_test.end_us = now_us + static_cast<uint64_t>(_link_test.duration_ms) * 1000;
			_link_test.next_send_us = now_us;
			_link_test.next_sequence = 0;
			_link_test.highest_echoed = 0;

		} else if (_link_test.pending_abort) {
			finish_link_test(now_us, actions);
		}

		return;
	}

	if (completed_message == static_cast<uint8_t>(MessageId::ControlRelease)
	    || completed_message == static_cast<uint8_t>(MessageId::PairDisarm)) {
		_state = LinkState::Ready;
		return;
	}

	if (completed_message == static_cast<uint8_t>(MessageId::EmergencyWithdraw)) {
		_state = LinkState::Discovered;
		_peer_session = 0;
		_remote = {};
	}
}

void Session::handle_reliable_event(const Frame &frame, const ReliableEvent &event,
				    SessionActions &actions, uint64_t now_us)
{
	if (acknowledge_duplicate(frame, event)) {
		return;
	}

	if (event.target_peer_session != _session_id) {
		send_ack(frame, AckResult::SessionMismatch);
		return;
	}

	const MessageId message_id = static_cast<MessageId>(frame.header.message_id);
	AckResult result = AckResult::Success;
	uint8_t detail = 0;

	if ((_link_test.active || _link_test.pending_start || _link_test.pending_abort)
	    && (message_id == MessageId::ControlEnable || message_id == MessageId::PairArm)) {
		finish_link_test(now_us, actions);
		cache_event(frame, event, AckResult::RejectedSafety, 0, now_us);
		send_ack(frame, AckResult::RejectedSafety);
		return;
	}

	if ((message_id == MessageId::ControlEnable && event.reason_code != ReasonCode::None)
	    || (message_id != MessageId::ControlEnable
		&& event.reason_code == ReasonCode::None)) {
		cache_event(frame, event, AckResult::BadArgument, 0, now_us);
		send_ack(frame, AckResult::BadArgument);
		return;
	}

	switch (message_id) {
	case MessageId::ControlEnable: {
			if (_role != Role::Slave || frame.header.source_role != Role::Master
			    || event.reason_code != ReasonCode::None) {
				result = AckResult::InvalidState;

			} else if (_state == LinkState::PairControlReady) {
				result = AckResult::Success;
				_control_enable_acked_us = now_us;
				_pair_control_ready_us = now_us;

			} else if (_state != LinkState::Ready || _pair_reattach_latched || _emergency_latched
				   || !_local.local_ready || !_local.pair_mode_selected
				   || !_local.local_rc_valid
				   || !_local.safety_gate_allow || _local.armed) {
				result = (_pair_reattach_latched || !_local.local_rc_valid
					  || !_local.safety_gate_allow) ? AckResult::RejectedSafety
					 : AckResult::InvalidState;

			} else {
				_state = LinkState::PairControlReady;
				_control_enable_acked_us = now_us;
				_pair_control_ready_us = now_us;
			}

			cache_event(frame, event, result, detail, now_us);
			send_ack(frame, result, detail);
			return;
		}

	case MessageId::ControlRelease: {
			if (_role != Role::Slave || frame.header.source_role != Role::Master) {
				cache_event(frame, event, AckResult::InvalidState, 0, now_us);
				send_ack(frame, AckResult::InvalidState);
				return;
			}

			clear_control(false, event.reason_code, false, actions);
			_state = LinkState::Ready;
			cache_event(frame, event, AckResult::Success, 0, now_us);
			send_ack(frame, AckResult::Success);
			return;
		}

	case MessageId::EmergencyWithdraw: {
			clear_control(_local.armed && !_local.airborne_latched, event.reason_code,
				      false, actions);
			_emergency_latched = true;
			_state = LinkState::Discovered;
			cache_event(frame, event, AckResult::Success, 0, now_us);
			send_ack(frame, AckResult::Success);
			return;
		}

	case MessageId::PairArm: {
			if (_role != Role::Slave || frame.header.source_role != Role::Master) {
				result = AckResult::InvalidState;

			} else if (_slave_arm_pending) {
				send_ack(frame, AckResult::Busy);
				return;

			} else if (_state != LinkState::PairControlReady || _pair_reattach_latched
				   || _emergency_latched || _local.armed || _local.airborne_latched
				   || _pair_control_ready_us == 0
				   || now_us < _pair_control_ready_us
				   || now_us - _pair_control_ready_us > kSensorFreshTimeoutUs
				   || !_local.pair_mode_selected || !_local.local_rc_valid
				   || !_local.safety_gate_allow
				   || (_remote.health_flags & Armed) != 0
				   || (_remote.health_flags & AirborneLatched) != 0
				   || (_remote.health_flags & PairModeSelected) == 0
				   || (_remote.health_flags & PairReattachLatched) != 0
				   || !peer_is_fresh(now_us)) {
				result = (_pair_reattach_latched || !_local.local_rc_valid
					  || !_local.safety_gate_allow) ? AckResult::RejectedSafety
					 : AckResult::InvalidState;

			} else {
				_slave_arm_pending = true;
				_slave_arm_event_id = event.event_id;
				_slave_arm_request = frame;
				_state = LinkState::PairArmPending;
				actions.request_arm = true;
				return;
			}

			cache_event(frame, event, result, detail, now_us);
			send_ack(frame, result, detail);
			return;
		}

	case MessageId::PairDisarm: {
			if (_role != Role::Slave || frame.header.source_role != Role::Master) {
				cache_event(frame, event, AckResult::InvalidState, 0, now_us);
				send_ack(frame, AckResult::InvalidState);
				return;
			}

			clear_control(true, event.reason_code, false, actions);
			_state = LinkState::Ready;
			cache_event(frame, event, AckResult::Success, 0, now_us);
			send_ack(frame, AckResult::Success);
			return;
		}

	default:
		result = AckResult::Unsupported;
		send_ack(frame, result);
		return;
	}
}

SessionActions Session::receive(const Frame &frame, uint64_t now_us)
{
	SessionActions actions{};

	if (_role == Role::Disabled
	    || !roles_are_opposite(frame.header.source_role, frame.header.dest_role)
	    || frame.header.dest_role != _role) {
		++_session_errors;
		return actions;
	}

	const MessageId message_id = static_cast<MessageId>(frame.header.message_id);

	if (frame.header.frame_type == FrameType::Command && message_id == MessageId::Heartbeat) {
		Heartbeat heartbeat{};

		if (!decode_heartbeat(frame.payload, frame.header.payload_len, heartbeat)) {
			return actions;
		}

		++_rx_frames;
		_last_peer_frame_us = now_us;
		_last_peer_heartbeat_us = now_us;
		_remote.session = frame.header.source_session;
		_remote.health_flags = heartbeat.health_flags;
		_remote.inbound_periodic_fresh = (heartbeat.health_flags & InboundPeriodicFresh) != 0;
		_remote.local_rc_valid = (heartbeat.health_flags & HealthLocalRcValid) != 0;
		_remote.safety_gate_allow = (heartbeat.health_flags & HealthSafetyGateAllow) != 0;
		_remote.airlink_control_active =
			(heartbeat.health_flags & HealthAirlinkControlActive) != 0;
		_remote.armed = (heartbeat.health_flags & Armed) != 0;
		_remote.airborne_latched = (heartbeat.health_flags & AirborneLatched) != 0;
		_remote.pair_mode_selected = (heartbeat.health_flags & PairModeSelected) != 0;
		_remote.pair_reattach_latched =
			(heartbeat.health_flags & PairReattachLatched) != 0;

		if (_peer_session != frame.header.source_session) {
			if (_link_test.active || _link_test.pending_start || _link_test.pending_abort) {
				finish_link_test(now_us, actions);
			}

			const bool was_pair_transaction = _state == LinkState::PairActive
							  || _state == LinkState::PairArmedGround
							  || _state == LinkState::PairArmPending;
			const bool was_pair_active = _state == LinkState::PairActive
						     || _state == LinkState::PairArmedGround;
			clear_control(_local.armed && !_local.airborne_latched
				      && was_pair_transaction,
				      ReasonCode::SessionChanged, was_pair_active, actions);
			_peer_session = frame.header.source_session;
			_state = LinkState::Discovered;
			_handshake = {};
			_pending = {};
			_completed_handshake_transaction = 0;
			_has_peer_sensor_sequence = false;
			_has_peer_control_sequence = false;
			_handshake_retry_after_us = now_us;

			if (was_pair_active) {
				actions.pair_lost = true;
			}
		}

		if (_role == Role::Master && _state == LinkState::Discovered
		    && now_us >= _handshake_retry_after_us) {
			start_handshake(now_us);
		}

		return actions;
	}

	if (_peer_session == 0 || frame.header.source_session != _peer_session) {
		++_session_errors;
		return actions;
	}

	_last_peer_frame_us = now_us;
	++_rx_frames;

	if (frame.header.frame_type == FrameType::Data) {
		if (message_id == MessageId::SensorState) {
			SensorState sensor{};

			if (_role == Role::Master && frame.header.source_role == Role::Slave
			    && decode_sensor_state(frame.payload, frame.header.payload_len, sensor)) {
				if (_has_peer_sensor_sequence
				    && frame.header.sequence == _last_peer_sensor_sequence
				    && _last_peer_sensor_us != 0 && now_us >= _last_peer_sensor_us
				    && now_us - _last_peer_sensor_us <= kSensorFreshTimeoutUs) {
					return actions;
				}

				_peer_sensor = sensor;
				_remote.sensor_status_flags = sensor.status_flags;
				_remote.airlink_control_active =
					(sensor.status_flags & AirlinkControlActive) != 0;
				_peer_sensor_consecutive =
					_last_peer_sensor_us != 0 && now_us >= _last_peer_sensor_us
					&& now_us - _last_peer_sensor_us <= kSensorFreshTimeoutUs
					? static_cast<uint8_t>(minimum<int>(
								       255, _peer_sensor_consecutive + 1)) : 1;
				_last_peer_sensor_us = now_us;
				_last_peer_sensor_sequence = frame.header.sequence;
				_has_peer_sensor_sequence = true;

				if (_state == LinkState::PairArmedGround
				    && _remote.airlink_control_active
				    && last_peer_sensor_age_us(now_us) <= kSensorFreshTimeoutUs) {
					_state = LinkState::PairActive;
					actions.pair_active = true;
				}
			}

		} else if (message_id == MessageId::LinkTestData) {
			process_link_test_data(frame, now_us);
		}

		return actions;
	}

	switch (message_id) {
	case MessageId::ActuatorCommand: {
			ActuatorCommand command{};

			if (_role == Role::Slave && frame.header.source_role == Role::Master
			    && decode_actuator_command(frame.payload, frame.header.payload_len, command)
				&& (!_has_peer_control_sequence || _last_peer_control_us == 0
				    || now_us < _last_peer_control_us
				    || now_us - _last_peer_control_us > kSensorFreshTimeoutUs
				    || frame.header.sequence != _last_peer_control_sequence)
			    && _local.armed && !_pair_reattach_latched && !_emergency_latched
			    && (_state == LinkState::PairArmPending || _state == LinkState::PairActive)
			    && (_pair_arm_commit_deadline_us == 0
				|| now_us <= _pair_arm_commit_deadline_us)) {
				_peer_command = command;
				_peer_control_consecutive =
					_last_peer_control_us != 0 && now_us >= _last_peer_control_us
					&& now_us - _last_peer_control_us <= kSensorFreshTimeoutUs
					? static_cast<uint8_t>(minimum<int>(
								       255, _peer_control_consecutive + 1)) : 1;
				_last_peer_control_us = now_us;
				_last_peer_control_sequence = frame.header.sequence;
				_has_peer_control_sequence = true;
				_pair_arm_commit_deadline_us = 0;

				if (_state == LinkState::PairArmPending) {
					_state = LinkState::PairActive;
					actions.pair_active = true;
				}
			}

			return actions;
		}

	case MessageId::Ack: {
			Ack ack{};

			if (decode_ack(frame.payload, frame.header.payload_len, ack)) {
				handle_ack(ack, actions, now_us);
			}

			return actions;
		}

	case MessageId::LinkHandshake: {
			Handshake handshake{};

			if (!decode_handshake(frame.payload, frame.header.payload_len, handshake)) {
				return actions;
			}

			if (handshake.target_peer_session != _session_id) {
				++_session_errors;
				return actions;
			}

			if (_role == Role::Slave && frame.header.source_role == Role::Master
			    && handshake.phase == HandshakePhase::Request
			    && handshake_flags_match(frame.header.flags, 0)) {
				if (_handshake.active && _handshake.transaction_id == handshake.transaction_id) {
					build_and_enqueue(_handshake, false);
					return actions;
				}

				Handshake report{};
				report.target_peer_session = _peer_session;
				report.transaction_id = handshake.transaction_id;
				report.phase = HandshakePhase::Report;
				report.capabilities = local_capabilities();
				report.state_flags = local_handshake_flags();
				report.config_crc32c = _config_crc32c;
				uint8_t payload[kHandshakePayloadSize] {};

				if (encode_handshake(report, payload)) {
					Frame frame_to_send{};

					if (queue_frame(FrameType::Command,
							static_cast<uint8_t>(MessageId::LinkHandshake),
							Response, payload, sizeof(payload), &frame_to_send)) {
						_handshake.active = true;
						_handshake.frame = frame_to_send;
						_handshake.sends = 1;
						_handshake.max_sends = 4;
						_handshake.last_send_us = now_us;
						_handshake.timeout_us = kHandshakeRetryDelayUs;
						_handshake.transaction_id = handshake.transaction_id;
						_handshake.message_id =
							static_cast<uint8_t>(MessageId::LinkHandshake);
						_state = LinkState::HandshakePending;
					}
				}

				return actions;
			}

			if (_role == Role::Master && _state == LinkState::HandshakePending
			    && _handshake.active && handshake.transaction_id == _handshake.transaction_id
			    && handshake.phase == HandshakePhase::Report
			    && handshake_flags_match(frame.header.flags, Response)) {
				if (!handshake_report_valid(frame, handshake)) {
					restart_handshake(now_us);
					return actions;
				}

				Handshake confirm = handshake;
				confirm.target_peer_session = _peer_session;
				confirm.phase = HandshakePhase::Confirm;
				confirm.capabilities = local_capabilities();
				confirm.state_flags = local_handshake_flags();
				confirm.config_crc32c = _config_crc32c;
				uint8_t payload[kHandshakePayloadSize] {};

				if (encode_handshake(confirm, payload)) {
					Frame frame_to_send{};

					if (queue_frame(FrameType::Command,
							static_cast<uint8_t>(MessageId::LinkHandshake),
							AckRequired, payload, sizeof(payload), &frame_to_send)) {
						_handshake.active = false;
						_pending.active = true;
						_pending.frame = frame_to_send;
						_pending.sends = 1;
						_pending.max_sends = 4;
						_pending.last_send_us = now_us;
						_pending.timeout_us = kHandshakeRetryDelayUs;
						_pending.transaction_id = handshake.transaction_id;
						_pending.message_id =
							static_cast<uint8_t>(MessageId::LinkHandshake);
					}
				}

				return actions;
			}

			if (_role == Role::Slave && _state == LinkState::HandshakePending
			    && handshake.phase == HandshakePhase::Confirm
			    && handshake_flags_match(frame.header.flags, AckRequired)) {
				if (!_handshake.active || handshake.transaction_id != _handshake.transaction_id
				    || handshake.capabilities != local_capabilities()
				    || handshake.config_crc32c != _config_crc32c) {
					return actions;
				}

				Ack ack{};
				ack.target_peer_session = _peer_session;
				ack.event_id = handshake.transaction_id;
				ack.acked_message_id = static_cast<uint8_t>(MessageId::LinkHandshake);
				ack.result = AckResult::Success;
				uint8_t payload[kAckPayloadSize] {};

				if (encode_ack(ack, payload)
				    && queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::Ack),
						   Response, payload, sizeof(payload))) {
					_handshake = {};
					_pending = {};
					_completed_handshake_transaction = handshake.transaction_id;
					_state = LinkState::Ready;
					_emergency_latched = false;
				}

				return actions;
			}

			if (_role == Role::Slave && _state == LinkState::Ready
			    && handshake.phase == HandshakePhase::Confirm
			    && handshake.transaction_id == _completed_handshake_transaction
			    && handshake_flags_match(frame.header.flags, AckRequired)) {
				Ack ack{};
				ack.target_peer_session = _peer_session;
				ack.event_id = handshake.transaction_id;
				ack.acked_message_id = static_cast<uint8_t>(MessageId::LinkHandshake);
				ack.result = AckResult::Success;
				uint8_t payload[kAckPayloadSize] {};

				if (encode_ack(ack, payload)) {
					queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::Ack),
						    Response, payload, sizeof(payload));
				}

				return actions;
			}

			return actions;
		}

	case MessageId::LinkTestControl: {
			LinkTestControl control{};
			const bool decoded = decode_link_test_control(frame.payload, frame.header.payload_len,
					   control);
			const uint32_t test_id = frame.header.payload_len >= 8
						 ? read_u32(frame.payload + 4) : 0;
			const uint8_t opcode = frame.header.payload_len >= 9 ? frame.payload[8] : 0;
			const uint8_t cache_message_id = static_cast<uint8_t>(0xA0u | opcode);
			DuplicateEntry *duplicate = find_duplicate(frame.header.source_session, test_id,
					     cache_message_id, now_us);
			auto send_test_ack = [this, &frame, test_id, opcode](AckResult result) {
				Ack ack{};
				ack.target_peer_session = frame.header.source_session;
				ack.event_id = test_id;
				ack.acked_message_id = static_cast<uint8_t>(MessageId::LinkTestControl);
				ack.result = result;
				ack.detail = opcode;
				uint8_t payload[kAckPayloadSize] {};

				if (encode_ack(ack, payload)) {
					queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::Ack),
						    Response, payload, sizeof(payload));
				}
			};

			if (duplicate != nullptr && duplicate->valid) {
				duplicate->timestamp_us = now_us;
				send_test_ack(duplicate->result);
				++_duplicate_events;
				return actions;
			}

			AckResult result = AckResult::Success;

			if (!decoded || test_id == 0) {
				if (test_id != 0) {
					send_test_ack(AckResult::BadArgument);
				}

				return actions;

			} else if (_role != Role::Slave || frame.header.source_role != Role::Master
				   || control.target_peer_session != _session_id) {
				result = control.target_peer_session != _session_id
					 ? AckResult::SessionMismatch : AckResult::InvalidState;

			} else if (_state != LinkState::Ready || _local.armed
				   || _local.airborne_latched || _pair_reattach_latched) {
				result = AckResult::InvalidState;

			} else if (control.opcode == LinkTestOpcode::Start) {
				if (_link_test.active) {
					result = AckResult::Busy;

				} else {
					_link_test = {};
					_link_test.active = true;
					_link_test.master = false;
					_link_test.test_id = control.test_id;
					_link_test.payload_len = control.payload_len;
					_link_test.rate_hz = control.rate_hz;
					_link_test.duration_ms = control.duration_ms;
					_link_test.started_us = now_us;
					_link_test.end_us = now_us
							    + static_cast<uint64_t>(control.duration_ms) * 1000;
				}

			} else if (!_link_test.active || _link_test.test_id != control.test_id) {
				result = AckResult::InvalidState;

			} else {
				_link_test.active = false;
				_link_test.end_us = now_us;
			}

			if (duplicate != nullptr) {
				*duplicate = {};
				duplicate->valid = true;
				duplicate->source_session = frame.header.source_session;
				duplicate->event_id = test_id;
				duplicate->message_id = cache_message_id;
				duplicate->result = result;
				duplicate->detail = opcode;
				duplicate->timestamp_us = now_us;
			}

			send_test_ack(result);
			return actions;
		}

	case MessageId::ControlEnable:
	case MessageId::ControlRelease:
	case MessageId::EmergencyWithdraw:
	case MessageId::PairArm:
	case MessageId::PairDisarm: {
			ReliableEvent event{};
			const bool valid = decode_reliable_event(frame.payload, frame.header.payload_len, event);

			if (!valid && (frame.header.flags & AckRequired) != 0) {
				const uint32_t event_id = frame.header.payload_len >= 8
							  ? read_u32(frame.payload + 4) : 0;

				if (event_id != 0) {
					Ack ack{};
					ack.target_peer_session = frame.header.source_session;
					ack.event_id = event_id;
					ack.acked_message_id = frame.header.message_id;
					ack.result = AckResult::BadArgument;
					uint8_t payload[kAckPayloadSize] {};

					if (encode_ack(ack, payload)) {
						queue_frame(FrameType::Command,
							    static_cast<uint8_t>(MessageId::Ack),
							    Response, payload, sizeof(payload));
					}
				}
			}

			if (valid) {
				handle_reliable_event(frame, event, actions, now_us);
			}

			return actions;
		}

	default:
		++_unknown_messages;

		if ((frame.header.flags & AckRequired) != 0) {
			Ack ack{};
			ack.target_peer_session = frame.header.source_session;
			ack.event_id = frame.header.payload_len >= 8 ? read_u32(frame.payload + 4) : 0;

			if (ack.event_id == 0) {
				ack.event_id = frame.header.sequence != 0 ? frame.header.sequence : 1;
			}

			ack.acked_message_id = frame.header.message_id;
			ack.result = AckResult::Unsupported;
			uint8_t payload[kAckPayloadSize] {};

			if (encode_ack(ack, payload)) {
				queue_frame(FrameType::Command, static_cast<uint8_t>(MessageId::Ack),
					    Response, payload, sizeof(payload));
			}
		}

		return actions;
	}
}

SessionActions Session::tick(uint64_t now_us)
{
	SessionActions actions{};

	if (_role == Role::Disabled) {
		return actions;
	}

	_local.pair_reattach_latched = _pair_reattach_latched;
	expire_pending(now_us, actions);

	if (_peer_session != 0 && _state >= LinkState::Discovered
	    && _state != LinkState::PairActive && !_link_test.active
	    && !_link_test.pending_start && !_link_test.pending_abort) {
		if (_last_peer_heartbeat_us == 0 || now_us < _last_peer_heartbeat_us
		    || now_us - _last_peer_heartbeat_us > kPeerHeartbeatTimeoutUs) {
			enter_link_down(actions, ReasonCode::HeartbeatTimeout);
		}
	}

	if (_state == LinkState::PairControlReady && _role == Role::Slave
	    && _pair_control_ready_us != 0 && now_us >= _pair_control_ready_us
	    && now_us - _pair_control_ready_us > kSensorFreshTimeoutUs) {
		_state = LinkState::Ready;
		_control_enable_acked_us = 0;

	} else if (_state == LinkState::PairControlReady && _role == Role::Master
		   && _pair_control_ready_us != 0 && now_us >= _pair_control_ready_us
		   && now_us - _pair_control_ready_us > kSensorFreshTimeoutUs) {
		_state = LinkState::Ready;
		_control_enable_acked_us = 0;
	}

	const bool control_transaction = _state == LinkState::PairActive
					 || _state == LinkState::PairArmedGround
					 || _state == LinkState::PairArmPending;

	if (control_transaction && (!_local.local_rc_valid || !_local.safety_gate_allow)) {
		clear_control(_local.armed && !_local.airborne_latched,
			      ReasonCode::LocalRcOrSafety, false, actions);
		_state = LinkState::PairLost;
		actions.pair_lost = true;
	}

	if (_state == LinkState::PairArmPending && _local.armed
	    && _pair_arm_commit_deadline_us != 0 && now_us > _pair_arm_commit_deadline_us) {
		clear_control(true, ReasonCode::ControlStale, true, actions);
		_state = LinkState::PairLost;
		actions.pair_lost = true;

	} else if (_state == LinkState::PairArmedGround && _pair_arm_commit_deadline_us != 0
		   && now_us > _pair_arm_commit_deadline_us) {
		clear_control(true, ReasonCode::ControlStale, true, actions);
		_state = LinkState::PairLost;
		actions.pair_lost = true;
	}

	bool critical_stream_stale = false;

	if (_state == LinkState::PairActive) {
		if (_role == Role::Master) {
			critical_stream_stale = last_peer_sensor_age_us(now_us) > kSensorFreshTimeoutUs
						|| !_remote.airlink_control_active;

		} else {
			critical_stream_stale = last_peer_control_age_us(now_us) > kSensorFreshTimeoutUs;
		}
	}

	if (critical_stream_stale) {
		clear_control(_local.armed && !_local.airborne_latched,
			      _role == Role::Master ? ReasonCode::SensorStale : ReasonCode::ControlStale,
			      true, actions);
		_state = LinkState::PairLost;
		actions.pair_lost = true;
	}

	if (_state == LinkState::Discovered && _role == Role::Master
	    && now_us >= _handshake_retry_after_us) {
		start_handshake(now_us);
	}

	update_link_test(now_us, actions);
	queue_periodic_heartbeat(now_us);
	return actions;
}

const char *link_state_name(LinkState state)
{
	switch (state) {
	case LinkState::Down: return "AIRLINK_DOWN";
	case LinkState::Discovered: return "AIRLINK_DISCOVERED";
	case LinkState::HandshakePending: return "LINK_HANDSHAKE_PENDING";
	case LinkState::Ready: return "AIRLINK_READY";
	case LinkState::PairEnablePending: return "PAIR_ENABLE_PENDING";
	case LinkState::PairControlReady: return "PAIR_CONTROL_READY";
	case LinkState::PairArmPending: return "PAIR_ARM_PENDING";
	case LinkState::PairArmedGround: return "PAIR_ARMED_GROUND";
	case LinkState::PairActive: return "PAIR_ACTIVE";
	case LinkState::PairLost: return "PAIR_LOST";
	}

	return "UNKNOWN";
}

} // namespace airlink
