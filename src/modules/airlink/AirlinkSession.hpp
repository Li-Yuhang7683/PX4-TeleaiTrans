/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "AirlinkProtocol.hpp"

#include <cstddef>
#include <cstdint>

namespace airlink
{

enum class LinkState : uint8_t {
	Down = 0,
	Discovered = 1,
	HandshakePending = 2,
	Ready = 3,
	PairEnablePending = 4,
	PairControlReady = 5,
	PairArmPending = 6,
	PairArmedGround = 7,
	PairActive = 8,
	PairLost = 9,
};

struct LocalStatus {
	bool local_ready{false};
	bool local_rc_valid{false};
	bool safety_gate_allow{false};
	bool armed{false};
	bool airborne_latched{false};
	bool pair_mode_selected{false};
	bool gs_link_valid{false};
	bool gs_position_active{false};
	bool single_landing_active{false};
	bool pair_reattach_latched{false};
};

struct SessionActions {
	bool request_arm{false};
	bool request_disarm{false};
	ReasonCode disarm_reason{ReasonCode::Other};
	bool clear_follower_control{false};
	bool control_enable_acked{false};
	bool pair_active{false};
	bool pair_lost{false};
	bool link_test_finished{false};
};

struct RemoteStatus {
	uint16_t health_flags{0};
	uint16_t sensor_status_flags{0};
	uint32_t session{0};
	bool inbound_periodic_fresh{false};
	bool local_rc_valid{false};
	bool safety_gate_allow{false};
	bool airlink_control_active{false};
	bool armed{false};
	bool airborne_latched{false};
	bool pair_mode_selected{false};
	bool pair_reattach_latched{false};
};

struct LinkTestRequest {
	uint16_t payload_len{256};
	uint16_t rate_hz{500};
	uint16_t duration_ms{5000};
};

struct LinkTestStats {
	uint32_t sent{0};
	uint32_t echoed{0};
	uint32_t duplicate{0};
	uint32_t reordered{0};
	uint32_t missing{0};
	uint32_t bad_echo{0};
	uint32_t rtt_min_us{0};
	uint32_t rtt_mean_us{0};
	uint32_t rtt_p50_us{0};
	uint32_t rtt_p95_us{0};
	uint32_t rtt_p99_us{0};
	uint32_t rtt_max_us{0};
	uint32_t rtt_jitter_us{0};
	uint32_t max_receive_gap_us{0};
	uint32_t link_interruptions{0};
	uint32_t offered_bytes_per_second{0};
	uint32_t echo_bytes_per_second{0};
	uint32_t bidirectional_bytes_per_second{0};
};

class Session
{
public:
	explicit Session(uint32_t config_crc32c);

	void start(Role role, uint32_t session_id, const LocalStatus &local, uint64_t now_us);
	void set_local_status(const LocalStatus &local);
	void set_armed(bool armed);
	bool set_role(Role role, uint32_t new_session_id, const LocalStatus &local, uint64_t now_us);

	LinkState state() const { return _state; }
	Role role() const { return _role; }
	uint32_t session_id() const { return _session_id; }
	uint32_t peer_session_id() const { return _peer_session; }
	uint32_t config_crc32c() const { return _config_crc32c; }
	const RemoteStatus &remote() const { return _remote; }
	const ActuatorCommand &peer_actuator_command() const { return _peer_command; }
	const SensorState &peer_sensor_state() const { return _peer_sensor; }
	bool pair_reattach_latched() const { return _pair_reattach_latched; }
	bool link_test_active() const { return _link_test.active; }
	uint32_t link_test_id() const { return _link_test.test_id; }
	bool link_test_was_aborted() const { return _link_test.aborted; }
	const LinkTestStats &link_test_stats() const { return _link_test.stats; }

	SessionActions tick(uint64_t now_us);
	SessionActions receive(const Frame &frame, uint64_t now_us);
	bool pop_tx(Frame &frame);
	size_t tx_queue_size() const { return _tx_count; }

	bool queue_sensor_state(const SensorState &sensor, uint64_t now_us);
	bool query_control_tx_allowed(uint64_t now_us) const;
	bool queue_actuator_command(const ActuatorCommand &command, float collective, uint64_t now_us);
	bool request_control_enable(uint64_t now_us);
	bool request_pair_arm(uint64_t now_us);
	bool request_pair_disarm(ReasonCode reason, uint64_t now_us);
	bool request_control_release(ReasonCode reason, uint64_t now_us);
	bool request_emergency_withdraw(ReasonCode reason, uint64_t now_us);
	bool notify_local_arm_result(bool success, uint64_t now_us);

	bool start_link_test(const LinkTestRequest &request, uint64_t now_us);
	bool abort_link_test(uint64_t now_us);

	uint32_t rx_frames() const { return _rx_frames; }
	uint32_t tx_frames() const { return _tx_frames; }
	uint32_t session_errors() const { return _session_errors; }
	uint32_t unknown_messages() const { return _unknown_messages; }
	uint32_t reliable_retries() const { return _reliable_retries; }
	uint32_t duplicate_events() const { return _duplicate_events; }
	uint32_t last_peer_age_us(uint64_t now_us) const;
	uint32_t last_peer_sensor_age_us(uint64_t now_us) const;
	uint32_t last_peer_control_age_us(uint64_t now_us) const;

private:
	static constexpr size_t kTxQueueSize{16};
	static constexpr size_t kDuplicateCacheSize{64};
	static constexpr uint64_t kHeartbeatPeriodUs{2'000};
	static constexpr uint64_t kPeerHeartbeatTimeoutUs{kPrecontrolHeartbeatTimeoutUs};
	static constexpr uint64_t kHandshakeRetryDelayUs{5'000};
	static constexpr uint64_t kHandshakeRestartDelayUs{50'000};
	static constexpr uint64_t kDuplicateRetentionUs{60'000'000};
	static constexpr uint32_t kRequiredCapabilities{Single | Pair | PairArmCapability | GsPosition
			| SingleLanding | LinkTest | SensorStateV1 | Output2M3s};

	struct PendingTemplate {
		bool active{false};
		Frame frame{};
		uint8_t sends{0};
		uint8_t max_sends{4};
		uint64_t last_send_us{0};
		uint64_t timeout_us{kReliableAckTimeoutUs};
		uint32_t transaction_id{0};
		uint8_t message_id{0};
	};

	struct DuplicateEntry {
		bool valid{false};
		uint32_t source_session{0};
		uint32_t event_id{0};
		uint8_t message_id{0};
		AckResult result{AckResult::InternalError};
		uint8_t detail{0};
		uint64_t timestamp_us{0};
	};

	struct LinkTestState {
		bool active{false};
		bool aborted{false};
		bool pending_start{false};
		bool pending_abort{false};
		bool master{false};
		uint32_t test_id{0};
		uint16_t payload_len{0};
		uint16_t rate_hz{0};
		uint16_t duration_ms{0};
		uint64_t started_us{0};
		uint64_t end_us{0};
		uint64_t next_send_us{0};
		uint64_t last_echo_arrival_us{0};
		uint32_t next_sequence{0};
		uint32_t highest_echoed{0};
		uint32_t echo_count{0};
		uint32_t send_count{0};
		uint64_t send_times_us[5000] {};
		uint32_t rtt_samples_us[5000] {};
		uint8_t echoed_bits[(5000 + 7) / 8] {};
		LinkTestStats stats{};
	};

	bool queue_frame(FrameType frame_type, uint8_t message_id, uint8_t flags,
			 const uint8_t *payload, uint16_t payload_len, Frame *saved = nullptr);
	bool request_event(MessageId message_id, ReasonCode reason, uint64_t now_us);
	bool enqueue_existing(const Frame &frame);
	bool build_and_enqueue(PendingTemplate &pending, bool retry);
	void start_handshake(uint64_t now_us);
	void restart_handshake(uint64_t now_us);
	uint32_t allocate_transaction();
	void send_ack(const Frame &request, AckResult result, uint8_t detail = 0);
	void handle_ack(const Ack &ack, SessionActions &actions, uint64_t now_us);
	void handle_reliable_event(const Frame &frame, const ReliableEvent &event,
				   SessionActions &actions, uint64_t now_us);
	bool acknowledge_duplicate(const Frame &frame, const ReliableEvent &event);
	void cache_event(const Frame &frame, const ReliableEvent &event,
			 AckResult result, uint8_t detail, uint64_t now_us);
	DuplicateEntry *find_duplicate(uint32_t source_session, uint32_t event_id,
				       uint8_t message_id, uint64_t now_us);
	DuplicateEntry *find_event_duplicate(uint32_t source_session, uint32_t event_id,
					     uint64_t now_us);
	void clear_control(bool disarm, ReasonCode reason, bool mark_reattach,
			   SessionActions &actions);
	uint16_t local_health_flags(uint64_t now_us) const;
	uint32_t local_capabilities() const;
	uint32_t local_handshake_flags() const;
	bool peer_is_fresh(uint64_t now_us) const;
	bool handshake_report_valid(const Frame &frame, const Handshake &handshake) const;
	void enter_link_down(SessionActions &actions, ReasonCode reason);
	void expire_pending(uint64_t now_us, SessionActions &actions);
	void queue_periodic_heartbeat(uint64_t now_us);
	bool active_data_streams_confirmed() const;
	void update_link_test(uint64_t now_us, SessionActions &actions);
	bool process_link_test_data(const Frame &frame, uint64_t now_us);
	void finish_link_test(uint64_t now_us, SessionActions &actions);
	void publish_link_test_stats(uint64_t now_us);
	LinkTestStats &mutable_link_test_stats() { return _link_test.stats; }

	Role _role{Role::Disabled};
	uint32_t _session_id{0};
	uint32_t _config_crc32c{0};
	uint32_t _peer_session{0};
	uint16_t _tx_sequence{0};
	uint16_t _last_peer_sensor_sequence{0};
	uint16_t _last_peer_control_sequence{0};
	LinkState _state{LinkState::Down};
	LocalStatus _local{};
	RemoteStatus _remote{};
	ActuatorCommand _peer_command{};
	SensorState _peer_sensor{};
	bool _pair_reattach_latched{false};
	bool _emergency_latched{false};
	bool _master_arm_pending{false};
	bool _slave_arm_pending{false};
	uint32_t _slave_arm_event_id{0};
	Frame _slave_arm_request{};
	uint64_t _control_enable_acked_us{0};
	uint64_t _pair_arm_commit_deadline_us{0};
	uint64_t _last_peer_frame_us{0};
	uint64_t _last_peer_heartbeat_us{0};
	uint64_t _last_peer_sensor_us{0};
	uint64_t _last_peer_control_us{0};
	uint64_t _last_local_sensor_sent_us{0};
	uint64_t _last_local_control_sent_us{0};
	uint64_t _last_control_sent_us{0};
	uint64_t _last_heartbeat_sent_us{0};
	uint64_t _handshake_retry_after_us{0};
	uint64_t _pair_control_ready_us{0};
	uint32_t _next_transaction_id{1};
	uint32_t _completed_handshake_transaction{0};
	uint32_t _tx_frames{0};
	uint32_t _rx_frames{0};
	uint32_t _session_errors{0};
	uint32_t _unknown_messages{0};
	uint32_t _reliable_retries{0};
	uint32_t _duplicate_events{0};
	uint8_t _peer_sensor_consecutive{0};
	uint8_t _peer_control_consecutive{0};
	uint8_t _local_sensor_consecutive{0};
	uint8_t _local_control_consecutive{0};
	bool _has_peer_sensor_sequence{false};
	bool _has_peer_control_sequence{false};

	Frame _tx_queue[kTxQueueSize] {};
	size_t _tx_head{0};
	size_t _tx_count{0};
	PendingTemplate _pending{};
	PendingTemplate _handshake{};
	DuplicateEntry _duplicates[kDuplicateCacheSize] {};
	LinkTestState _link_test{};
};

const char *link_state_name(LinkState state);

} // namespace airlink
