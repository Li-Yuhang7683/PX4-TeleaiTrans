/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * AIRLINK endpoint role
 *
 * The role must be changed while disarmed. A role change creates a new session
 * and requires a new heartbeat discovery, handshake and control enable.
 *
 * @min 0
 * @max 2
 * @value 0 Disabled
 * @value 1 Master
 * @value 2 Slave
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_INT32(AIRLINK_ROLE, 0);

/**
 * Select the paired-controller mode
 *
 * Enables use of the paired controller command topic and requires an explicit
 * AIRLINK handshake, CONTROL_ENABLE and PAIR_ARM sequence before paired arming.
 * It does not itself arm either aircraft.
 *
 * @boolean
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_INT32(AIRLINK_PAIR_EN, 0);

/**
 * SINGLE_LANDING impact acceleration threshold
 *
 * Acceleration magnitude in the PX4 body-frame estimate that triggers local
 * disarm. This is an initial bench/HIL value, not a flight-qualified limit.
 *
 * @unit m/s^2
 * @min 10
 * @max 200
 * @decimal 1
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_FLOAT(AIRLINK_IMPACT, 35.0f);

/**
 * SINGLE_LANDING impact confirmation time
 *
 * Continuous time above AIRLINK_IMPACT before local disarm.
 *
 * @unit ms
 * @min 5
 * @max 1000
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_INT32(AIRLINK_IMP_MS, 20);

/**
 * SINGLE_LANDING flip angle threshold
 *
 * Angle of the body down axis away from world down before local disarm.
 *
 * @unit deg
 * @min 20
 * @max 180
 * @decimal 1
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_FLOAT(AIRLINK_FLIP_DEG, 70.0f);

/**
 * SINGLE_LANDING flip confirmation time
 *
 * Continuous time above AIRLINK_FLIP_DEG before local disarm.
 *
 * @unit ms
 * @min 10
 * @max 2000
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_INT32(AIRLINK_FLIP_MS, 150);

/**
 * SINGLE_LANDING maximum duration
 *
 * Continuous SINGLE_LANDING time before forced local disarm.
 *
 * @unit ms
 * @min 1000
 * @max 60000
 * @reboot_required false
 * @group AIRLINK
 */
PARAM_DEFINE_INT32(AIRLINK_LAND_MS, 15000);
