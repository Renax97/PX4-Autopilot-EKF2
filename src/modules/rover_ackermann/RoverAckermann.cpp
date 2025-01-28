/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "RoverAckermann.hpp"

using namespace time_literals;

RoverAckermann::RoverAckermann() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) // TODO: Which work queue ?
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	updateParams();
}

bool RoverAckermann::init() // TODO: Is this necessary ?
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverAckermann::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ra_max_str_rate.get() > FLT_EPSILON && _param_ra_max_str_ang.get() > FLT_EPSILON) {
		_steering_with_rate_limit.setSlewRate((M_DEG_TO_RAD_F * _param_ra_max_str_rate.get()) / _param_ra_max_str_ang.get());
	}
}

void RoverAckermann::Run()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	_ackermann_att_control.updateAttControl();
	_ackermann_rate_control.updateRateControl();

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	bool manual_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
				   && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled
				   && !_vehicle_control_mode.flag_control_rates_enabled;

	if (manual_mode_enabled) { // Manual mode
		generateSteeringSetpoint();
	}

	if (_rover_throttle_setpoint_sub.updated() || _rover_steering_setpoint_sub.updated()) {
		generateActuatorSetpoint();
	}

}

void RoverAckermann::generateSteeringSetpoint()
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		rover_steering_setpoint.timestamp = hrt_absolute_time();
		rover_steering_setpoint.normalized_steering_angle = manual_control_setpoint.roll;
		_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = hrt_absolute_time();
		rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
		rover_throttle_setpoint.throttle_body_y = 0.f;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	}
}

void RoverAckermann::generateActuatorSetpoint()
{
	if (_rover_throttle_setpoint_sub.updated()) { // Publish to motor
		rover_throttle_setpoint_s rover_throttle_setpoint;
		_rover_throttle_setpoint_sub.update(&rover_throttle_setpoint);
		actuator_motors_s actuator_motors{};
		actuator_motors.reversible_flags = _param_r_rev.get();
		actuator_motors.control[0] = rover_throttle_setpoint.throttle_body_x;
		actuator_motors.timestamp = hrt_absolute_time();
		_actuator_motors_pub.publish(actuator_motors);
	}


	if (_rover_steering_setpoint_sub.updated()) { // Publish to servo
		rover_steering_setpoint_s rover_steering_setpoint;
		_rover_steering_setpoint_sub.update(&rover_steering_setpoint);

		if (_param_ra_max_str_rate.get() > FLT_EPSILON
		    && _param_ra_max_str_ang.get() > FLT_EPSILON) { // Apply slew rate if configured
			_steering_with_rate_limit.update(rover_steering_setpoint.normalized_steering_angle, _dt);

		} else {
			_steering_with_rate_limit.setForcedValue(rover_steering_setpoint.normalized_steering_angle);
		}

		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = _steering_with_rate_limit.getState();
		actuator_servos.timestamp = hrt_absolute_time();
		_actuator_servos_pub.publish(actuator_servos);
	}
}

int RoverAckermann::task_spawn(int argc, char *argv[])
{
	RoverAckermann *instance = new RoverAckermann();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverAckermann::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverAckermann::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover ackermann module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_ackermann", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_ackermann_main(int argc, char *argv[])
{
	return RoverAckermann::main(argc, argv);
}
