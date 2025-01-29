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

#include "AckermannAttControl.hpp"

using namespace time_literals;

AckermannAttControl::AckermannAttControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_attitude_status_pub.advertise();
	updateParams();
}

void AckermannAttControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	_pid_yaw.setGains(_param_ro_yaw_p.get(), _param_ro_yaw_i.get(), 0.f);
	_pid_yaw.setIntegralLimit(_max_yaw_rate);
	_pid_yaw.setOutputLimit(_max_yaw_rate);
	_yaw_with_yaw_rate_limit.setSlewRate(_max_yaw_rate);
}

void AckermannAttControl::updateAttControl()
{
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	// Estimate forward speed based on throttle
	if (_actuator_motors_sub.updated()) {
		actuator_motors_s actuator_motors;
		_actuator_motors_sub.copy(&actuator_motors);
		_estimated_forward_speed = _param_ro_max_thr_speed.get() > FLT_EPSILON ? math::interpolate<float>
					   (actuator_motors.control[0], -1.f, 1.f, -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get()) : 0.f;
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		if (_vehicle_control_mode.flag_control_manual_enabled || _vehicle_control_mode.flag_control_offboard_enabled) {
			generateAttitudeSetpoint();
		}

		if (_rover_attitude_setpoint_sub.updated()) {
			generateRateSetpoint();
		}

	} else { // Reset controller and slew rate when attitude control is not active
		_pid_yaw.resetIntegral();
		_yaw_with_yaw_rate_limit.setForcedValue(0.f);
	}

	// Publish attitude controller status (logging only)
	rover_attitude_status_s rover_attitude_status;
	rover_attitude_status.timestamp = _timestamp;
	rover_attitude_status.measured_yaw = _vehicle_yaw;
	rover_attitude_status.adjusted_yaw_setpoint = _yaw_with_yaw_rate_limit.getState();
	rover_attitude_status.pid_yaw_integral = _pid_yaw.getIntegral();
	_rover_attitude_status_pub.publish(rover_attitude_status);

}

void AckermannAttControl::generateAttitudeSetpoint()
{
	bool stab_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
				 && !_vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_control_attitude_enabled;

	if (stab_mode_enabled && _manual_control_setpoint_sub.updated()) { // Stab Mode
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			bool necessary_parameters_set = _max_yaw_rate > FLT_EPSILON
							&&  _param_ro_max_thr_speed.get() > FLT_EPSILON; // TODO: Check this
			rover_throttle_setpoint_s rover_throttle_setpoint{};
			rover_throttle_setpoint.timestamp = _timestamp;
			rover_throttle_setpoint.throttle_body_x = necessary_parameters_set ? manual_control_setpoint.throttle : 0.f;
			rover_throttle_setpoint.throttle_body_y = 0.f;
			_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

			float yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
						  _param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);

			if (fabsf(yaw_rate_setpoint) > FLT_EPSILON
			    || fabsf(rover_throttle_setpoint.throttle_body_x) < FLT_EPSILON) { // Closed loop yaw rate control
				_stab_yaw_ctl = false;
				_yaw_with_yaw_rate_limit.setForcedValue(0.f);
				rover_rate_setpoint_s rover_rate_setpoint{};
				rover_rate_setpoint.timestamp = _timestamp;
				rover_rate_setpoint.yaw_rate_setpoint = necessary_parameters_set ? matrix::sign(_estimated_forward_speed) *
									yaw_rate_setpoint : 0.f;
				_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

			} else { // Closed loop yaw control if the yaw rate input is zero (keep current yaw)
				if (!_stab_yaw_ctl) {
					_stab_yaw_setpoint = _vehicle_yaw;
					_stab_yaw_ctl = true;
				}

				rover_attitude_setpoint_s rover_attitude_setpoint{};
				rover_attitude_setpoint.timestamp = _timestamp;
				rover_attitude_setpoint.yaw_setpoint = _stab_yaw_setpoint;
				_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
			}


		}

	} else if (
		_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Rate Control TODO: Does this even work in theory?
		if (_offboard_control_mode_sub.updated()) {
			_offboard_control_mode_sub.copy(&_offboard_control_mode);
		}

		bool offboard_attitude_control = _offboard_control_mode.position && !_offboard_control_mode.velocity
						 && _offboard_control_mode.attitude;

		if (offboard_attitude_control && _trajectory_setpoint_sub.updated()) {
			trajectory_setpoint_s trajectory_setpoint{};
			_trajectory_setpoint_sub.copy(&trajectory_setpoint);
			rover_attitude_setpoint_s rover_attitude_setpoint{};
			rover_attitude_setpoint.timestamp = _timestamp;
			rover_attitude_setpoint.yaw_setpoint = trajectory_setpoint.yaw;
			_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
		}
	}
}

void AckermannAttControl::generateRateSetpoint()
{
	rover_attitude_setpoint_s rover_attitude_setpoint;
	_rover_attitude_setpoint_sub.update(&rover_attitude_setpoint);

	// Apply slew rate if configured
	if (_max_yaw_rate > FLT_EPSILON) {
		// Calculate yaw rate limit for slew rate
		float max_possible_yaw_rate = _param_ra_wheel_base.get() > FLT_EPSILON ? fabsf(_estimated_forward_speed) * tanf(
						      _param_ra_max_str_ang.get()) / _param_ra_wheel_base.get() :
					      _max_yaw_rate; // Maximum possible yaw rate at current velocity
		float yaw_rate_limit = math::min(max_possible_yaw_rate, _max_yaw_rate);
		_yaw_with_yaw_rate_limit.setSlewRate(yaw_rate_limit);
		_yaw_with_yaw_rate_limit.update(matrix::wrap_pi(rover_attitude_setpoint.yaw_setpoint), _dt);

		if (fabsf(matrix::wrap_pi(_yaw_with_yaw_rate_limit.getState() - _vehicle_yaw)) > fabsf(matrix::wrap_pi(
					rover_attitude_setpoint.yaw_setpoint - _vehicle_yaw))) {
			_yaw_with_yaw_rate_limit.setForcedValue(_vehicle_yaw);
		}

	} else {
		_yaw_with_yaw_rate_limit.setForcedValue(rover_attitude_setpoint.yaw_setpoint);
	}

	_pid_yaw.setSetpoint(
		matrix::wrap_pi(_yaw_with_yaw_rate_limit.getState() -
				_vehicle_yaw));  // Error as setpoint to take care of wrapping
	float yaw_rate_setpoint = math::constrain(_pid_yaw.update(0.f, _dt), -_max_yaw_rate, _max_yaw_rate);

	rover_rate_setpoint_s rover_rate_setpoint{};
	rover_rate_setpoint.timestamp = _timestamp;
	rover_rate_setpoint.yaw_rate_setpoint = yaw_rate_setpoint;
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}
