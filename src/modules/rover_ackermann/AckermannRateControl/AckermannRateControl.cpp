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

#include "AckermannRateControl.hpp"

using namespace time_literals;

AckermannRateControl::AckermannRateControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_status_pub.advertise();
	updateParams();
}

void AckermannRateControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	_pid_yaw_rate.setGains(_param_ro_yaw_rate_p.get(), _param_ro_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);
	_yaw_rate_with_accel_limit.setSlewRate(_param_ro_max_yaw_accel.get() * M_DEG_TO_RAD_F);
}

void AckermannRateControl::updateRateControl()
{
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = fabsf(vehicle_angular_velocity.xyz[2]) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F ?
				    vehicle_angular_velocity.xyz[2] : 0.f;
	}

	if (_vehicle_control_mode.flag_control_rates_enabled) {
		if (_vehicle_control_mode.flag_control_manual_enabled || _vehicle_control_mode.flag_control_offboard_enabled) {
			generateRateSetpoint();
		}

		if (_rover_rate_setpoint_sub.updated()) {
			generateSteeringSetpoint();
		}

	} else { // Reset controller and slew rate when rate control is not active
		_pid_yaw_rate.resetIntegral();
		_yaw_rate_with_accel_limit.setForcedValue(0.f);
	}

	// Publish rate controller status (logging only)
	rover_rate_status_s rover_rate_status;
	rover_rate_status.timestamp = _timestamp;
	rover_rate_status.measured_yaw_rate = _vehicle_yaw_rate;
	rover_rate_status.adjusted_yaw_rate_setpoint = _yaw_rate_with_accel_limit.getState();
	rover_rate_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	_rover_rate_status_pub.publish(rover_rate_status);

}

void AckermannRateControl::generateRateSetpoint()
{
	bool acro_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
				 && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled;

	if (acro_mode_enabled && _manual_control_setpoint_sub.updated()) { // Acro Mode
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			bool necessary_parameters_set = _max_yaw_rate > FLT_EPSILON &&  _param_ro_max_thr_speed.get() > FLT_EPSILON;
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = necessary_parameters_set ? math::interpolate<float>
								(manual_control_setpoint.roll,
										-1.f, 1.f, -_max_yaw_rate, _max_yaw_rate) : 0.f;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
			rover_throttle_setpoint_s rover_throttle_setpoint{};
			rover_throttle_setpoint.timestamp = _timestamp;
			rover_throttle_setpoint.throttle_body_x = necessary_parameters_set ? manual_control_setpoint.throttle : 0.f;
			rover_throttle_setpoint.throttle_body_y = 0.f;
			_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
		}

	} else if (
		_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Rate Control TODO: Does this even work in theory?
		if (_offboard_control_mode_sub.updated()) {
			_offboard_control_mode_sub.copy(&_offboard_control_mode);
		}

		bool offboard_rate_control = _offboard_control_mode.body_rate && !_offboard_control_mode.position
					     && !_offboard_control_mode.velocity && !_offboard_control_mode.attitude;

		if (offboard_rate_control && _trajectory_setpoint_sub.updated()) {
			trajectory_setpoint_s trajectory_setpoint{};
			_trajectory_setpoint_sub.copy(&trajectory_setpoint);
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = trajectory_setpoint.yawspeed;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
		}
	}
}

void AckermannRateControl::generateSteeringSetpoint()
{
	// Estimate forward velocity based on throttle setpoint (Necessary for yaw rate -> Steering angle mapping)
	if (_rover_throttle_setpoint_sub.updated()) {
		rover_throttle_setpoint_s rover_throttle_setpoint;
		_rover_throttle_setpoint_sub.copy(&rover_throttle_setpoint);
		_estimated_forward_speed = _param_ro_max_thr_speed.get() > FLT_EPSILON ? math::interpolate<float>
					   (rover_throttle_setpoint.throttle_body_x,
					    -1.f, 1.f, -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get()) : 0.f;
	}

	// Set up feasible yaw rate setpoint
	rover_rate_setpoint_s rover_rate_setpoint;
	_rover_rate_setpoint_sub.update(&rover_rate_setpoint);
	float max_possible_yaw_rate = _param_ra_wheel_base.get() > FLT_EPSILON ? fabsf(_estimated_forward_speed) * tanf(
					      _param_ra_max_str_ang.get()) / _param_ra_wheel_base.get() :
				      _max_yaw_rate; // Maximum possible yaw rate at current velocity
	float yaw_rate_limit = math::min(max_possible_yaw_rate, _max_yaw_rate);
	float constrained_yaw_rate = math::constrain(rover_rate_setpoint.yaw_rate_setpoint, -yaw_rate_limit, yaw_rate_limit);

	if (_param_ro_max_yaw_accel.get() > FLT_EPSILON) { // Apply slew rate if configured
		_yaw_rate_with_accel_limit.update(constrained_yaw_rate, _dt);

		if (fabsf(_yaw_rate_with_accel_limit.getState() - _vehicle_yaw_rate) > fabsf(constrained_yaw_rate -
				_vehicle_yaw_rate)) {
			_yaw_rate_with_accel_limit.setForcedValue(_vehicle_yaw_rate);
		}

	} else {
		_yaw_rate_with_accel_limit.setForcedValue(constrained_yaw_rate);
	}

	// Feed forward
	float steering_setpoint = fabsf(_estimated_forward_speed) > FLT_EPSILON ? atanf(_yaw_rate_with_accel_limit.getState() *
				  _param_ra_wheel_base.get() / _estimated_forward_speed) : 0.f;

	// Feedback (Only when driving forwards because backwards is NMP and can introduce instability)
	if (_estimated_forward_speed > FLT_EPSILON) {
		_pid_yaw_rate.setSetpoint(_yaw_rate_with_accel_limit.getState());
		steering_setpoint += _pid_yaw_rate.update(_vehicle_yaw_rate, _dt);
	}

	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = _timestamp;
	rover_steering_setpoint.normalized_steering_angle = math::interpolate<float>(steering_setpoint,
			-_param_ra_max_str_ang.get(), _param_ra_max_str_ang.get(), -1.f, 1.f); // Normalize steering setpoint
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
}
