/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Kim Lindberg Schwaner <kim.schwaner@gmail.com>
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

/**
 * @file sr_att_control.cpp
 *
 * @author Kim Lindberg Schwaner <kim.schwaner@gmail.com>
 *
 * @todo Use uORB cpp wrappers
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_debug.h>
#include "sr_att_control.h"

namespace singlerotor
{

AttitudeController::AttitudeController()
{
	_param_handles.roll_p = param_find("SR_ROLL_P");
	_param_handles.roll_rate_p = param_find("SR_ROLLRATE_P");
	_param_handles.roll_rate_i = param_find("SR_ROLLRATE_I");
	_param_handles.roll_rate_d = param_find("SR_ROLLRATE_D");
	_param_handles.pitch_p = param_find("SR_PITCH_P");
	_param_handles.pitch_rate_p = param_find("SR_PITCHRATE_P");
	_param_handles.pitch_rate_i = param_find("SR_PITCHRATE_I");
	_param_handles.pitch_rate_d = param_find("SR_PITCHRATE_D");
	_param_handles.yaw_p = param_find("SR_YAW_P");
	_param_handles.yaw_rate_p = param_find("SR_YAWRATE_P");
	_param_handles.yaw_rate_i = param_find("SR_YAWRATE_I");
	_param_handles.yaw_rate_d = param_find("SR_YAWRATE_D");
}

AttitudeController::~AttitudeController()
{
}

int AttitudeController::run(int argc, char *argv[])
{
	_is_running = true;
	_should_stop = false;
	_control_loop_perf = perf_alloc(PC_ELAPSED, "sr_att_control_loop");
	subscribe_all();
	advertise_open_all();
	params_update();

	// Wake up on attitude or control parameter updates. We don't care about
	// returned events.
	struct pollfd fds[2];
	fds[0].fd = _sub_handles.parameter_update;
	fds[0].events = POLLIN;
	fds[1].fd = _sub_handles.vehicle_attitude;
	fds[1].events = POLLIN;

	// This is the main loop of the task.
	while (!_should_stop) {
		// Wait for event on file descriptors.
		int ret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (ret < 0) {
			// Error.
			warn("AttitudeController: poll error %d, %d", ret, errno);
			continue;
		} else if (ret == 0) {
			// Timeout.
			continue;
		}

		// If parameters were updated we clear the parameter_update topic
		// updated flag and then update the local parameter cache.
		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_update), _sub_handles.parameter_update, &_parameter_update);
			params_update();
		}

		// If vehicle attitude has changed, we run the control loop with
		// the updates values.
		if (fds[1].revents & POLLIN) {
			control_main();
		}
	}

	// Clean-up and exit
	advertise_close_all();
	unsubscribe_all();
	perf_free(_control_loop_perf);
	_is_running = false;
	return EXIT_SUCCESS;
}

void AttitudeController::stop()
{
	_should_stop = true;
}

bool AttitudeController::is_running() const
{
	return _is_running;
}

void AttitudeController::print_info_screen(FILE *out) const
{
	const char *CL = "\033[K"; // clear line

	fprintf(out, "\033[2J"); // clear screen
	fprintf(out, "\033[H"); // move cursor home

	fprintf(out, "%slast control update: %lld\n", CL, _control_last_run);

	fprintf(out, "%sattitude time: %lld\n", CL, _vehicle_attitude.timestamp);
	fprintf(out, "%sroll: %f\n", CL, _vehicle_attitude.roll);
	fprintf(out, "%spitch: %f\n", CL, _vehicle_attitude.pitch);
	fprintf(out, "%syaw: %f\n", CL, _vehicle_attitude.yaw);
	fprintf(out, "%srollspeed: %f\n", CL, _vehicle_attitude.rollspeed);
	fprintf(out, "%spitchspeed: %f\n", CL, _vehicle_attitude.pitchspeed);
	fprintf(out, "%syawspeed: %f\n", CL, _vehicle_attitude.yawspeed);
	fprintf(out, "%srollacc: %f\n", CL, _vehicle_attitude.rollacc);
	fprintf(out, "%spitchacc: %f\n", CL, _vehicle_attitude.pitchacc);
	fprintf(out, "%syawacc: %f\n", CL, _vehicle_attitude.yawacc);

	fprintf(out, "%smanual x: %f\n", CL, _manual_control_setpoint.x);
	fprintf(out, "%smanual y: %f\n", CL, _manual_control_setpoint.y);
	fprintf(out, "%smanual z: %f\n", CL, _manual_control_setpoint.z);
	fprintf(out, "%smanual r: %f\n", CL, _manual_control_setpoint.r);
	fprintf(out, "%smanual aux1: %f\n", CL, _manual_control_setpoint.aux1);
	fprintf(out, "%smanual aux2: %f\n", CL, _manual_control_setpoint.aux2);
	fprintf(out, "%smanual aux3: %f\n", CL, _manual_control_setpoint.aux3);
	fprintf(out, "%smanual aux4: %f\n", CL, _manual_control_setpoint.aux4);
	fprintf(out, "%smanual aux5: %f\n", CL, _manual_control_setpoint.aux5);

	fprintf(out, "%sflag_control_manual_enabled: %s\n", CL, _vehicle_control_mode.flag_control_manual_enabled ? "true" : "false");
	fprintf(out, "%sflag_control_auto_enabled: %s\n", CL, _vehicle_control_mode.flag_control_auto_enabled ? "true" : "false");
	fprintf(out, "%sflag_control_rates_enabled: %s\n", CL, _vehicle_control_mode.flag_control_rates_enabled ? "true" : "false");
	fprintf(out, "%sflag_control_attitude_enabled: %s\n", CL, _vehicle_control_mode.flag_control_attitude_enabled ? "true" : "false");

	for (int i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		fprintf(out, "%sactuator_controls_0.control[%d]: %f\n", CL, i, _actuator_controls_0.control[i]);
	}
}

void AttitudeController::subscribe_all()
{
	_sub_handles.actuator_armed = orb_subscribe(ORB_ID(actuator_armed));
	_sub_handles.vehicle_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_handles.vehicle_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
	_sub_handles.vehicle_attitude_setpoint = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_sub_handles.vehicle_rates_setpoint = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_sub_handles.manual_control_setpoint = orb_subscribe(ORB_ID(manual_control_setpoint));
	_sub_handles.parameter_update = orb_subscribe(ORB_ID(parameter_update));
}

void AttitudeController::unsubscribe_all()
{
	orb_unsubscribe(_sub_handles.actuator_armed);
	orb_unsubscribe(_sub_handles.vehicle_control_mode);
	orb_unsubscribe(_sub_handles.vehicle_attitude);
	orb_unsubscribe(_sub_handles.vehicle_attitude_setpoint);
	orb_unsubscribe(_sub_handles.vehicle_rates_setpoint);
	orb_unsubscribe(_sub_handles.manual_control_setpoint);
	orb_unsubscribe(_sub_handles.parameter_update);
}

void AttitudeController::advertise_open_all()
{
	_pub_handles.actuator_controls_0 = orb_advertise(ORB_ID(actuator_controls_0), &_actuator_controls_0);
	_pub_handles.vehicle_rates_setpoint = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_vehicle_rates_setpoint);
	_pub_handles.vehicle_control_debug = orb_advertise(ORB_ID(vehicle_control_debug), &_vehicle_control_debug);
}

void AttitudeController::advertise_close_all()
{
	close(_pub_handles.actuator_controls_0);
	close(_pub_handles.vehicle_rates_setpoint);
	close(_pub_handles.vehicle_control_debug);
}

void AttitudeController::get_orb_updates()
{
	bool updated;

	orb_check(_sub_handles.actuator_armed, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _sub_handles.vehicle_attitude, &_vehicle_attitude);
	}

	orb_check(_sub_handles.vehicle_control_mode, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _sub_handles.vehicle_control_mode, &_vehicle_control_mode);
	}

	orb_check(_sub_handles.vehicle_attitude, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _sub_handles.vehicle_attitude, &_vehicle_attitude);
	}

	orb_check(_sub_handles.vehicle_attitude_setpoint, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _sub_handles.vehicle_attitude_setpoint, &_vehicle_attitude_setpoint);
	}

	orb_check(_sub_handles.vehicle_rates_setpoint, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _sub_handles.vehicle_rates_setpoint, &_vehicle_rates_setpoint);
	}

	orb_check(_sub_handles.manual_control_setpoint, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _sub_handles.manual_control_setpoint, &_manual_control_setpoint);
	}
}

void AttitudeController::params_update()
{
	param_get(_param_handles.roll_p, &_control_params.att_p(0));
	param_get(_param_handles.roll_rate_p, &_control_params.rate_p(0));
	param_get(_param_handles.roll_rate_i, &_control_params.rate_i(0));
	param_get(_param_handles.roll_rate_d, &_control_params.rate_d(0));
	param_get(_param_handles.pitch_p, &_control_params.att_p(1));
	param_get(_param_handles.pitch_rate_p, &_control_params.rate_p(1));
	param_get(_param_handles.pitch_rate_i, &_control_params.rate_i(1));
	param_get(_param_handles.pitch_rate_d, &_control_params.rate_d(1));
	param_get(_param_handles.yaw_p, &_control_params.att_p(2));
	param_get(_param_handles.yaw_rate_p, &_control_params.rate_p(2));
	param_get(_param_handles.yaw_rate_i, &_control_params.rate_i(2));
	param_get(_param_handles.yaw_rate_d, &_control_params.rate_d(2));
}

void AttitudeController::control_main()
{
	perf_begin(_control_loop_perf);

	// Update from all subscribed topics which have new data.
	get_orb_updates();

	// Time since last run hrt_absolute_time() returns time in microseconds
	float dt = (hrt_absolute_time() - _control_last_run) / 1000000.0f;
	_control_last_run = hrt_absolute_time();

	// Clamp dt's
	// TODO: review this
	if (dt < 0.002f) {
		dt = 0.002f;
	} else if (dt > 0.02f) {
		dt = 0.02f;
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		// Attitude stabilization is active: Run attitude control loop.
		control_attitude();
		_vehicle_rates_setpoint.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(vehicle_rates_setpoint), _pub_handles.vehicle_rates_setpoint, &_vehicle_rates_setpoint);
	} else {
		// Attitude stabilization deactivated. We try to get the
		// vehicle_rates_setpoint from elsewhere.

		// TODO
	}

	if (_vehicle_control_mode.flag_control_rates_enabled) {
		// Angular rate stabilization is active: Run rate control loop.
		control_rates(dt); // this sets _actuator_controls_0 values
	}

	// Manual control
	// TODO: Stabilized mode
	if (_vehicle_control_mode.flag_control_manual_enabled) {
		// Manual control input pass-through
		_actuator_controls_0.control[0] = _manual_control_setpoint.y; // pitch
		_actuator_controls_0.control[1] = -_manual_control_setpoint.x; // roll
		_actuator_controls_0.control[2] = _manual_control_setpoint.z; // collective
		_actuator_controls_0.control[3] = _manual_control_setpoint.r; // yaw
		_actuator_controls_0.control[4] = 0.0;
		_actuator_controls_0.control[5] = 0.0;
		_actuator_controls_0.control[6] = 0.0;
		_actuator_controls_0.control[7] = _manual_control_setpoint.aux1; // throttle
	}

	// Timestamp and publish
	_actuator_controls_0.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(actuator_controls_0), _pub_handles.actuator_controls_0, &_actuator_controls_0);
	_vehicle_control_debug.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_control_debug), _pub_handles.vehicle_control_debug, &_vehicle_control_debug);

	perf_end(_control_loop_perf);
}

void AttitudeController::control_attitude()
{
	float e_roll = _vehicle_attitude_setpoint.roll_body - _vehicle_attitude.roll;
	float e_pitch = _vehicle_attitude_setpoint.pitch_body - _vehicle_attitude.pitch;
	float e_yaw = _vehicle_attitude_setpoint.yaw_body - _vehicle_attitude.yaw;

	float p_roll = e_roll * _control_params.att_p(0);
	float p_pitch = e_pitch * _control_params.att_p(1);
	float p_yaw = e_yaw * _control_params.att_p(2);

	_vehicle_rates_setpoint.roll = p_roll;
	_vehicle_rates_setpoint.pitch = p_pitch;
	_vehicle_rates_setpoint.yaw = p_yaw;

	// Debug output
	_vehicle_control_debug.roll_p = p_roll;
	_vehicle_control_debug.pitch_p = p_pitch;
	_vehicle_control_debug.yaw_p = p_yaw;
	_vehicle_control_debug.roll_i = 0.0;
	_vehicle_control_debug.pitch_i = 0.0;
	_vehicle_control_debug.yaw_i = 0.0;
	_vehicle_control_debug.roll_d = 0.0;
	_vehicle_control_debug.pitch_d = 0.0;
	_vehicle_control_debug.yaw_d = 0.0;
}

void AttitudeController::control_rates(float dt)
{
	// If disarmed, reset integral
	if (!_actuator_armed.armed) {
		_i_rollrate = 0.0;
		_i_pitchrate = 0.0;
		_i_yawrate = 0.0;
	}

	float e_rollrate = _vehicle_rates_setpoint.roll - _vehicle_attitude.rollspeed;
	float e_pitchrate = _vehicle_rates_setpoint.pitch - _vehicle_attitude.pitchspeed;
	float e_yawrate = _vehicle_rates_setpoint.yaw - _vehicle_attitude.yawspeed;

	float p_rollrate = e_rollrate * _control_params.rate_p(0);
	float p_pitchrate = e_pitchrate * _control_params.rate_p(1);
	float p_yawrate = e_yawrate * _control_params.rate_p(2);

	float i_rollrate = _i_rollrate + e_rollrate * dt * _control_params.rate_i(0);
	float i_pitchrate = _i_pitchrate + e_pitchrate * dt * _control_params.rate_i(1);
	float i_yawrate = _i_yawrate + e_yawrate * dt * _control_params.rate_i(2);

	// TODO Windup guard


	float d_rollrate = (e_rollrate - _e_rollrate_prev) / dt * _control_params.rate_d(0);
	float d_pitchrate = (e_pitchrate - _e_pitchrate_prev) / dt * _control_params.rate_d(1);
	float d_yawrate = (e_yawrate - _e_yawrate_prev) / dt * _control_params.rate_d(2);

	_e_rollrate_prev = e_rollrate;
	_e_pitchrate_prev = e_pitchrate;
	_e_yawrate_prev = e_yawrate;

	_actuator_controls_0.control[0] = p_pitchrate + i_pitchrate + d_pitchrate;
	_actuator_controls_0.control[1] = p_rollrate + i_rollrate + d_rollrate;
	_actuator_controls_0.control[2] = _vehicle_attitude_setpoint.thrust; // Pass through thrust?
	_actuator_controls_0.control[3] = p_yawrate + i_yawrate + d_yawrate;

	// Debug output
	_vehicle_control_debug.roll_rate_p = p_rollrate;
	_vehicle_control_debug.pitch_rate_p = p_pitchrate;
	_vehicle_control_debug.yaw_rate_p = p_yawrate;
	_vehicle_control_debug.roll_rate_i = i_rollrate;
	_vehicle_control_debug.pitch_rate_i = i_pitchrate;
	_vehicle_control_debug.yaw_rate_i = i_yawrate;
	_vehicle_control_debug.roll_rate_d = d_rollrate;
	_vehicle_control_debug.pitch_rate_d = d_pitchrate;
	_vehicle_control_debug.yaw_rate_d = d_yawrate;
}

} // namespace singlerotor
