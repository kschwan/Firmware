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
 * @file sr_pos_control.cpp
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
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include "sr_pos_control.h"

namespace singlerotor
{

PositionController::PositionController()
{
	_param_handles.yaw_manual_sens = param_find("SRP_MAN_YAW_SENS");

	// Initialize values to 0
	// TODO: review if this is necessary
	memset(&_vehicle_control_mode, 0, sizeof(_vehicle_control_mode));
	memset(&_vehicle_attitude, 0, sizeof(_vehicle_attitude));
	memset(&_vehicle_attitude_setpoint, 0, sizeof(_vehicle_attitude_setpoint));
	memset(&_manual_control_setpoint, 0, sizeof(_manual_control_setpoint));
	memset(&_parameter_update, 0, sizeof(_parameter_update));
	memset(&_vehicle_local_position, 0, sizeof(_vehicle_local_position));

	_yaw_manual_sens = 0.0f;
}

PositionController::~PositionController()
{
}

int PositionController::run(int argc, char *argv[])
{
	_is_running = true;
	_should_stop = false;
	_control_loop_perf = perf_alloc(PC_ELAPSED, "sr_pos_control_loop");
	subscribe_all();
	advertise_open_all();
	params_update();

	struct pollfd fds[2];
	fds[0].fd = _sub_handles.parameter_update;
	fds[0].events = POLLIN;
	fds[1].fd = _sub_handles.vehicle_local_position;
	fds[1].events = POLLIN;

	// This is the main loop of the task.
	while (!_should_stop) {
		// Wait for event on file descriptors.
		int ret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (ret < 0) {
			// Error.
			warn("PositionController: poll error %d, %d", ret, errno);
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

		// If vehicle position has updated, we run the control loop with
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

void PositionController::stop()
{
	_should_stop = true;
}

bool PositionController::is_running() const
{
	return _is_running;
}

void PositionController::subscribe_all()
{
	_sub_handles.vehicle_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_handles.vehicle_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
	_sub_handles.vehicle_local_position = orb_subscribe(ORB_ID(vehicle_local_position));
	_sub_handles.manual_control_setpoint = orb_subscribe(ORB_ID(manual_control_setpoint));
	_sub_handles.parameter_update = orb_subscribe(ORB_ID(parameter_update));
}

void PositionController::unsubscribe_all()
{
	orb_unsubscribe(_sub_handles.vehicle_control_mode);
	orb_unsubscribe(_sub_handles.vehicle_attitude);
	orb_unsubscribe(_sub_handles.vehicle_local_position);
	orb_unsubscribe(_sub_handles.manual_control_setpoint);
	orb_unsubscribe(_sub_handles.parameter_update);
}

void PositionController::advertise_open_all()
{
	_pub_handles.vehicle_attitude_setpoint = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_vehicle_attitude_setpoint);
}

void PositionController::advertise_close_all()
{
	close(_pub_handles.vehicle_attitude_setpoint);
}

void PositionController::get_orb_updates()
{
	bool updated;

	orb_check(_sub_handles.vehicle_control_mode, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _sub_handles.vehicle_control_mode, &_vehicle_control_mode);
	}

	orb_check(_sub_handles.vehicle_attitude, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _sub_handles.vehicle_attitude, &_vehicle_attitude);
	}

	orb_check(_sub_handles.manual_control_setpoint, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _sub_handles.manual_control_setpoint, &_manual_control_setpoint);
	}

	orb_check(_sub_handles.vehicle_local_position, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _sub_handles.vehicle_local_position, &_vehicle_local_position);
	}
}

void PositionController::params_update()
{
	param_get(_param_handles.yaw_manual_sens, &_yaw_manual_sens);

	// TODO get all!
}

void PositionController::control_main()
{
	perf_begin(_control_loop_perf);
	get_orb_updates();

	// Manual (stabilized) control
	if (_vehicle_control_mode.flag_control_manual_enabled) {

		if (isfinite(_manual_control_setpoint.x)
		    && isfinite(_manual_control_setpoint.y)
		    && isfinite(_manual_control_setpoint.z)
		    && isfinite(_manual_control_setpoint.r)) {
			_vehicle_attitude_setpoint.roll_body = _manual_control_setpoint.y;
			_vehicle_attitude_setpoint.pitch_body = -_manual_control_setpoint.x;
			_vehicle_attitude_setpoint.yaw_body = _vehicle_attitude_setpoint.yaw_body + _manual_control_setpoint.r * _yaw_manual_sens;

			// Keep yaw angle within bounds [-pi ; pi]
			if (_vehicle_attitude_setpoint.yaw_body > M_PI) {
				_vehicle_attitude_setpoint.yaw_body -= 2 * M_PI;
			} else if (_vehicle_attitude_setpoint.yaw_body < -M_PI) {
				_vehicle_attitude_setpoint.yaw_body += 2 * M_PI;
			}
		}

		// Publish
		_vehicle_attitude_setpoint.R_valid = false;
		_vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _pub_handles.vehicle_attitude_setpoint, &_vehicle_attitude_setpoint);
	}

	perf_end(_control_loop_perf);
}

} // namespace singlerotor
