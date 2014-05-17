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
#include "sr_pos_control.h"

namespace singlerotor
{

PositionController::PositionController()
{
}

PositionController::~PositionController()
{
}

int PositionController::run(int argc, char *argv[])
{
	_is_running = true;
	_should_stop = false;
	_control_loop_perf = perf_alloc(PC_ELAPSED, "sr_att_control_loop");
	subscribe_all();
	advertise_open_all();
	params_update();

	// This is the main loop of the task.
	while (!_should_stop) {
		_vehicle_attitude_setpoint.roll_body = 0.0f;
		_vehicle_attitude_setpoint.pitch_body = 0.0f;
		_vehicle_attitude_setpoint.yaw_body = 0.0f;
		_vehicle_attitude_setpoint.R_valid = false;

		orb_publish(ORB_ID(vehicle_attitude_setpoint), _pub_handles.vehicle_attitude_setpoint, &_vehicle_attitude_setpoint);

		sleep(1);
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
}

void PositionController::unsubscribe_all()
{
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
}

void PositionController::params_update()
{
}

} // namespace singlerotor
