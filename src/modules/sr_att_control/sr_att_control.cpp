/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 *           Thomas Larsen <thola11@student.sdu.dk>
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
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 * @author Thomas Larsen <thola11@student.sdu.dk>
 */

#include <unistd.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
//#include <systemlib/pid/pid.h>// ?
#include <systemlib/err.h>
#include "sr_att_control.h"

namespace singlerotor
{

AttitudeController::AttitudeController()
{
}

AttitudeController::~AttitudeController()
{
}

int AttitudeController::run(int argc, char *argv[])
{
	_is_running = true;
	_should_stop = false;
	subscribe_all();

	while (!_should_stop) {
		sleep(1);
	}

	unsubscribe_all();
	_is_running = false;
	return 0;
}

void AttitudeController::stop()
{
	_should_stop = true;
}

bool AttitudeController::is_running() const
{
	return _is_running;
}

void AttitudeController::subscribe_all()
{
	_sub_handles.actuator_armed = orb_subscribe(ORB_ID(actuator_armed));
	_sub_handles.vehicle_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_sub_handles.vehicle_attitude = orb_subscribe(ORB_ID(vehicle_attitude));
	_sub_handles.vehicle_attitude_setpoint = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_sub_handles.vehicle_rates_setpoint = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_sub_handles.manual_control_setpoint = orb_subscribe(ORB_ID(manual_control_setpoint));
}

void AttitudeController::unsubscribe_all()
{
	orb_unsubscribe(_sub_handles.actuator_armed);
	orb_unsubscribe(_sub_handles.vehicle_control_mode);
	orb_unsubscribe(_sub_handles.vehicle_attitude);
	orb_unsubscribe(_sub_handles.vehicle_attitude_setpoint);
	orb_unsubscribe(_sub_handles.vehicle_rates_setpoint);
	orb_unsubscribe(_sub_handles.manual_control_setpoint);
}


} // namespace singlerotor
