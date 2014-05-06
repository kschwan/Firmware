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
 * @file sr_att_control.h
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 * @author Thomas Larsen <thola11@student.sdu.dk>
 */

#ifndef SR_ATT_CONTROL_H
#define SR_ATT_CONTROL_H

#include <cxx/cstdio>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>

namespace singlerotor
{

class AttitudeController
{
public:
	AttitudeController();
	~AttitudeController();

	int run(int argc, char *argv[]);
	void stop();
	bool is_running() const;

	void print_info_screen(FILE *out);

private:
	void subscribe_all();
	void unsubscribe_all();

private:
	bool _is_running;
	bool _should_stop;

	struct params_t {
		float roll_p;
		float roll_rate_p;
		float roll_rate_i;
		float roll_rate_d;
		float pitch_p;
		float pitch_rate_p;
		float pitch_rate_i;
		float pitch_rate_d;
		float yaw_p;
		float yaw_rate_p;
		float yaw_rate_i;
		float yaw_rate_d;
	} _params;

	// uORB subscription handles
	struct sub_handles_t {
		int actuator_armed;
		int vehicle_control_mode;
		int vehicle_attitude;
		int vehicle_attitude_setpoint;
		int vehicle_rates_setpoint;
		int manual_control_setpoint;
	} _sub_handles;

	// uORB topic data structures
	struct actuator_armed_s                 _actuator_armed;
	struct vehicle_control_mode_s           _vehicle_control_mode;
	struct vehicle_attitude_s               _vehicle_attitude;
	struct vehicle_attitude_setpoint_s      _vehicle_attitude_setpoint;
	struct vehicle_rates_setpoint_s         _vehicle_rates_setpoint;
	struct manual_control_setpoint_s        _manual_control_setpoint;
	struct actuator_controls_s              _actuator_controls;

};

} // namespace singlerotor

#endif // SR_ATT_CONTROL_H
