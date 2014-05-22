/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Kim Lindberg Schwaner <kim.schwaner@gmail.com
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
 * @file sr_pos_control.h
 *
 * @author Kim Lindberg Schwaner <kim.schwaner@gmail.com>
 */

#ifndef SR_POS_CONTROL_H
#define SR_POS_CONTROL_H

#include <stdio.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>

namespace singlerotor
{

/**
 * Singlerotor position control class
 */
class PositionController
{
public:
	/**
	 * Constructor
	 */
	PositionController();

	/**
	 * Destructor
	 *
	 * Frees resources. User is responsible for first calling stop() before
	 * deleting an PositionController object.
	 */
	~PositionController();

	int run(int argc, char *argv[]);
	void stop();
	bool is_running() const;

private:
	void subscribe_all();
	void unsubscribe_all();
	void advertise_open_all();
	void advertise_close_all();
	void get_orb_updates();
	void params_update();
	void control_main();

private:
	bool _is_running;
	bool _should_stop;
	perf_counter_t _control_loop_perf;

	struct {
		param_t yaw_manual_sens;
	} _param_handles;

	// uORB subscription handles
	struct {
		int vehicle_control_mode;
		int vehicle_local_position;
		int vehicle_attitude;
		int manual_control_setpoint;
		int parameter_update;
	} _sub_handles;

	// uORB publications
	struct {
		orb_advert_t vehicle_attitude_setpoint;
	} _pub_handles;

	vehicle_control_mode_s _vehicle_control_mode; /**< uORB vehicle_control_mode topic data */
	vehicle_local_position_s _vehicle_local_position;
	vehicle_attitude_s _vehicle_attitude;
	vehicle_attitude_setpoint_s _vehicle_attitude_setpoint; /**< uORB vehicle_attitude_setpoint topic data */
	manual_control_setpoint_s _manual_control_setpoint; /**< uORB manual_control_setpoint topic data */
	parameter_update_s _parameter_update; /**< uORB parameter_update topic data */

	float _yaw_manual_sens;

};

} // namespace singlerotor

#endif // SR_POS_CONTROL_H
