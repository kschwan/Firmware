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
 * @file sr_att_control.h
 *
 * @author Kim Lindberg Schwaner <kim.schwaner@gmail.com>
 */

#ifndef SR_ATT_CONTROL_H
#define SR_ATT_CONTROL_H

#include <stdio.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_debug.h>

namespace singlerotor
{

/**
 * Singlerotor attitude control class
 */
class AttitudeController
{
public:
	/**
	 * Constructor
	 */
	AttitudeController();

	/**
	 * Destructor
	 *
	 * Frees resources. User is responsible for first calling stop() before
	 * deleting an AttitudeController object.
	 */
	~AttitudeController();

	int run(int argc, char *argv[]);
	void stop();
	bool is_running() const;
	void print_info_screen(FILE *out) const;

private:
	void subscribe_all();
	void unsubscribe_all();
	void advertise_open_all();
	void advertise_close_all();
	void get_orb_updates();
	void params_update();
	void control_main();
	void control_attitude();
	void control_rates(float dt);

private:
	bool _is_running;
	bool _should_stop;
	perf_counter_t _control_loop_perf;

	// Control parameter vectors
	struct {
		math::Vector<3> att_p; /**< P gain for attitude error */
		math::Vector<3> rate_p; /**< P gain for angular rate error */
		math::Vector<3> rate_i; /**< I gain for angular rate error */
		math::Vector<3> rate_d; /**< D gain for angular rate error */
	} _control_params;

	// Parameter handles
	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
	} _param_handles;

	// uORB subscription handles
	struct {
		int actuator_armed;
		int vehicle_control_mode;
		int vehicle_attitude;
		int vehicle_attitude_setpoint;
		int vehicle_rates_setpoint;
		int manual_control_setpoint;
		int parameter_update;
	} _sub_handles;

	// uORB publications
	struct {
		orb_advert_t vehicle_rates_setpoint;
		orb_advert_t actuator_controls_0; /**< attitude actuator controls publication */
		orb_advert_t vehicle_control_debug;
	} _pub_handles;

	actuator_armed_s _actuator_armed; /**< uORB actuator_armed topic data */
	vehicle_control_mode_s _vehicle_control_mode; /**< uORB vehicle_control_mode topic data */
	vehicle_attitude_s _vehicle_attitude; /**< uORB vehicle_attitude topic data */
	vehicle_attitude_setpoint_s _vehicle_attitude_setpoint; /**< uORB vehicle_attitude_setpoint topic data */
	vehicle_rates_setpoint_s _vehicle_rates_setpoint; /**< uORB vehicle_rates_setpoint topic data */
	manual_control_setpoint_s _manual_control_setpoint; /**< uORB manual_control_setpoint topic data */
	parameter_update_s _parameter_update; /**< uORB parameter_update topic data */
	actuator_controls_s _actuator_controls_0; /**< uORB actuator_controls_0 topic data */
	vehicle_control_debug_s _vehicle_control_debug;

	hrt_abstime _control_last_run;

	float _i_rollrate;
	float _i_pitchrate;
	float _i_yawrate;

	float _e_rollrate_prev;
	float _e_pitchrate_prev;
	float _e_yawrate_prev;
};

} // namespace singlerotor

#endif // SR_ATT_CONTROL_H
