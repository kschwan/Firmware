/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Kim Lindberg Schwaner <kschw10@student.sdu.dk>
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
 * @file motor_rpm.h
 *
 * Driver for the motor RPM measurement device.
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 */

#ifndef MOTOR_RPM_H
#define MOTOR_RPM_H

#include <termios.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/encoders.h>
#include <uORB/topics/debug_key_value.h>

class MotorRPM
{
public:
	MotorRPM();
	~MotorRPM();
	bool uart_init(char const *device);
	void uart_deinit();
	void update();

private:
	struct Cmd {
		static const uint8_t REQUEST = 0x03;
	};

	int _fd;
	struct termios _uart_config, _orig_uart_config;
	bool _uart_init_ok;
	uint64_t _time_last_update;
	uint8_t _iobuf[8];
	uORB::Publication<encoders_s> _pub_encoders;
	uORB::Publication<debug_key_value_s> _pub_debug;
};

#endif // MOTOR_RPM_H