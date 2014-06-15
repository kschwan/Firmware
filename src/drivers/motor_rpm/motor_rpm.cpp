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
 * @file motor_rpm.cpp
 *
 * Driver for the motor RPM measurement device.
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 */

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/encoders.h>
#include "motor_rpm.h"

MotorRPM::MotorRPM()
	: _fd(-1)
	, _uart_init_ok(false)
	, _time_last_update(0)
	, _pub_encoders(nullptr, ORB_ID(encoders))
{
}

MotorRPM::~MotorRPM()
{
	uart_deinit();
}

bool MotorRPM::uart_init(char const *device)
{
	if (device != nullptr) {
		return false;
	}

	// Open device
	_fd = open(device, O_RDWR | O_NOCTTY);

	if (_fd == -1) {
		fprintf(stderr, "Failed to open %s\n", device);
		return false;
	}

	// Read current port settings
	if (tcgetattr(_fd, &_orig_uart_config) < 0 || tcgetattr(_fd, &_uart_config) < 0) {
		fprintf(stderr, "Failed to read port settings on %s\n", device);
		return false;
	}

	// Configure the serial port in "raw" mode
	_uart_config.c_iflag &= ~(BRKINT | ICRNL | IGNBRK | IGNCR | INLCR | ISTRIP | IXON | PARMRK);
	_uart_config.c_oflag &= ~OPOST;
	_uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	_uart_config.c_cflag &= ~(CSIZE | PARENB);
	_uart_config.c_cflag |= CS8;

	// Baud rate
	if (cfsetispeed(&_uart_config, B57600) < 0 || cfsetospeed(&_uart_config, B57600)) {
		fprintf(stderr, "Failed to set baud rate on %s\n", device);
		return false;
	}

	// Apply new settings when any pending data has been flushed
	if (tcsetattr(_fd, TCSANOW, &_uart_config) < 0) {
		fprintf(stderr, "Failed to set new port settings on %s\n", device);
		return false;
	}

	// Flush
	tcflush(_fd, TCIOFLUSH);

	_uart_init_ok = true;
	return true;
}

void MotorRPM::uart_deinit()
{
	if (_uart_init_ok) {
		// Restore original settings when any pending data has been flushed
		tcsetattr(_fd, TCSANOW, &_orig_uart_config);
		tcflush(_fd, TCIOFLUSH);
		close(_fd);
		_uart_init_ok = false;
	}
}

void MotorRPM::update()
{
	int nbytes;

	if (!_uart_init_ok) {
		return;
	}

	// Send request
	_iobuf[0] = Cmd::REQUEST;
	nbytes = write(_fd, _iobuf, 1);

	if (nbytes < 0) {
		// handle error?
	}

	// Read response
	nbytes = read(_fd, _iobuf, 4); // TODO: how many bytes to read?

	if (nbytes < 0) {
		// handle error?
	} else if (nbytes == 0) {
		// no bytes read / end-of-file ?
	}

	// asdf
	_pub_encoders.counts[0] = 0;

	for (int i = 0; i < 4; i++) {
		_pub_encoders.counts[0] |= static_cast<uint8_t>(_iobuf[i] << 8 * i);
	}

	uint64_t time_now = hrt_absolute_time();
	uint64_t time_diff = time_now - _time_last_update;
	_time_last_update = time_now;

	// Calculate ticks per second
	_pub_encoders.velocity[0] = _pub_encoders.counts[0] / (time_diff / 1000000);

	// Publish topic
	_pub_encoders.timestamp = time_now;
	_pub_encoders.update();
}
