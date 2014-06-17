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
 * @file motor_rpm_main.cpp
 *
 * Driver for the motor RPM measurement device.
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <systemlib/systemlib.h>
#include "motor_rpm.h"

static bool task_should_exit = false;
static bool task_running = false;
static char const *task_argv[2] = {nullptr};

extern "C" __EXPORT int motor_rpm_main(int argc, char *argv[]);

/**
 * Callback function to be called when the task exits.
 */
static void motor_rpm_atexit(void)
{
	task_running = false;
	puts("stopped");
}

/**
 * The motor RPM task
 */
static int motor_rpm_task(int argc, char *argv[])
{
	atexit(&motor_rpm_atexit);
	MotorRPM mrpm;

	// Argument sanity check
	if (argc != 2) {
		return EXIT_FAILURE;
	} else if (strncmp(argv[1], "/dev", 4) != 0) {
		fprintf(stderr, "%s: bad argument '%s'\n", argv[0], argv[1]);
		return EXIT_FAILURE;
	}

	// Init UART for the MotorRPM
	if (!mrpm.uart_init(argv[1])) {
		fprintf(stderr, "%s: %s could not be initialized", argv[0], argv[1]);
		return EXIT_FAILURE;
	}

	// Wait a little before first update
	usleep(20000);

	// Start task loop
	task_running = true;

	while (!task_should_exit) {
		mrpm.update();
		usleep(10000); // Update rate = 10 ms ~ 100Hz
	}

	return EXIT_SUCCESS;
}

/**
 * Prints usage help
 */
static void usage()
{
	printf("%s", "Usage: motor_rpm [option]...\n\n");
}

/**
 * Program entry point. Manages the background task
 */
int motor_rpm_main(int argc, char *argv[])
{
	char const *device = nullptr;
	int c;

	if (argc < 1) {
		usage();
		return EXIT_FAILURE;
	}

	while ((c = getopt(argc - 1, &argv[1], "d:")) != EOF) {
		switch (c) {
		case 'd':
			device = optarg;
			break;

		case '?':
			if (optopt == 'd') {
				fprintf(stderr, "%s: option -%c requires an argument.\n", argv[0], optopt);
			} else {
				fprintf(stderr, "%s: unknown option character 0x%x.\n", argv[0], optopt);
			}

			return EXIT_FAILURE;

		default:
			abort();
		}
	}

	if (!strcmp(argv[1], "start")) {
		if (task_running) {
			fprintf(stderr, "%s: daemon already running\n", argv[0]);
		} else {
			task_argv[0] = device;
			task_should_exit = false;
			return task_spawn_cmd("motor_rpmd",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_MAX - 5,
					      1024,
					      static_cast<main_t>(&motor_rpm_task),
					      task_argv);
		}
	} else if (!strcmp(argv[1], "stop")) {
		if (task_running) {
			printf("%s: stopping daemon...\n", argv[0]);
			task_should_exit = true;
		} else {
			printf("%s: daemon already stopped\n", argv[0]);
		}
	}

	return EXIT_SUCCESS;
}