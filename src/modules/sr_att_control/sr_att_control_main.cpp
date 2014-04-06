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
 * @file sr_att_control_main.cpp
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 * @author Thomas Larsen <thola11@student.sdu.dk>
 */

#include <cxx/cstdlib>
#include <cxx/cstdio>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/getopt_long.h>
#include "sr_att_control.h"

// sys/types.h defines ERROR only for c (and not c++)?
#ifndef ERROR
static const int ERROR = -1;
#endif

/**
 * Entry point of the sr_att_control application when started from the shell and
 * manager of the sr_att_control daemon.
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sr_att_control_main(int argc, char *argv[]);

static singlerotor::AttitudeController *att_control; /**< A pointer to an AttitudeController object */

/**
 * Callback function to be called when the sr_att_control background loop exits.
 * This frees the memory allocated for the AttitudeController.
 */
static void att_control_atexit(void)
{
	delete att_control;
	att_control = nullptr;
	printf("stopped\n");
}

/**
 * Wrapper function for calling AttitudeController::run()
 *
 * It allows passing a pointer-to-C++-member-function as the task entry point.
 * An object to invoke the function on is necessary. Furthermore, an atexit()
 * handler can be registered here to allow the run-loop a graceful return.
 *
 * All this has the added benefit of keeping NuttX task-handling stuff seperate
 * from the AttitudeController class.
 *
 * @param argc the argument count.
 * @param argv the argument list.
 *
 * @return the task exit value.
 */
static int att_control_run_wrapper(int argc, char *argv[])
{
	atexit(&att_control_atexit);
	return att_control->run(argc, argv);
}

/**
 * Starts a task using att_control_run_wrapper() as the entry point
 */
static int att_control_start()
{
	return task_spawn_cmd("sr_att_control",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_MAX - 5,
			      2048,
			      static_cast<main_t>(&att_control_run_wrapper),
			      nullptr);
}

/**
 * Print the usage help.
 */
static void usage()
{
	printf("Usage: sr_att_control [options]\n\n");
	printf("  -h, --help\tthis help\n");
	printf("      --start\tstarts the daemon\n");
	printf("      --stop\tstops the daemon\n");
	printf("  -s, --status\tshow status\n");
}

int sr_att_control_main(int argc, char *argv[])
{
	int opt;
	int opt_idx = 0;
	bool continue_parse = true;

	static GETOPT_LONG_OPTION_T options[] = {
		{"start", NO_ARG, NULL, 'a'},
		{"stop", NO_ARG, NULL, 'b'},
		{"status", NO_ARG, NULL, 's'},
		{"help", NO_ARG, NULL, 'h'},
		{NULL, NULL, NULL, NULL}
	};

	optind = 0; // Reset optind

	while (continue_parse) {
		opt = getopt_long(argc, argv, "sh", options, &opt_idx);

		if (opt == EOF) {
			break;
		}

		switch (opt) {
		case 0:
			// A flag was set
			break;

		case 'a':
			if (att_control != nullptr && att_control->isRunning()) {
				fprintf(stderr, "already running\n");
			} else {
				att_control = new singlerotor::AttitudeController;

				if (att_control == nullptr) {
					fprintf(stderr, "alloc failed\n");
					return EXIT_FAILURE;
				} else {
					if (att_control_start() < 0) {
						delete att_control;
						att_control = nullptr;
						fprintf(stderr, "start failed\n");
						return EXIT_FAILURE;
					} else {
						printf("started\n");
					}
				}
			}

			continue_parse = false;
			break;

		case 'b':
			if (att_control != nullptr && att_control->isRunning()) {
				att_control->stop();
				printf("stopping...\n");
			} else {
				printf("already stopped\n");
			}

			continue_parse = false;
			break;

		case 's':
			if (att_control == nullptr || !att_control->isRunning()) {
				printf("stopped\n");
			} else {
				printf("running\n");
			}

			continue_parse = false;
			break;

		case 'h':
		case '?':
		default:
			usage();
			continue_parse = false;
			break;
		}
	}

	return EXIT_SUCCESS;
}
