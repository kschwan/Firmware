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
 * @file sr_att_control_main.cpp
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <systemlib/systemlib.h>
#include <systemlib/getopt_long.h>
#include "sr_att_control.h"

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

static void info_screen(const char *path)
{
	FILE *out;
	int fd = -1;

	if (path) {
		fd = open(path, O_WRONLY);

		if (fd < 0) {
			printf("error opening %s\n", path);
			return;
		}

		out = fdopen(fd, "w");
		printf("opening infoscreen on %s\n", path);
	} else {
		out = stdout;
		printf("opening infoscreen on stdout\n");
	}

	fflush(out);

	char c;
	struct pollfd fds;
	int ret;

	for (;;) {
		att_control->print_info_screen(out);

		// Sleep waiting for user input (to cancel the loop)
		for (int k = 0; k < 5; k++) {
			fds.fd = 0; /* stdin */
			fds.events = POLLIN;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {
				read(0, &c, 1);

				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					return;
				}
			}

			usleep(200000);
		}

		sleep(1);
	}

	if (fd < 0) {
		close(fd);
	}
}

/**
 * Print the usage help.
 */
static void usage()
{
	printf("%s", "Usage: sr_att_control [option]...\n\n");
	printf("%-30s%-s\n", "  -h, --help", "this help");
	printf("%-30s%-s\n", "      --start", "starts the daemon");
	printf("%-30s%-s\n", "  -i, --infoscreen=DEVICE", "show information screen on DEVICE");
	printf("%-30s%-s\n", "      --stop", "stops the daemon");
	printf("%-30s%-s\n", "  -s, --status", "show status");
}

int sr_att_control_main(int argc, char *argv[])
{
	int opt;
	int opt_idx = 0;
	bool continue_parse = true;

	static GETOPT_LONG_OPTION_T options[] = {
		{"start", NO_ARG, 0, 'a'},
		{"stop", NO_ARG, 0, 'b'},
		{"status", NO_ARG, 0, 's'},
		{"infoscreen", OPTIONAL_ARG, 0, 'i'},
		{"help", NO_ARG, 0, 'h'},
		{0, 0, 0, 0}
	};

	optind = 0; // Reset optind

	while (continue_parse) {
		opt = getopt_long(argc, argv, "sih", options, &opt_idx);

		if (opt == EOF) {
			break;
		}

		switch (opt) {
		case 0:
			// A flag was set
			break;

		case 'a':
			if (att_control != nullptr && att_control->is_running()) {
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
			if (att_control != nullptr && att_control->is_running()) {
				att_control->stop();
				printf("stopping...\n");
			} else {
				printf("already stopped\n");
			}

			continue_parse = false;
			break;

		case 's':
			if (att_control == nullptr || !att_control->is_running()) {
				printf("stopped\n");
			} else {
				printf("running\n");
			}

			continue_parse = false;
			break;

		case 'i':

			if (att_control->is_running()) {
				info_screen(optarg);
			} else {
				printf("not running\n");
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
