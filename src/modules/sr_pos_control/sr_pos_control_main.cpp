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
 * @file sr_pos_control_main.cpp
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
#include "sr_pos_control.h"

extern "C" __EXPORT int sr_pos_control_main(int argc, char *argv[]);

static singlerotor::PositionController *pos_control;

static void pos_control_atexit(void)
{
	delete pos_control;
	pos_control = nullptr;
	printf("stopped\n");
}


static int pos_control_run_wrapper(int argc, char *argv[])
{
	atexit(&pos_control_atexit);
	return pos_control->run(argc, argv);
}

static int pos_control_start()
{
	return task_spawn_cmd("sr_pos_control",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_MAX - 5,
			      2048,
			      static_cast<main_t>(&pos_control_run_wrapper),
			      nullptr);
}

/**
 * Print the usage help.
 */
static void usage()
{
	printf("%s", "Usage: sr_pos_control [option]...\n\n");
	printf("%-30s%-s\n", "  -h, --help", "this help");
	printf("%-30s%-s\n", "      --start", "starts the daemon");
	printf("%-30s%-s\n", "      --stop", "stops the daemon");
	printf("%-30s%-s\n", "  -s, --status", "show status");
}

int sr_pos_control_main(int argc, char *argv[])
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
			if (pos_control != nullptr && pos_control->is_running()) {
				fprintf(stderr, "already running\n");
			} else {
				pos_control = new singlerotor::PositionController;

				if (pos_control == nullptr) {
					fprintf(stderr, "alloc failed\n");
					return EXIT_FAILURE;
				} else {
					if (pos_control_start() < 0) {
						delete pos_control;
						pos_control = nullptr;
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
			if (pos_control != nullptr && pos_control->is_running()) {
				pos_control->stop();
				printf("stopping...\n");
			} else {
				printf("already stopped\n");
			}

			continue_parse = false;
			break;

		case 's':
			if (pos_control == nullptr || !pos_control->is_running()) {
				printf("stopped\n");
			} else {
				printf("running\n");
			}

			continue_parse = false;
			break;

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
