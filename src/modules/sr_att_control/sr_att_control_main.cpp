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
#include <systemlib/getopt_long.h>
#include "sr_att_control.h"

// sys/types.h defines ERROR only for c (and not c++)?
#ifdef ERROR
#  undef ERROR
#endif
static const int ERROR = -1;

/**
 * Entry point of the sr_att_control application when started from the shell and
 * manager of the sr_att_control daemon.
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sr_att_control_main(int argc, char *argv[]);

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
			// start
			continue_parse = false;
			break;

		case 'b':
			// stop
			continue_parse = false;
			break;

		case 's':
			// status
			continue_parse = false;
			break;

		case 'h':
			usage();
			continue_parse = false;
			break;

		case '?':
			// getopt_long itself prints an error message here
			usage();
			continue_parse = false;
			break;

		default:
			abort();
		}
	}

	return EXIT_SUCCESS;
}
