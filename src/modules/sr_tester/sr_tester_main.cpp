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
 * @file sr_tester_main.cpp
 *
 * @author Kim Lindberg Schwaner <kschw10@student.sdu.dk>
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <nuttx/analog/adc.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>

static int adc_logger_thread_handle;

extern "C" __EXPORT int sr_tester_main(int argc, char *argv[]);
int adc_logger_thread_main(int argc, char *argv[]);

static void usage()
{
	printf("Usage: sr_tester [options]\n\n");
	printf("  help\tthis help\n");
	printf("  adc\tstart adc logger\n");
}

int adc_logger_thread_main(int argc, char *argv[])
{
	struct debug_key_value_s dbg;
	struct adc_msg_s sample[12];
	ssize_t count;

	int fd = open(ADC_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		warnx("ERROR: can't open ADC device");
		return EXIT_FAILURE;
	}

	strcpy(dbg.key, "potmeter");
	dbg.value = 0.0f;

	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	// Main loop
	while (1) {
		count = read(fd, sample, sizeof(sample));

		if (count < 0) {
			close(fd);
			return EXIT_FAILURE;
		}

		int num_channels = count / sizeof(sample[0]);

		// TODO: Decide which channel want to forward on mavlink
		dbg.value = static_cast<float>(sample[0].am_data); // BEWARE uint32_t -> float?

		dbg.timestamp_ms = hrt_absolute_time() / 1000.0f;
		orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);

		usleep(10000); // Sample rate?
	}

	close(pub_dbg);

	return EXIT_SUCCESS;
}

int sr_tester_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage();
	} else if (!strcmp(argv[1], "help")) {
		usage();
	} else if (!strcmp(argv[1], "adc")) {
		adc_logger_thread_handle = task_spawn_cmd("adc_logger",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT,
					   1024,
					   static_cast<main_t>(adc_logger_thread_main),
					   nullptr);
	} else {
		usage();
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
