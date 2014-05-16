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
 * @file sr_att_control.cpp
 *
 * @author Kim Lindberg Schwaner <kim.schwaner@gmail.com>
 */

#include <systemlib/param/param.h>

PARAM_DEFINE_FLOAT(SR_ROLL_P, 2.0f);
PARAM_DEFINE_FLOAT(SR_ROLLRATE_P, 0.2f);
PARAM_DEFINE_FLOAT(SR_ROLLRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(SR_ROLLRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(SR_PITCH_P, 2.0f);
PARAM_DEFINE_FLOAT(SR_PITCHRATE_P, 0.2f);
PARAM_DEFINE_FLOAT(SR_PITCHRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(SR_PITCHRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(SR_YAW_P, 2.0f);
PARAM_DEFINE_FLOAT(SR_YAWRATE_P, 0.2f);
PARAM_DEFINE_FLOAT(SR_YAWRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(SR_YAWRATE_D, 0.0f);

// PARAM_DEFINE_FLOAT(SR_YAW_FF, 0.5f);
// PARAM_DEFINE_FLOAT(SR_MAN_R_MAX, 35.0f);
// PARAM_DEFINE_FLOAT(SR_MAN_P_MAX, 35.0f);
// PARAM_DEFINE_FLOAT(SR_MAN_Y_MAX, 120.0f);
