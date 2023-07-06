/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * Fail safe input mode
 *
 * This is the protocol used between the ground station and the autopilot.
 *
 * Recommended is Auto, RC only or MAVLink gimbal protocol v2.
 * The rest will be deprecated.
 *
 * @value -1 DISABLED
 * @value 0 Auto (RC and MAVLink gimbal protocol v2)
 * @value 1 RC
 * @value 2 MAVLINK_ROI (protocol v1, to be deprecated)
 * @value 3 MAVLINK_DO_MOUNT (protocol v1, to be deprecated)
 * @value 4 MAVlink gimbal protocol v2
 * @min 0
 * @max 1
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_EN, 0);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 50
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_GPS_TC, 0);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 30000
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_GPS_GT, 3000);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 30000
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_GPS_FGT, 3000);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 50
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_GPS_MINS, 10);

//**
 * Camera trigger activation time
 *
 * This parameter sets the time the trigger needs to pulled high or low.
 *
 * @unit ms
 * @min 0.0
 * @max 3000
 * @decimal 1
 * @reboot_required false
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(C_FAIL_GPS_DST, 3.0f);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 50
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_FLOAT(C_FAIL_GPS_VVAR, 1.0f);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 50
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_GPS_HDOP, 3);

/**
 * Fail safe output mode
 *
 * This is the protocol used between the autopilot and a connected gimbal.
 *
 * Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
 *
 * @value 0 AUX
 * @value 1 MAVLink gimbal protocol v1
 * @value 2 MAVLink gimbal protocol v2
 * @min 0
 * @max 50
 * @group COLUGO
 * @reboot_required false
 */
PARAM_DEFINE_INT32(C_FAIL_GPS_STAT, 2);
