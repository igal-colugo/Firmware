/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file gps_parameter_switcher.cpp
 *
 * GPS Parameter Switcher Module
 *
 * Monitors GPS selection and automatically switches between two sets of
 * parameters (GPS0 and GPS1) when the active GPS changes.
 */

#include "gps_parameter_switcher.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <lib/systemlib/mavlink_log.h>
#include <parameters/param.h>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{200_ms}; ///< Schedule interval (5 Hz)

GpsParameterSwitcher::GpsParameterSwitcher() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

bool GpsParameterSwitcher::init()
{
	ScheduleOnInterval(SCHEDULE_INTERVAL);
	return true;
}

void GpsParameterSwitcher::Run()
{
	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		_enabled = _param_gps_param_switch_en.get() != 0;
	}

	if (!_enabled) {
		return;
	}

	// Check for GPS position updates
	vehicle_gps_position_s gps_pos;

	if (_vehicle_gps_position_sub.update(&gps_pos)) {
		// GPS selection: 0 = GPS1, 1 = GPS2, 2 = GPS3, 3 = Blending
		// We only care about GPS1 (0) and GPS2 (1)
		uint8_t selected_gps = gps_pos.selected;

		// Initialize on first valid GPS reading or switch if GPS changed
		if (selected_gps <= 1) {
			if (_current_gps_index == 255) {
				// First time - initialize with current GPS
				PX4_INFO("GPS parameter switcher initialized with GPS%d", selected_gps);
				switchParameters(selected_gps);
				_current_gps_index = selected_gps;

			} else if (selected_gps != _current_gps_index) {
				// GPS changed - switch parameters
				PX4_INFO("GPS parameter switch: GPS%d -> GPS%d", _current_gps_index, selected_gps);
				switchParameters(selected_gps);
				_current_gps_index = selected_gps;
			}
		}
	}
}

void GpsParameterSwitcher::switchParameters(uint8_t gps_index)
{
	if (gps_index > 1) {
		return; // Only support GPS0 and GPS1
	}

	const char *suffix = (gps_index == 0) ? "_GPS0" : "_GPS1";
	const char *base_params[] = {
		"EKF2_GPS_P_GATE",
		"EKF2_REQ_EPH",
		"EKF2_REQ_PDOP",
		"EKF2_REQ_VDRIFT",
		"FW_L1_R_SLEW_MAX",
		"FW_RR_P",
		"FW_THR_SLEW_MAX",
		"FW_T_CLMB_MAX",
		"MC_PITCHRATE_D",
		"MC_PITCHRATE_I",
		"MC_PITCHRATE_P",
		"MC_ROLLRATE_D",
		"MPC_ACC_HOR_MAX",
		"MPC_ACC_UP_MAX",
		"MPC_MAN_TILT_MAX",
		"MPC_TILTMAX_AIR",
		"MPC_VEL_MANUAL",
		"MPC_XY_VEL_MAX",
		"MPC_Z_VEL_MAX_DN",
		"MPC_Z_VEL_MAX_UP",
		"VT_FW_QC_P"
	};

	const int num_params = sizeof(base_params) / sizeof(base_params[0]);

	int params_applied = 0;

	for (int i = 0; i < num_params; i++) {
		char gps_param_name[32];
		snprintf(gps_param_name, sizeof(gps_param_name), "%s%s", base_params[i], suffix);

		param_t handle = param_find(gps_param_name);

		if (handle != PARAM_INVALID) {
			float value;
			int result = param_get(handle, &value);

			if (result == 0) {
				// Apply the GPS-specific parameter value to the base parameter
				param_t base_handle = param_find(base_params[i]);

				if (base_handle != PARAM_INVALID) {
					param_set(base_handle, &value);
					params_applied++;
					PX4_DEBUG("Switched %s to %.3f (from %s)", base_params[i], (double)value, gps_param_name);
				} else {
					PX4_WARN("Base parameter %s not found", base_params[i]);
				}
			}
		} else {
			PX4_DEBUG("GPS-specific parameter %s not found, skipping", gps_param_name);
		}
	}

	if (params_applied > 0) {
		PX4_INFO("Applied %d parameters for GPS%d", params_applied, gps_index);
	} else {
		PX4_WARN("No GPS-specific parameters found for GPS%d. Set parameters with _GPS0/_GPS1 suffix.", gps_index);
	}

	// Trigger parameter update notification
	param_notify_changes();
}

void GpsParameterSwitcher::applyParameter(const char *param_name, float value)
{
	param_t handle = param_find(param_name);

	if (handle != PARAM_INVALID) {
		param_set(handle, &value);
	}
}

int GpsParameterSwitcher::task_spawn(int argc, char *argv[])
{
	GpsParameterSwitcher *instance = new GpsParameterSwitcher();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		PX4_ERR("init failed");
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

int GpsParameterSwitcher::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GpsParameterSwitcher::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module monitors GPS selection and automatically switches between two sets of
parameters (GPS0 and GPS1) when the active GPS changes.

When GPS0 is selected, parameters with _GPS0 suffix are applied.
When GPS1 is selected, parameters with _GPS1 suffix are applied.

Supported parameters:
- EKF2_GPS_P_GATE
- EKF2_REQ_EPH
- EKF2_REQ_PDOP
- EKF2_REQ_VDRIFT
- FW_L1_R_SLEW_MAX
- FW_RR_P
- FW_THR_SLEW_MAX
- FW_T_CLMB_MAX
- MC_PITCHRATE_D
- MC_PITCHRATE_I
- MC_PITCHRATE_P
- MC_ROLLRATE_D
- MPC_ACC_HOR_MAX
- MPC_ACC_UP_MAX
- MPC_MAN_TILT_MAX
- MPC_TILTMAX_AIR
- MPC_VEL_MANUAL
- MPC_XY_VEL_MAX
- MPC_Z_VEL_MAX_DN
- MPC_Z_VEL_MAX_UP
- VT_FW_QC_P

Example: Set EKF2_GPS_P_GATE_GPS0=5.0 and EKF2_GPS_P_GATE_GPS1=10.0
When GPS0 is active, EKF2_GPS_P_GATE will be set to 5.0
When GPS1 is active, EKF2_GPS_P_GATE will be set to 10.0
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gps_parameter_switcher", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gps_parameter_switcher_main(int argc, char *argv[])
{
	return GpsParameterSwitcher::main(argc, argv);
}

