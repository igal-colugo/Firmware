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
 * @file gps_parameter_switcher_params.c
 *
 * Parameters for GPS Parameter Switcher module
 */

#ifndef PARAM_DEFINE_INT32
#define PARAM_DEFINE_INT32(name, def) /* parameter definition */
#endif

#ifndef PARAM_DEFINE_FLOAT
#define PARAM_DEFINE_FLOAT(name, def) /* parameter definition */
#endif

/**
 * GPS Parameter Switching Enable
 *
 * Enable automatic parameter switching based on GPS selection.
 * When enabled, parameters with _GPS0 and _GPS1 suffixes will be
 * automatically applied when GPS0 or GPS1 is selected.
 *
 * @group GPS Parameter Switcher
 * @boolean
 */
PARAM_DEFINE_INT32(GPS_P_SWCH_EN, 1);

/**
 * GPS-specific parameter variants for GPS0 and GPS1
 * These parameters store the values to be applied when the corresponding GPS is selected.
 * Users should set these parameters to the desired values for each GPS.
 *
 * @group GPS Parameter Switcher
 */

// EKF2 Parameters
PARAM_DEFINE_FLOAT(EKF2_GPS_P_GATE_GPS0, 5.0f);
PARAM_DEFINE_FLOAT(EKF2_GPS_P_GATE_GPS1, 5.0f);
PARAM_DEFINE_FLOAT(EKF2_REQ_EPH_GPS0, 3.0f);
PARAM_DEFINE_FLOAT(EKF2_REQ_EPH_GPS1, 3.0f);
PARAM_DEFINE_FLOAT(EKF2_REQ_PDOP_GPS0, 2.5f);
PARAM_DEFINE_FLOAT(EKF2_REQ_PDOP_GPS1, 2.5f);
PARAM_DEFINE_FLOAT(EKF2_REQ_VDRIFT_GPS0, 0.2f);
PARAM_DEFINE_FLOAT(EKF2_REQ_VDRIFT_GPS1, 0.2f);

// Fixed-wing Parameters
PARAM_DEFINE_FLOAT(FW_L1_R_SLEW_MAX_GPS0, 90.0f);
PARAM_DEFINE_FLOAT(FW_L1_R_SLEW_MAX_GPS1, 90.0f);
PARAM_DEFINE_FLOAT(FW_RR_P_GPS0, 0.05f);
PARAM_DEFINE_FLOAT(FW_RR_P_GPS1, 0.05f);
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX_GPS0, 0.0f);
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX_GPS1, 0.0f);
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX_GPS0, 5.0f);
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX_GPS1, 5.0f);

// Multicopter Rate Control Parameters
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D_GPS0, 0.003f);
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D_GPS1, 0.003f);
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I_GPS0, 0.2f);
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I_GPS1, 0.2f);
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P_GPS0, 0.15f);
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P_GPS1, 0.15f);
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D_GPS0, 0.003f);
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D_GPS1, 0.003f);

// Multicopter Position Control Parameters
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX_GPS0, 5.0f);
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX_GPS1, 5.0f);
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX_GPS0, 4.0f);
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX_GPS1, 4.0f);
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX_GPS0, 35.0f);
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX_GPS1, 35.0f);
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR_GPS0, 45.0f);
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR_GPS1, 45.0f);
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL_GPS0, 10.0f);
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL_GPS1, 10.0f);
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX_GPS0, 12.0f);
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX_GPS1, 12.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN_GPS0, 1.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN_GPS1, 1.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP_GPS0, 3.0f);
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP_GPS1, 3.0f);

// VTOL Parameters
PARAM_DEFINE_INT32(VT_FW_QC_P_GPS0, 0);
PARAM_DEFINE_INT32(VT_FW_QC_P_GPS1, 0);

