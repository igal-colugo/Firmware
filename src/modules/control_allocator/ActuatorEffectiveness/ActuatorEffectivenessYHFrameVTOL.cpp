/****************************************************************************
 *
 *   Copyright (c) 2024 Colugo Systems. All rights reserved.
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
 * @file ActuatorEffectivenessYHFrameVTOL.cpp
 *
 * Actuator effectiveness for Colugo Y H-Frame VTOL
 */

#include "ActuatorEffectivenessYHFrameVTOL.hpp"
#include <px4_platform_common/log.h>
#include "../ControlAllocation/ControlAllocation.hpp"

using namespace matrix;

ActuatorEffectivenessYHFrameVTOL::ActuatorEffectivenessYHFrameVTOL(ModuleParams *parent)
	: ActuatorEffectivenessStandardVTOL(parent)
{
	PX4_INFO("Y H-Frame VTOL actuator effectiveness initialized");
}

bool ActuatorEffectivenessYHFrameVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// First, get the standard VTOL effectiveness matrix
	bool success = ActuatorEffectivenessStandardVTOL::getEffectivenessMatrix(configuration, EffectivenessUpdateReason::CONFIGURATION_UPDATE);

	if (!success) {
		PX4_ERR("Y H-Frame VTOL: Failed to get standard VTOL effectiveness matrix");
		return false;
	}

	// Apply Y H-Frame specific constraints
	applyYHFrameConstraints(configuration);

	return true;
}

void ActuatorEffectivenessYHFrameVTOL::applyYHFrameConstraints(Configuration &configuration)
{
	// Only apply constraints once and log the application
	if (_constraints_applied) {
		return;
	}

	// Get the multicopter motors effectiveness matrix (matrix[0])
	EffectivenessMatrix &effectiveness = configuration.effectiveness_matrices[0];

	const int num_motors = configuration.num_actuators_matrix[0];

	if (num_motors < 4) {
		PX4_ERR("Y H-Frame VTOL: Expected at least 4 motors, got %d", num_motors);
		return;
	}

	//PX4_INFO("==============================================");
//	PX4_INFO("Y H-Frame VTOL: Applying custom motor mixing");
	//PX4_INFO("==============================================");

	// Log original effectiveness values before modification
//	PX4_INFO("Original effectiveness matrix (first 4 motors):");
	//PX4_INFO("Motor 0 (Front Right): Roll=%.3f, Pitch=%.3f, Yaw=%.3f",
	//	 (double)effectiveness(0, 0), (double)effectiveness(1, 0), (double)effectiveness(2, 0));
	//PX4_INFO("Motor 1 (Rear Left):   Roll=%.3f, Pitch=%.3f, Yaw=%.3f",
	//	 (double)effectiveness(0, 1), (double)effectiveness(1, 1), (double)effectiveness(2, 1));
	//PX4_INFO("Motor 2 (Front Left):  Roll=%.3f, Pitch=%.3f, Yaw=%.3f",
	//	 (double)effectiveness(0, 2), (double)effectiveness(1, 2), (double)effectiveness(2, 2));
	//PX4_INFO("Motor 3 (Rear Right):  Roll=%.3f, Pitch=%.3f, Yaw=%.3f",
	//	 (double)effectiveness(0, 3), (double)effectiveness(1, 3), (double)effectiveness(2, 3));

	// Apply Y H-Frame constraints:
	// Front motors (0, 2): Zero yaw effectiveness
	effectiveness(ControlAllocation::ControlAxis::YAW, 0) = 0.0f;  // Motor 0 (Front Right)
	effectiveness(ControlAllocation::ControlAxis::YAW, 2) = 0.0f;  // Motor 2 (Front Left)

	// Rear motors (1, 3): Zero roll effectiveness
	effectiveness(ControlAllocation::ControlAxis::ROLL, 1) = 0.0f; // Motor 1 (Rear Left)
	effectiveness(ControlAllocation::ControlAxis::ROLL, 3) = 0.0f; // Motor 3 (Rear Right)
/*
	PX4_INFO("----------------------------------------------");
	PX4_INFO("Modified effectiveness matrix:");
	PX4_INFO("Motor 0 (Front Right): Roll=%.3f, Pitch=%.3f, Yaw=%.3f (YAW ZEROED)",
		 (double)effectiveness(0, 0), (double)effectiveness(1, 0), (double)effectiveness(2, 0));
	PX4_INFO("Motor 1 (Rear Left):   Roll=%.3f, Pitch=%.3f, Yaw=%.3f (ROLL ZEROED)",
		 (double)effectiveness(0, 1), (double)effectiveness(1, 1), (double)effectiveness(2, 1));
	PX4_INFO("Motor 2 (Front Left):  Roll=%.3f, Pitch=%.3f, Yaw=%.3f (YAW ZEROED)",
		 (double)effectiveness(0, 2), (double)effectiveness(1, 2), (double)effectiveness(2, 2));
	PX4_INFO("Motor 3 (Rear Right):  Roll=%.3f, Pitch=%.3f, Yaw=%.3f (ROLL ZEROED)",
		 (double)effectiveness(0, 3), (double)effectiveness(1, 3), (double)effectiveness(2, 3));
	PX4_INFO("==============================================");
	PX4_INFO("Y H-Frame VTOL motor mixing ACTIVE");
	PX4_INFO("==============================================");
*/
	_constraints_applied = true;
}
