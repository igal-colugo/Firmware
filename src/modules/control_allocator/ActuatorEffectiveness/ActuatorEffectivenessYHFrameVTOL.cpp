/****************************************************************************
 *
 *   Copyright (c) 2020-2025 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessYHFrameVTOL.hpp"

using namespace matrix;

ActuatorEffectivenessYHFrameVTOL::ActuatorEffectivenessYHFrameVTOL(ModuleParams *parent)
	: ActuatorEffectivenessStandardVTOL(parent)
{
}

bool
ActuatorEffectivenessYHFrameVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// Call parent to set up standard VTOL configuration
	bool success = ActuatorEffectivenessStandardVTOL::getEffectivenessMatrix(configuration, external_update);

	if (success) {
		// Apply Y H-Frame specific constraints to the effectiveness matrix
		applyYHFrameConstraints(configuration);
	}

	return success;
}

void
ActuatorEffectivenessYHFrameVTOL::applyYHFrameConstraints(Configuration &configuration)
{
	// Get the MC motors effectiveness matrix (matrix 0)
	EffectivenessMatrix &effectiveness = configuration.effectiveness_matrices[0];

	// Standard H-frame motor layout:
	// Motor 0: Front Right
	// Motor 1: Rear Left
	// Motor 2: Front Left
	// Motor 3: Rear Right

	// Effectiveness matrix rows:
	// Row 0: Roll moment
	// Row 1: Pitch moment
	// Row 2: Yaw moment
	// Row 3-5: Thrust (X, Y, Z)

	// Front motors (0, 2): Zero yaw effectiveness
	// Remove yaw contribution while preserving pitch and roll
	effectiveness(2, 0) = 0.0f; // Front right motor: zero yaw
	effectiveness(2, 2) = 0.0f; // Front left motor: zero yaw

	// Rear motors (1, 3): Zero roll effectiveness
	// Remove roll contribution while preserving pitch and yaw
	effectiveness(0, 1) = 0.0f; // Rear left motor: zero roll
	effectiveness(0, 3) = 0.0f; // Rear right motor: zero roll
}
