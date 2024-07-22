/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file standard.h
* VTOL with fixed multirotor motor configurations (such as quad) and a pusher
* (or puller aka tractor) motor for forward flight.
*
* @author Simon Wilks 		<simon@uaventure.com>
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
* @author Sander Smeets 	<sander@droneslab.com>
*
*/

#ifndef STANDARD_H
#define STANDARD_H
#include "vtol_type.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include "colugoTransHelper.h"
//#include <uORB/topics/debug_vect_clg.h>
#include <uORB/topics/debug_vect.h>


class Standard : public VtolType
{

public:

	Standard(VtolAttitudeControl *_att_controller);
	~Standard() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

private:


	struct {
		float pusher_ramp_dt;
		float back_trans_ramp;
		float pitch_setpoint_offset;
		float reverse_output;
		float reverse_delay;
	} _params_standard;

	struct {
		param_t pusher_ramp_dt;
		param_t back_trans_ramp;
		param_t pitch_setpoint_offset;
		param_t reverse_output;
		param_t reverse_delay;
	} _params_handles_standard;


	struct
	{
		hrt_abstime throttle_trans_reached_time;

	} _colugo_trans_to_fw;


	colugoTransHelper _cth{};
	//COLUGO_FW_TRANS_STAGE _colugo_fw_trans_stage;

	struct {
		colugoTransHelper::vtol_mode flight_mode;			// indicates in which mode the vehicle is in
		hrt_abstime transition_start;	// at what time did we start a transition (front- or backtransition)
	} _vtol_schedule;


	float _pusher_throttle{0.0f};
	float _reverse_output{0.0f};
	float _airspeed_trans_blend_margin{0.0f};
	//my debug logs...
	//struct debug_vect_clg_s _dbg_vect_clg;
	//orb_advert_t pub_dbg_vect_clg = orb_advertise(ORB_ID(debug_vect_clg), &_dbg_vect_clg);
	//debug for mavlink...
	struct debug_vect_s _dbg_vect_for_mav;
	orb_advert_t pub_dbg_vect_for_mav = orb_advertise(ORB_ID(debug_vect), &_dbg_vect_for_mav);
	void parameters_update() override;

	//cologo staff
	void publishDebugForMavIfneeded();


};
#endif
