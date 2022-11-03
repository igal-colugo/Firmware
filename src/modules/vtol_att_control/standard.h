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
#include <uORB/topics/colugo_actuator.h>
#include <lib/colugo/colugoTransHelper.h>

//debug
//#include <uORB/topics/debug_key_value.h>
//#include <uORB/uORB.h>
//#include <string.h>
//#include <uORB/topics/debug_value.h>
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


	struct {
		int32_t _param_c_debug;
		float 	_param_c_wafp;
		float 	_param_c_wasp;
		float 	_param_c_pi_fp;
		float 	_param_c_pi_sp;
		float	_param_c_pi_mc_pos;
		float 	_param_c_fl_fp;
		float 	_param_c_fl_sp;
		float	_param_c_fl_mc_pos;
		float _param_c_tm_to_pos1;
	} _params_colugo;


	struct {
		param_t _param_c_debug;
		param_t _param_c_wafp;
		param_t _param_c_wasp;
		param_t _param_c_pi_fp;
		param_t _param_c_pi_sp;
		param_t _param_c_pi_mc_pos;
		param_t _param_c_fl_fp;
		param_t _param_c_fl_sp;
		param_t _param_c_fl_mc_pos;
		param_t _param_c_tm_to_pos1;
	} _params_handles_colugo;

//	bool _fw_trans_latch = false;

	struct
	{
		bool _reached_blend_atlist_once;
		bool _reached_trans_atlist_once;
		hrt_abstime blend_speed_reached_time;// at what time did we reached blend speed

	} _colugo_trans_to_fw;


	enum class vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE
	};

	struct {
		vtol_mode flight_mode;			// indicates in which mode the vehicle is in
		hrt_abstime transition_start;	// at what time did we start a transition (front- or backtransition)
		//hrt_abstime blend_speed_treached;// at what time did we reached blend speed
		//hrt_abstime blend_speed_reached;//time after reaching blend speed for actuator posiotion 1
		//bool need_update_blend_time_reached;//bool latch helper for intermidiate time
	} _vtol_schedule;

	const float COLUGO_ACTUATOR_MC_POS{-1.0f};
	//float _colugoActuatorPos{COLUGO_ACTUATOR_MC_POS};
	float _pusher_throttle{0.0f};
	float _reverse_output{0.0f};
	float _airspeed_trans_blend_margin{0.0f};

	//ColugoTransHelper _colugo_trans_helper = ColugoTransHelper();
    	uORB::Publication<colugo_actuator_s> _colugo_actuator_pub{ORB_ID(colugo_actuator)};

	struct debug_vect_s dbg_vect_clg;
	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect_clg);

	void parameters_update() override;
	//cologo staff
	void publishColugoActuatorIfneeded(float val);
	void resetColugoTransitionStruct();
	/*
get the postion of pitch control for mc to fw trasition (to move the free wing to correct location before lock)
*/
	float getColugoToFwPitchTransition();

	/*
get the postion of flaps control for mc to fw trasition (to move the free wing to correct location before lock)
*/
	float getColugoToFwFlapsTransition();
	float getColugoActuatorToFwTransition();

	//is the airspeed is higher than blend air speed
	bool isAirspeedAbovePos1ForTransition();
	//is the airspeed is higher than transition air speed
	bool isAirspeedAbovePos2ForTransition();
	//is time since trasition command > _param_c_tm_to_pos1
	bool isTimeToColugoPos1();

	//same as getColugoToFwFlapsTransition - but pos #1 is time based
	float getColugoToFwFlapsTransitionTimeBased();

	//same as getColugoToFwPitchTransition - but pos #1 is time based
	float getColugoToFwPitchTransitionTimeBased();

};
#endif
