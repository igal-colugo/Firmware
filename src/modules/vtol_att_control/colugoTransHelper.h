#pragma once

#include "vtol_type.h"
#include <uORB/Publication.hpp>
#include <uORB/topics/colugo_actuator.h>
#include <uORB/topics/colugo_transition.h>
#include <parameters/param.h>

//static const float CST_TRANSITION_VS_M_S = -2.6;

//static const float CST_VERTICAL_TRANS_S = 10.0;//VERTICAL TRASITION TIME IN SECONDS
//static const float CST_LOCK_AIRSPEED = 9.0;//horizontal airspeed for final lock of the wing
static const float CST_LOCKING_TIME_S = 1.2;//allowed time in seconds for the lock pin to reach final postion

static const float COLUGO_ACTUATOR_MC_POS{-1.0f};

/*
enum class COLUGO_FW_TRANS_STAGE{
		TRANS_IDLE = 0,
		TRANS_START,
		TRANS_REACHED_THROTLE,
		TRANS_COLUGO_ACT_TIME_FIRST_POS,
		TRANS_CONTROL_ACT_TIME_FIRST_POS,
		TRANS_CONTROL_ACT_TIME_FIRST_POS_ENDED,
		TRANS_TIME_SCND_POS,
		TRANS_ALLOW_FW
	};
	*/
enum class COLUGO_FW_VTRANS_STAGE{ //fixed wing vertical algorithm stages ....
		VTRANS_IDLE = 0,
		VTRANS_VERTICAL_START,//starting vertical ascend
		VTRANS_REACHED_SEMI_LOCK_POS, // finished 90% of time ascending colugo lock goes to semi out position
        	VTRANS_FARWARD_START,//starting pusher motor, surfaces slew to transition position
		VTRANS_REACHED_LOCK_SPEED,//reached blend speed colugo pin goes to fully lock position
		VTRANS_ALLOW_FW //safe to complete transition to fixed wing.
	};


class colugoTransHelper {
public:

	enum class vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE,
		PRE_TRANSITION_TO_FW

	};

    colugoTransHelper();

    void setColugoActuatorPos();
    void publishColugoActuator();
    void updateColugoTransitionState(float airSpd, vtol_mode fm, hrt_abstime tt);
    float getPusherThr(float thr);
    COLUGO_FW_VTRANS_STAGE getInnerState(){return _transStage;}
    void parameters_update();
    int32_t getColugoDebugVal(){return _params_colugo._param_c_debug;}
    float getColugoPiMcPos(){return _params_colugo._param_c_pi_mc_pos;}
    float getColugoPiFp(){return _params_colugo._param_c_pi_fp;}
    float getColugoPiSp(){return _params_colugo._param_c_pi_sp;}
    float getColugoTrFwSrvSlew(){return _params_colugo._param_c_tr_fw_srv_slew;}
    float getColugoTmToPos1(){return _params_colugo._param_c_tm_to_pos1;};
    float getColugoFlapsMcPos(){return _params_colugo._param_c_fl_mc_pos;}
    float getColugoFlapsFrstPos(){return _params_colugo._param_c_fl_fp;}
    float getColugoFlapsScndPos(){return _params_colugo._param_c_fl_sp;}
    float getColugoLockTimeToPos1(){return _params_colugo._param_c_tm_to_col_pos1;}
    float getColugoTimeToPos2(){return _params_colugo._param_c_tm_to_pos2;};
    float getColugoLockTransToFwFrstPos(){return _params_colugo._param_c_wafp;}
    float getColugoLockTransToFwScndPos(){return _params_colugo._param_c_wasp;}
    void lockColugoActuator();
    float getColugoTransToFwSlewedPitch();
    float getColugoTransToFwSlewedFlaps();



private:

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
		float _param_c_tm_to_col_pos1;
		float _param_c_tm_to_pos2;
		float _param_c_tr_fw_srv_slew;
		float _param_airspeed_blend;
		float _param_c_z_tr_spd_ms;
		float _param_c_z_tr_time_s;
		float _param_c_z_lck_tming;

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
		param_t _param_c_tm_to_col_pos1;
		param_t _param_c_tm_to_pos2;
		param_t _param_c_tr_fw_srv_slew;
		param_t _param_airspeed_blend;
		param_t _param_c_z_tr_spd_ms;
		param_t _param_c_z_tr_time_s;
		param_t _param_c_z_lck_tming;

	} _params_handles_colugo;

        //position of the wing lock actuator - range: -1.0 ~ 1.0
    float _wingLockActuatorPos = COLUGO_ACTUATOR_MC_POS;
    uORB::Publication<colugo_actuator_s> _colugo_actuator_pub{ORB_ID(colugo_actuator)};
    uORB::Publication<colugo_transition_s> _colugo_transition_pub{ORB_ID(colugo_transition)};
    vtol_mode _currentMode;
    hrt_abstime _startTime, _reachedLockSpeedTime, _FarwardStageStartTime;
    COLUGO_FW_VTRANS_STAGE _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
    float _airspeed;

	//methods
    void updateInnerStage();
    float getSlewedPosition(float startPos, float endPos);
};
