#include "colugoTransHelper.h"

#include <mathlib/math/Limits.hpp>

colugoTransHelper::colugoTransHelper() {
    // Constructor code here
    	_params_handles_colugo._param_c_wafp  = param_find("C_WAFP");
	_params_handles_colugo._param_c_wasp  = param_find("C_WASP");
	_params_handles_colugo._param_c_pi_fp = param_find("C_PI_FP");
	_params_handles_colugo._param_c_pi_sp = param_find("C_PI_SP");
	_params_handles_colugo._param_c_pi_mc_pos = param_find("C_PI_MC_POS");
	_params_handles_colugo._param_c_fl_fp = param_find("C_FL_FP");
	_params_handles_colugo._param_c_fl_sp = param_find("C_FL_SP");
	_params_handles_colugo._param_c_fl_mc_pos = param_find("C_FL_MC_POS");
	_params_handles_colugo._param_c_tm_to_pos1 = param_find("C_TM_TO_POS1");
	_params_handles_colugo._param_c_tm_to_col_pos1 = param_find("C_TM_TO_COL_POS1");
	_params_handles_colugo._param_c_tm_to_pos2 = param_find("C_TM_TO_POS2");
	_params_handles_colugo._param_c_tr_fw_srv_slew = param_find("C_TR_FW_SRV_SLEW");

	_params_handles_colugo._param_c_debug = param_find("C_DEBUG");
}
//@note colugoTransHelper
void colugoTransHelper::setColugoActuatorPos() {
	float res = COLUGO_ACTUATOR_MC_POS;
	if(_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS){
		res = _params_colugo._param_c_wafp;
	}
	if(_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED){
		res = _params_colugo._param_c_wasp;
	}
	_wingLockActuatorPos = res;
}
void colugoTransHelper::lockColugoActuator(){
	if(fabsf(_wingLockActuatorPos - _params_colugo._param_c_wasp) > FLT_EPSILON){
		_wingLockActuatorPos = _params_colugo._param_c_wasp;
		publishColugoActuator();
	}
}

void colugoTransHelper::publishColugoActuator()
{
	colugo_actuator_s colugo_act{};
	colugo_act.actuator_state = _wingLockActuatorPos;
	colugo_act.timestamp = hrt_absolute_time();
	_colugo_actuator_pub.publish(colugo_act);

}

void colugoTransHelper::updateColugoTransitionState(float airSpd, mode fm, hrt_abstime tt){

	_airspeed = airSpd;
		switch (fm)
		{
		case mode::FIXED_WING:
			_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
			break;
		case mode::TRANSITION_TO_FW:
			if(fm != _currentMode){//we just entered tansition...
				_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START;
				_startTime = tt;
				break;
			}
			else{//we are in trasition...
				updateInnerStage();
			}
			break;
		case mode::TRANSITION_TO_MC:
			_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
			break;
		case mode::ROTARY_WING:
		/* code */
			break;

	default:
		break;
	}
	_currentMode = fm;
}
void colugoTransHelper::updateInnerStage(){
	uint64_t debugDiff = (hrt_absolute_time() - _startTime);
	switch (_transStage)
	{
	case COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START:
		if(debugDiff > ((CST_VERTICAL_TRANS_S * 0.8f) * 1000000)){
			_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS;
		}
		break;
	case COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS:
		if(debugDiff > (CST_VERTICAL_TRANS_S * 1000000)){
			_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START;
		}
		break;
	case COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START:
		if(_airspeed > CST_LOCK_AIRSPEED){
			_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED;
			_reachedLockSpeedTime = hrt_absolute_time();

		}
		break;
	case COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED:
	//give the pin some time to move to final position...
		if(hrt_absolute_time() - _reachedLockSpeedTime > (CST_LOCKING_TIME_S * 1000000)){
			_transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_ALLOW_FW;
		}
		break;
	case COLUGO_FW_VTRANS_STAGE::VTRANS_ALLOW_FW:
	case COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE:
		break;
	}



//publish only if not idel...
	if(COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE != _transStage){
		colugo_transition_s colugo_trans{};
		colugo_trans.transition_state = static_cast<uint8_t>(_transStage);
		colugo_trans.timestamp = hrt_absolute_time();
		_colugo_transition_pub.publish(colugo_trans);
	}

}

//while we are in vertical lift stage - dont push farward...
float colugoTransHelper::getPusherThr(float thr){
	return COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START == _transStage ? 0 : thr;
}


////////////////////////////
//colugo parametes update
void colugoTransHelper::parameters_update(){
//colugo debug
	int32_t d;
	param_get(_params_handles_colugo._param_c_debug, &d);
	_params_colugo._param_c_debug = d;

	/*params for transition from mc to fw*/
	float v;
	param_get(_params_handles_colugo._param_c_wafp, &v);
	_params_colugo._param_c_wafp = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_wasp, &v);
	_params_colugo._param_c_wasp = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_pi_fp, &v);
	_params_colugo._param_c_pi_fp = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_pi_sp, &v);
	_params_colugo._param_c_pi_sp = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_fl_fp, &v);
	_params_colugo._param_c_fl_fp = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_fl_sp, &v);
	_params_colugo._param_c_fl_sp = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_fl_mc_pos, &v);
	_params_colugo._param_c_fl_mc_pos = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_pi_mc_pos, &v);
	_params_colugo._param_c_pi_mc_pos = math::constrain(v, -1.0f, 1.0f);

	param_get(_params_handles_colugo._param_c_tm_to_pos1, &v);
	_params_colugo._param_c_tm_to_pos1 = math::constrain(v, 0.0f, 100.0f);

	param_get(_params_handles_colugo._param_c_tm_to_col_pos1, &v);
	_params_colugo._param_c_tm_to_col_pos1 = math::constrain(v, 0.0f, 100.0f);

	param_get(_params_handles_colugo._param_c_tm_to_pos2, &v);
	_params_colugo._param_c_tm_to_pos2 = math::constrain(v, 0.0f, 100.0f);

	param_get(_params_handles_colugo._param_c_tr_fw_srv_slew, &v);
	_params_colugo._param_c_tr_fw_srv_slew = math::constrain(v, 0.0f, 10.0f);
/////////////////////////
}

