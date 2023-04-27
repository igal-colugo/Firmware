#include "colugoTransHelper.h"

colugoTransHelper::colugoTransHelper() {
    // Constructor code here
}
//@note colugoTransHelper
void colugoTransHelper::setColugoActuatorPos(float newVal) {
	_wingLockActuatorPos = newVal;
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
