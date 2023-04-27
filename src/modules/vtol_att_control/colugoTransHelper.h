#pragma once

#include "vtol_type.h"
#include <uORB/Publication.hpp>
#include <uORB/topics/colugo_actuator.h>
#include <uORB/topics/colugo_transition.h>

static const float CST_TRANSITION_VS_M_S = -2.6;

static const float CST_VERTICAL_TRANS_S = 10.0;//VERTICAL TRASITION TIME IN SECONDS
static const float CST_LOCK_AIRSPEED = 9.0;//horizontal airspeed for final lock of the wing
static const float CST_LOCKING_TIME_S = 1.2;//allowed time in seconds for the lock pin to reach final postion

static const float COLUGO_ACTUATOR_MC_POS{-1.0f};


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
enum class COLUGO_FW_VTRANS_STAGE{
		VTRANS_IDLE = 0,
		VTRANS_VERTICAL_START,
		VTRANS_REACHED_SEMI_LOCK_POS,
        	VTRANS_FARWARD_START,
		VTRANS_REACHED_LOCK_SPEED,
		VTRANS_ALLOW_FW
	};


class colugoTransHelper {
public:

    colugoTransHelper();

    void setColugoActuatorPos(float newVal);
    void publishColugoActuator();
    void updateColugoTransitionState(float airSpd, mode fm, hrt_abstime tt);
    float getPusherThr(float thr);
    COLUGO_FW_VTRANS_STAGE getInnerState(){return _transStage;};

private:
        //position of the wing lock actuator - range: -1.0 ~ 1.0
    float _wingLockActuatorPos = COLUGO_ACTUATOR_MC_POS;
    uORB::Publication<colugo_actuator_s> _colugo_actuator_pub{ORB_ID(colugo_actuator)};
    uORB::Publication<colugo_transition_s> _colugo_transition_pub{ORB_ID(colugo_transition)};
    mode _currentMode;
    hrt_abstime _startTime, _reachedLockSpeedTime;
    COLUGO_FW_VTRANS_STAGE _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
    float _airspeed;

	//methods
    void updateInnerStage();
};
