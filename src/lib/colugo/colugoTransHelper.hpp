#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/colugo_actuator.h>

static const float CST_TRANSITION_VS_M_S = -4.0;
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
        VTRANS_FARWARD_START,
		VTRANS_REACHED_LOCK_SPEED,
		VTRANS_ALLOW_FW
	};


class colugoTransHelper {
public:
    colugoTransHelper();
    void setColugoActuatorPos(float newVal);
    void publishColugoActuator();

private:
        //position of the wing lock actuator - range: -1.0 ~ 1.0
    float _wingLockActuatorPos = COLUGO_ACTUATOR_MC_POS;
    uORB::Publication<colugo_actuator_s> _colugo_actuator_pub{ORB_ID(colugo_actuator)};
};
