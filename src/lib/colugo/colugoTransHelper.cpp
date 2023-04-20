#include "colugoTransHelper.hpp"

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
