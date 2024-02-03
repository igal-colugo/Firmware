#include "colugoTransHelper.h"

#include "control_allocator/ActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.hpp"
#include <mathlib/math/Limits.hpp>

colugoTransHelper::colugoTransHelper()
{
    // Constructor code here
    _params_handles_colugo._param_c_wafp = param_find("C_WAFP");
    _params_handles_colugo._param_c_wasp = param_find("C_WASP");
    _params_handles_colugo._param_c_pi_fp = param_find("C_PI_FP");
    _params_handles_colugo._param_c_pi_sp = param_find("C_PI_SP");
    _params_handles_colugo._param_c_pi_mc_pos = param_find("C_PI_MC_POS");

    _params_handles_colugo._param_c_fl_fp = param_find("C_FL_FP");
    _params_handles_colugo._param_c_fl_sp = param_find("C_FL_SP");
    _params_handles_colugo._param_c_fl_mc_pos = param_find("C_FL_MC_POS");

    _params_handles_colugo._param_c_tr_fw_srv_slew = param_find("C_TR_FW_SRV_SLEW");
    _params_handles_colugo._param_airspeed_blend = param_find("VT_ARSP_BLEND");

    _params_handles_colugo._param_c_z_tr_spd_ms = param_find("C_Z_TR_SPD_MS");
    _params_handles_colugo._param_c_z_tr_time_s = param_find("C_Z_TR_TIME_S");
    _params_handles_colugo._param_c_z_lck_tming = param_find("C_Z_LCK_TMING");

    _params_handles_colugo._param_c_debug = param_find("C_DEBUG");

    // find left and right aileron surface
    findAileronServosTorqParam();
}
//@note colugoTransHelper
void colugoTransHelper::setColugoActuatorPos()
{
    float res = COLUGO_ACTUATOR_MC_POS;
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS)
    {
        res = _params_colugo._param_c_wafp;
    }
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED)
    {
        res = _params_colugo._param_c_wasp;
    }
    _wingLockActuatorPos = res;
}
void colugoTransHelper::lockColugoActuator()
{
    if (fabsf(_wingLockActuatorPos - _params_colugo._param_c_wasp) > FLT_EPSILON)
    {
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

void colugoTransHelper::updateColugoTransitionState(float airSpd, vtol_mode fm, hrt_abstime tt)
{
    _airspeed = airSpd;
    switch (fm)
    {
    case vtol_mode::FW_MODE:
	//reset ailrons direction
        if (fm != _currentMode){
            param_set(_param_ailron_l_r, &_ailron_l_r);
        }

        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;
    case vtol_mode::PRE_TRANSITION_TO_FW:
    case vtol_mode::TRANSITION_TO_FW:
        if (fm != _currentMode)
        { // we just entered tansition...
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START;

	    //make both aileorns go t same direction...
	    float opsitDir = -_ailron_l_r;
	    param_set(_param_ailron_l_r, &opsitDir);
            _startTime = tt;
            break;
        }
        else
        { // we are in trasition...
            updateInnerStage();
        }
        break;
    case vtol_mode::TRANSITION_TO_MC:
        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;
    case vtol_mode::MC_MODE:
        //saftey measure makesure back to correct position when goes back for safty reasons
        if (fm != _currentMode){
            param_set(_param_ailron_l_r, &_ailron_l_r);
        }
        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;

    default:
        break;
    }
    _currentMode = fm;
}
void colugoTransHelper::updateInnerStage()
{
    uint64_t debugDiff = (hrt_absolute_time() - _startTime);
    switch (_transStage)
    {
    case COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START:
        if (debugDiff > ((_params_colugo._param_c_z_tr_time_s * _params_colugo._param_c_z_lck_tming * 0.01f) * 1000000))
        {
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS;
        }
        break;
    case COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS:
        if (debugDiff > (_params_colugo._param_c_z_tr_time_s * 1000000))
        {
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START;
            _FarwardStageStartTime = hrt_absolute_time();
        }
        break;
    case COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START:
        // speed is high enough to cuople wing to colugo pin...
        if (_airspeed > _params_colugo._param_airspeed_blend
            // past enough time for movment of flaps and elevator to final postion for lock
            && ((hrt_absolute_time() - _FarwardStageStartTime) * 1e-6f) > (getColugoTrFwSrvSlew() * 1.1f))
        {
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED;
            _reachedLockSpeedTime = hrt_absolute_time();
        }
        break;
    case COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED:
        // give the pin some time to move to final position...
        if (hrt_absolute_time() - _reachedLockSpeedTime > (CST_LOCKING_TIME_S * 1000000))
        {
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_ALLOW_FW;
        }
        break;
    case COLUGO_FW_VTRANS_STAGE::VTRANS_ALLOW_FW:
    case COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE:
        break;
    }

    // publish only if not idel...
    if (COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE != _transStage)
    {
        colugo_transition_s colugo_trans{};
        colugo_trans.transition_state = static_cast<uint8_t>(_transStage);
        colugo_trans.vz = -_params_colugo._param_c_z_tr_spd_ms;
        colugo_trans.timestamp = hrt_absolute_time();
        _colugo_transition_pub.publish(colugo_trans);
    }
}

// while we are in vertical lift stage - dont push farward...
float colugoTransHelper::getPusherThr(float thr)
{
    return COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START <= _transStage ? thr : 0;
}

////////////////////////////
// colugo parametes update
void colugoTransHelper::parameters_update()
{
    // colugo debug
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

    param_get(_params_handles_colugo._param_c_tr_fw_srv_slew, &v);
    _params_colugo._param_c_tr_fw_srv_slew = math::constrain(v, 0.0f, 10.0f);

    param_get(_params_handles_colugo._param_c_z_tr_spd_ms, &v);
    _params_colugo._param_c_z_tr_spd_ms = math::constrain(v, 0.0f, 100.0f);

    param_get(_params_handles_colugo._param_c_z_tr_time_s, &v);
    _params_colugo._param_c_z_tr_time_s = math::constrain(v, 0.0f, 100.0f);

    param_get(_params_handles_colugo._param_c_z_lck_tming, &v);
    _params_colugo._param_c_z_lck_tming = math::constrain(v, 0.0f, 100.0f);

    param_get(_params_handles_colugo._param_airspeed_blend, &v);
    _params_colugo._param_airspeed_blend = math::constrain(v, 0.0f, 100.0f);
    /////////////////////////
    findAileronServosTorqParam();
}

float colugoTransHelper::getColugoTransToFwSlewedPitch()
{
    float res = getColugoPiMcPos(); // _params_colugo._param_c_pi_mc_pos;
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START)
    {
        res = getSlewedPosition(getColugoPiMcPos(), // _params_colugo._param_c_pi_mc_pos,
                                getColugoPiFp()     //_params_colugo._param_c_pi_fp
        );
    }
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED)
    {
        res = getColugoPiSp(); // _params_colugo._param_c_pi_sp;
    }
    return res;
}
float colugoTransHelper::getColugoTransToFwSlewedFlaps()
{
    float res = getColugoFlapsMcPos();
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_FARWARD_START)
    {
        res = getSlewedPosition(getColugoFlapsMcPos(), getColugoFlapsFrstPos());
    }
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED)
    {
        res = getColugoFlapsScndPos(); //_params_colugo._param_c_fl_sp;
    }
    return res;
}

float colugoTransHelper::getSlewedPosition(float startPos, float endPos)
{
    float res = endPos;
    // that means we want some kind of slew...
    if (getColugoTrFwSrvSlew() > 0)
    {

        // range...
        float fullRange = endPos - startPos;

        float partialTime = (hrt_absolute_time() - _FarwardStageStartTime) * 1e-6f;

        float progress = (partialTime / getColugoTrFwSrvSlew());

        progress = math::constrain(progress, 0.0f, 1.0f);
        float servRelativePos = fullRange * progress;
        res = startPos + servRelativePos;
    }
    return res;
}

void colugoTransHelper::findAileronServosTorqParam()
{
    for (int i = 0; i < ActuatorEffectivenessControlSurfaces::MAX_COUNT; ++i)
    {
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", i);
        param_t aileronCandidate = param_find(buffer);
        if (aileronCandidate != PARAM_INVALID)
        {
            int32_t type;
            param_get(aileronCandidate, &type);
	    if(type == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::LeftAileron)){
		char trqBuffer[17];
		snprintf(trqBuffer, sizeof(trqBuffer), "CA_SV_CS%u_TRQ_R", i);
		_param_ailron_l_r = param_find(trqBuffer);
		param_get(_param_ailron_l_r, &_ailron_l_r);
	    }
	    else if(type == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::RightAileron)){
		//talk with amit - myabe do hewre somthing...

	    }
        }
    }
}
