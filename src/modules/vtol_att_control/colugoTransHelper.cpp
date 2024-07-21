#include "colugoTransHelper.h"
#include <mathlib/math/Limits.hpp>

colugoTransHelper::colugoTransHelper()
{
    // Constructor code here
    _params_handles_colugo._param_c_debug = param_find("C_DEBUG");
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

    _params_handles_colugo._param_c_pwm_main_mc = param_find("C_PWM_MAIN_MC");
    _params_handles_colugo._param_c_pwm_aux_mc  = param_find("C_PWM_AUX_MC");
    _params_handles_colugo._param_c_pwm_main_fw = param_find("C_PWM_MAIN_FW");
    _params_handles_colugo._param_c_pwm_aux_fw  = param_find("C_PWM_AUX_FW");
    _pwmMainRev = param_find("PWM_MAIN_REV");
    _pwmAuxRev  = param_find("PWM_AUX_REV");
}


bool colugoTransHelper::delayAfterMcReached()
{
    if (_MCstarted == 0)
    {
        return false;
    }
    return ((hrt_absolute_time() - _MCstarted) * 1e-6f) < 1.0;
}
//@note colugoTransHelper
void colugoTransHelper::setColugoActuatorPos()
{
    float res = COLUGO_ACTUATOR_MC_POS;
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_SEMI_LOCK_POS)
    {
        res = _params_colugo._param_c_wafp;
    }
    if (_transStage >= COLUGO_FW_VTRANS_STAGE::VTRANS_REACHED_LOCK_SPEED || delayAfterMcReached())
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

void colugoTransHelper::updateColugoTransitionState(float airSpd, vtol_mode vtolFlightMode, hrt_abstime absTime)
{
    _airspeed = airSpd;
    switch (vtolFlightMode)
    {
    case vtol_mode::FW_MODE:
        // reset ailrons direction only once
        if (vtolFlightMode != _currentMode)
        {
           setServosBitmaskToFW();
        }

        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;
    case vtol_mode::PRE_TRANSITION_TO_FW:
    case vtol_mode::TRANSITION_TO_FW:
        _MCstarted = 0; // reset
        if (vtolFlightMode != _currentMode)
        { // we just entered tansition...
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START;
            // start counter for inner stage states
            _toFwStartTime = absTime;
            break;
        }
        else
        { // we are in trasition...
            updateInnerStage();
        }
        break;

    case vtol_mode::TRANSITION_TO_MC:
        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        //maube change surfaces position only after reducing - speed - ask amit...
        // saftey measure make sure back to correct position when goes back to MC for safety reasons
        if (vtolFlightMode != _currentMode)
        {
            setServosBitmaskToMC();
        }

        break;

    case vtol_mode::MC_MODE:
        // saftey measure make sure back to correct position when goes back to MC for safety reasons
        if (vtolFlightMode != _currentMode)
        {
            _MCstarted = hrt_absolute_time();
        }
        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;

    default:
        break;
    }
    _currentMode = vtolFlightMode;
}
void colugoTransHelper::setServosBitmaskToMC(){
    setParamValue(_pwmMainRev, _params_colugo._param_c_pwm_main_mc);
    setParamValue(_pwmAuxRev, _params_colugo._param_c_pwm_aux_mc);

}

void colugoTransHelper::setServosBitmaskToFW(){
    setParamValue(_pwmMainRev, _params_colugo._param_c_pwm_main_fw);
    setParamValue(_pwmAuxRev, _params_colugo._param_c_pwm_aux_fw);

}

void colugoTransHelper::updateInnerStage()
{
    uint64_t debugDiff = (hrt_absolute_time() - _toFwStartTime);
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
    int32_t iPrm;
    param_get(_params_handles_colugo._param_c_debug, &iPrm);
    _params_colugo._param_c_debug = iPrm;

    param_get(_params_handles_colugo._param_c_pwm_main_mc, &iPrm);
    _params_colugo._param_c_pwm_main_mc = math::constrain(iPrm, (int32_t)0, C_MAX_MAIN_BITMASK);
    param_get(_params_handles_colugo._param_c_pwm_aux_mc, &iPrm);
    _params_colugo._param_c_pwm_aux_mc = math::constrain(iPrm, (int32_t)0, C_MAX_AUX_BITMASK);
    param_get(_params_handles_colugo._param_c_pwm_main_fw, &iPrm);
    _params_colugo._param_c_pwm_main_fw = math::constrain(iPrm, (int32_t)0, C_MAX_MAIN_BITMASK);
    param_get(_params_handles_colugo._param_c_pwm_aux_fw, &iPrm);
    _params_colugo._param_c_pwm_aux_fw = math::constrain(iPrm, (int32_t)0, C_MAX_AUX_BITMASK);

    float fPrm;
    param_get(_params_handles_colugo._param_c_wafp, &fPrm);
    _params_colugo._param_c_wafp = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_wasp, &fPrm);
    _params_colugo._param_c_wasp = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_pi_fp, &fPrm);
    _params_colugo._param_c_pi_fp = math::constrain(fPrm, -C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_pi_sp, &fPrm);
    _params_colugo._param_c_pi_sp = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_fl_fp, &fPrm);
    _params_colugo._param_c_fl_fp = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_fl_sp, &fPrm);
    _params_colugo._param_c_fl_sp = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_fl_mc_pos, &fPrm);
    _params_colugo._param_c_fl_mc_pos = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_pi_mc_pos, &fPrm);
    _params_colugo._param_c_pi_mc_pos = math::constrain(fPrm, C_MIN_SURFACE_RANGE, C_MAX_SURFACE_RANGE);

    param_get(_params_handles_colugo._param_c_tr_fw_srv_slew, &fPrm);
    _params_colugo._param_c_tr_fw_srv_slew = math::constrain(fPrm, 0.0f, 10.0f);

    param_get(_params_handles_colugo._param_c_z_tr_spd_ms, &fPrm);
    _params_colugo._param_c_z_tr_spd_ms = math::constrain(fPrm, 0.0f, 100.0f);

    param_get(_params_handles_colugo._param_c_z_tr_time_s, &fPrm);
    _params_colugo._param_c_z_tr_time_s = math::constrain(fPrm, 0.0f, 100.0f);

    param_get(_params_handles_colugo._param_c_z_lck_tming, &fPrm);
    _params_colugo._param_c_z_lck_tming = math::constrain(fPrm, 0.0f, 100.0f);

    param_get(_params_handles_colugo._param_airspeed_blend, &fPrm);
    _params_colugo._param_airspeed_blend = math::constrain(fPrm, 0.0f, 100.0f);
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

/**
 * @brief
 * need to set ailerons positions slowly.. and gently so the wing will gradualy will go up...when we are trasnitioning to FW
 *
 */
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



void colugoTransHelper::setParamValue(param_t prm, int32_t val)
{
    int32_t curval;
    if (prm != PARAM_INVALID)
    {
        param_get(prm, &curval);
        if (curval != val)
        {
            param_set(prm, &val);
        }
    }
}

