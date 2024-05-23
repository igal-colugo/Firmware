#include "colugoTransHelper.h"
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

    _params_handles_colugo._param_c_srv_to_rev = param_find("C_SRV_TO_REV");

    _params_handles_colugo._param_c_debug = param_find("C_DEBUG");
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

void colugoTransHelper::updateColugoTransitionState(float airSpd, vtol_mode fm, hrt_abstime tt)
{
    _airspeed = airSpd;
    switch (fm)
    {
    case vtol_mode::FW_MODE:
        // reset ailrons direction only once
        if (fm != _currentMode)
        {
            setAsAileronsAndElevator();
        }

        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;
    case vtol_mode::PRE_TRANSITION_TO_FW:
    case vtol_mode::TRANSITION_TO_FW:
        _MCstarted = 0; // reset
        if (fm != _currentMode)
        { // we just entered tansition...
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START;

            // make both aileorns go the same direction...
           // setAsLeftFlap();
            _toFwStartTime = tt;
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
        // saftey measure make sure back to correct position when goes back to MC for safety reasons
        if (fm != _currentMode)
        {
            _MCstarted = hrt_absolute_time();
            setAsLeftFlap();
        }
        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;

    default:
        break;
    }
    _currentMode = fm;
    updateOnLAndOrTakeoff();
}
/**
 * @brief
 * need to go to "MC" postions and functions of servos on ARM, and go back to FW postions and functions when disarmed..
 *
 */
void colugoTransHelper::updateOnLAndOrTakeoff()
{
    /* Update land detector */
    if (_vehicle_land_detected_sub.updated())
    {
        const bool was_landed = _vehicle_land_detected.landed;

        _vehicle_land_detected_sub.copy(&_vehicle_land_detected);
        // landed
        if (!was_landed && _vehicle_land_detected.landed)
        {
            // we just landed - revert to FW functions
            setAsAileronsAndElevator();
        }
        // we are airborne read fucntions.. and - put all servos to MC state
        else if (was_landed && !_vehicle_land_detected.landed)
        {
            // findAileronFuncs();
            setAsLeftFlap();
        }
    }
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
    int32_t d;
    param_get(_params_handles_colugo._param_c_debug, &d);
    _params_colugo._param_c_debug = d;

    /*params for transition from mc to fw*/
    param_get(_params_handles_colugo._param_c_srv_to_rev, &d);
    // make sure its in correct
    _params_colugo._param_c_srv_to_rev = 0 <= d <= 14 ? d : 0; // math::constrain(d, 0, 16);
    updateServoToReverseDuringMC();
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
    findAileronAndElevFuncs();
}

void colugoTransHelper::updateServoToReverseDuringMC(){
     uint16_t itoShL = _params_colugo._param_c_srv_to_rev <= 8 ? _params_colugo._param_c_srv_to_rev : _params_colugo._param_c_srv_to_rev - 8;

    _bitMaskForReverseSrvInMC = _params_colugo._param_c_srv_to_rev == 0 ? 0 : 1 << (itoShL - 1);
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

void colugoTransHelper::findAileronAndElevFuncs()
{
    // call once, before changes are made programatically
    if (!_registeredFuncs)
    {
        char buffer[17];
        int32_t ctrlType;
        for (int i = 0; i < ActuatorEffectivenessControlSurfaces::MAX_COUNT; ++i)
        {
            snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", i);
            param_get(param_find(buffer), &ctrlType);

            if (ctrlType == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::LeftAileron))
            {
                // take only the first one...
                if (_flightsurfaces_tr_colugo._leftAileronCsTypeNo < 0)
                {
                    _flightsurfaces_tr_colugo._leftAileronCsTypeNo = i;
                }
            }
            if (ctrlType == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::RightAileron))
            {
                if (_flightsurfaces_tr_colugo._rightAileronCsTypeNo)
                {
                    _flightsurfaces_tr_colugo._rightAileronCsTypeNo = i;
                }
            }
            if (ctrlType == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::Elevator))
            {
                if (_flightsurfaces_tr_colugo._elevatorCsTypeNo)
                {
                    _flightsurfaces_tr_colugo._elevatorCsTypeNo = i;
                }
            }
        }

        //get servo reverse bit mask
        getReverseServoBitmask();

        _registeredFuncs = true;
    }
}

void colugoTransHelper::setAsLeftFlap()
{
    if (_registeredFuncs)
    {
        setSurfaceType(ActuatorEffectivenessControlSurfaces::Type::LeftFlaps, _flightsurfaces_tr_colugo._leftAileronCsTypeNo, "R", 0.0f);
        setSurfaceType(ActuatorEffectivenessControlSurfaces::Type::LeftFlaps, _flightsurfaces_tr_colugo._rightAileronCsTypeNo, "R", 0.0f);
        setSurfaceType(ActuatorEffectivenessControlSurfaces::Type::LeftFlaps, _flightsurfaces_tr_colugo._elevatorCsTypeNo, "P", 0.0f);
        uint32_t val = _originpwmRevVal ^ _bitMaskForReverseSrvInMC;
        setReverseServoBitmask(val);
    }
}

void colugoTransHelper::setReverseServoBitmask(int32_t val){
    if(_params_colugo._param_c_srv_to_rev > 0){
        if(_pwmrev_param != PARAM_INVALID){
            int32_t curval;
            param_get(_pwmrev_param, &curval);
            if(curval != val){
                param_set(_pwmrev_param, &val);
            }
        }
    }
}

void colugoTransHelper::getReverseServoBitmask(){
    if(_params_colugo._param_c_srv_to_rev > 0){
        char sPrm[14];
        snprintf(sPrm, sizeof(sPrm), "PWM_MAIN_REV");
        if(_params_colugo._param_c_srv_to_rev > 8)
            snprintf(sPrm, sizeof(sPrm), "PWM_AUX_REV");

        _pwmrev_param = param_find(sPrm);
        if(_pwmrev_param != PARAM_INVALID){
            param_get(_pwmrev_param, &_originpwmRevVal);
            //pwmRev = pwmRev ^ _bitMaskForReverseSrvInMC;
            //param_set(prm, &pwmRev);
        }
    }
}


void colugoTransHelper::setAsAileronsAndElevator()
{
    if (_registeredFuncs)
    {
        setSurfaceType(ActuatorEffectivenessControlSurfaces::Type::LeftAileron, _flightsurfaces_tr_colugo._leftAileronCsTypeNo, "R", -0.5f);
        setSurfaceType(ActuatorEffectivenessControlSurfaces::Type::RightAileron, _flightsurfaces_tr_colugo._rightAileronCsTypeNo, "R", 0.5f);
        setSurfaceType(ActuatorEffectivenessControlSurfaces::Type::Elevator, _flightsurfaces_tr_colugo._elevatorCsTypeNo, "P", 1.0f);
        setReverseServoBitmask(_originpwmRevVal);
    }
}



 void colugoTransHelper::setSurfaceType(ActuatorEffectivenessControlSurfaces::Type srface, int32_t srvNo, char torqType[], float torqVal){
        int32_t iSurface = static_cast<int32_t>(srface);
        if (srvNo >= 0){
            char buffer[22];
            snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", srvNo);

            int32_t iCurrVal;
            param_get(param_find(buffer), &iCurrVal);

            //update only if needed...
            if(iCurrVal != iSurface){
                param_set(param_find(buffer), &iSurface);
                snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRQ_%s", srvNo, torqType);
                param_set(param_find(buffer), &torqVal);

            }
        }
    }
