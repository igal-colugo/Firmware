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

    _params_handles_colugo._param_c_tr_srv_rev_no = param_find("C_TR_SRV_REV_NO");

    _params_handles_colugo._param_c_debug = param_find("C_DEBUG");
}

bool colugoTransHelper::delayAfterMcReached(){
    if(_MCstarted == 0){
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
	//reset ailrons direction only once
        if (fm != _currentMode){
            setAsAilerons();
        }

        _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
        break;
    case vtol_mode::PRE_TRANSITION_TO_FW:
    case vtol_mode::TRANSITION_TO_FW:
        _MCstarted = 0;//reset
        if (fm != _currentMode)
        { // we just entered tansition...
            _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_VERTICAL_START;

	    //make both aileorns go the same direction...
	        setAsElevator();
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
        //saftey measure make sure back to correct position when goes back to MC for safety reasons
        if (fm != _currentMode){
            _MCstarted = hrt_absolute_time();
            setAsElevator();
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
void colugoTransHelper::updateOnLAndOrTakeoff(){
     /* Update land detector */
        if (_vehicle_land_detected_sub.updated())
        {
            const bool was_landed = _vehicle_land_detected.landed;

             _vehicle_land_detected_sub.copy(&_vehicle_land_detected);
             //landed
             if(!was_landed && _vehicle_land_detected.landed){
                //we just landed - revert to FW functions
                setAsAilerons();
             }
             //we are airborne read fucntions.. and - put all servos to MC state
             else if (was_landed && !_vehicle_land_detected.landed){
               // findAileronFuncs();
               setAsElevator();
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
    param_get(_params_handles_colugo._param_c_tr_srv_rev_no, &d);
    //make sure its in correct
    _params_colugo._param_c_tr_srv_rev_no = 0 <= d <= 16 ? d : 0;//math::constrain(d, 0, 16);

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
    findAileronFuncs();
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

void colugoTransHelper::findAileronFuncs()
{
    //call once, before changes are made programatically
    if(!_registeredFuncs){
            for (int i = 0; i < ActuatorEffectivenessControlSurfaces::MAX_COUNT; ++i) {
                char buffer[17];
                snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", i);
                int32_t type;
                param_get(param_find(buffer), &type);
                if(type == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::LeftAileron)){
                    _ailerons_tr_colugo._leftAileronCsTypeNo = i;
                }
                if(type == static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::RightAileron)){
                    _ailerons_tr_colugo._rightAileronCsTypeNo = i;
                }

            }
            _registeredFuncs = true;
    }

    /*
    _servo_tr_to_reverse_colugo._servo_to_reverse_during_tr = 0;
    if(_params_colugo._param_c_tr_srv_rev_no > 0){
        _servo_tr_to_reverse_colugo._servo_to_reverse_during_tr = _params_colugo._param_c_tr_srv_rev_no;
        //this mneans its an AUX servo... 9~14 (16 in cube+)
        if(_servo_tr_to_reverse_colugo._servo_to_reverse_during_tr > 8){
            param_get(param_find("PWM_AUX_REV"), &_servo_tr_to_reverse_colugo._originalVal);
          //  _servo_tr_to_reverse_colugo._originalVal = reverseMask;

        }
        //this means its main servo 1~8
        else{
          //  bitNo = bitNo << (_servo_tr_to_reverse_colugo._servo_to_reverse_during_tr - 1);
            param_get(param_find("PWM_MAIN_REV"), &_servo_tr_to_reverse_colugo._originalVal);
         //   _servo_tr_to_reverse_colugo._originalVal = (reverseMask  & bitNo);
        }
    }
    */

}

void colugoTransHelper::setAsElevator(){
    if(_registeredFuncs){
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", _ailerons_tr_colugo._leftAileronCsTypeNo);
        int32_t val = static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::Elevator);
        param_set(param_find(buffer), &val);
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", _ailerons_tr_colugo._rightAileronCsTypeNo);
      //  val = static_cast<uint8_t>(ActuatorEffectivenessControlSurfaces::Type::Eleהכלי אצלך?
        param_set(param_find(buffer), &val);
    }


   /*
    int32_t newVal;
    if(_servo_tr_to_reverse_colugo._servo_to_reverse_during_tr > 8){
        newVal = _servo_tr_to_reverse_colugo._originalVal ^ (1 << (_servo_tr_to_reverse_colugo._servo_to_reverse_during_tr - 9));
        param_set(param_find("PWM_AUX_REV"), &newVal);

    }
    else{
        newVal = _servo_tr_to_reverse_colugo._originalVal ^ (1 << (_servo_tr_to_reverse_colugo._servo_to_reverse_during_tr - 1));
        param_set(param_find("PWM_MAIN_REV"), &newVal);
    }
    */
}
void colugoTransHelper::setAsAilerons(){
    if(_registeredFuncs){
        char buffer[17];
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", _ailerons_tr_colugo._leftAileronCsTypeNo);
        int32_t val = static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::LeftAileron);
        param_set(param_find(buffer), &val);
        snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TYPE", _ailerons_tr_colugo._rightAileronCsTypeNo);
        val = static_cast<int32_t>(ActuatorEffectivenessControlSurfaces::Type::RightAileron);
        param_set(param_find(buffer), &val);

    }

    /*
    if(_servo_tr_to_reverse_colugo._servo_to_reverse_during_tr > 8){
        param_set(param_find("PWM_AUX_REV"), &_servo_tr_to_reverse_colugo._originalVal);
    }
    else{
        param_set(param_find("PWM_MAIN_REV"), &_servo_tr_to_reverse_colugo._originalVal);

    }
    */
}
