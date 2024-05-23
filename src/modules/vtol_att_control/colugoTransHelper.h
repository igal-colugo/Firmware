#pragma once

#include "vtol_type.h"
#include <uORB/Publication.hpp>
#include <uORB/topics/colugo_actuator.h>
#include <uORB/topics/colugo_transition.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_land_detected.h>
#include "control_allocator/ActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.hpp"
//#include <uORB/topics/vehicle_command.h>

//static const float CST_TRANSITION_VS_M_S = -2.6;

//static const float CST_VERTICAL_TRANS_S = 10.0;//VERTICAL TRASITION TIME IN SECONDS
//static const float CST_LOCK_AIRSPEED = 9.0;//horizontal airspeed for final lock of the wing
static const float CST_LOCKING_TIME_S = 1.2;//allowed time in seconds for the lock pin to reach final postion

static const float COLUGO_ACTUATOR_MC_POS{-1.0f};

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
    float getColugoFlapsMcPos(){return _params_colugo._param_c_fl_mc_pos;}
    float getColugoFlapsFrstPos(){return _params_colugo._param_c_fl_fp;}
    float getColugoFlapsScndPos(){return _params_colugo._param_c_fl_sp;}
    float getColugoLockTransToFwFrstPos(){return _params_colugo._param_c_wafp;}
    float getColugoLockTransToFwScndPos(){return _params_colugo._param_c_wasp;}
    void lockColugoActuator();
    float getColugoTransToFwSlewedPitch();
    float getColugoTransToFwSlewedFlaps();



private:

	struct {
		int32_t _param_c_debug;
		float 	_param_c_wafp;	   //colugo pin (wing actuator) first position
		float 	_param_c_wasp;	   //colugo pin (wing actuator) second position
		float 	_param_c_pi_fp;    //pitch servo first position during transition to FW mode
		float 	_param_c_pi_sp;	   //pitch servo second position during transition to FW mode
		float	_param_c_pi_mc_pos;//pitch servo position during MC mode
		float 	_param_c_fl_fp;
		float 	_param_c_fl_sp;
		float	_param_c_fl_mc_pos;
		float _param_c_tr_fw_srv_slew;
		float _param_airspeed_blend;
		float _param_c_z_tr_spd_ms;
		float _param_c_z_tr_time_s;
		float _param_c_z_lck_tming;
		int32_t _param_c_tr_srv_rev_no;

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
		param_t _param_c_tr_fw_srv_slew;
		param_t _param_airspeed_blend;
		param_t _param_c_z_tr_spd_ms;
		param_t _param_c_z_tr_time_s;
		param_t _param_c_z_lck_tming;
		param_t _param_c_tr_srv_rev_no;

	} _params_handles_colugo;

struct {
		int32_t _leftAileronCsTypeNo  = -1;
		int32_t _rightAileronCsTypeNo = -1;
		int32_t _elevatorCsTypeNo     = -1;
	} _flightsurfaces_tr_colugo;

	// Subscribers
	//uORB::Subscription _command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

        //position of the wing lock actuator - range: -1.0 ~ 1.0
    float _wingLockActuatorPos = COLUGO_ACTUATOR_MC_POS;
    uORB::Publication<colugo_actuator_s> _colugo_actuator_pub{ORB_ID(colugo_actuator)};
    uORB::Publication<colugo_transition_s> _colugo_transition_pub{ORB_ID(colugo_transition)};
    vtol_mode _currentMode;
    hrt_abstime _toFwStartTime = 0, _reachedLockSpeedTime = 0, _FarwardStageStartTime = 0, _MCstarted = 0;
    COLUGO_FW_VTRANS_STAGE _transStage = COLUGO_FW_VTRANS_STAGE::VTRANS_IDLE;
    float _airspeed;
    vehicle_land_detected_s _vehicle_land_detected{};

	//this flag to mark that registered the fictions at list once
    bool _registeredFuncs = false;


	//methods
    void updateInnerStage();

    /**
 * @brief
 * need to set ailerons positions slowly.. and gently so the wing will gradualy will go up...when we are trasnitioning to FW
 *
 */
    float getSlewedPosition(float startPos, float endPos);
    void findAileronAndElevFuncs();
    void setAsLeftFlap();
    void setAsAileronsAndElevator();
    //returns true when we are in MC mode for less than 1 sec
    bool delayAfterMcReached();
    void setSurfaceType(ActuatorEffectivenessControlSurfaces::Type srface, int32_t srvNo, char torqType[], float torqVal);

    /**
 * @brief
 * need to go to "MC" postions and functions of servos on ARM, and go back to FW postions and functions when disarmed..
 *
 */
    void updateOnLAndOrTakeoff();

};
