/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include<uORB/topics/vehicle_status.h>

#include "diagnostic.h"
#include "fail_handler.h"
#include <list>

using namespace time_literals;

extern "C" __EXPORT int colugo_failsafe_module_main(int argc, char *argv[]);

class ColugoFailsafeModule : public ModuleBase<ColugoFailsafeModule>, public ModuleParams
{
    friend GPSFailHandler;

  public:
    struct __attribute__((packed, aligned(1))) ColugoFailsafeModuleParameters
    {
        int8_t system_monitor_enable;
        int8_t system_monitor_debug_enable;
        int32_t system_monitor_dual_fail_taking_care;
        int32_t system_monitor_gps_taking_care;
        int32_t system_monitor_gps_grace_time;
        int32_t system_monitor_gps_fail_grace_time;
        int32_t system_monitor_gps_min_satellites;
        int32_t system_monitor_gps_hdop;
        int32_t system_monitor_gps_status;
        float system_monitor_gps_distance;
        float system_monitor_gps_velocity_variance;
        int32_t system_monitor_communication_grace_time;
        int32_t system_monitor_communication_fail_grace_time;
        int32_t system_monitor_communication_lost_time;
        int32_t system_monitor_communication_taking_care;
    } parameters;

    std::list<FailHandler *> list_handlers;
    Diagnostic diagnostic_status;

    struct vehicle_status_s vehicle_status;

    ColugoFailsafeModule(int example_param, bool example_flag);

    virtual ~ColugoFailsafeModule() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static ColugoFailsafeModule *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

    bool check_system();

    bool update_gps_parameters();

    bool update_handler(FailHandler *handler);

    bool gps_fail_handler();

    bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN, const float param3 = NAN, const float param4 = NAN,
                              const double param5 = static_cast<double>(NAN), const double param6 = static_cast<double>(NAN), const float param7 = NAN);

  private:
    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_update(bool force = false);

    DEFINE_PARAMETERS((ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,                  /**< example parameter */
                      (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,                /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_EN>) _param_col_fail_en,                        /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_GPS_TC>) _param_col_gps_taking_care,            /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_GPS_GT>) _param_col_gps_grace_time,             /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_GPS_FGT>) _param_col_gps_fail_grace_time,       /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_GPS_MINS>) _param_col_gps_min_satellites,       /**< another parameter */
                      (ParamFloat<px4::params::C_FAIL_GPS_DST>) _param_col_gps_distance,            /**< another parameter */
                      (ParamFloat<px4::params::C_FAIL_GPS_VVAR>) _param_col_gps_velocity_variance,  /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_GPS_HDOP>) _param_col_gps_hdop,                 /**< another parameter */
                      (ParamInt<px4::params::C_FAIL_GPS_STAT>) _param_col_gps_status                /**< another parameter */
    )

    // Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
