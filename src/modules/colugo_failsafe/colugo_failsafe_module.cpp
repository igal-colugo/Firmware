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

#include "colugo_failsafe_module.h"

#include "commander/px4_custom_mode.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

int ColugoFailsafeModule::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int ColugoFailsafeModule::custom_command(int argc, char *argv[])
{
    /*
    if (!is_running()) {
        print_usage("not running");
        return 1;
    }

    // additional custom commands can be handled like this:
    if (!strcmp(argv[0], "do-something")) {
        get_instance()->do_something();
        return 0;
    }
     */

    return print_usage("unknown command");
}

int ColugoFailsafeModule::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024, (px4_main_t) &run_trampoline, (char *const *) argv);

    if (_task_id < 0)
    {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

ColugoFailsafeModule *ColugoFailsafeModule::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF)
    {
        switch (ch)
        {
        case 'p':
            example_param = (int) strtol(myoptarg, nullptr, 10);
            break;

        case 'f':
            example_flag = true;
            break;

        case '?':
            error_flag = true;
            break;

        default:
            PX4_WARN("unrecognized flag");
            error_flag = true;
            break;
        }
    }

    if (error_flag)
    {
        return nullptr;
    }

    ColugoFailsafeModule *instance = new ColugoFailsafeModule(example_param, example_flag);

    if (instance == nullptr)
    {
        PX4_ERR("alloc failed");
    }

    return instance;
}

ColugoFailsafeModule::ColugoFailsafeModule(int example_param, bool example_flag) : ModuleParams(nullptr)
{
    uint32_t i = 0;

    list_handlers.clear();
    list_handlers.push_back(new GPSFailHandler(i++));
}
//@note colugo failsafe module run function
void ColugoFailsafeModule::run()
{
    // Example: run the loop synchronized to the sensor_combined topic publication
    int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

    px4_pollfd_struct_t fds[2];

    fds[0].fd = sensor_combined_sub;
    fds[0].events = POLLIN;

    fds[1].fd = vehicle_status_sub;
    fds[1].events = POLLIN;

    // initialize parameters
    parameters_update(true);

    while (!should_exit())
    {
        if (parameters.system_monitor_enable > 0)
        {
            // wait for up to 1000ms for data
            int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

            if (pret == 0)
            {
                // Timeout: let the loop run anyway, don't do `continue` here
            }
            else if (pret < 0)
            {
                // this is undesirable but not much we can do
                PX4_ERR("poll error %d, %d", pret, errno);
                px4_usleep(50000);
                continue;
            }
            else if (fds[0].revents & POLLIN)
            {
                struct sensor_combined_s sensor_combined;
                orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
                // TODO: do something with the data...
            }
            else if (fds[1].revents & POLLIN)
            {
                orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
                // TODO: do something with the data...
            }

            check_system();

            const bool is_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

            if (is_armed)
            {
                // check indicators and handle fail if needed
                for (auto fail_handler : list_handlers)
                {
                    uint32_t handler_id = fail_handler->get_id() + 1;
                    if (handler_id & diagnostic_status.indicators.array)
                    {
                        if (fail_handler->classType() == CLASS_GPS_HANDLER)
                        {
                            // handle fail once
                            if (fail_handler->is_handled == false)
                            {
                                gps_fail_handler();
                                fail_handler->is_handled = true;
                            }
                        }
                    }
                    else
                    {
                        // reset handle fail once flag
                        fail_handler->is_handled = false;
                    }
                }
            }
        }

        parameters_update();
    }

    orb_unsubscribe(sensor_combined_sub);
}

void ColugoFailsafeModule::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force)
    {
        // clear update
        parameter_update_s update;
        _parameter_update_sub.copy(&update);

        // update parameters from storage
        updateParams();

        parameters.system_monitor_enable = _param_col_fail_en.get();
        //@note update gps parameters
        bool return_value = update_gps_parameters();
    }
}

int ColugoFailsafeModule::print_usage(const char *reason)
{
    if (reason)
    {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("module", "colugo_failsafe");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

bool ColugoFailsafeModule::check_system()
{
    bool return_value = false;

    for (auto fail_handler : list_handlers)
    {
        //@todo Vlad:implement cyclic processing
        update_handler(fail_handler);

        if (fail_handler->is_fail())
        {
            diagnostic_status.indicators.array = diagnostic_status.indicators.array | (1U << fail_handler->get_id()); // set bit
        }
        else
        {
            diagnostic_status.indicators.array = diagnostic_status.indicators.array & (~(1U << fail_handler->get_id())); // clear bit
        }
    }

    return return_value;
}

bool ColugoFailsafeModule::update_gps_parameters()
{
    bool return_value = true;

    parameters.system_monitor_gps_taking_care = _param_col_gps_taking_care.get();
    parameters.system_monitor_gps_grace_time = _param_col_gps_grace_time.get() * 1000;           // convert to us
    parameters.system_monitor_gps_fail_grace_time = _param_col_gps_fail_grace_time.get() * 1000; // convert to us
    parameters.system_monitor_gps_min_satellites = _param_col_gps_min_satellites.get();
    parameters.system_monitor_gps_distance = _param_col_gps_distance.get();
    parameters.system_monitor_gps_velocity_variance = _param_col_gps_velocity_variance.get();
    parameters.system_monitor_gps_hdop = _param_col_gps_hdop.get();
    parameters.system_monitor_gps_status = _param_col_gps_status.get();

    return return_value;
}

bool ColugoFailsafeModule::update_handler(FailHandler *handler)
{
    bool return_value = false;

    if (handler->classType() == CLASS_GPS_HANDLER)
    {
        //@added update gps handler
        double_t *new_parameters = new double_t[10]{parameters.system_monitor_gps_grace_time, parameters.system_monitor_gps_fail_grace_time,   parameters.system_monitor_gps_min_satellites,
                                                    parameters.system_monitor_gps_distance,   parameters.system_monitor_gps_velocity_variance, parameters.system_monitor_gps_hdop,
                                                    parameters.system_monitor_gps_status};

        handler->set_parameters(new_parameters);
        handler->set_taking_care(parameters.system_monitor_gps_taking_care);

        delete[] new_parameters;
    }

    return return_value;
}

bool ColugoFailsafeModule::gps_fail_handler()
{
    bool return_value = false;

    if (parameters.system_monitor_gps_taking_care == 0)
    {
        //@todo create send message to QGroundControl
        // send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND);
    }
    else if (parameters.system_monitor_gps_taking_care == 1)
    {
        send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);
    }
    else if (parameters.system_monitor_gps_taking_care == 2)
    {
        send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_RTL);
    }
    else if (parameters.system_monitor_gps_taking_care == 3)
    {
        send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LAND);
    }

    return return_value;
}

bool ColugoFailsafeModule::send_vehicle_command(const uint32_t cmd, const float param1, const float param2, const float param3, const float param4, const double param5, const double param6,
                                                const float param7)
{
    vehicle_command_s vcmd{};
    vcmd.command = cmd;
    vcmd.param1 = param1;
    vcmd.param2 = param2;
    vcmd.param3 = param3;
    vcmd.param4 = param4;
    vcmd.param5 = param5;
    vcmd.param6 = param6;
    vcmd.param7 = param7;

    uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
    vcmd.source_system = vehicle_status_sub.get().system_id;
    vcmd.target_system = vehicle_status_sub.get().system_id;
    vcmd.source_component = vehicle_status_sub.get().component_id;
    vcmd.target_component = vehicle_status_sub.get().component_id;

    uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
    vcmd.timestamp = hrt_absolute_time();

    return vcmd_pub.publish(vcmd);
}
//@note colugo failsafe module main function
int colugo_failsafe_module_main(int argc, char *argv[])
{
    return ColugoFailsafeModule::main(argc, argv);
}
