/****************************************************************************
 *
 *   Copyright (C) 2017-2019 Intel Corporation. All rights reserved.
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

#include "da35efi_serial.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace da35efi
{

DA35EFISerial *g_dev_serial{nullptr};

static int start(const char *port)
{
    if (g_dev_serial != nullptr)
    {
        PX4_ERR("already started");
        return PX4_ERROR;
    }

    g_dev_serial = new DA35EFISerial(port);

    if (g_dev_serial == nullptr)
    {
        PX4_ERR("object instantiate failed");
        return PX4_ERROR;
    }

    // Initialize the sensor.
    if (g_dev_serial->init() != PX4_OK)
    {
        PX4_ERR("driver start failed");
        delete g_dev_serial;
        g_dev_serial = nullptr;
        return PX4_ERROR;
    }

    // Start the driver.
    g_dev_serial->start();

    return PX4_OK;
}

static int status()
{
    if (g_dev_serial == nullptr)
    {
        PX4_ERR("driver not running");
        return PX4_ERROR;
    }

    g_dev_serial->print_info();

    return PX4_OK;
}

static int stop()
{
    if (g_dev_serial != nullptr)
    {
        delete g_dev_serial;
        g_dev_serial = nullptr;
    }

    PX4_INFO("driver stopped");
    return PX4_OK;
}

static int usage()
{
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description

Serial bus driver for the HFE da35efi engine controller.

Most boards are configured to enable/start the driver on a specified UART using the DA35EFI_CFG parameter.

### Examples

Attempt to start driver on a specified serial device.
$ da35efi start -d /dev/ttyS1
Stop driver
$ da35efi stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("da35efi", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("drivers");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
    return PX4_OK;
}

} // namespace da35efi

extern "C" __EXPORT int da35efi_main(int argc, char *argv[])
{
    const char *myoptarg = nullptr;

    int ch = 0;
    int myoptind = 1;

    const char *port = nullptr;
    // uint8_t communication = 0;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF)
    {
        switch (ch)
        {
        case 'd':
            port = myoptarg;
            break;

            // case 'c':
            // {
            //     int com = -1;

            //     if (px4_get_parameter_value(myoptarg, com) != 0)
            //     {
            //         PX4_ERR("communication failed");
            //         return -1;
            //     }
            //     communication = (uint8_t) com;
            //     break;
            // }

        default:
            PX4_WARN("Unknown option");
            return da35efi::usage();
        }
    }

    if (myoptind >= argc)
    {
        return da35efi::usage();
    }

    if (!strcmp(argv[myoptind], "start"))
    {
        return da35efi::start(port);
    }
    else if (!strcmp(argv[myoptind], "status"))
    {
        return da35efi::status();
    }
    else if (!strcmp(argv[myoptind], "stop"))
    {
        return da35efi::stop();
    }

    return da35efi::usage();
}
