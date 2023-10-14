/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#if defined(CONFIG_SYSTEM_CDCACM)
__BEGIN_DECLS
#include <arch/board/board.h>
#include <builtin/builtin.h>
#include <nuttx/wqueue.h>
#include <syslog.h>

#include <sys/ioctl.h>
#include <termios.h>

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);
__END_DECLS

#include <px4_platform_common/shutdown.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

#define MODULE_NAME "cdc_acm"

#define USB_DEVICE_PATH "/dev/ttyACM0"

#if defined(CONFIG_SERIAL_PASSTHRU_UBLOX)
#undef SERIAL_PASSTHRU_UBLOX_DEV
#if defined(CONFIG_SERIAL_PASSTHRU_GPS1) && defined(CONFIG_BOARD_SERIAL_GPS1)
#define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS1
#elif defined(CONFIG_SERIAL_PASSTHRU_GPS2) && defined(CONFIG_BOARD_SERIAL_GPS2)
#define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS2
#elif defined(CONFIG_SERIAL_PASSTHRU_GPS3) && defined(CONFIG_BOARD_SERIAL_GPS3)
#define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS3
#elif defined(CONFIG_SERIAL_PASSTHRU_GPS4) && defined(CONFIG_BOARD_SERIAL_GPS4)
#define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS4
#elif defined(CONFIG_SERIAL_PASSTHRU_GPS5) && defined(CONFIG_BOARD_SERIAL_GPS5)
#define SERIAL_PASSTHRU_UBLOX_DEV CONFIG_BOARD_SERIAL_GPS5
#endif
#if !defined(SERIAL_PASSTHRU_UBLOX_DEV)
#error "CONFIG_SERIAL_PASSTHRU_GPSn and CONFIG_BOARD_SERIAL_GPSn must be defined"
#endif
#endif

static struct work_s usb_serial_work;

enum class UsbAutoStartState
{
    disconnected,
    connecting,
    connected,
    disconnecting,
} usb_auto_start_state{UsbAutoStartState::disconnected};

static void mavlink_usb_check(void *arg)
{
    int rescheduled = -1;

    uORB::SubscriptionData<actuator_armed_s> actuator_armed_sub{ORB_ID(actuator_armed)};

    const bool armed = actuator_armed_sub.get().armed;
    bool vbus_present = (board_read_VBUS_state() == PX4_OK);
    static bool vbus_present_prev = (board_read_VBUS_state() == PX4_OK);
    bool locked_out = false;

    // If the hardware support RESET lockout that has nArmed ANDed with VBUS
    // vbus_sense may drop during a param save which uses
    // BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE to prevent external resets
    // while writing the params.  If we are not armed and nARMRED is low
    // we are in such a lock out so ignore changes on VBUS_SENSE during this
    // time.
#if defined(BOARD_GET_EXTERNAL_LOCKOUT_STATE)
    locked_out = BOARD_GET_EXTERNAL_LOCKOUT_STATE() == 0;

    if (locked_out)
    {
        vbus_present = vbus_present_prev;
    }

#endif

    if (!armed && !locked_out)
    {
        switch (usb_auto_start_state)
        {
        case UsbAutoStartState::disconnected:
            if (vbus_present && vbus_present_prev)
            {
                if (sercon_main(0, nullptr) == EXIT_SUCCESS)
                {
                    usb_auto_start_state = UsbAutoStartState::connecting;
                    rescheduled = work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, nullptr, USEC2TICK(100000));
                }
            }
            else if (vbus_present && !vbus_present_prev)
            {
                // check again sooner if USB just connected
                rescheduled = work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, nullptr, USEC2TICK(100000));
            }

            break;

        case UsbAutoStartState::connecting:
            if (vbus_present && vbus_present_prev)
            {
                struct termios uart_config;

                static const char *mavlink_argv[]{"mavlink", "start", "-d", USB_DEVICE_PATH, "-b", "57600", nullptr};
                char **exec_argv = nullptr;

                exec_argv = (char **) mavlink_argv;

                sched_lock();

                if (exec_builtin(exec_argv[0], exec_argv, nullptr, 0) > 0)
                {
                    usb_auto_start_state = UsbAutoStartState::connected;
                }
                else
                {
                    usb_auto_start_state = UsbAutoStartState::disconnecting;
                }

                sched_unlock();
            }
            else
            {
                usb_auto_start_state = UsbAutoStartState::disconnecting;
            }

            break;

        case UsbAutoStartState::connected:
            if (!vbus_present && !vbus_present_prev)
            {
                sched_lock();

                static const char app[]{"mavlink"};
                static const char *stop_argv[]{"mavlink", "stop", "-d", USB_DEVICE_PATH, NULL};
                exec_builtin(app, (char **) stop_argv, NULL, 0);

                sched_unlock();

                usb_auto_start_state = UsbAutoStartState::disconnecting;
            }

            break;

        case UsbAutoStartState::disconnecting:
            // serial disconnect if unused
            serdis_main(0, NULL);
            usb_auto_start_state = UsbAutoStartState::disconnected;
            break;
        }
    }

    vbus_present_prev = vbus_present;

    if (rescheduled != PX4_OK)
    {
        work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, NULL, USEC2TICK(1000000));
    }
}

void cdcacm_init(void)
{
    work_queue(LPWORK, &usb_serial_work, mavlink_usb_check, nullptr, 0);
}

#endif // CONFIG_SYSTEM_CDCACM
