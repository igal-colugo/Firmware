/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef HFE_STATUS_HPP
#define HFE_STATUS_HPP

#include <uORB/topics/hfe_da35efi_status.h>

class MavlinkStreamHfeStatus : public MavlinkStream
{
  public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamHfeStatus(mavlink);
    }

    static constexpr const char *get_name_static()
    {
        return "HFE_STATUS";
    }
    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_DA35EFI_STATUS;
    }

    const char *get_name() const override
    {
        return MavlinkStreamHfeStatus::get_name_static();
    }
    uint16_t get_id() override
    {
        return get_id_static();
    }

    unsigned get_size() override
    {
        return _hfe_da35efi_sub.advertised() ? MAVLINK_MSG_ID_DA35EFI_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

  private:
    explicit MavlinkStreamHfeStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
    {
    }

    uORB::Subscription _hfe_da35efi_sub{ORB_ID(hfe_da35efi_status)};

    bool send() override
    {
        hfe_da35efi_status_s hfe_da35efi_status_status;

        if (_hfe_da35efi_sub.update(&hfe_da35efi_status_status))
        {
            mavlink_da35efi_status_t msg{};

            msg.seconds = hfe_da35efi_status_status.seconds;
            msg.pulse_width_1 = hfe_da35efi_status_status.pulse_width_1;
            msg.pulse_width_2 = hfe_da35efi_status_status.pulse_width_2;
            msg.rpm = hfe_da35efi_status_status.rpm;
            msg.engine = hfe_da35efi_status_status.engine;
            msg.barometer = hfe_da35efi_status_status.barometer;
            msg.coolant = hfe_da35efi_status_status.coolant;
            msg.tps = hfe_da35efi_status_status.tps;
            msg.battery_voltage = hfe_da35efi_status_status.battery_voltage;
            msg.injector_duty_cycle = hfe_da35efi_status_status.injector_duty_cycle;
            msg.fuel_flow_rate_instantaneos = hfe_da35efi_status_status.fuel_flow_rate_instantaneos;
            msg.scaled_flow_rate = hfe_da35efi_status_status.scaled_flow_rate;

            mavlink_msg_da35efi_status_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // EFI_STATUS_HPP
