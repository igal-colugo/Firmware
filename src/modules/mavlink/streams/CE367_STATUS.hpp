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

#ifndef CE367_STATUS_HPP
#define CE367_STATUS_HPP

#include <uORB/topics/currawong_ce367ecu_status.h>

class MavlinkStreamCE367Status : public MavlinkStream
{
  public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamCE367Status(mavlink);
    }

    static constexpr const char *get_name_static()
    {
        return "CE367_STATUS";
    }
    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_CE367_STATUS;
    }

    const char *get_name() const override
    {
        return get_name_static();
    }
    uint16_t get_id() override
    {
        return get_id_static();
    }

    unsigned get_size() override
    {
        return _currawong_ce367ecu_status_sub.advertised() ? (MAVLINK_MSG_ID_CE367_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
    }

  private:
    explicit MavlinkStreamCE367Status(Mavlink *mavlink) : MavlinkStream(mavlink)
    {
    }

    uORB::Subscription _currawong_ce367ecu_status_sub{ORB_ID(currawong_ce367ecu_status), 0};

    bool send() override
    {
        currawong_ce367ecu_status_s ce367_status;

        if (_currawong_ce367ecu_status_sub.update(&ce367_status))
        {
            mavlink_ce367_status_t msg{};

            msg.ecu_throttle = ce367_status.ecu_throttle;
            msg.ecu_fuel_used = ce367_status.ecu_fuel_used;
            msg.ecu_rpm = ce367_status.ecu_rpm;

            mavlink_msg_ce367_status_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // ASIO_STATUS_HPP
