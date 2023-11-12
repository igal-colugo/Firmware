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

#ifndef ASIO_STATUS_HPP
#define ASIO_STATUS_HPP

#include <uORB/topics/arial_obox_status.h>

class MavlinkStreamAsioStatus : public MavlinkStream
{
  public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamAsioStatus(mavlink);
    }

    static constexpr const char *get_name_static()
    {
        return "ASIO_STATUS";
    }
    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_ASIO_STATUS;
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
        return _arial_obox_status_sub.advertised() ? (MAVLINK_MSG_ID_ASIO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
    }

  private:
    explicit MavlinkStreamAsioStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
    {
    }

    uORB::Subscription _arial_obox_status_sub{ORB_ID(arial_obox_status), 0};

    bool send() override
    {
        arial_obox_status_s asio_status;

        if (_arial_obox_status_sub.update(&asio_status))
        {
            mavlink_asio_status_t msg{};
            //     uint32_t error;            /*<  */
            //     float pitch_offset;        /*< [deg] Pitch offset*/
            //     float roll_offset;         /*< [deg] Roll offset*/
            //     float reserved_2;          /*<  */
            //     uint32_t version;          /*<  asio software version*/
            //     int16_t temperature;       /*< [cdegC] Internal temperature of the system, set to -1 if invalid*/
            //     uint8_t nav_state;         /*<  */
            //     uint8_t nav_output;        /*<  */
            //     uint8_t gps_jamming_state; /*<  */
            //     uint8_t active_sensor;     /*<  */
            //     uint8_t active_rec;        /*<  */
            //     uint8_t active_db_rec;     /*<  */
            //     uint8_t active_nav;        /*<  Current NAV_MODE*/
            //     uint8_t active_slam;       /*<  Current SLAM_MODE*/
            //     uint8_t battery;           /*< [%] Set to 255 for unknown/irrelevant*/
            //     uint8_t sys_state;         /*<  Current system state*/
            //     uint8_t reserved;          /*<  */
            //     uint8_t map_warning;       /*<  */
            //     uint8_t calibration_state; /*<  */

            msg.error = asio_status.error;
            msg.pitch_offset = asio_status.pitch_offset;
            msg.roll_offset = asio_status.roll_offset;
            msg.reserved_2 = asio_status.reserved_2;
            msg.version = asio_status.version;
            msg.temperature = asio_status.temperature;
            msg.nav_state = asio_status.nav_state;
            msg.nav_output = asio_status.nav_output;
            msg.gps_jamming_state = asio_status.gps_jamming_state;
            msg.active_sensor = asio_status.active_sensor;
            msg.active_rec = asio_status.active_rec;
            msg.active_db_rec = asio_status.active_db_rec;
            msg.active_nav = asio_status.active_nav;
            msg.active_slam = asio_status.active_slam;
            msg.battery = asio_status.battery;
            msg.sys_state = asio_status.sys_state;
            msg.reserved = asio_status.reserved;
            msg.map_warning = asio_status.map_warning;
            msg.calibration_state = asio_status.calibration_state;

            mavlink_msg_asio_status_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // ASIO_STATUS_HPP
