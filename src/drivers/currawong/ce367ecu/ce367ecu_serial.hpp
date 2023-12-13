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

#pragma once

#include <fcntl.h>

#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>

#include <stdlib.h>
#include <string.h>
#include <termios.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/gimbal_device_information.h>
#include <uORB/topics/gimbal_device_set_attitude.h>

#include "stm32.h"

extern struct fdcan_driver_s g_fdcan0;

using namespace time_literals;
using namespace math;

#define CE367ECU_BAUDRATE B57600
#define SYNC_0 0xA5 // first header byte
#define SYNC_1 0x5A

#define CE367ECU_PACKETLEN_MAX (100U)
#define CE367ECU_PACKETLEN_MIN 4                                               // minimum number of bytes in a packet.  this is a packet with no data bytes
#define CE367ECU_DATALEN_MAX (CE367ECU_PACKETLEN_MAX - CE367ECU_PACKETLEN_MIN) // max bytes for data portion of packet
#define CE367ECU_MEASURE_INTERVAL 200_ms                                       // 5Hz
#define CMD_GET_ANGLES_EXT_PACKET_LEN (59U)

class CE367ECUSerial : public px4::ScheduledWorkItem
{
  public:
#pragma region Methods
    CE367ECUSerial(const char *serial_port);
    ~CE367ECUSerial() override;

    int init();
    /**
     * Diagnostics - print some basic information about the driver.
     */
    void print_info();
    /**
     * Initialise the automatic measurement state machine and start it.
     */
    void start();
    /**
     * Stop the automatic measurement state machine.
     */
    void stop();

#pragma endregion

  private:
#pragma region Nested types
    union ViewControlMode {
        uint8_t control_byte;
        struct
        {
            uint8_t mode : 4;

            uint8_t flag_0 : 1;
            uint8_t flag_1 : 1;
            uint8_t control_flag_auto_task : 1;
            uint8_t control_flag_high_res_speed : 1;

        } byte;
    };

    union ControlGimbalMovementCommand {
        uint8_t control_array[19];

        struct __attribute__((packed, aligned(1)))
        {
            uint8_t header[4] = {0xFF, 0x01, 0x0F, 0x10};

            ViewControlMode roll_mode;
            ViewControlMode pitch_mode;
            ViewControlMode yaw_mode;

            int16_t roll_speed_units;
            int16_t roll_angle_units;

            int16_t pitch_speed_units;
            int16_t pitch_angle_units;

            int16_t yaw_speed_units;
            int16_t yaw_angle_units;

        } control_command;
    };

    union AnglesPacket {
        uint8_t packet_array[58];

        struct
        {
            uint8_t header_byte_1;
            uint8_t header_byte_2;
            uint8_t header_byte_3;
            uint8_t header_byte_4;

            int16_t roll_imu_angle_units;
            int16_t roll_rc_target_angle_units;
            int32_t roll_stator_rel_angle_units;
            uint8_t roll_reserved_bytes[10] = {};

            int16_t pitch_imu_angle_units;
            int16_t pitch_rc_target_angle_units;
            int32_t pitch_stator_rel_angle_units;
            uint8_t pitch_reserved_bytes[10] = {};

            int16_t yaw_imu_angle_units;
            int16_t yaw_rc_target_angle_units;
            int32_t yaw_stator_rel_angle_units;
            uint8_t yaw_reserved_bytes[10] = {};

            uint8_t checksum;

        } __attribute__((packed, aligned(1))) packet;
    };

    // viewpro control mode
    enum class ViewControlModeEnum : uint8_t
    {
        MODE_NO_CONTRL = 0x00,
        MODE_SPEED = 0x01,
        MODE_ANGLE = 0x02,
        MODE_SPEED_ANGLE = 0x03,
        MODE_RC = 0x04,
        MODE_ANGLE_REF_FRAME = 0x05,
        MODE_RC_HIGH_RES = 0x06
    };

    // Function Feedback Info packet info_type values
    enum class FunctionFeedbackInfo : uint8_t
    {
        SUCCESS = 0,
        FAILED_TO_TAKE_PHOTO = 1,
        HDR_ON = 2,
        HDR_OFF = 3,
        FAILED_TO_RECORD_VIDEO = 4
    };

    // parsing state
    enum class ParseState : uint8_t
    {
        WAITING_FOR_SYNC_0,
        WAITING_FOR_SYNC_1,
        WAITING_FOR_ADDRESS_HI,
        WAITING_FOR_ADDRESS_LOW,
        WAITING_FOR_MESSAGE_TYPE,
        WAITING_FOR_MESSAGE_SIZE,
        WAITING_FOR_DATA,
        WAITING_FOR_CHECKSUM_0,
        WAITING_FOR_CHECKSUM_1
    };

    struct __attribute__((packed, aligned(1))) transmitted_msg
    {
        uint8_t sync_0;
        uint8_t sync_1;
        uint16_t device_address;
        uint8_t message_type;
        uint8_t message_size;
        uint8_t *data;
        uint16_t check_sum;
    };

    struct __attribute__((__packed__)) recieved_msg
    {
        uint8_t sync_0;
        uint8_t sync_1;
        uint16_t device_address;
        uint8_t message_type;
        uint8_t message_size;
        uint8_t *data;
        uint16_t check_sum;
    };

#pragma endregion

#pragma region Fields

    const char *_serial_port{nullptr};

    int _file_descriptor{-1};

    uint8_t _buffer[150];
    uint8_t _buffer_len{0};

    hrt_abstime _measurement_time{0};

    perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME ": comms_error")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": sample")};

    recieved_msg recieved_message = {};
    transmitted_msg transmitted_message = {};

    ParseState parsing_state = ParseState::WAITING_FOR_SYNC_0;
    uint32_t data_bytes_received = 0;

    //@note control command to gimbal
    ControlGimbalMovementCommand cmd_control_movement = {};
    //@note reported data from gimbal
    AnglesPacket angles_packet = {};

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    uint8_t _msg_buff[CE367ECU_PACKETLEN_MAX];
    uint8_t _msg_buff_len;

    // variables for sending packets to gimbal
    uint32_t _last_send_ms; // system time (in milliseconds) of last packet sent to gimbal
    uint16_t _last_seq;     // last sequence number used (should be increment for each send)

    uint32_t _last_current_angle_rad_ms; // system time _current_angle_rad was updated

    uint16_t _count_received_packets;
    uint16_t _count_broken_packets;

    // Subscribers
    // MavlinkV1
    uORB::Subscription _attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};
    // MavlinkV2
    uORB::Subscription _set_attitude_sub{ORB_ID(gimbal_device_set_attitude)};
    uORB::Subscription _trigger_sub{ORB_ID(camera_trigger)};

#pragma endregion

#pragma region Methods
    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet(const uint8_t *databuff, uint8_t databuff_len, bool checksum = false, uint8_t header_size = 0);

    uint8_t two_complement_checksum(const uint8_t *pBuffer, uint32_t length);
    /**
     * Calculates the 16 byte crc value for the data frame.
     * @param data_frame The data frame to compute a checksum for.
     * @param crc16_length The length of the data frame.
     */
    uint16_t crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length);
    /**
     * Encode sequential bytes in a memory buffer using the fletcher algorithm
     */
    uint16_t fletcher_encode(const uint8_t *buffer, uint16_t count);

    /**
     * Reads the data measrurement from serial UART.
     */
    int collect();

    /**
     * Sends a data request message to the sensor.
     */
    int measure();

    int update();

    /**
     * Opens and configures the UART serial communications port.
     * @param speed The baudrate (speed) to configure the serial UART port.
     */
    int open_serial_port(const speed_t speed = CE367ECU_BAUDRATE);

    void Run() override;

    bool rotate_gimbal(float roll, float pitch, float yaw, ViewControlModeEnum mode);
    bool request_get_angles_ext();

#pragma endregion
};
