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

using namespace time_literals;
using namespace math;

#define VIEWPRO_BAUDRATE B115200
#define VIEWPRO_HEADER_BYTE_1 0x3e // first header byte
#define VIEWPRO_HEADER_BYTE_2 0x3d
#define VIEWPRO_HEADER_BYTE_3 0x36
#define VIEWPRO_HEADER_BYTE_4 0x73

#define VIEWPRO_PACKETLEN_MAX (100U)
#define VIEWPRO_PACKETLEN_MIN 4                                             // minimum number of bytes in a packet.  this is a packet with no data bytes
#define VIEWPRO_DATALEN_MAX (VIEWPRO_PACKETLEN_MAX - VIEWPRO_PACKETLEN_MIN) // max bytes for data portion of packet
#define VIEWPRO_MSG_BUF_DATA_START 8                                        // data starts at this byte in _msg_buf
#define VIEWPRO_MEASURE_INTERVAL 200_ms                                     // 5Hz
#define CMD_GET_ANGLES_EXT_PACKET_LEN (59U)

class ViewPro : public px4::ScheduledWorkItem
{
  public:
#pragma region Methods
    ViewPro(const char *serial_port);
    ~ViewPro() override;

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

    // Photo Function packet func_type values
    enum class PhotoFunction : uint8_t
    {
        TAKE_PICTURE = 0,
        HDR_TOGGLE = 1,
        RECORD_VIDEO_TOGGLE = 2,
        LOCK_MODE = 3,
        FOLLOW_MODE = 4,
        FPV_MODE = 5
    };

    // parsing state
    enum class ParseState : uint8_t
    {
        WAITING_FOR_HEADER_BYTE_1,
        WAITING_FOR_HEADER_BYTE_2,
        WAITING_FOR_HEADER_BYTE_3,
        WAITING_FOR_HEADER_BYTE_4,
        WAITING_FOR_ROLL_STATUS,
        WAITING_FOR_PITCH_STATUS,
        WAITING_FOR_YAW_STATUS,
        WAITING_FOR_CHECKSUM
    };

    struct __attribute__((__packed__)) reading_msg
    {
        uint8_t slave_addr;
        uint8_t function;
        uint8_t len;
        uint8_t low_timestamp_high_byte;
        uint8_t low_timestamp_low_byte;
        uint8_t high_timestamp_high_byte;
        uint8_t high_timestamp_low_byte;
        uint8_t temp_high;
        uint8_t temp_low;
        uint8_t num_detections_high_byte;
        uint8_t num_detections_low_byte;
        uint8_t first_dist_high_byte;
        uint8_t first_dist_low_byte;
        uint8_t first_amplitude_high_byte;
        uint8_t first_amplitude_low_byte;
        uint8_t second_dist_high_byte;
        uint8_t second_dist_low_byte;
        uint8_t second_amplitude_high_byte;
        uint8_t second_amplitude_low_byte;
        uint8_t third_dist_high_byte;
        uint8_t third_dist_low_byte;
        uint8_t third_amplitude_high_byte;
        uint8_t third_amplitude_low_byte;
        uint16_t crc; /* little-endian */
    };

    // parser state and unpacked fields
    struct PACKED
    {
        uint16_t data_len;            // expected number of data bytes
        uint8_t command_id;           // command id
        uint16_t data_bytes_received; // number of data bytes received so far
        uint16_t check_sum;           // latest message's crc
        ParseState state;             // state of incoming message processing
    } _parsed_msg;

    struct Quaternion
    {
        double w;
        double x;
        double y;
        double z;
    };

    struct EulerAngles
    {
        double roll;
        double pitch;
        double yaw;
    };

#pragma endregion

#pragma region Fields

    const double angle_units_to_degree_ratio = 0.02197265625;
    const double speed_units_to_degree_ratio = 0.1220740379;

    const char *_serial_port{nullptr};

    int _file_descriptor{-1};

    uint8_t _buffer[sizeof(AnglesPacket)];
    uint8_t _buffer_len{0};

    hrt_abstime _measurement_time{0};

    perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME ": comms_error")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": sample")};

    //@note control command to gimbal
    ControlGimbalMovementCommand cmd_control_movement = {};
    //@note reported data from gimbal
    AnglesPacket angles_packet = {};

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    uint8_t _msg_buff[VIEWPRO_PACKETLEN_MAX];
    uint8_t _msg_buff_len;
    const uint8_t _msg_buff_data_start = 4; // data starts at this byte of _msg_buff

    // variables for sending packets to gimbal
    uint32_t _last_send_ms; // system time (in milliseconds) of last packet sent to gimbal
    uint16_t _last_seq;     // last sequence number used (should be increment for each send)

    // actual attitude received from gimbal
    // Vector3f _current_angle_rad;                // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    // Vector3f _current_imu_angle_degree;         // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    // Vector3f _current_rc_target_degree;         // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)

    // Vector3f _current_stator_rel_angle_degree;  // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    // Vector3f _current_imu_angle_rad;            // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    // Vector3f _current_rc_target_rad;            // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)

    // Vector3f _current_stator_rel_angle_rad;     // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)

    uint32_t _last_current_angle_rad_ms;     // system time _current_angle_rad was updated
    uint32_t _last_req_current_angle_rad_ms; // system time that this driver last requested current angle

    uint32_t _last_req_get_picture_ms;          // system time that this driver last requested get picture
    uint32_t _period_getting_picture_ms = 1000; // period of getting pictures in ms

    // variables for camera state
    bool _last_record_video; // last record_video state sent to gimbal

    uint16_t _count_received_packets;
    uint16_t _count_broken_packets;

    EulerAngles angles = EulerAngles();

    // Subscribers
    //MavlinkV1
    uORB::Subscription _attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};
    //MavlinkV2
    uORB::Subscription _set_attitude_sub{ORB_ID(gimbal_device_set_attitude)};
    uORB::Subscription _trigger_sub{ORB_ID(camera_trigger)};

#pragma endregion

#pragma region Methods
    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet(const uint8_t *command_header, uint8_t command_header_len, const uint8_t *databuff, uint8_t databuff_len);
    bool send_packet(const uint8_t *databuff, uint8_t databuff_len, bool checksum = false, uint8_t header_size = 0);

    uint8_t two_complement_checksum(const uint8_t *pBuffer, uint32_t length);

    /**
     * Calculates the 16 byte crc value for the data frame.
     * @param data_frame The data frame to compute a checksum for.
     * @param crc16_length The length of the data frame.
     */
    uint16_t crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length);

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
    int open_serial_port(const speed_t speed = VIEWPRO_BAUDRATE);

    void Run() override;

    void send_target_angles(float roll_rad, float pitch_rad, float yaw_rad, bool yaw_is_ef);
    bool rotate_gimbal(float roll, float pitch, float yaw, ViewControlModeEnum mode);
    bool request_get_angles_ext();
    bool take_picture();
    bool request_photograph();
    Quaternion convert_to_quaternion(double roll, double pitch, double yaw);
    EulerAngles convert_to_euler_angles(Quaternion q);

#pragma endregion
};
