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

#define DA35EFI_BAUDRATE B115200
#define SYNC_0 0x00 // first header byte
#define SYNC_1 0xD5

#define DA35EFI_PACKETLEN_MAX (220U)
#define DA35EFI_PACKETLEN_MIN 4                                             // minimum number of bytes in a packet.  this is a packet with no data bytes
#define DA35EFI_DATALEN_MAX (DA35EFI_PACKETLEN_MAX - DA35EFI_PACKETLEN_MIN) // max bytes for data portion of packet
#define DA35EFI_MEASURE_INTERVAL 200_ms                                     // 5Hz
#define CMD_REQUEST_REAL_TIME_PACKET_LEN (213U)

//! Byte swap unsigned short
inline uint16_t swap_uint16(uint16_t val)
{
    return (val << 8) | (val >> 8);
}
//! Byte swap short
inline int16_t swap_int16(int16_t val)
{
    return (val << 8) | ((val >> 8) & 0xFF);
}
//! Byte swap unsigned int
inline uint32_t swap_uint32(uint32_t val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | (val >> 16);
}
//! Byte swap int
inline int32_t swap_int32(int32_t val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | ((val >> 16) & 0xFFFF);
}

inline int64_t swap_int64(int64_t val)
{
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL) | ((val >> 8) & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) | ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | ((val >> 32) & 0xFFFFFFFFULL);
}

inline uint64_t swap_uint64(uint64_t val)
{
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL) | ((val >> 8) & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) | ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | (val >> 32);
}

class DA35EFISerial : public px4::ScheduledWorkItem
{
  public:
#pragma region Methods
    DA35EFISerial(const char *serial_port);
    ~DA35EFISerial() override;

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

    union RealTimeDataPacket {
        uint8_t packet_array[212];

        struct
        {
            uint16_t seconds;
            uint16_t pulseWidth1;
            uint16_t pulseWidth2;
            uint16_t rpm;
            int16_t advance;
            uint8_t squirt;
            uint8_t engine;
            uint8_t afrtgt1;
            uint8_t afrtgt2;
            uint8_t wbo2_en1;
            uint8_t wbo2_en2;
            int16_t barometer;
            int16_t map;
            int16_t mat;
            int16_t coolant;
            int16_t tps;
            int16_t batteryVoltage;
            int16_t afr1;
            int16_t afr2;
            int16_t knock;
            int16_t egocor1;
            int16_t egocor2;
            int16_t aircor;
            int16_t warmcor;
            int16_t accelEnrich;
            int16_t tpsfuelcut;
            int16_t baroCorrection;
            int16_t gammaEnrich;
            int16_t ve1;
            int16_t ve2;
            int16_t iacstep;
            int16_t cold_adv_deg;
            int16_t TPSdot;
            int16_t MAPdot;
            int16_t dwell;
            int16_t MAF;
            uint8_t fuelload;
            int16_t fuelcor;
            uint8_t portStatus;
            uint8_t knockRetard;
            int16_t EAEfcor1;
            int16_t egoV1;
            int16_t egoV2;
            uint16_t status1;
            uint16_t status2;
            uint16_t status3;
            uint16_t status4;
            uint16_t looptime;
            uint16_t status5;
            uint16_t tpsADC;
            int16_t fuelload2;
            int16_t ignload;
            int16_t ignload2;
            uint8_t synccnt;
            int8_t timing_err;
            int32_t delta;
            uint32_t wallfuel1;
            uint16_t gpioadc0;
            uint16_t gpioadc1;
            uint16_t gpioadc2;
            uint16_t gpioadc3;
            uint16_t gpioadc4;
            uint16_t gpioadc5;
            uint16_t gpioadc6;
            uint16_t gpioadc7;
            uint16_t gpiopwmin0;
            uint16_t gpiopwmin1;
            uint16_t gpiopwmin2;
            uint16_t gpiopwmin3;
            uint16_t adc6;
            uint16_t adc7;
            int32_t wallfuel2;
            uint16_t EAEFuelCorr2;
            uint8_t boostduty;
            uint8_t syncreason;
            uint16_t user0;
            int16_t inj_adv1;
            int16_t inj_adv2;
            uint16_t pulseWidth3;
            uint16_t pulseWidth4;
            int16_t vetrim1curr;
            int16_t vetrim2curr;
            int16_t vetrim3curr;
            int16_t vetrim4curr;
            uint16_t maf;
            int16_t eaeload1;
            int16_t afrload1;
            int16_t RPMdot;
            uint8_t gpioport0;
            uint8_t gpioport1;
            uint16_t gpioport2;
            int16_t cl_idle_targ_rpm;
            uint16_t maf_volts;
            int16_t airtemp;
            uint16_t dwell_trl;
            uint16_t fuel_pct;
            int16_t boost_targ;
            int16_t ext_advance;
            int16_t base_advance;
            int16_t idle_cor_advance;
            int16_t mat_retard;
            int16_t flex_advance;
            int16_t adv1;
            int16_t adv2;
            int16_t adv3;
            int16_t revlim_retard;
            int16_t nitrous_retard;
            uint16_t deadtime1;
            uint16_t n2o_addfuel;
            uint8_t portbde;
            uint8_t portam;
            uint8_t port_plus;
            uint8_t can_error_cnt;
            uint8_t can_error;

        } __attribute__((packed, aligned(1))) packet;
    };

    // parsing state
    enum class ParseState : uint8_t
    {
        WAITING_FOR_SYNC_0,
        WAITING_FOR_SYNC_1,
        WAITING_FOR_FLAG,
        WAITING_FOR_DATA,
        WAITING_FOR_CHECKSUM_0,
        WAITING_FOR_CHECKSUM_1,
        WAITING_FOR_CHECKSUM_2,
        WAITING_FOR_CHECKSUM_3
    };

    struct __attribute__((packed, aligned(1))) transmitted_msg
    {
        uint16_t size;
        uint8_t flag;
        uint8_t *data;
        uint32_t check_sum;
    };

    struct __attribute__((__packed__)) recieved_msg
    {
        uint16_t size;
        uint8_t flag;
        uint8_t *data;
        uint32_t check_sum;
    };

#pragma endregion

#pragma region Fields

    const char *_serial_port{nullptr};

    int _file_descriptor{-1};

    uint8_t _buffer[250];
    uint8_t _buffer_len{0};

    hrt_abstime _measurement_time{0};

    perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME ": comms_error")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": sample")};

    recieved_msg recieved_message = {};
    transmitted_msg transmitted_message = {};

    ParseState parsing_state = ParseState::WAITING_FOR_SYNC_0;
    uint32_t data_bytes_received = 0;

    RealTimeDataPacket realtime_data_packet = {};

    // buffer holding bytes from latest packet.  This is only used to calculate the crc
    uint8_t _msg_buff[DA35EFI_PACKETLEN_MAX];
    uint8_t _msg_buff_len;

    // variables for sending packets to gimbal
    uint32_t _last_send_ms; // system time (in milliseconds) of last packet sent to gimbal
    uint16_t _last_seq;     // last sequence number used (should be increment for each send)

    uint32_t _last_current_angle_rad_ms; // system time _current_angle_rad was updated

    uint32_t _last_current_real_time_ms;

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
    bool send_packet(const uint8_t *databuff, uint16_t databuff_len, bool checksum = false);

    uint8_t two_complement_checksum(const uint8_t *pBuffer, uint32_t length);
    /**
     * Calculates the 16 byte crc value for the data frame.
     * @param data_frame The data frame to compute a checksum for.
     * @param crc16_length The length of the data frame.
     */
    uint16_t crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length);
    /**
     * Calculates the 32 byte crc value for the data frame.
     * @param data_frame The data frame to compute a checksum for.
     * @param crc32_length The length of the data frame.
     */
    uint32_t crc32_calc(const unsigned char *data_frame, const uint32_t crc32_length);
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
    int open_serial_port(const speed_t speed = DA35EFI_BAUDRATE);

    void Run() override;

    bool request_realtime_data();

#pragma endregion
};
