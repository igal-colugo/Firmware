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

#include "ViewPro.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include <lib/drivers/device/Device.hpp>

ViewPro::ViewPro(const char *serial_port) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port))
{
    _serial_port = strdup(serial_port);

    device::Device::DeviceId device_id;
    device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

    uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

    if (bus_num < 10)
    {
        device_id.devid_s.bus = bus_num;
    }

    _count_received_packets = 0;
    _count_broken_packets = 0;
}

ViewPro::~ViewPro()
{
    stop();

    free((char *) _serial_port);
    perf_free(_comms_error);
    perf_free(_sample_perf);
}

int ViewPro::init()
{
    if (open_serial_port() != PX4_OK)
    {
        return PX4_ERROR;
    }

    hrt_abstime time_now = hrt_absolute_time();

    const hrt_abstime timeout_usec = time_now + 500000_us; // timeout 0.5sec

    while (time_now < timeout_usec)
    {
        if (measure() == PX4_OK)
        {
            px4_usleep(VIEWPRO_MEASURE_INTERVAL);

            if (collect() == PX4_OK)
            {
                // The file descriptor can only be accessed by the process that opened it,
                // so closing here allows the port to be opened from scheduled work queue.
                stop();
                return PX4_OK;
            }
        }

        px4_usleep(1000);
        time_now = hrt_absolute_time();
    }

    PX4_ERR("No readings from ViewPro");
    return PX4_ERROR;
}

int ViewPro::collect()
{
    int return_value = 0;

    perf_begin(_sample_perf);

    const int buffer_size = sizeof(_buffer);
    const int message_size = sizeof(AnglesPacket);

    static uint8_t count_data = 0;
    static uint8_t header_size = 0;

    // read incomming bytes
    int bytes_read = ::read(_file_descriptor, _buffer + _buffer_len, buffer_size - _buffer_len);

    if (bytes_read < 1)
    {
        // Trigger a new measurement.
        return_value = measure();
        return return_value;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    _buffer_len += bytes_read;

    if (_buffer_len < message_size)
    {
        // Return on next scheduled cycle to collect remaining data.
        return PX4_OK;
    }

    // process bytes received
    for (int16_t i = 0; i < bytes_read; i++)
    {
        const int16_t received_byte = _buffer[i];

        // sanity check byte
        if ((received_byte < 0) || (received_byte > 0xFF))
        {
            continue;
        }

        _msg_buff[_msg_buff_len++] = received_byte;

        // protect against overly long messages
        if (_msg_buff_len >= VIEWPRO_PACKETLEN_MAX)
        {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state)
        {

        case ParseState::WAITING_FOR_HEADER_BYTE_1:
            _parsed_msg.data_bytes_received++;
            header_size++;
            if (received_byte == VIEWPRO_HEADER_BYTE_1)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER_BYTE_2;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER_BYTE_2:
            _parsed_msg.data_bytes_received++;
            header_size++;
            if (received_byte == VIEWPRO_HEADER_BYTE_2)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER_BYTE_3;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER_BYTE_3:
            _parsed_msg.data_bytes_received++;
            header_size++;
            if (received_byte == VIEWPRO_HEADER_BYTE_3)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER_BYTE_4;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER_BYTE_4:
            _parsed_msg.data_bytes_received++;
            header_size++;
            if (received_byte == VIEWPRO_HEADER_BYTE_4)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_ROLL_STATUS;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_ROLL_STATUS:
            _parsed_msg.data_bytes_received++;
            count_data++;
            if (count_data >= 18)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_PITCH_STATUS;
                count_data = 0;
            }
            break;

        case ParseState::WAITING_FOR_PITCH_STATUS:
            _parsed_msg.data_bytes_received++;
            count_data++;
            if (count_data >= 18)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_YAW_STATUS;
                count_data = 0;
            }
            break;

        case ParseState::WAITING_FOR_YAW_STATUS:
            _parsed_msg.data_bytes_received++;
            count_data++;
            if (count_data >= 18)
            {
                _parsed_msg.state = ParseState::WAITING_FOR_CHECKSUM;
                count_data = 0;
            }
            break;

        case ParseState::WAITING_FOR_CHECKSUM:
            _parsed_msg.check_sum = received_byte;

            // check crc
            // uint8_t debug_buffer[58] = {0x3E, 0x3D, 0x36, 0x73, 0xCC, 0xFD, 0xD3, 0xFD, 0x61, 0xFD, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
            // 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // 0x00, 0x00};//, 0xFF}; uint16_t size_debug_buffer = sizeof(debug_buffer) / sizeof(uint8_t); const uint8_t expected_debug_check_sum = two_complement_checksum(debug_buffer +
            // header_size, _parsed_msg.data_bytes_received - header_size);

            const uint8_t expected_check_sum = two_complement_checksum(_msg_buff + header_size, _parsed_msg.data_bytes_received - header_size);
            _parsed_msg.data_bytes_received++;
            if (expected_check_sum == _parsed_msg.check_sum)
            {
                // successfully received a message, do something with it
                process_packet();

                _count_received_packets++;
            }
            else
            {
                _count_broken_packets++;
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser)
        {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER_BYTE_1;
            _msg_buff_len = 0;
            header_size = 0;
            _parsed_msg.data_bytes_received = 0;
        }
    }

    perf_end(_sample_perf);

    // Trigger the next measurement.
    return_value = measure();
    return return_value;
}

int ViewPro::measure()
{
    request_get_angles_ext();

    // send_target_angles(radians(30.0f), radians(10.0f), radians(90.0f), false);

    _measurement_time = hrt_absolute_time();
    _buffer_len = 0;

    return PX4_OK;
}

int ViewPro::update()
{
    int return_value = 0;

    gimbal_device_attitude_status_s attitude_status{};

    if (attitude_status_sub.update(&attitude_status))
    {
    }

    return return_value;
}

int ViewPro::open_serial_port(const speed_t speed)
{
    // File descriptor already initialized?
    if (_file_descriptor > 0)
    {
        // PX4_INFO("serial port already open");
        return PX4_OK;
    }

    // Configure port flags for read/write, non-controlling, non-blocking.
    int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

    // Open the serial port.
    _file_descriptor = ::open(_serial_port, flags);

    if (_file_descriptor < 0)
    {
        PX4_ERR("open failed (%i)", errno);
        return PX4_ERROR;
    }

    termios uart_config = {};

    // Store the current port configuration. attributes.
    if (tcgetattr(_file_descriptor, &uart_config))
    {
        PX4_ERR("Unable to get termios from %s.", _serial_port);
        ::close(_file_descriptor);
        _file_descriptor = -1;
        return PX4_ERROR;
    }

    // Clear: data bit size, two stop bits, parity, hardware flow control.
    uart_config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);

    // Set: 8 data bits, enable receiver, ignore modem status lines.
    uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

    // Clear: echo, echo new line, canonical input and extended input.
    uart_config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

    // Clear ONLCR flag (which appends a CR for every LF).
    uart_config.c_oflag &= ~ONLCR;

    // Set the input baud rate in the uart_config struct.
    int termios_state = cfsetispeed(&uart_config, speed);

    if (termios_state < 0)
    {
        PX4_ERR("CFG: %d ISPD", termios_state);
        ::close(_file_descriptor);
        return PX4_ERROR;
    }

    // Set the output baud rate in the uart_config struct.
    termios_state = cfsetospeed(&uart_config, speed);

    if (termios_state < 0)
    {
        PX4_ERR("CFG: %d OSPD", termios_state);
        ::close(_file_descriptor);
        return PX4_ERROR;
    }

    // Apply the modified port attributes.
    termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

    if (termios_state < 0)
    {
        PX4_ERR("baud %d ATTR", termios_state);
        ::close(_file_descriptor);
        return PX4_ERROR;
    }

    // Flush the hardware buffers.
    tcflush(_file_descriptor, TCIOFLUSH);

    PX4_DEBUG("opened UART port %s", _serial_port);

    return PX4_OK;
}

void ViewPro::print_info()
{
    perf_print_counter(_comms_error);
    perf_print_counter(_sample_perf);
}

void ViewPro::Run()
{
    // Ensure the serial port is open.
    open_serial_port();
    collect();
    update();
}

void ViewPro::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(VIEWPRO_MEASURE_INTERVAL, VIEWPRO_MEASURE_INTERVAL);
}

void ViewPro::stop()
{
    // Ensure the serial port is closed.
    ::close(_file_descriptor);
    _file_descriptor = -1;

    // Clear the work queue schedule.
    ScheduleClear();
}

// process successfully decoded packets held in the _parsed_msg structure
void ViewPro::process_packet()
{
    // flag to warn of unexpected data buffer length
    bool unexpected_len = false;

    // process packet depending upon command id

    if (_parsed_msg.data_bytes_received != CMD_GET_ANGLES_EXT_PACKET_LEN)
    {
        unexpected_len = true;
    }

    _last_current_angle_rad_ms = hrt_absolute_time();

    memcpy(&angles_packet.packet_array, &_msg_buff, sizeof(_msg_buff) / sizeof(uint8_t));

    //_current_imu_angle_degree.x = angles_packet.packet.roll_imu_angle_units * angle_units_to_degree_ratio;
    //_current_imu_angle_degree.y = angles_packet.packet.pitch_imu_angle_units * angle_units_to_degree_ratio;
    //_current_imu_angle_degree.z = angles_packet.packet.yaw_imu_angle_units * angle_units_to_degree_ratio;

    //_current_imu_angle_rad.x = radians(_current_imu_angle_degree.x);
    //_current_imu_angle_rad.y = radians(_current_imu_angle_degree.y);
    //_current_imu_angle_rad.z = radians(_current_imu_angle_degree.z);

    //_current_angle_rad.x = _current_imu_angle_rad.x; // roll
    //_current_angle_rad.y = _current_imu_angle_rad.y; // pitch
    //_current_angle_rad.z = _current_imu_angle_rad.z; // yaw

    // handle unexpected data buffer length
    if (unexpected_len)
    {
    }
}

// methods to send commands to gimbal
// returns true on success, false if outgoing serial buffer is full
bool ViewPro::send_packet(const uint8_t *databuff, uint8_t databuff_len, bool checksum, uint8_t header_size)
{
    // calculate and sanity check packet size
    uint16_t packet_size = 0;

    packet_size = (checksum) ? databuff_len + 1 : databuff_len;
    if (packet_size > VIEWPRO_PACKETLEN_MAX)
    {
        return false;
    }

    // Flush the receive buffer.
    tcflush(_file_descriptor, TCIFLUSH);

    // buffer for holding outgoing packet
    uint8_t send_buff[packet_size] = {};
    uint8_t send_buff_ofs = 0;

    // DATA
    if (databuff_len != 0)
    {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    if (checksum)
    {
        const uint8_t crc = two_complement_checksum(send_buff + header_size, sizeof(send_buff) / sizeof(uint8_t) - header_size);
        send_buff[send_buff_ofs++] = crc;
    }

    // send packet
    int num_bytes = ::write(_file_descriptor, send_buff, sizeof(send_buff));

    if (num_bytes != packet_size)
    {
        PX4_INFO("measurement error: %i, errno: %i", num_bytes, errno);
        return PX4_ERROR;
    }

    _measurement_time = hrt_absolute_time();
    _buffer_len = 0;

    return true;
}

uint16_t ViewPro::crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < crc16_length; i++)
    {
        crc ^= data_frame[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
//@added By Vlad
/*
  Fletcher algorithm for checksum calculation
*/
uint8_t ViewPro::two_complement_checksum(const uint8_t *pBuffer, uint32_t length)
{
    uint8_t return_value = 0;

    uint32_t sum = 0;
    for (uint32_t i = 0; i < (length - 1); i++)
    {
        sum += *(pBuffer + i);
    }
    // modulo 256 sum
    sum %= 256;

    return_value = sum;

    return return_value;
}

// send target roll,pitch and yaw angles to gimbal angles units radians
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
void ViewPro::send_target_angles(float roll_rad, float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // stop gimbal if no recent actual angles
    uint32_t now_ms = hrt_absolute_time();
    if (now_ms - _last_current_angle_rad_ms >= 300000)
    {
        rotate_gimbal(0, 0, 0, ViewControlModeEnum::MODE_ANGLE);
        return;
    }

    rotate_gimbal(roll_rad, pitch_rad, yaw_rad, ViewControlModeEnum::MODE_ANGLE);
}

// rotate gimbal
bool ViewPro::rotate_gimbal(float roll, float pitch, float yaw, ViewControlModeEnum mode)
{
    bool return_value = false;

    cmd_control_movement.control_command.header[0] = (uint8_t) 0xFF;
    cmd_control_movement.control_command.header[1] = (uint8_t) 0x01;
    cmd_control_movement.control_command.header[2] = (uint8_t) 0x0F;
    cmd_control_movement.control_command.header[3] = (uint8_t) 0x10;

    cmd_control_movement.control_command.roll_mode.byte.mode = (uint8_t) mode;
    cmd_control_movement.control_command.pitch_mode.byte.mode = (uint8_t) mode;
    cmd_control_movement.control_command.yaw_mode.byte.mode = (uint8_t) mode;

    if (mode == ViewControlModeEnum::MODE_ANGLE)
    {
        //@todo Vlad check angles limits

        // convert radians to degrees and to units
        cmd_control_movement.control_command.roll_angle_units = degrees(roll) / angle_units_to_degree_ratio;
        cmd_control_movement.control_command.pitch_angle_units = degrees(pitch) / angle_units_to_degree_ratio;
        cmd_control_movement.control_command.yaw_angle_units = degrees(yaw) / angle_units_to_degree_ratio;
    }
    if (mode == ViewControlModeEnum::MODE_SPEED)
    {
        // convert radians/second to degrees/second and to units/second
        cmd_control_movement.control_command.roll_speed_units = degrees(roll) / speed_units_to_degree_ratio;
        cmd_control_movement.control_command.pitch_speed_units = degrees(pitch) / speed_units_to_degree_ratio;
        cmd_control_movement.control_command.yaw_speed_units = degrees(yaw) / speed_units_to_degree_ratio;
    }

    return_value = send_packet(cmd_control_movement.control_array, (sizeof(cmd_control_movement.control_array) / sizeof(uint8_t)), true,
                               sizeof(cmd_control_movement.control_command.header) / sizeof(uint8_t));

    return return_value;
}

//@todo Vlad document added functions
//@note -------------------- Other commands -----------------------------
bool ViewPro::request_get_angles_ext()
{
    bool returnValue = false;
    uint8_t send_message[] = {0x3e, 0x3D, 0x00, 0x3D, 0x00};

    returnValue = send_packet(send_message, (sizeof(send_message) / sizeof(uint8_t)), true);

    return returnValue;
}

float ViewPro::radians(float degrees)
{
    float return_value = 0.0;

    return_value = degrees * (MATH_PI / 180.0f);

    return return_value;
}

float ViewPro::degrees(float radians)
{
    float return_value = 0.0;

    return_value = radians * (180.0f / MATH_PI);

    return return_value;
}
