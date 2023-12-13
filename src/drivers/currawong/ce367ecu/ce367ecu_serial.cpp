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

#include "ce367ecu_serial.hpp"

#pragma region Module methods

CE367ECUSerial::CE367ECUSerial(const char *serial_port) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port))
{
     int res = stm32_fdcansockinitialize(0);

    // int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // fdcan_driver_s g_fdcan0;
    //fdcan_receive(&g_fdcan0);
    // arm_netinitialize();

    _serial_port = strdup(serial_port);

    device::Device::DeviceId device_id;
    device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

    uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

    if (bus_num < 10)
    {
        device_id.devid_s.bus = bus_num;
    }

    recieved_message = {};
    recieved_message.data = (uint8_t *) malloc(10);
    transmitted_message = {};
    transmitted_message.data = (uint8_t *) malloc(10);

    _count_received_packets = 0;
    _count_broken_packets = 0;
}

CE367ECUSerial::~CE367ECUSerial()
{
    stop();

    free((char *) _serial_port);
    perf_free(_comms_error);
    perf_free(_sample_perf);
}

int CE367ECUSerial::init()
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
            px4_usleep(CE367ECU_MEASURE_INTERVAL);

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

int CE367ECUSerial::collect()
{
    int return_value = 0;

    perf_begin(_sample_perf);

    const int buffer_size = sizeof(_buffer);

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

    if (_buffer_len < bytes_read)
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
        if (_msg_buff_len >= CE367ECU_PACKETLEN_MAX)
        {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (parsing_state)
        {

        case ParseState::WAITING_FOR_SYNC_0:
            data_bytes_received++;
            header_size++;
            if (received_byte == SYNC_0)
            {
                parsing_state = ParseState::WAITING_FOR_SYNC_1;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_SYNC_1:
            data_bytes_received++;
            header_size++;
            if (received_byte == SYNC_1)
            {
                parsing_state = ParseState::WAITING_FOR_ADDRESS_HI;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_ADDRESS_HI:
            data_bytes_received++;
            header_size++;

            parsing_state = ParseState::WAITING_FOR_ADDRESS_LOW;

            break;

        case ParseState::WAITING_FOR_ADDRESS_LOW:
            data_bytes_received++;
            header_size++;

            parsing_state = ParseState::WAITING_FOR_MESSAGE_TYPE;

            break;

        case ParseState::WAITING_FOR_MESSAGE_TYPE:
            data_bytes_received++;
            header_size++;

            parsing_state = ParseState::WAITING_FOR_MESSAGE_SIZE;

            break;

        case ParseState::WAITING_FOR_MESSAGE_SIZE:
            data_bytes_received++;
            recieved_message.message_size = received_byte;

            if (recieved_message.message_size > 0)
            {
                recieved_message.data = (uint8_t *) realloc(recieved_message.data, recieved_message.message_size);
                parsing_state = ParseState::WAITING_FOR_DATA;
            }
            else
            {
                parsing_state = ParseState::WAITING_FOR_CHECKSUM_0;
            }

            break;

        case ParseState::WAITING_FOR_DATA:
            data_bytes_received++;
            count_data++;
            *(recieved_message.data++) = received_byte;

            if (count_data >= recieved_message.message_size)
            {
                parsing_state = ParseState::WAITING_FOR_CHECKSUM_0;
                count_data = 0;
            }
            break;

        case ParseState::WAITING_FOR_CHECKSUM_0:
            data_bytes_received++;
            recieved_message.check_sum = received_byte;
            recieved_message.check_sum = recieved_message.check_sum << 8;

            parsing_state = ParseState::WAITING_FOR_CHECKSUM_1;

            break;

        case ParseState::WAITING_FOR_CHECKSUM_1:
            data_bytes_received++;
            recieved_message.check_sum = received_byte;

            const uint16_t expected_check_sum = fletcher_encode(_msg_buff, data_bytes_received - 2);
            if (expected_check_sum == recieved_message.check_sum)
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
            parsing_state = ParseState::WAITING_FOR_SYNC_0;
            _msg_buff_len = 0;
            header_size = 0;
            data_bytes_received = 0;
        }
    }

    perf_end(_sample_perf);

    // Trigger the next measurement.
    return_value = measure();
    return return_value;
}

int CE367ECUSerial::measure()
{
    request_get_angles_ext();

    _measurement_time = hrt_absolute_time();
    _buffer_len = 0;

    return PX4_OK;
}

int CE367ECUSerial::update()
{
    int return_value = 0;

    gimbal_device_attitude_status_s gimbal_device_attitude_status{};
    gimbal_device_set_attitude_s gimbal_device_set_attitude{};

    // MavlinkV1
    if (_attitude_status_sub.updated())
    {
        if (_attitude_status_sub.copy(&gimbal_device_attitude_status))
        {
        }
    }
    // MavlinkV2
    if (_set_attitude_sub.updated())
    {
        if (_set_attitude_sub.copy(&gimbal_device_set_attitude))
        {
        }
    }

    // send_target_angles(angles.roll, angles.pitch, angles.yaw, false);

    return return_value;
}

int CE367ECUSerial::open_serial_port(const speed_t speed)
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

void CE367ECUSerial::print_info()
{
    perf_print_counter(_comms_error);
    perf_print_counter(_sample_perf);
}

void CE367ECUSerial::Run()
{
    // Ensure the serial port is open.
    open_serial_port();
    collect();
    update();
}

void CE367ECUSerial::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(CE367ECU_MEASURE_INTERVAL, CE367ECU_MEASURE_INTERVAL);
}

void CE367ECUSerial::stop()
{
    // Ensure the serial port is closed.
    ::close(_file_descriptor);
    _file_descriptor = -1;

    // Clear the work queue schedule.
    ScheduleClear();
}

#pragma endregion

#pragma region CE367ECUSerial methods
// process successfully decoded packets held in the _parsed_msg structure
void CE367ECUSerial::process_packet()
{
    // flag to warn of unexpected data buffer length
    bool unexpected_len = false;

    // process packet depending upon command id

    if (data_bytes_received != CMD_GET_ANGLES_EXT_PACKET_LEN)
    {
        unexpected_len = true;
    }

    _last_current_angle_rad_ms = hrt_absolute_time();

    memcpy(&angles_packet.packet_array, &_msg_buff, sizeof(_msg_buff) / sizeof(uint8_t));

    if (unexpected_len)
    {
    }
}
// methods to send commands to gimbal
// returns true on success, false if outgoing serial buffer is full
bool CE367ECUSerial::send_packet(const uint8_t *databuff, uint8_t databuff_len, bool checksum, uint8_t header_size)
{
    // calculate and sanity check packet size
    uint16_t packet_size = 0;

    packet_size = (checksum) ? databuff_len + 2 : databuff_len;
    if (packet_size > CE367ECU_PACKETLEN_MAX)
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
        const uint16_t crc = fletcher_encode(send_buff, sizeof(send_buff) / sizeof(uint8_t));
        send_buff[send_buff_ofs++] = (uint8_t) (crc & 0x00FF);
        send_buff[send_buff_ofs++] = (uint8_t) ((crc & 0xFF00) >> 8);
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

uint16_t CE367ECUSerial::crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length)
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
/*
  Fletcher algorithm for checksum calculation
*/
uint8_t CE367ECUSerial::two_complement_checksum(const uint8_t *pBuffer, uint32_t length)
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
// Encode sequential bytes in a memory buffer using the fletcher algorithm
uint16_t CE367ECUSerial::fletcher_encode(const uint8_t *buffer, uint16_t count)
{
    uint16_t i;
    uint8_t c0 = 0;
    uint8_t c1 = 0;
    uint16_t checksum;

    // Iterate over the data
    for (i = 0; i < count; i++)
    {
        c0 = (uint8_t) (c0 + buffer[i]);
        c1 = (uint8_t) (c0 + c1);
    }
    checksum = (uint8_t) (c0 - c1);
    checksum <<= 8;
    checksum += (uint8_t) (c1 - 2 * c0);

    return checksum;
}
// rotate gimbal
bool CE367ECUSerial::rotate_gimbal(float roll, float pitch, float yaw, ViewControlModeEnum mode)
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
        cmd_control_movement.control_command.roll_angle_units = 0;
        cmd_control_movement.control_command.pitch_angle_units = 0;
        cmd_control_movement.control_command.yaw_angle_units = 0;
    }
    if (mode == ViewControlModeEnum::MODE_SPEED)
    {
        // convert radians/second to degrees/second and to units/second
        cmd_control_movement.control_command.roll_speed_units = 0;
        cmd_control_movement.control_command.pitch_speed_units = 0;
        cmd_control_movement.control_command.yaw_speed_units = 0;
    }
    if (mode == ViewControlModeEnum::MODE_SPEED_ANGLE)
    {
        //@todo Vlad check angles limits

        // convert radians to degrees and to units
        cmd_control_movement.control_command.roll_angle_units = 0;
        cmd_control_movement.control_command.pitch_angle_units = 0;
        cmd_control_movement.control_command.yaw_angle_units = 0;

        cmd_control_movement.control_command.roll_speed_units = 5.0;
        cmd_control_movement.control_command.pitch_speed_units = 5.0;
        cmd_control_movement.control_command.yaw_speed_units = 5.0;
    }
    if (mode == ViewControlModeEnum::MODE_ANGLE_REF_FRAME)
    {
        //@todo Vlad check angles limits

        // convert radians to degrees and to units
        cmd_control_movement.control_command.roll_angle_units = 0;
        cmd_control_movement.control_command.pitch_angle_units = 0;
        cmd_control_movement.control_command.yaw_angle_units = 0;
    }

    return_value = send_packet(cmd_control_movement.control_array, (sizeof(cmd_control_movement.control_array) / sizeof(uint8_t)), true,
                               sizeof(cmd_control_movement.control_command.header) / sizeof(uint8_t));

    return return_value;
}

bool CE367ECUSerial::request_get_angles_ext()
{
    bool returnValue = false;
    uint8_t send_message[] = {0x3e, 0x3D, 0x00, 0x3D, 0x00};

    returnValue = send_packet(send_message, (sizeof(send_message) / sizeof(uint8_t)), true);

    return returnValue;
}

#pragma endregion
