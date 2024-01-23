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

#pragma region Module methods

DA35EFISerial::DA35EFISerial(const char *serial_port) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port))
{
    _serial_port = strdup(serial_port);

    device::Device::DeviceId device_id;
    device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SERIAL;

    uint8_t bus_num = atoi(&_serial_port[strlen(_serial_port) - 1]); // Assuming '/dev/ttySx'

    if (bus_num < 10)
    {
        device_id.devid_s.bus = bus_num;
    }

    recieved_message = {};
    recieved_message.data = nullptr;
    transmitted_message = {};
    transmitted_message.data = nullptr;

    _count_received_packets = 0;
    _count_broken_packets = 0;
}

DA35EFISerial::~DA35EFISerial()
{
    stop();

    free((char *) _serial_port);
    perf_free(_comms_error);
    perf_free(_sample_perf);
}

int DA35EFISerial::init()
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
            px4_usleep(DA35EFI_MEASURE_INTERVAL);

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

int DA35EFISerial::collect()
{
    int return_value = 0;

    perf_begin(_sample_perf);

    const int buffer_size = sizeof(_buffer);
    uint8_t *data_index = nullptr;
    static uint8_t count_data = 0;

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
        const u_int8_t received_byte = _buffer[i];

        _msg_buff[_msg_buff_len++] = received_byte;

        // protect against overly long messages
        if (_msg_buff_len >= DA35EFI_PACKETLEN_MAX)
        {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (parsing_state)
        {
        case ParseState::WAITING_FOR_SYNC_0:
            data_bytes_received++;
            if (received_byte == SYNC_0)
            {
                parsing_state = ParseState::WAITING_FOR_SYNC_1;
                *((uint8_t *) (&recieved_message.size) + 1) = received_byte;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_SYNC_1:
            data_bytes_received++;
            if (received_byte == SYNC_1)
            {
                parsing_state = ParseState::WAITING_FOR_FLAG;
                *((uint8_t *) (&recieved_message.size) + 0) = received_byte;
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_FLAG:
            data_bytes_received++;
            if (received_byte == 0x01) // realtime data
            {
                if (recieved_message.data == nullptr)
                {
                    recieved_message.data = (uint8_t *) malloc(recieved_message.size);
                    if (recieved_message.data != nullptr)
                    {
                        data_index = recieved_message.data;
                        parsing_state = ParseState::WAITING_FOR_DATA;
                    }
                    else
                    {
                        reset_parser = true;
                    }
                }
            }
            else
            {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_DATA:
            data_bytes_received++;
            count_data++;
            *(data_index++) = received_byte;

            if (count_data >= recieved_message.size - 1) // exclude flag byte
            {
                parsing_state = ParseState::WAITING_FOR_CHECKSUM_0;
                count_data = 0;
            }
            break;

        case ParseState::WAITING_FOR_CHECKSUM_0:
            data_bytes_received++;

            *((uint8_t *) (&recieved_message.check_sum) + 3) = received_byte;

            parsing_state = ParseState::WAITING_FOR_CHECKSUM_1;

            break;

        case ParseState::WAITING_FOR_CHECKSUM_1:
            data_bytes_received++;
            *((uint8_t *) (&recieved_message.check_sum) + 2) = received_byte;

            parsing_state = ParseState::WAITING_FOR_CHECKSUM_2;

            break;

        case ParseState::WAITING_FOR_CHECKSUM_2:
            data_bytes_received++;
            *((uint8_t *) (&recieved_message.check_sum) + 1) = received_byte;

            parsing_state = ParseState::WAITING_FOR_CHECKSUM_3;

            break;

        case ParseState::WAITING_FOR_CHECKSUM_3:
            data_bytes_received++;
            *((uint8_t *) (&recieved_message.check_sum) + 0) = received_byte;

            const uint32_t expected_check_sum = crc32_calc(_msg_buff + 2, data_bytes_received - 6); // exclude 2 bytes size and 4 bytes crc32
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
            data_bytes_received = 0;
            if (recieved_message.data != nullptr)
            {
                free(recieved_message.data);
                recieved_message.data = nullptr;
            }
        }
    }

    perf_end(_sample_perf);

    // Trigger the next measurement.
    return_value = measure();

    return return_value;
}

int DA35EFISerial::measure()
{
    request_realtime_data();

    _measurement_time = hrt_absolute_time();
    _buffer_len = 0;

    return PX4_OK;
}

int DA35EFISerial::update()
{
    int return_value = 0;

    // gimbal_device_attitude_status_s gimbal_device_attitude_status{};
    // gimbal_device_set_attitude_s gimbal_device_set_attitude{};

    // if (_attitude_status_sub.updated())
    // {
    //     if (_attitude_status_sub.copy(&gimbal_device_attitude_status))
    //     {
    //     }
    // }
    // if (_set_attitude_sub.updated())
    // {
    //     if (_set_attitude_sub.copy(&gimbal_device_set_attitude))
    //     {
    //     }
    // }

    // send_target_angles(angles.roll, angles.pitch, angles.yaw, false);

    return return_value;
}

int DA35EFISerial::open_serial_port(const speed_t speed)
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

void DA35EFISerial::print_info()
{
    perf_print_counter(_comms_error);
    perf_print_counter(_sample_perf);
}

void DA35EFISerial::Run()
{
    // Ensure the serial port is open.
    open_serial_port();
    collect();
    // update();
}

void DA35EFISerial::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(DA35EFI_MEASURE_INTERVAL, DA35EFI_MEASURE_INTERVAL);
}

void DA35EFISerial::stop()
{
    // Ensure the serial port is closed.
    ::close(_file_descriptor);
    _file_descriptor = -1;

    // Clear the work queue schedule.
    ScheduleClear();
}

#pragma endregion

#pragma region DA35EFISerial methods
// process successfully decoded packets held in the _parsed_msg structure
void DA35EFISerial::process_packet()
{
    // flag to warn of unexpected data buffer length
    bool unexpected_len = false;

    // process packet depending upon command id

    if (data_bytes_received != CMD_REQUEST_REAL_TIME_PACKET_LEN)
    {
        unexpected_len = true;
    }

    _last_current_real_time_ms = hrt_absolute_time();

    if (recieved_message.data != nullptr)
    {
        memcpy(&realtime_data_packet.packet_array, recieved_message.data, recieved_message.size - 1);

        // big endian swap variables
        realtime_data_packet.packet.seconds = swap_uint16(realtime_data_packet.packet.seconds);
        realtime_data_packet.packet.pulseWidth1 = swap_uint16(realtime_data_packet.packet.pulseWidth1);
        realtime_data_packet.packet.pulseWidth2 = swap_uint16(realtime_data_packet.packet.pulseWidth2);
        realtime_data_packet.packet.rpm = swap_uint16(realtime_data_packet.packet.rpm);
        realtime_data_packet.packet.advance = swap_int16(realtime_data_packet.packet.advance);
        realtime_data_packet.packet.barometer = swap_int16(realtime_data_packet.packet.barometer);
        realtime_data_packet.packet.map = swap_int16(realtime_data_packet.packet.map);
        realtime_data_packet.packet.mat = swap_int16(realtime_data_packet.packet.mat);
        realtime_data_packet.packet.coolant = swap_int16(realtime_data_packet.packet.coolant);
        realtime_data_packet.packet.tps = swap_int16(realtime_data_packet.packet.tps);
        realtime_data_packet.packet.batteryVoltage = swap_int16(realtime_data_packet.packet.batteryVoltage);
        realtime_data_packet.packet.afr1 = swap_int16(realtime_data_packet.packet.afr1);
        realtime_data_packet.packet.afr2 = swap_int16(realtime_data_packet.packet.afr2);
        realtime_data_packet.packet.knock = swap_int16(realtime_data_packet.packet.knock);
        realtime_data_packet.packet.egocor1 = swap_int16(realtime_data_packet.packet.egocor1);
        realtime_data_packet.packet.egocor2 = swap_int16(realtime_data_packet.packet.egocor2);
        realtime_data_packet.packet.aircor = swap_int16(realtime_data_packet.packet.aircor);
        realtime_data_packet.packet.warmcor = swap_int16(realtime_data_packet.packet.warmcor);
        realtime_data_packet.packet.accelEnrich = swap_int16(realtime_data_packet.packet.accelEnrich);
        realtime_data_packet.packet.tpsfuelcut = swap_int16(realtime_data_packet.packet.tpsfuelcut);
        realtime_data_packet.packet.baroCorrection = swap_int16(realtime_data_packet.packet.baroCorrection);
        realtime_data_packet.packet.gammaEnrich = swap_int16(realtime_data_packet.packet.gammaEnrich);
        realtime_data_packet.packet.ve1 = swap_int16(realtime_data_packet.packet.ve1);
        realtime_data_packet.packet.ve2 = swap_int16(realtime_data_packet.packet.ve2);
        realtime_data_packet.packet.iacstep = swap_int16(realtime_data_packet.packet.iacstep);
        realtime_data_packet.packet.cold_adv_deg = swap_int16(realtime_data_packet.packet.cold_adv_deg);
        realtime_data_packet.packet.TPSdot = swap_int16(realtime_data_packet.packet.TPSdot);
        realtime_data_packet.packet.MAPdot = swap_int16(realtime_data_packet.packet.MAPdot);
        realtime_data_packet.packet.dwell = swap_int16(realtime_data_packet.packet.dwell);
        realtime_data_packet.packet.MAF = swap_int16(realtime_data_packet.packet.MAF);
        realtime_data_packet.packet.fuelcor = swap_int16(realtime_data_packet.packet.fuelcor);
        realtime_data_packet.packet.EAEfcor1 = swap_int16(realtime_data_packet.packet.EAEfcor1);
        realtime_data_packet.packet.egoV1 = swap_int16(realtime_data_packet.packet.egoV1);
        realtime_data_packet.packet.egoV2 = swap_int16(realtime_data_packet.packet.egoV2);
        realtime_data_packet.packet.status1 = swap_uint16(realtime_data_packet.packet.status1);
        realtime_data_packet.packet.status2 = swap_uint16(realtime_data_packet.packet.status2);
        realtime_data_packet.packet.status3 = swap_uint16(realtime_data_packet.packet.status3);
        realtime_data_packet.packet.status4 = swap_uint16(realtime_data_packet.packet.status4);
        realtime_data_packet.packet.looptime = swap_uint16(realtime_data_packet.packet.looptime);
        realtime_data_packet.packet.status5 = swap_uint16(realtime_data_packet.packet.status5);
        realtime_data_packet.packet.tpsADC = swap_uint16(realtime_data_packet.packet.tpsADC);
        realtime_data_packet.packet.fuelload2 = swap_int16(realtime_data_packet.packet.fuelload2);
        realtime_data_packet.packet.ignload = swap_int16(realtime_data_packet.packet.ignload);
        realtime_data_packet.packet.ignload2 = swap_int16(realtime_data_packet.packet.ignload2);
        realtime_data_packet.packet.delta = swap_int32(realtime_data_packet.packet.delta);
        realtime_data_packet.packet.wallfuel1 = swap_uint32(realtime_data_packet.packet.wallfuel1);
        realtime_data_packet.packet.gpioadc0 = swap_uint16(realtime_data_packet.packet.gpioadc0);
        realtime_data_packet.packet.gpioadc1 = swap_uint16(realtime_data_packet.packet.gpioadc1);
        realtime_data_packet.packet.gpioadc2 = swap_uint16(realtime_data_packet.packet.gpioadc2);
        realtime_data_packet.packet.gpioadc3 = swap_uint16(realtime_data_packet.packet.gpioadc3);
        realtime_data_packet.packet.gpioadc4 = swap_uint16(realtime_data_packet.packet.gpioadc4);
        realtime_data_packet.packet.gpioadc5 = swap_uint16(realtime_data_packet.packet.gpioadc5);
        realtime_data_packet.packet.gpioadc6 = swap_uint16(realtime_data_packet.packet.gpioadc6);
        realtime_data_packet.packet.gpioadc7 = swap_uint16(realtime_data_packet.packet.gpioadc7);
        realtime_data_packet.packet.gpiopwmin0 = swap_uint16(realtime_data_packet.packet.gpiopwmin0);
        realtime_data_packet.packet.gpiopwmin1 = swap_uint16(realtime_data_packet.packet.gpiopwmin1);
        realtime_data_packet.packet.gpiopwmin2 = swap_uint16(realtime_data_packet.packet.gpiopwmin2);
        realtime_data_packet.packet.gpiopwmin3 = swap_uint16(realtime_data_packet.packet.gpiopwmin3);
        realtime_data_packet.packet.adc6 = swap_uint16(realtime_data_packet.packet.adc6);
        realtime_data_packet.packet.adc7 = swap_uint16(realtime_data_packet.packet.adc7);
        realtime_data_packet.packet.wallfuel2 = swap_int32(realtime_data_packet.packet.wallfuel2);
        realtime_data_packet.packet.EAEFuelCorr2 = swap_uint16(realtime_data_packet.packet.EAEFuelCorr2);
        realtime_data_packet.packet.user0 = swap_uint16(realtime_data_packet.packet.user0);
        realtime_data_packet.packet.inj_adv1 = swap_int16(realtime_data_packet.packet.inj_adv1);
        realtime_data_packet.packet.inj_adv2 = swap_int16(realtime_data_packet.packet.inj_adv2);
        realtime_data_packet.packet.pulseWidth3 = swap_uint16(realtime_data_packet.packet.pulseWidth3);
        realtime_data_packet.packet.pulseWidth4 = swap_uint16(realtime_data_packet.packet.pulseWidth4);
        realtime_data_packet.packet.vetrim1curr = swap_int16(realtime_data_packet.packet.vetrim1curr);
        realtime_data_packet.packet.vetrim2curr = swap_int16(realtime_data_packet.packet.vetrim2curr);
        realtime_data_packet.packet.vetrim3curr = swap_int16(realtime_data_packet.packet.vetrim3curr);
        realtime_data_packet.packet.vetrim4curr = swap_int16(realtime_data_packet.packet.vetrim4curr);
        realtime_data_packet.packet.maf = swap_uint16(realtime_data_packet.packet.maf);
        realtime_data_packet.packet.eaeload1 = swap_int16(realtime_data_packet.packet.eaeload1);
        realtime_data_packet.packet.afrload1 = swap_int16(realtime_data_packet.packet.afrload1);
        realtime_data_packet.packet.RPMdot = swap_int16(realtime_data_packet.packet.RPMdot);
        realtime_data_packet.packet.gpioport2 = swap_uint16(realtime_data_packet.packet.gpioport2);
        realtime_data_packet.packet.cl_idle_targ_rpm = swap_int16(realtime_data_packet.packet.cl_idle_targ_rpm);
        realtime_data_packet.packet.maf_volts = swap_uint16(realtime_data_packet.packet.maf_volts);
        realtime_data_packet.packet.airtemp = swap_int16(realtime_data_packet.packet.airtemp);
        realtime_data_packet.packet.dwell_trl = swap_uint16(realtime_data_packet.packet.dwell_trl);
        realtime_data_packet.packet.fuel_pct = swap_uint16(realtime_data_packet.packet.fuel_pct);
        realtime_data_packet.packet.boost_targ = swap_int16(realtime_data_packet.packet.boost_targ);
        realtime_data_packet.packet.ext_advance = swap_int16(realtime_data_packet.packet.ext_advance);
        realtime_data_packet.packet.base_advance = swap_int16(realtime_data_packet.packet.base_advance);
        realtime_data_packet.packet.idle_cor_advance = swap_int16(realtime_data_packet.packet.idle_cor_advance);
        realtime_data_packet.packet.mat_retard = swap_int16(realtime_data_packet.packet.mat_retard);
        realtime_data_packet.packet.flex_advance = swap_int16(realtime_data_packet.packet.flex_advance);
        realtime_data_packet.packet.adv1 = swap_int16(realtime_data_packet.packet.adv1);
        realtime_data_packet.packet.adv2 = swap_int16(realtime_data_packet.packet.adv2);
        realtime_data_packet.packet.adv3 = swap_int16(realtime_data_packet.packet.adv3);
        realtime_data_packet.packet.revlim_retard = swap_int16(realtime_data_packet.packet.revlim_retard);
        realtime_data_packet.packet.nitrous_retard = swap_int16(realtime_data_packet.packet.nitrous_retard);
        realtime_data_packet.packet.deadtime1 = swap_uint16(realtime_data_packet.packet.deadtime1);
        realtime_data_packet.packet.n2o_addfuel = swap_uint16(realtime_data_packet.packet.n2o_addfuel);

        if (unexpected_len)
        {
        }

        _injector_duty_cycle = ((float) realtime_data_packet.packet.pulseWidth1 * (float) realtime_data_packet.packet.rpm) / 60000.0f;
        _fuel_flow_rate_instantaneos = (static_injector_flow_rate / 60.0f) * _injector_duty_cycle * density_of_fuel; // g/s
        _scaled_flow_rate = _fuel_flow_rate_instantaneos * (scalar_divisor / 1000.0f);                               // g/s

        //@todo Vlad insert relevant data to publish structure and publish it

        _hfe_da35efi_status.seconds = realtime_data_packet.packet.seconds;
        _hfe_da35efi_status.pulse_width_1 = realtime_data_packet.packet.pulseWidth1 / 0.000666f;
        _hfe_da35efi_status.pulse_width_2 = realtime_data_packet.packet.pulseWidth2 / 0.000666f;
        _hfe_da35efi_status.rpm = realtime_data_packet.packet.rpm;
        _hfe_da35efi_status.engine = realtime_data_packet.packet.engine;
        _hfe_da35efi_status.barometer = realtime_data_packet.packet.barometer / 10.0f;
        _hfe_da35efi_status.coolant = realtime_data_packet.packet.coolant / 10.0f;
        _hfe_da35efi_status.tps = realtime_data_packet.packet.tps / 10.0f;
        _hfe_da35efi_status.battery_voltage = realtime_data_packet.packet.batteryVoltage / 10.0f;

        _hfe_da35efi_status.injector_duty_cycle = _injector_duty_cycle;
        _hfe_da35efi_status.fuel_flow_rate_instantaneos = _fuel_flow_rate_instantaneos;
        _hfe_da35efi_status.scaled_flow_rate = _scaled_flow_rate;

        _da35efi_status_pub.publish(_hfe_da35efi_status);
    }
}
// returns true on success, false if outgoing serial buffer is full
bool DA35EFISerial::send_packet(const uint8_t *databuff, uint16_t databuff_len, bool checksum)
{
    // calculate and sanity check packet size
    uint16_t packet_size = 0;

    packet_size = (checksum) ? databuff_len + 2 + 4 : databuff_len; // 2 bytes size and 4 bytes crc32
    if (packet_size > DA35EFI_PACKETLEN_MAX)
    {
        return false;
    }

    // Flush the receive buffer.
    tcflush(_file_descriptor, TCIFLUSH);

    // buffer for holding outgoing packet
    uint8_t send_buff[packet_size] = {}; // packet size + 2 bytes size
    uint8_t send_buff_ofs = 0;

    send_buff[0] = *(((uint8_t *) &databuff_len) + 1);
    send_buff_ofs++;
    send_buff[1] = *(((uint8_t *) &databuff_len) + 0);
    send_buff_ofs++;

    // DATA
    if (databuff_len != 0)
    {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    if (checksum)
    {
        const uint32_t crc = crc32_calc(send_buff + 2, databuff_len);

        send_buff[send_buff_ofs++] = *(((uint8_t *) &crc) + 3);
        send_buff[send_buff_ofs++] = *(((uint8_t *) &crc) + 2);
        send_buff[send_buff_ofs++] = *(((uint8_t *) &crc) + 1);
        send_buff[send_buff_ofs++] = *(((uint8_t *) &crc) + 0);
    }

    // send packet
    int num_bytes = ::write(_file_descriptor, send_buff, sizeof(send_buff));

    if (num_bytes != sizeof(send_buff))
    {
        PX4_INFO("measurement error: %i, errno: %i", num_bytes, errno);
        return PX4_ERROR;
    }

    _measurement_time = hrt_absolute_time();
    _buffer_len = 0;

    return true;
}

uint16_t DA35EFISerial::crc16_calc(const unsigned char *data_frame, const uint8_t crc16_length)
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
/*----------------------------------------------------------------------------*\
 *  NAME:
 *     Crc32_ComputeBuf() - computes the CRC-32 value of a memory buffer
 *  DESCRIPTION:
 *     Computes or accumulates the CRC-32 value for a memory buffer.
 *  ARGUMENTS:
 *     buf     - buffer to compute CRC-32 value for
 *     bufLen  - number of bytes in buffer
 *  RETURNS:
 *     crc32 - computed CRC-32 value
 *  ERRORS:
 *     (no errors are possible)
\*----------------------------------------------------------------------------*/
uint32_t DA35EFISerial::crc32_calc(const unsigned char *data_frame, const uint32_t crc32_length)
{
    static const unsigned long crcTable[256] = {
        0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3, 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07,
        0x90BF1D91, 0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9,
        0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3,
        0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
        0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934, 0x9609A88E,
        0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
        0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A,
        0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
        0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615,
        0x73DC1683, 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1, 0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB,
        0x196C3671, 0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1,
        0xA6BC5767, 0x3FB506DD, 0x48B2364B, 0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
        0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 0x9B64C2B0, 0xEC63F226, 0x756AA39C,
        0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242,
        0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777, 0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45, 0xA00AE278,
        0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
        0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B,
        0x2D02EF8D};

    uint32_t crc32 = 0xFFFFFFFF;
    uint8_t *byteBuf;
    uint32_t i;

    /** accumulate crc32 for buffer **/
    byteBuf = (uint8_t *) data_frame;
    for (i = 0; i < crc32_length; i++)
    {
        crc32 = (crc32 >> 8) ^ crcTable[(crc32 ^ byteBuf[i]) & 0xFF];
    }

    return (crc32 ^ 0xFFFFFFFF);
}
/*
  Fletcher algorithm for checksum calculation
*/
uint8_t DA35EFISerial::two_complement_checksum(const uint8_t *pBuffer, uint32_t length)
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
uint16_t DA35EFISerial::fletcher_encode(const uint8_t *buffer, uint16_t count)
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

bool DA35EFISerial::request_realtime_data()
{
    bool returnValue = false;
    uint8_t send_message[] = {0x41}; //'A'

    returnValue = send_packet(send_message, (sizeof(send_message) / sizeof(uint8_t)), true);

    return returnValue;
}

#pragma endregion
