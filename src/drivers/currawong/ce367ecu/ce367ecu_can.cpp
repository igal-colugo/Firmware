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

/**
 * @file ce367ecu_can.cpp
 * @author Vlad Smirnov <vlad@colugo-sys.com>
 *
 * Driver for the Currawong ce367ecu engine controller connected over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Currawong datasheet CE367 ECU Type B. Other models of Currawong engine controllers
 * will NOT work with this driver in its current form.
 *
 */

#include "ce367ecu_can.hpp"

CE367ECUCan::CE367ECUCan(int can_port, bool is_collector) : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::can)
{
    int res = 0;

    res = stm32_fdcansockinitialize(0);
    res = stm32_fdcansockinitialize(1);

    netlib_ifup("can0");
    netlib_ifup("can1");

    _is_collector = is_collector;
    _can_port = can_port;
    _real_number_devices = 1;
    _ecu_id = 0xFFFF;
    _pmu_id = 0xFFFF;

    _initialized = true;

    if (PX4_OK != param_get(param_find("CE367ECU_MTR"), &_motor_index))
    {
        PX4_ERR("Could not get param CE367ECU_MTR");
    }
    if (_motor_index < 1)
    {
        _motor_index = 1;
    }
    if (_motor_index > 8)
    {
        _motor_index = 8;
    }
    _motor_index = _motor_index - 1;
    if (PX4_OK != param_get(param_find("CE367ECU_MTR_MIN"), &_motor_minimum_value))
    {
        PX4_ERR("Could not get param CE367ECU_MTR_MIN");
    }
    if (PX4_OK != param_get(param_find("CE367ECU_MTR_MAX"), &_motor_maximum_value))
    {
        PX4_ERR("Could not get param CE367ECU_MTR_MAX");
    }

    // int task_id = px4_task_spawn_cmd("ce367ecu_collect", SCHED_DEFAULT, SCHED_PRIORITY_SLOW_DRIVER, 1024, collect, (char *const *) nullptr);
}

CE367ECUCan::~CE367ECUCan()
{
    netlib_ifdown("can0");
    netlib_ifdown("can1");
}

void CE367ECUCan::Run()
{
    //@example send_data(1, "00000000#AAAABBBBCCCCDDDDAAAABBBB");
    if (_is_collector == false)
    {
        update();
    }
    else
    {
        collect();
    }
    // px4_sleep(50);
}

void CE367ECUCan::start(uint32_t interval_us)
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(interval_us);
}

void CE367ECUCan::stop()
{
    // Clear the work queue schedule.
    ScheduleClear();
}

int CE367ECUCan::update()
{
    int return_value = 0;

    //@example
    /*
    canfd_frame out_frame = {};
    memset(&out_frame, 0, sizeof(canfd_frame)); //init CAN FD frame, e.g. LEN = 0

    PiccoloFrameID piccolo_frame_id = {};
    memset(&piccolo_frame_id, 0, sizeof(PiccoloFrameID));

    piccolo_frame_id.word.frame_format_flag = 1;
    piccolo_frame_id.word.group_id = 0x08;
    piccolo_frame_id.word.message_type = 0x03;
    piccolo_frame_id.word.device_address = 0xFFFF;

    out_frame.can_id = piccolo_frame_id.frame_id; // 0x8803FFFF; // 0x8300FFFF;
    out_frame.len = 4;
    out_frame.data[0] = 55;
    out_frame.data[1] = 55;
    out_frame.data[2] = 55;
    out_frame.data[3] = 55;
    // out_frame.data[4] = 55;
    // out_frame.data[5] = 55;
    // out_frame.data[6] = 55;
    // out_frame.data[7] = 55;

    return_value = write_frame(can_port, real_number_devices, &out_frame, 0);
    */

    //@note Vlad in implementation state

    uint16_t servoCmdRate = 2;
    uint16_t ecuCmdRate = 2;

    //@todo Vlad:subscribe to pwm uorb messages

    // MavlinkV1
    if (_actuator_outputs_sub.updated())
    {
        if (_actuator_outputs_sub.copy(&_actuator_outputs))
        {
            send_ecu_messages((((float) _actuator_outputs.output[_motor_index] - (float) _motor_minimum_value) / abs((float) _motor_maximum_value - (float) _motor_minimum_value)) * 100.0f);
        }
    }

    if (_vehicle_command_sub.update(&_vehicle_command))
    {
        if (_vehicle_command.command == 60601)
        {
            start_cranking_engine();
        }
    }

    // Transmit ecu throttle commands at regular intervals
    if (_ecu_tx_counter >= 10)
    {
        _ecu_tx_counter = 0;
        //@todo Vlad:set scaled value from throttle motor
        // send_ecu_messages(50.0);
        // start_cranking_engine();
    }

    /*
    // Transmit ESC commands at regular intervals
    if (_esc_tx_counter++ > escCmdRate)
    {
        _esc_tx_counter = 0;
        send_esc_messages();
        }
        // Transmit servo commands at regular intervals
        if (_servo_tx_counter++ > servoCmdRate)
        {
            _servo_tx_counter = 0;
            send_servo_messages();
        }
    */

    _ecu_tx_counter++;

    return return_value;
}

/*
    send CAN-frames via CAN_RAW sockets.
    <device> <can_frame>
    <can_frame>
    <can_id>#{data} for 'classic' CAN 2.0 data frames
    <can_id>#R{len} for 'classic' CAN 2.0 data frames
    <can_id>##<flags>{data} for CAN FD frames
    <can_id>
    3 (SFF) or 8 (EFF) hex chars
    {data}
    0..8 (0..64 CAN FD) ASCII hex-values (optionally separated by '.'
    {len}
    an optional 0..8 value as RTR frames can contain a valid dlc field
    <flags>
    a single ASCII Hex value (0 .. F) which defines canfd_frame.flags
    Examples:
    5A1#11.2233.44556677.88 / 123#DEADBEEF / 5AA# / 123##1 / 213##311223344 /
    1F334455#1122334455667788 / 123#R / 00000123#R3
*/
int CE367ECUCan::send_data(int can_port, char *data)
{
    int return_value = 0;
    char *port = nullptr;
    int raw_socket = 0; /* can raw socket */
    int required_mtu = 0;
    int mtu = 0;
    int enable_canfd = 1;
    struct sockaddr_can addr = {};
    struct canfd_frame frame = {};
    struct ifreq ifr = {};

    if (can_port == 0)
    {
        port = "can0";
    }
    if (can_port == 1)
    {
        port = "can1";
    }

    /* parse CAN frame */
    required_mtu = parse_canframe(data, &frame);
    if (!required_mtu)
    {
        return -1;
    }

    /* open socket */
    if ((raw_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        return -1;
    }

    strncpy(ifr.ifr_name, port, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex)
    {
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (required_mtu > (int) CAN_MTU)
    {
        /* check if the frame fits into the CAN netdevice */
        if (ioctl(raw_socket, SIOCGIFMTU, &ifr) < 0)
        {
            return -1;
        }
        mtu = ifr.ifr_mtu;

        if (mtu != CANFD_MTU)
        {
            return -1;
        }

        /* interface is ok - try to switch the socket into CAN FD mode */
        if (setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)))
        {
            return -1;
        }

        /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
        frame.len = can_dlc2len(can_len2dlc(frame.len));
    }

    /* disable default receive filter on this RAW socket */
    /* This is obsolete as we do not read from the socket at all, but for */
    /* this reason we can remove the receive list in the Kernel to save a */
    /* little (really a very little!) CPU usage.                          */
    setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if (bind(raw_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        return -1;
    }

    /* send frame */
    if (write(raw_socket, &frame, required_mtu) != required_mtu)
    {
        return -1;
    }

    close(raw_socket);

    return return_value;
}

void CE367ECUCan::collect()
{
    canfd_frame recv_frame = {};
    // CAN Frame ID components
    uint8_t frame_id_group;   // Piccolo message group
    uint16_t frame_id_device; // Device identifier
    // int return_value = read_frame(can_port, real_number_devices, &recv_frame, 0);

    // Look for any message responses on the CAN bus
    if (read_frame(_can_port, _real_number_devices, &recv_frame, 0) >= 0)
    {
        // Extract group and device ID values from the frame identifier
        frame_id_group = (recv_frame.can_id >> 24) & 0x1F;
        frame_id_device = (recv_frame.can_id >> 8) & 0xFF;

        if (MessageGroup(frame_id_group) == MessageGroup::ACTUATOR)
        {
            // ESC messages exist in the ACTUATOR group
            if (ActuatorType(frame_id_device) == ActuatorType::SERVO)
            {
                //     if (handle_servo_message(rxFrame))
                //     {
                //         // Returns true if the message was successfully decoded
                //     }
            }
            if (ActuatorType(frame_id_device) == ActuatorType::ESC)
            {
                //     if (handle_esc_message(rxFrame))
                //     {
                //         // Returns true if the message was successfully decoded
                //     }
            }
        }
        else if (MessageGroup(frame_id_group) == MessageGroup::ECU_OUT)
        {
            if (handle_ecu_message(&recv_frame))
            {
                // Returns true if the message was successfully decoded
            }
        }
        else if (PMUMessageGroup(frame_id_group) == PMUMessageGroup::PMU)
        {
            if (handle_pmu_message(&recv_frame))
            {
                // Returns true if the message was successfully decoded
            }
        }
        else
        {
            int debug_1 = 1;
            debug_1 = debug_1 + 1;
        }
    }

    _publisher_counter++;
    if (_publisher_counter > 1) // every 100 ms
    {
        _publisher_counter = 0;
        _currawong_ce367ecu_status_pub.publish(_currawong_ce367ecu_status);
    }
}

int CE367ECUCan::read_frame(int can_port, int real_number_devices, canfd_frame *recv_frame, uint64_t time_out)
{
    int return_value = 0;

    fd_set rdfs;
    char *port = nullptr;
    int raw_socket = 0; //[MAXSOCK] = {};
    unsigned char down_causes_exit = 1;
    unsigned char view = 0;
    int count = 0;
    int rcvbuf_size = 0;
    int opt, ret;
    int currmax, numfilter;
    int join_filter;
    char *ptr, *nptr;
    struct sockaddr_can addr = {};
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
    struct iovec iov = {};
    struct msghdr msg = {};
    struct cmsghdr *cmsg = nullptr;
    struct can_filter *rfilter = nullptr;
    can_err_mask_t err_mask;
    struct canfd_frame *frame = recv_frame;
    int nbytes, i, maxdlen;
    struct ifreq ifr = {};
    struct timeval tv, last_tv;
    struct timeval timeout, timeout_config = {0, 0}, *timeout_current = NULL;

    last_tv.tv_sec = 0;
    last_tv.tv_usec = 0;

    if (can_port == 0)
    {
        port = "can0";
    }
    if (can_port == 1)
    {
        port = "can1";
    }

    //@note open socket
    ptr = port;
    nptr = strchr(ptr, ',');

    raw_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (raw_socket < 0)
    {
        return -1;
    }

    if (nptr)
    {
        nbytes = nptr - ptr; /* interface name is up the first ',' */
    }
    else
    {
        nbytes = strlen(ptr); /* no ',' found => no filter definitions */
    }

    if (nbytes >= IFNAMSIZ)
    {
        return -1;
    }
    if (nbytes > max_devname_len)
    {
        max_devname_len = nbytes; /* for nice printing */
    }

    addr.can_family = AF_CAN;

    memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
    strncpy(ifr.ifr_name, ptr, nbytes);

    if (strcmp(ANYDEV, ifr.ifr_name))
    {
        if (ioctl(raw_socket, SIOCGIFINDEX, &ifr) < 0)
        {
            return -1;
        }
        addr.can_ifindex = ifr.ifr_ifindex;
    }
    else
    {
        addr.can_ifindex = 0; /* any can interface */
    }

    if (nptr)
    {
        /* found a ',' after the interface name => check for filters */
        /* determine number of filters to alloc the filter space */
        numfilter = 0;
        ptr = nptr;
        while (ptr)
        {
            numfilter++;
            ptr++;                  /* hop behind the ',' */
            ptr = strchr(ptr, ','); /* exit condition */
        }

        rfilter = (can_filter *) (malloc(sizeof(struct can_filter) * numfilter));
        if (!rfilter)
        {
            return -1;
        }

        numfilter = 0;
        err_mask = 0;
        join_filter = 0;

        while (nptr)
        {
            ptr = nptr + 1;          /* hop behind the ',' */
            nptr = strchr(ptr, ','); /* update exit condition */

            if (sscanf(ptr, "%" SCNx32 ":%" SCNx32, &rfilter[numfilter].can_id, &rfilter[numfilter].can_mask) == 2)
            {
                rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
                if (*(ptr + 8) == ':')
                {
                    rfilter[numfilter].can_id |= CAN_EFF_FLAG;
                }
                numfilter++;
            }
            else if (sscanf(ptr, "%" SCNx32 "~%" SCNx32, &rfilter[numfilter].can_id, &rfilter[numfilter].can_mask) == 2)
            {
                rfilter[numfilter].can_id |= CAN_INV_FILTER;
                rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
                if (*(ptr + 8) == '~')
                {
                    rfilter[numfilter].can_id |= CAN_EFF_FLAG;
                }
                numfilter++;
            }
            else if (*ptr == 'j' || *ptr == 'J')
            {
                join_filter = 1;
            }
            else if (sscanf(ptr, "#%" SCNx32, &err_mask) != 1)
            {
                return -1;
            }
        }

        if (err_mask)
        {
            setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
        }
        if (join_filter && setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join_filter, sizeof(join_filter)) < 0)
        {
            return -1;
        }
        if (numfilter)
        {
            setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, numfilter * sizeof(struct can_filter));
        }

        free(rfilter);

    } /* if (nptr) */

    /* try to switch the socket into CAN FD mode */
    setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

    if (rcvbuf_size)
    {
        int curr_rcvbuf_size;
        socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

        /* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
        if (setsockopt(raw_socket, SOL_SOCKET, SO_RCVBUFFORCE, &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
        {
            if (setsockopt(raw_socket, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
            {
                return -1;
            }
            if (getsockopt(raw_socket, SOL_SOCKET, SO_RCVBUF, &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0)
            {
                return -1;
            }
        }
    }

    if (bind(raw_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        return -1;
    }

    /* these settings are static and can be held out of the hot path */
    iov.iov_base = frame;
    msg.msg_name = &addr;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = &ctrlmsg;

    FD_ZERO(&rdfs);
    FD_SET(raw_socket, &rdfs);

    if (timeout_current)
    {
        *timeout_current = timeout_config;
    }

    if ((ret = select(raw_socket + 1, &rdfs, NULL, NULL, timeout_current)) <= 0)
    {
        return -1;
    }

    //@note recieve data from socket
    /* check all CAN RAW sockets */
    if (FD_ISSET(raw_socket, &rdfs))
    {
        int idx;

        /* these settings may be modified by recvmsg() */
        iov.iov_len = sizeof(canfd_frame);
        msg.msg_namelen = sizeof(addr);
        msg.msg_controllen = sizeof(ctrlmsg);
        msg.msg_flags = 0;

        nbytes = recvmsg(raw_socket, &msg, 0);
        idx = idx2dindex(addr.can_ifindex, raw_socket);

        if (nbytes < 0)
        {
            if ((errno == ENETDOWN) && !down_causes_exit)
            {
                // continue;
            }
            else
            {
                return -1;
            }
        }

        if ((size_t) nbytes == CAN_MTU)
        {
            maxdlen = CAN_MAX_DLEN;
        }
        else if ((size_t) nbytes == CANFD_MTU)
        {
            maxdlen = CANFD_MAX_DLEN;
        }
        else
        {
            return -1;
        }

        for (cmsg = CMSG_FIRSTHDR(&msg); cmsg && (cmsg->cmsg_level == SOL_SOCKET); cmsg = CMSG_NXTHDR(&msg, cmsg))
        {
            if (cmsg->cmsg_type == SO_TIMESTAMP)
            {
                memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
            }
            else if (cmsg->cmsg_type == SO_TIMESTAMPING)
            {
                struct timespec *stamp = (struct timespec *) CMSG_DATA(cmsg);

                /*
                 * stamp[0] is the software timestamp
                 * stamp[1] is deprecated
                 * stamp[2] is the raw hardware timestamp
                 * See chapter 2.1.2 Receive timestamps in
                 * linux/Documentation/networking/timestamping.txt
                 */
                tv.tv_sec = stamp[2].tv_sec;
                tv.tv_usec = stamp[2].tv_nsec / 1000;
            }
            else if (cmsg->cmsg_type == SO_RXQ_OVFL)
            {
                memcpy(&dropcnt[i], CMSG_DATA(cmsg), sizeof(__u32));
            }
        }

        /* check for (unlikely) dropped frames on this specific socket */
        if (dropcnt[0] != last_dropcnt[0])
        {
            __u32 frames = dropcnt[0] - last_dropcnt[0];

            last_dropcnt[0] = dropcnt[0];
        }

        /* once we detected a EFF frame indent SFF frames accordingly */
        if (frame->can_id & CAN_EFF_FLAG)
        {
            view |= CANLIB_VIEW_INDENT_SFF;
        }
    }
    //@note close socket
    close(raw_socket);

    return return_value;
}
/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
int CE367ECUCan::write_frame(int can_port, int real_number_devices, canfd_frame *out_frame, uint64_t time_out)
{
    int return_value = 0;
    char *port = nullptr;
    int raw_socket = 0; /* can raw socket */
    int required_mtu = 0;
    int mtu = 0;
    int enable_canfd = 1;
    struct sockaddr_can addr = {};
    struct canfd_frame *frame = out_frame;
    struct ifreq ifr = {};

    if (can_port == 0)
    {
        port = "can0";
    }
    if (can_port == 1)
    {
        port = "can1";
    }

    /* open socket */
    if ((raw_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        return -1;
    }

    strncpy(ifr.ifr_name, port, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex)
    {
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    /* check if the frame fits into the CAN netdevice */
    if (ioctl(raw_socket, SIOCGIFMTU, &ifr) < 0)
    {
        return -1;
    }
    mtu = ifr.ifr_mtu;

    if (mtu != CANFD_MTU)
    {
        return -1;
    }

    /* interface is ok - try to switch the socket into CAN FD mode */
    if (setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)))
    {
        return -1;
    }

    /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
    frame->len = can_dlc2len(can_len2dlc(frame->len));

    /* disable default receive filter on this RAW socket */
    /* This is obsolete as we do not read from the socket at all, but for */
    /* this reason we can remove the receive list in the Kernel to save a */
    /* little (really a very little!) CPU usage.                          */
    setsockopt(raw_socket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if (bind(raw_socket, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        return -1;
    }

    /* send frame */
    //@note Vlad: size is 8 bytes message id and 8 bytes of data == 16 bytes
    return_value = write(raw_socket, frame, sizeof(can_frame));

    close(raw_socket);

    return return_value;
}

void CE367ECUCan::print_info()
{
    perf_print_counter(_comms_error);
    perf_print_counter(_sample_perf);
}

#pragma region Can send functions
/* get data length from can_dlc with sanitized can_dlc */
unsigned char CE367ECUCan::can_dlc2len(unsigned char can_dlc)
{
    return dlc2len[can_dlc & 0x0F];
}
/* map the sanitized data length to an appropriate data length code */
unsigned char CE367ECUCan::can_len2dlc(unsigned char len)
{
    if (len > 64)
        return 0xF;

    return len2dlc[len];
}

unsigned char CE367ECUCan::asc2nibble(char c)
{

    if ((c >= '0') && (c <= '9'))
        return c - '0';

    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;

    return 16; /* error */
}

int CE367ECUCan::hexstring2data(char *arg, unsigned char *data, int maxdlen)
{

    int len = strlen(arg);
    int i;
    unsigned char tmp;

    if (!len || len % 2 || len > maxdlen * 2)
        return 1;

    memset(data, 0, maxdlen);

    for (i = 0; i < len / 2; i++)
    {

        tmp = asc2nibble(*(arg + (2 * i)));
        if (tmp > 0x0F)
            return 1;

        data[i] = (tmp << 4);

        tmp = asc2nibble(*(arg + (2 * i) + 1));
        if (tmp > 0x0F)
            return 1;

        data[i] |= tmp;
    }

    return 0;
}

int CE367ECUCan::parse_canframe(char *cs, struct canfd_frame *cf)
{
    /* documentation see lib.h */

    int i, idx, dlen, len;
    int maxdlen = CAN_MAX_DLEN;
    int ret = CAN_MTU;
    unsigned char tmp;

    len = strlen(cs);
    // printf("'%s' len %d\n", cs, len);

    memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

    if (len < 4)
        return 0;

    if (cs[3] == CANID_DELIM)
    { /* 3 digits */

        idx = 4;
        for (i = 0; i < 3; i++)
        {
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (2 - i) * 4);
        }
    }
    else if (cs[8] == CANID_DELIM)
    { /* 8 digits */

        idx = 9;
        for (i = 0; i < 8; i++)
        {
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (7 - i) * 4);
        }
        if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
            cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */
    }
    else
        return 0;

    if ((cs[idx] == 'R') || (cs[idx] == 'r'))
    { /* RTR frame */
        cf->can_id |= CAN_RTR_FLAG;

        /* check for optional DLC value for CAN 2.0B frames */
        if (cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
            cf->len = tmp;

        return ret;
    }

    if (cs[idx] == CANID_DELIM)
    { /* CAN FD frame escape char '##' */

        maxdlen = CANFD_MAX_DLEN;
        ret = CANFD_MTU;

        /* CAN FD frame <canid>##<flags><data>* */
        if ((tmp = asc2nibble(cs[idx + 1])) > 0x0F)
            return 0;

        cf->flags = tmp;
        idx += 2;
    }

    for (i = 0, dlen = 0; i < maxdlen; i++)
    {

        if (cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
            idx++;

        if (idx >= len) /* end of string => end of data */
            break;

        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] = (tmp << 4);
        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] |= tmp;
        dlen++;
    }
    cf->len = dlen;

    return ret;
}

void CE367ECUCan::fprint_canframe(FILE *stream, struct canfd_frame *cf, char *eol, int sep, int maxdlen)
{
    /* documentation see lib.h */

    char buf[CL_CFSZ]; /* max length */

    sprint_canframe(buf, cf, sep, maxdlen);
    fprintf(stream, "%s", buf);
    if (eol)
        fprintf(stream, "%s", eol);
}

void CE367ECUCan::sprint_canframe(char *buf, struct canfd_frame *cf, int sep, int maxdlen)
{
    /* documentation see lib.h */

    int i, offset;
    int len = (cf->len > maxdlen) ? maxdlen : cf->len;

    if (cf->can_id & CAN_ERR_FLAG)
    {
        put_eff_id(buf, cf->can_id & (CAN_ERR_MASK | CAN_ERR_FLAG));
        buf[8] = '#';
        offset = 9;
    }
    else if (cf->can_id & CAN_EFF_FLAG)
    {
        put_eff_id(buf, cf->can_id & CAN_EFF_MASK);
        buf[8] = '#';
        offset = 9;
    }
    else
    {
        put_sff_id(buf, cf->can_id & CAN_SFF_MASK);
        buf[3] = '#';
        offset = 4;
    }

    /* standard CAN frames may have RTR enabled. There are no ERR frames with RTR */
    if (maxdlen == CAN_MAX_DLEN && cf->can_id & CAN_RTR_FLAG)
    {
        buf[offset++] = 'R';
        /* print a given CAN 2.0B DLC if it's not zero */
        if (cf->len && cf->len <= CAN_MAX_DLC)
            buf[offset++] = hex_asc_upper_lo(cf->len);

        buf[offset] = 0;
        return;
    }

    if (maxdlen == CANFD_MAX_DLEN)
    {
        /* add CAN FD specific escape char and flags */
        buf[offset++] = '#';
        buf[offset++] = hex_asc_upper_lo(cf->flags);
        if (sep && len)
            buf[offset++] = '.';
    }

    for (i = 0; i < len; i++)
    {
        put_hex_byte(buf + offset, cf->data[i]);
        offset += 2;
        if (sep && (i + 1 < len))
            buf[offset++] = '.';
    }

    buf[offset] = 0;
}

void CE367ECUCan::fprint_long_canframe(FILE *stream, struct canfd_frame *cf, char *eol, int view, int maxdlen)
{
    /* documentation see lib.h */

    char buf[CL_LONGCFSZ];

    sprint_long_canframe(buf, cf, view, maxdlen);
    fprintf(stream, "%s", buf);
    if ((view & CANLIB_VIEW_ERROR) && (cf->can_id & CAN_ERR_FLAG))
    {
        fprintf(stream, "\n\t%s", buf);
    }
    if (eol)
        fprintf(stream, "%s", eol);
}

void CE367ECUCan::sprint_long_canframe(char *buf, struct canfd_frame *cf, int view, int maxdlen)
{
    /* documentation see lib.h */

    int i, j, dlen, offset;
    int len = (cf->len > maxdlen) ? maxdlen : cf->len;

    /* initialize space for CAN-ID and length information */
    memset(buf, ' ', 15);

    if (cf->can_id & CAN_ERR_FLAG)
    {
        put_eff_id(buf, cf->can_id & (CAN_ERR_MASK | CAN_ERR_FLAG));
        offset = 10;
    }
    else if (cf->can_id & CAN_EFF_FLAG)
    {
        put_eff_id(buf, cf->can_id & CAN_EFF_MASK);
        offset = 10;
    }
    else
    {
        if (view & CANLIB_VIEW_INDENT_SFF)
        {
            put_sff_id(buf + 5, cf->can_id & CAN_SFF_MASK);
            offset = 10;
        }
        else
        {
            put_sff_id(buf, cf->can_id & CAN_SFF_MASK);
            offset = 5;
        }
    }

    /* The len value is sanitized by maxdlen (see above) */
    if (maxdlen == CAN_MAX_DLEN)
    {
        buf[offset + 1] = '[';
        buf[offset + 2] = len + '0';
        buf[offset + 3] = ']';

        /* standard CAN frames may have RTR enabled */
        if (cf->can_id & CAN_RTR_FLAG)
        {
            sprintf(buf + offset + 5, " remote request");
            return;
        }
    }
    else
    {
        buf[offset] = '[';
        buf[offset + 1] = (len / 10) + '0';
        buf[offset + 2] = (len % 10) + '0';
        buf[offset + 3] = ']';
    }
    offset += 5;

    if (view & CANLIB_VIEW_BINARY)
    {
        dlen = 9; /* _10101010 */
        if (view & CANLIB_VIEW_SWAP)
        {
            for (i = len - 1; i >= 0; i--)
            {
                buf[offset++] = (i == len - 1) ? ' ' : SWAP_DELIMITER;
                for (j = 7; j >= 0; j--)
                    buf[offset++] = (1 << j & cf->data[i]) ? '1' : '0';
            }
        }
        else
        {
            for (i = 0; i < len; i++)
            {
                buf[offset++] = ' ';
                for (j = 7; j >= 0; j--)
                    buf[offset++] = (1 << j & cf->data[i]) ? '1' : '0';
            }
        }
    }
    else
    {
        dlen = 3; /* _AA */
        if (view & CANLIB_VIEW_SWAP)
        {
            for (i = len - 1; i >= 0; i--)
            {
                if (i == len - 1)
                    buf[offset++] = ' ';
                else
                    buf[offset++] = SWAP_DELIMITER;

                put_hex_byte(buf + offset, cf->data[i]);
                offset += 2;
            }
        }
        else
        {
            for (i = 0; i < len; i++)
            {
                buf[offset++] = ' ';
                put_hex_byte(buf + offset, cf->data[i]);
                offset += 2;
            }
        }
    }

    buf[offset] = 0; /* terminate string */

    /*
     * The ASCII & ERRORFRAME output is put at a fixed len behind the data.
     * For now we support ASCII output only for payload length up to 8 bytes.
     * Does it make sense to write 64 ASCII byte behind 64 ASCII HEX data on the console?
     */
    if (len > CAN_MAX_DLEN)
        return;

    if (cf->can_id & CAN_ERR_FLAG)
        sprintf(buf + offset, "%*s", dlen * (8 - len) + 13, "ERRORFRAME");
    else if (view & CANLIB_VIEW_ASCII)
    {
        j = dlen * (8 - len) + 4;
        if (view & CANLIB_VIEW_SWAP)
        {
            sprintf(buf + offset, "%*s", j, "`");
            offset += j;
            for (i = len - 1; i >= 0; i--)
                if ((cf->data[i] > 0x1F) && (cf->data[i] < 0x7F))
                    buf[offset++] = cf->data[i];
                else
                    buf[offset++] = '.';

            sprintf(buf + offset, "`");
        }
        else
        {
            sprintf(buf + offset, "%*s", j, "'");
            offset += j;
            for (i = 0; i < len; i++)
                if ((cf->data[i] > 0x1F) && (cf->data[i] < 0x7F))
                    buf[offset++] = cf->data[i];
                else
                    buf[offset++] = '.';

            sprintf(buf + offset, "'");
        }
    }
}

#pragma endregion

#pragma region Can dump functions

int CE367ECUCan::idx2dindex(int ifidx, int socket)
{

    int i;
    struct ifreq ifr;

    for (i = 0; i < MAXIFNAMES; i++)
    {
        if (dindex[i] == ifidx)
            return i;
    }

    /* create new interface index cache entry */

    /* remove index cache zombies first */
    for (i = 0; i < MAXIFNAMES; i++)
    {
        if (dindex[i])
        {
            ifr.ifr_ifindex = dindex[i];
            if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
                dindex[i] = 0;
        }
    }

    for (i = 0; i < MAXIFNAMES; i++)
        if (!dindex[i]) /* free entry */
            break;

    if (i == MAXIFNAMES)
    {
        fprintf(stderr, "Interface index cache only supports %d interfaces.\n", MAXIFNAMES);
        exit(1);
    }

    dindex[i] = ifidx;

    ifr.ifr_ifindex = ifidx;
    if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
        perror("SIOCGIFNAME");

    if (max_devname_len < (int) strlen(ifr.ifr_name))
        max_devname_len = strlen(ifr.ifr_name);

    strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
    printf("new index %d (%s)\n", i, devname[i]);
#endif

    return i;
}

#pragma endregion Can dump functions

#pragma region Currawong functions

void CE367ECUCan::send_ecu_messages(float throttle_percent)
{
    canfd_frame txFrame{};

    // No ECU node id set, don't send anything
    if (_ecu_id == 0)
    {
        return;
    }

    PiccoloFrameID piccolo_frame_id = {};
    memset(&piccolo_frame_id, 0, sizeof(PiccoloFrameID));

    piccolo_frame_id.word.frame_format_flag = 1;
    // piccolo_frame_id.word.group_id = 0x08;
    // piccolo_frame_id.word.message_type = 0x03;
    piccolo_frame_id.word.device_address = 0xFFFF;

    encodeECU_ThrottleCommandPacket(&txFrame, throttle_percent);
    txFrame.can_id |= (uint8_t) _ecu_id;
    txFrame.can_id |= piccolo_frame_id.frame_id;

    write_frame(_can_port, _real_number_devices, &txFrame, 0);
}

bool CE367ECUCan::handle_ecu_message(canfd_frame *frame)
{
    bool valid = true;

    // There are differences between Ardupilot EFI_State and types/scaling of Piccolo packets.
    // First decode to Piccolo structs, and then store the data we need in internal_state with any scaling required.

    // Structs to decode Piccolo messages into
    ECU_TelemetryFast_t telemetry_fast = {};
    ECU_TelemetrySlow0_t telemetry_slow0 = {};
    ECU_TelemetrySlow1_t telemetry_slow1 = {};
    ECU_TelemetrySlow2_t telemetry_slow2 = {};

    // Throw the message at the decoding functions
    if (decodeECU_TelemetryFastPacketStructure(frame, &telemetry_fast))
    {
        _currawong_ce367ecu_status.ecu_throttle = telemetry_fast.throttle;
        _currawong_ce367ecu_status.ecu_rpm = telemetry_fast.rpm;
        _currawong_ce367ecu_status.ecu_fuel_used = telemetry_fast.fuelUsed;
    }
    else if (decodeECU_TelemetrySlow0PacketStructure(frame, &telemetry_slow0))
    {
    }
    else if (decodeECU_TelemetrySlow1PacketStructure(frame, &telemetry_slow1))
    {
    }
    else if (decodeECU_TelemetrySlow2PacketStructure(frame, &telemetry_slow2))
    {
    }
    else
    {
        valid = false;
    }

    if (valid)
    {
        // internal_state.last_updated_ms = AP_HAL::millis();
    }

    return valid;
}

void CE367ECUCan::send_pmu_messages(float throttle_percent)
{
    canfd_frame txFrame{};

    // No ECU node id set, don't send anything
    if (_pmu_id == 0)
    {
        return;
    }

    PiccoloFrameID piccolo_frame_id = {};
    memset(&piccolo_frame_id, 0, sizeof(PiccoloFrameID));

    piccolo_frame_id.word.frame_format_flag = 1;
    // piccolo_frame_id.word.group_id = 0x08;
    // piccolo_frame_id.word.message_type = 0x03;
    piccolo_frame_id.word.device_address = 0xFFFF;

    encodeECU_ThrottleCommandPacket(&txFrame, throttle_percent); //_ecu_info.command);
    txFrame.can_id |= (uint8_t) _pmu_id;
    txFrame.can_id |= piccolo_frame_id.frame_id;

    write_frame(_can_port, _real_number_devices, &txFrame, 0);
}

void CE367ECUCan::start_cranking_engine()
{
    canfd_frame txFrame{};

    // No ECU node id set, don't send anything
    if (_pmu_id == 0)
    {
        return;
    }

    PiccoloFrameID piccolo_frame_id = {};
    memset(&piccolo_frame_id, 0, sizeof(PiccoloFrameID));

    piccolo_frame_id.word.frame_format_flag = 1;
    // piccolo_frame_id.word.group_id = 0x08;
    // piccolo_frame_id.word.message_type = 0x03;
    piccolo_frame_id.word.device_address = 0xFFFF;

    encodePMU_StartPacketStructure(&txFrame, nullptr);
    txFrame.can_id |= (uint8_t) _pmu_id;
    txFrame.can_id |= piccolo_frame_id.frame_id;

    write_frame(_can_port, _real_number_devices, &txFrame, 0);
}

void CE367ECUCan::stop_cranking_engine()
{
    canfd_frame txFrame{};

    // No ECU node id set, don't send anything
    if (_pmu_id == 0)
    {
        return;
    }

    PiccoloFrameID piccolo_frame_id = {};
    memset(&piccolo_frame_id, 0, sizeof(PiccoloFrameID));

    piccolo_frame_id.word.frame_format_flag = 1;
    // piccolo_frame_id.word.group_id = 0x08;
    // piccolo_frame_id.word.message_type = 0x03;
    piccolo_frame_id.word.device_address = 0xFFFF;

    encodePMU_StopPacketStructure(&txFrame, nullptr);
    txFrame.can_id |= (uint8_t) _pmu_id;
    txFrame.can_id |= piccolo_frame_id.frame_id;

    write_frame(_can_port, _real_number_devices, &txFrame, 0);
}

void CE367ECUCan::reset_cranking_engine()
{
    canfd_frame txFrame{};

    // No ECU node id set, don't send anything
    if (_pmu_id == 0)
    {
        return;
    }

    PiccoloFrameID piccolo_frame_id = {};
    memset(&piccolo_frame_id, 0, sizeof(PiccoloFrameID));

    piccolo_frame_id.word.frame_format_flag = 1;
    // piccolo_frame_id.word.group_id = 0x08;
    // piccolo_frame_id.word.message_type = 0x03;
    piccolo_frame_id.word.device_address = 0xFFFF;

    encodePMU_ResetPacketStructure(&txFrame, nullptr);
    txFrame.can_id |= (uint8_t) _pmu_id;
    txFrame.can_id |= piccolo_frame_id.frame_id;

    write_frame(_can_port, _real_number_devices, &txFrame, 0);
}

bool CE367ECUCan::handle_pmu_message(canfd_frame *frame)
{
    bool valid = true;

    // There are differences between Ardupilot EFI_State and types/scaling of Piccolo packets.
    // First decode to Piccolo structs, and then store the data we need in internal_state with any scaling required.

    // Structs to decode Piccolo messages into
    PMU_Voltages_t voltages = {};
    PMU_Currents_t currents = {};
    PMU_BatteryStatuses_t battery_statuses = {};
    PMU_Temperatures_t temperatures = {};
    PMU_Miscellaneous_t miscellaneous = {};

    // Throw the message at the decoding functions
    if (decodePMU_VoltagesPacketStructure(frame, &voltages))
    {
    }
    else if (decodePMU_CurrentsPacketStructure(frame, &currents))
    {
    }
    else if (decodePMU_BatteryStatusesPacketStructure(frame, &battery_statuses))
    {
    }
    else if (decodePMU_TemperaturesPacketStructure(frame, &temperatures))
    {
    }
    else if (decodePMU_MiscellaneousPacketStructure(frame, &miscellaneous))
    {
    }
    else
    {
        valid = false;
    }

    if (valid)
    {
        // internal_state.last_updated_ms = AP_HAL::millis();
    }

    return valid;
}

#pragma endregion Currawong functions
