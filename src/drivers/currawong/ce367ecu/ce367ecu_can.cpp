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
 * @file TattuCan.cpp
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 *
 * Driver for the Tattu 12S 1600mAh Smart Battery connected over CAN.
 *
 * This driver simply decodes the CAN frames based on the specification
 * as provided in the Tattu datasheet DOC 001 REV D, which is highly
 * specific to the 12S 1600mAh battery. Other models of Tattu batteries
 * will NOT work with this driver in its current form.
 *
 */

#include "ce367ecu_can.hpp"

extern orb_advert_t mavlink_log_pub;

CE367ECUCan::CE367ECUCan() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::can)
{
    int res = stm32_fdcansockinitialize(0);
    res = stm32_fdcansockinitialize(1);
    netlib_ifup("can0");
    netlib_ifup("can1");
}

CE367ECUCan::~CE367ECUCan()
{
    netlib_ifdown("can0");
    netlib_ifdown("can1");
}

void CE367ECUCan::Run()
{
    collect();
    update();
    // px4_sleep(50);
}

void CE367ECUCan::start()
{
    // Schedule the driver at regular intervals.
    ScheduleOnInterval(CE367ECU_MEASURE_INTERVAL, CE367ECU_MEASURE_INTERVAL);
}

void CE367ECUCan::stop()
{
    // Clear the work queue schedule.
    ScheduleClear();
}

int CE367ECUCan::update()
{
    int return_value = 0;

    int s; /* can raw socket */
    int required_mtu;
    int mtu;
    int enable_canfd = 1;
    struct sockaddr_can addr;
    struct canfd_frame frame;
    struct ifreq ifr;

    /* parse CAN frame */
    required_mtu = this->parse_canframe("00000005#AAAABBBBCCCCDDDD", &frame);
    if (!required_mtu)
    {
        fprintf(stderr, "\nWrong CAN-frame format!\n\n");
        return -1;
    }

    /* open socket */
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        return -1;
    }

    strncpy(ifr.ifr_name, "can1", IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (!ifr.ifr_ifindex)
    {
        perror("if_nametoindex");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (required_mtu > (int) CAN_MTU)
    {
        /* check if the frame fits into the CAN netdevice */
        if (ioctl(s, SIOCGIFMTU, &ifr) < 0)
        {
            perror("SIOCGIFMTU");
            return -1;
        }
        mtu = ifr.ifr_mtu;

        if (mtu != CANFD_MTU)
        {
            printf("CAN interface is not CAN FD capable - sorry.\n");
            return -1;
        }

        /* interface is ok - try to switch the socket into CAN FD mode */
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)))
        {
            printf("error when enabling CAN FD support\n");
            return -1;
        }

        /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
        frame.len = can_dlc2len(can_len2dlc(frame.len));
    }

    /* disable default receive filter on this RAW socket */
    /* This is obsolete as we do not read from the socket at all, but for */
    /* this reason we can remove the receive list in the Kernel to save a */
    /* little (really a very little!) CPU usage.                          */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return -1;
    }

    /* send frame */
    if (write(s, &frame, required_mtu) != required_mtu)
    {
        perror("write");
        return -1;
    }

    close(s);

    return return_value;
}

int CE367ECUCan::collect()
{
    int return_value = 0;

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

static int snprintf_error_data(char *buf, size_t len, uint8_t err, const char **arr, int arr_len)
{
    int i, n = 0, count = 0;

    if (!err || len <= 0)
        return 0;

    for (i = 0; i < arr_len; i++)
    {
        if (err & (1 << i))
        {
            if (count)
                n += snprintf(buf + n, len - n, ",");
            n += snprintf(buf + n, len - n, "%s", arr[i]);
            count++;
        }
    }

    return n;
}

static int snprintf_error_lostarb(char *buf, size_t len, const struct canfd_frame *cf)
{
    if (len <= 0)
        return 0;
    return snprintf(buf, len, "{at bit %d}", cf->data[0]);
}

static int snprintf_error_ctrl(char *buf, size_t len, const struct canfd_frame *cf)
{
    int n = 0;

    if (len <= 0)
        return 0;

    n += snprintf(buf + n, len - n, "{");
    n += snprintf_error_data(buf + n, len - n, cf->data[1], controller_problems, ARRAY_SIZE(controller_problems));
    n += snprintf(buf + n, len - n, "}");

    return n;
}

static int snprintf_error_prot(char *buf, size_t len, const struct canfd_frame *cf)
{
    int n = 0;

    if (len <= 0)
        return 0;

    n += snprintf(buf + n, len - n, "{{");
    n += snprintf_error_data(buf + n, len - n, cf->data[2], protocol_violation_types, ARRAY_SIZE(protocol_violation_types));
    n += snprintf(buf + n, len - n, "}{");
    if (cf->data[3] > 0 && cf->data[3] < ARRAY_SIZE(protocol_violation_locations))
        n += snprintf(buf + n, len - n, "%s", protocol_violation_locations[cf->data[3]]);
    n += snprintf(buf + n, len - n, "}}");

    return n;
}

#pragma endregion
