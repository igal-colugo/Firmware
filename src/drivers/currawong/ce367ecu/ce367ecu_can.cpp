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

    fd_set rdfs;
    int s[MAXSOCK];
    unsigned char timestamp = 0;
    unsigned char hwtimestamp = 0;
    unsigned char down_causes_exit = 1;
    unsigned char dropmonitor = 0;
    unsigned char extra_msg_info = 0;
    unsigned char silent = SILENT_INI;
    unsigned char silentani = 0;
    unsigned char color = 0;
    unsigned char view = 0;
    unsigned char logfrmt = 0;
    int count = 0;
    int rcvbuf_size = 0;
    int opt, ret;
    int currmax, numfilter;
    int join_filter;
    char *ptr, *nptr;
    struct sockaddr_can addr;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3 * sizeof(struct timespec) + sizeof(__u32))];
    struct iovec iov;
    struct msghdr msg;
    struct cmsghdr *cmsg;
    struct can_filter *rfilter;
    can_err_mask_t err_mask;
    struct canfd_frame frame;
    int nbytes, i, maxdlen;
    struct ifreq ifr;
    struct timeval tv, last_tv;
    struct timeval timeout, timeout_config = {0, 0}, *timeout_current = NULL;
    FILE *logfile = NULL;

    last_tv.tv_sec = 0;
    last_tv.tv_usec = 0;

    // while ((opt = getopt(argc, argv, "t:HciaSs:lDdxLn:r:heT:?")) != -1)
    // {
    //     switch (opt)
    //     {
    //     case 't':
    //         timestamp = optarg[0];
    //         if ((timestamp != 'a') && (timestamp != 'A') && (timestamp != 'd') && (timestamp != 'z'))
    //         {
    //             fprintf(stderr, "%s: unknown timestamp mode '%c' - ignored\n", basename(argv[0]), optarg[0]);
    //             timestamp = 0;
    //         }
    //         break;

    //     case 'H':
    //         hwtimestamp = 1;
    //         break;

    //     case 'c':
    //         color++;
    //         break;

    //     case 'i':
    //         view |= CANLIB_VIEW_BINARY;
    //         break;

    //     case 'a':
    //         view |= CANLIB_VIEW_ASCII;
    //         break;

    //     case 'S':
    //         view |= CANLIB_VIEW_SWAP;
    //         break;

    //     case 'e':
    //         view |= CANLIB_VIEW_ERROR;
    //         break;

    //     case 's':
    //         silent = atoi(optarg);
    //         if (silent > SILENT_ON)
    //         {
    //             print_usage(basename(argv[0]));
    //             exit(1);
    //         }
    //         break;

    //     case 'l':
    //         log = 1;
    //         break;

    //     case 'D':
    //         down_causes_exit = 0;
    //         break;

    //     case 'd':
    //         dropmonitor = 1;
    //         break;

    //     case 'x':
    //         extra_msg_info = 1;
    //         break;

    //     case 'L':
    //         logfrmt = 1;
    //         break;

    //     case 'n':
    //         count = atoi(optarg);
    //         if (count < 1)
    //         {
    //             print_usage(basename(argv[0]));
    //             exit(1);
    //         }
    //         break;

    //     case 'r':
    //         rcvbuf_size = atoi(optarg);
    //         if (rcvbuf_size < 1)
    //         {
    //             print_usage(basename(argv[0]));
    //             exit(1);
    //         }
    //         break;

    //     case 'T':
    //         errno = 0;
    //         timeout_config.tv_usec = strtol(optarg, NULL, 0);
    //         if (errno != 0)
    //         {
    //             print_usage(basename(argv[0]));
    //             exit(1);
    //         }
    //         timeout_config.tv_sec = timeout_config.tv_usec / 1000;
    //         timeout_config.tv_usec = (timeout_config.tv_usec % 1000) * 1000;
    //         timeout_current = &timeout;
    //         break;
    //     default:
    //         print_usage(basename(argv[0]));
    //         exit(1);
    //         break;
    //     }
    // }

    // if (optind == argc)
    // {
    //     print_usage(basename(argv[0]));
    //     exit(0);
    // }

    if (silent == SILENT_INI)
    {
        silent = SILENT_OFF; /* default output */
    }

    currmax = 1; // argc - optind; /* find real number of CAN devices */

    if (currmax > MAXSOCK)
    {
        fprintf(stderr, "More than %d CAN devices given on commandline!\n", MAXSOCK);
        return 1;
    }

    for (i = 0; i < currmax; i++)
    {

        ptr = "can1"; // argv[optind + i];
        nptr = strchr(ptr, ',');

#ifdef DEBUG
        printf("open %d '%s'.\n", i, ptr);
#endif

        s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s[i] < 0)
        {
            perror("socket");
            return 1;
        }

        cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

        if (nptr)
            nbytes = nptr - ptr; /* interface name is up the first ',' */
        else
            nbytes = strlen(ptr); /* no ',' found => no filter definitions */

        if (nbytes >= IFNAMSIZ)
        {
            fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
            return 1;
        }

        if (nbytes > max_devname_len)
            max_devname_len = nbytes; /* for nice printing */

        addr.can_family = AF_CAN;

        memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
        strncpy(ifr.ifr_name, ptr, nbytes);

#ifdef DEBUG
        printf("using interface name '%s'.\n", ifr.ifr_name);
#endif

        if (strcmp(ANYDEV, ifr.ifr_name))
        {
            if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0)
            {
                perror("SIOCGIFINDEX");
                exit(1);
            }
            addr.can_ifindex = ifr.ifr_ifindex;
        }
        else
            addr.can_ifindex = 0; /* any can interface */

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
                fprintf(stderr, "Failed to create filter space!\n");
                return 1;
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
                        rfilter[numfilter].can_id |= CAN_EFF_FLAG;
                    numfilter++;
                }
                else if (sscanf(ptr, "%" SCNx32 "~%" SCNx32, &rfilter[numfilter].can_id, &rfilter[numfilter].can_mask) == 2)
                {
                    rfilter[numfilter].can_id |= CAN_INV_FILTER;
                    rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
                    if (*(ptr + 8) == '~')
                        rfilter[numfilter].can_id |= CAN_EFF_FLAG;
                    numfilter++;
                }
                else if (*ptr == 'j' || *ptr == 'J')
                {
                    join_filter = 1;
                }
                else if (sscanf(ptr, "#%" SCNx32, &err_mask) != 1)
                {
                    fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
                    return 1;
                }
            }

            if (err_mask)
                setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

            if (join_filter && setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join_filter, sizeof(join_filter)) < 0)
            {
                perror("setsockopt CAN_RAW_JOIN_FILTERS not supported by your Linux Kernel");
                return 1;
            }

            if (numfilter)
                setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, numfilter * sizeof(struct can_filter));

            free(rfilter);

        } /* if (nptr) */

        /* try to switch the socket into CAN FD mode */
        setsockopt(s[i], SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

        if (rcvbuf_size)
        {

            int curr_rcvbuf_size;
            socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

            /* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
            if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUFFORCE, &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
            {
#ifdef DEBUG
                printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
                if (setsockopt(s[i], SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
                {
                    perror("setsockopt SO_RCVBUF");
                    return 1;
                }

                if (getsockopt(s[i], SOL_SOCKET, SO_RCVBUF, &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0)
                {
                    perror("getsockopt SO_RCVBUF");
                    return 1;
                }

                /* Only print a warning the first time we detect the adjustment */
                /* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
                if (!i && curr_rcvbuf_size < rcvbuf_size * 2)
                    fprintf(stderr, "The socket receive buffer size was "
                                    "adjusted due to /proc/sys/net/core/rmem_max.\n");
            }
        }

        if (timestamp || logfrmt)
        {

            if (hwtimestamp)
            {
                const int timestamping_flags = (SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_RAW_HARDWARE);

                if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMPING, &timestamping_flags, sizeof(timestamping_flags)) < 0)
                {
                    perror("setsockopt SO_TIMESTAMPING is not supported by your Linux kernel");
                    return 1;
                }
            }
            else
            {
                const int timestamp_on = 1;

                if (setsockopt(s[i], SOL_SOCKET, SO_TIMESTAMP, &timestamp_on, sizeof(timestamp_on)) < 0)
                {
                    perror("setsockopt SO_TIMESTAMP");
                    return 1;
                }
            }
        }

        if (dropmonitor)
        {

            const int dropmonitor_on = 1;

            if (setsockopt(s[i], SOL_SOCKET, SO_RXQ_OVFL, &dropmonitor_on, sizeof(dropmonitor_on)) < 0)
            {
                perror("setsockopt SO_RXQ_OVFL not supported by your Linux Kernel");
                return 1;
            }
        }

        if (bind(s[i], (struct sockaddr *) &addr, sizeof(addr)) < 0)
        {
            perror("bind");
            return 1;
        }
    }

    /* these settings are static and can be held out of the hot path */
    iov.iov_base = &frame;
    msg.msg_name = &addr;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = &ctrlmsg;

    // while (running)
    //{

    FD_ZERO(&rdfs);
    for (i = 0; i < currmax; i++)
        FD_SET(s[i], &rdfs);

    if (timeout_current)
        *timeout_current = timeout_config;

    if ((ret = select(s[currmax - 1] + 1, &rdfs, NULL, NULL, timeout_current)) <= 0)
    {
        // perror("select");
        running = 0;
        return 0; // continue;
    }

    for (i = 0; i < currmax; i++)
    { /* check all CAN RAW sockets */

        if (FD_ISSET(s[i], &rdfs))
        {

            int idx;

            /* these settings may be modified by recvmsg() */
            iov.iov_len = sizeof(frame);
            msg.msg_namelen = sizeof(addr);
            msg.msg_controllen = sizeof(ctrlmsg);
            msg.msg_flags = 0;

            nbytes = recvmsg(s[i], &msg, 0);
            idx = idx2dindex(addr.can_ifindex, s[i]);

            if (nbytes < 0)
            {
                if ((errno == ENETDOWN) && !down_causes_exit)
                {
                    fprintf(stderr, "%s: interface down\n", devname[idx]);
                    continue;
                }
                perror("read");
                return 1;
            }

            if ((size_t) nbytes == CAN_MTU)
                maxdlen = CAN_MAX_DLEN;
            else if ((size_t) nbytes == CANFD_MTU)
                maxdlen = CANFD_MAX_DLEN;
            else
            {
                fprintf(stderr, "read: incomplete CAN frame\n");
                return 1;
            }

            if (count && (--count == 0))
                running = 0;

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
                    memcpy(&dropcnt[i], CMSG_DATA(cmsg), sizeof(__u32));
            }

            /* check for (unlikely) dropped frames on this specific socket */
            if (dropcnt[i] != last_dropcnt[i])
            {

                __u32 frames = dropcnt[i] - last_dropcnt[i];

                if (silent != SILENT_ON)
                    printf("DROPCOUNT: dropped %" PRId32 " CAN frame%s on '%s' socket (total drops %" PRId32 ")\n", (uint32_t) frames, (frames > 1) ? "s" : "", devname[idx],
                           (uint32_t) dropcnt[i]);

                last_dropcnt[i] = dropcnt[i];
            }

            /* once we detected a EFF frame indent SFF frames accordingly */
            if (frame.can_id & CAN_EFF_FLAG)
                view |= CANLIB_VIEW_INDENT_SFF;

            if ((logfrmt) && (silent == SILENT_OFF))
            {
                char buf[CL_CFSZ]; /* max length */

                /* print CAN frame in log file style to stdout */
                sprint_canframe(buf, &frame, 0, maxdlen);
                printf("(%010ju.%06ld) %*s %s\n", (uintmax_t) tv.tv_sec, tv.tv_usec, max_devname_len, devname[idx], buf);
                goto out_fflush; /* no other output to stdout */
            }

            if (silent != SILENT_OFF)
            {
                if (silent == SILENT_ANI)
                {
                    printf("%c\b", anichar[silentani %= MAXANI]);
                    silentani++;
                }
                goto out_fflush; /* no other output to stdout */
            }

            printf(" %s", (color > 2) ? col_on[idx % MAXCOL] : "");

            switch (timestamp)
            {

            case 'a': /* absolute with timestamp */
                printf("(%010ju.%06ld) ", (uintmax_t) tv.tv_sec, tv.tv_usec);
                break;

            case 'A': /* absolute with date */
            {
                struct tm tm;
                char timestring[25];

                tm = *localtime(&tv.tv_sec);
                strftime(timestring, 24, "%Y-%m-%d %H:%M:%S", &tm);
                printf("(%s.%06ld) ", timestring, tv.tv_usec);
            }
            break;

            case 'd': /* delta */
            case 'z': /* starting with zero */
            {
                struct timeval diff;

                if (last_tv.tv_sec == 0) /* first init */
                    last_tv = tv;
                diff.tv_sec = tv.tv_sec - last_tv.tv_sec;
                diff.tv_usec = tv.tv_usec - last_tv.tv_usec;
                if (diff.tv_usec < 0)
                    diff.tv_sec--, diff.tv_usec += 1000000;
                if (diff.tv_sec < 0)
                    diff.tv_sec = diff.tv_usec = 0;
                printf("(%03ju.%06ld) ", (uintmax_t) diff.tv_sec, diff.tv_usec);

                if (timestamp == 'd')
                    last_tv = tv; /* update for delta calculation */
            }
            break;

            default: /* no timestamp output */
                break;
            }

            printf(" %s", (color && (color < 3)) ? col_on[idx % MAXCOL] : "");
            printf("%*s", max_devname_len, devname[idx]);

            if (extra_msg_info)
            {

                if (msg.msg_flags & MSG_DONTROUTE)
                    printf("  TX %s", extra_m_info[frame.flags & 3]);
                else
                    printf("  RX %s", extra_m_info[frame.flags & 3]);
            }

            printf("%s  ", (color == 1) ? col_off : "");

            fprint_long_canframe(stdout, &frame, NULL, view, maxdlen);

            printf("%s", (color > 1) ? col_off : "");
            printf("\n");
        }

    out_fflush:
        fflush(stdout);
    }
    //}

    for (i = 0; i < currmax; i++)
        close(s[i]);

    return 0;

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
