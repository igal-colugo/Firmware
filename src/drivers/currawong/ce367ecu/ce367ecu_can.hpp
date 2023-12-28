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

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <netpacket/can.h>
#include <netutils/netlib.h>
#include <nuttx/can.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/battery_status.h>

#include <fcntl.h>
#include <poll.h>

#include <arch/board/board.h>
#include <px4_platform_common/log.h>

#include "stm32.h"

#include "ECUDefines.hpp"
#include "ECUPackets.hpp"
#include "ECUProtocol.hpp"
#include "ECUSettings.hpp"

using namespace time_literals;

#pragma region Can send definitions

/* Compatibility for NuttX */
typedef uint8_t __u8;
typedef uint32_t __u32;

#define CE367ECU_CAN_MEASURE_INTERVAL 50_ms

/* buffer sizes for CAN frame string representations */

#define CL_ID (sizeof("12345678##1"))
#define CL_DATA sizeof(".AA")
#define CL_BINDATA sizeof(".10101010")

/* CAN FD ASCII hex short representation with DATA_SEPERATORs */
#define CL_CFSZ (2 * CL_ID + 64 * CL_DATA)

/* CAN FD ASCII hex long representation with binary output */
#define CL_LONGCFSZ (2 * CL_ID + sizeof("   [255]  ") + (64 * CL_BINDATA))

#define CANLIB_VIEW_ASCII 0x1
#define CANLIB_VIEW_BINARY 0x2
#define CANLIB_VIEW_SWAP 0x4
#define CANLIB_VIEW_ERROR 0x8
#define CANLIB_VIEW_INDENT_SFF 0x10

#define SWAP_DELIMITER '`'

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'

const char hex_asc_upper[] = "0123456789ABCDEF";

#define hex_asc_upper_lo(x) hex_asc_upper[((x) & 0x0F)]
#define hex_asc_upper_hi(x) hex_asc_upper[((x) & 0xF0) >> 4]

#define put_sff_id(buf, id) _put_id(buf, 2, id)
#define put_eff_id(buf, id) _put_id(buf, 7, id)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/* CAN DLC to real data length conversion helpers */

static const unsigned char dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static const unsigned char len2dlc[] = {0,  1,  2,  3,  4,  5,  6,  7,  8, /* 0 - 8 */
                                        9,  9,  9,  9,                     /* 9 - 12 */
                                        10, 10, 10, 10,                    /* 13 - 16 */
                                        11, 11, 11, 11,                    /* 17 - 20 */
                                        12, 12, 12, 12,                    /* 21 - 24 */
                                        13, 13, 13, 13, 13, 13, 13, 13,    /* 25 - 32 */
                                        14, 14, 14, 14, 14, 14, 14, 14,    /* 33 - 40 */
                                        14, 14, 14, 14, 14, 14, 14, 14,    /* 41 - 48 */
                                        15, 15, 15, 15, 15, 15, 15, 15,    /* 49 - 56 */
                                        15, 15, 15, 15, 15, 15, 15, 15};   /* 57 - 64 */
static const char *error_classes[] = {
    "tx-timeout", "lost-arbitration", "controller-problem", "protocol-violation", "transceiver-status", "no-acknowledgement-on-tx", "bus-off", "bus-error", "restarted-after-bus-off",
};
static const char *controller_problems[] = {
    "rx-overflow", "tx-overflow", "rx-error-warning", "tx-error-warning", "rx-error-passive", "tx-error-passive", "back-to-error-active",
};
static const char *protocol_violation_types[] = {
    "single-bit-error", "frame-format-error", "bit-stuffing-error", "tx-dominant-bit-error", "tx-recessive-bit-error", "bus-overload", "active-error", "error-on-tx",
};
static const char *protocol_violation_locations[] = {
    "unspecified",   "unspecified",       "id.28-to-id.21", "start-of-frame",         "bit-srtr",      "bit-ide",        "id.20-to-id.18",     "id.17-to-id.13",
    "crc-sequence",  "reserved-bit-0",    "data-field",     "data-length-code",       "bit-rtr",       "reserved-bit-1", "id.4-to-id.0",       "id.12-to-id.5",
    "unspecified",   "active-error-flag", "intermission",   "tolerate-dominant-bits", "unspecified",   "unspecified",    "passive-error-flag", "error-delimiter",
    "crc-delimiter", "acknowledge-slot",  "end-of-frame",   "acknowledge-delimiter",  "overload-flag", "unspecified",    "unspecified",        "unspecified",
};

#pragma endregion Can send definitions

#pragma region Can dump definitions

/* for hardware timestamps - since Linux 2.6.30 */
#ifndef SO_TIMESTAMPING
#define SO_TIMESTAMPING 37
#endif

/* from #include <linux/net_tstamp.h> - since Linux 2.6.30 */
#define SOF_TIMESTAMPING_SOFTWARE (1 << 4)
#define SOF_TIMESTAMPING_RX_SOFTWARE (1 << 3)
#define SOF_TIMESTAMPING_RAW_HARDWARE (1 << 6)

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define SILENT_INI 42 /* detect user setting on commandline */
#define SILENT_OFF 0  /* no silent mode */
#define SILENT_ANI 1  /* silent mode with animation */
#define SILENT_ON 2   /* silent mode (completely silent) */

#define ATTBOLD "\33[1m"

#define FGBLACK "\33[30m"
#define FGRED "\33[31m"
#define FGGREEN "\33[32m"
#define FGYELLOW "\33[33m"
#define FGBLUE "\33[34m"
#define FGMAGENTA "\33[35m"
#define FGCYAN "\33[36m"
#define FGWHITE "\33[37m"

#define BOLD ATTBOLD
#define RED ATTBOLD FGRED
#define GREEN ATTBOLD FGGREEN
#define YELLOW ATTBOLD FGYELLOW
#define BLUE ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN ATTBOLD FGCYAN

#define MAXANI 4

/* reset to default */

#define ATTRESET "\33[0m"

const char col_on[MAXCOL][19] = {BLUE, RED, GREEN, BOLD, MAGENTA, CYAN};
const char col_off[] = ATTRESET;

static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ + 1];
static int dindex[MAXIFNAMES];
static int max_devname_len; /* to prevent frazzled device name output */
const int canfd_on = 1;

const char anichar[MAXANI] = {'|', '/', '-', '\\'};
const char extra_m_info[4][4] = {"- -", "B -", "- E", "B E"};

#pragma endregion Can dump definitions

#pragma region Currawong definitions

// maximum number of ESC allowed on CAN bus simultaneously
#define PICCOLO_CAN_MAX_NUM_ESC 16
#define PICCOLO_CAN_MAX_GROUP_ESC (PICCOLO_CAN_MAX_NUM_ESC / 4)

#define PICCOLO_CAN_MAX_NUM_SERVO 16
#define PICCOLO_CAN_MAX_GROUP_SERVO (PICCOLO_CAN_MAX_NUM_SERVO / 4)

#define PICCOLO_MSG_RATE_HZ_MIN 1
#define PICCOLO_MSG_RATE_HZ_MAX 500
#define PICCOLO_MSG_RATE_HZ_DEFAULT 50

#pragma endregion Currawong definitions

typedef struct __attribute__((packed))
{
    int16_t manufacturer;
    uint16_t sku;
    uint16_t voltage;
    int16_t current;
    int16_t temperature;
    uint16_t remaining_percent;
    uint16_t cycle_life;
    int16_t health_status;
    uint16_t cell_1_voltage;
    uint16_t cell_2_voltage;
    uint16_t cell_3_voltage;
    uint16_t cell_4_voltage;
    uint16_t cell_5_voltage;
    uint16_t cell_6_voltage;
    uint16_t cell_7_voltage;
    uint16_t cell_8_voltage;
    uint16_t cell_9_voltage;
    uint16_t cell_10_voltage;
    uint16_t cell_11_voltage;
    uint16_t cell_12_voltage;
    uint16_t standard_capacity;
    uint16_t remaining_capacity_mah;
    uint32_t error_info;
} CE367ECUCanMessage;
union PiccoloFrameID {
    uint32_t frame_id;
    struct __attribute__((packed))
    {
        uint16_t device_address;
        uint8_t message_type;
        uint8_t group_id : 5;
        uint8_t error_message_frame_flag : 1;
        uint8_t remote_transmission_request_flag : 1;
        uint8_t frame_format_flag : 1;
    } word;
};

class CE367ECUCan : public px4::ScheduledWorkItem
{
  public:
    // Piccolo actuator types differentiate between actuator frames
    enum class ActuatorType : uint8_t
    {
        SERVO = 0x00,
        ESC = 0x20,
    };

    struct CurrawongECU_Info_t
    {
        float command;
    } _ecu_info;

  public:
    CE367ECUCan();
    virtual ~CE367ECUCan();

    int can_port = 0;
    int real_number_devices = 0;

    void print_info();
    void start();
    void stop();

  private:
    uint16_t _esc_tx_counter = 0;
    uint16_t _servo_tx_counter = 0;
    uint16_t _ecu_tx_counter = 0;

    // Piccolo CAN parameters
    int32_t _esc_bm; //! ESC selection bitmask
    int32_t _srv_bm; //! Servo selection bitmask
    int16_t _ecu_id; //! ECU Node ID

    bool _initialized{false};

    void Run() override;

    int send_data(int can_port, char *data);
    int update();
    void collect();

#pragma region Can send functions

    static inline void put_hex_byte(char *buf, __u8 byte)
    {
        buf[0] = hex_asc_upper_hi(byte);
        buf[1] = hex_asc_upper_lo(byte);
    }
    static inline void _put_id(char *buf, int end_offset, canid_t id)
    {
        /* build 3 (SFF) or 8 (EFF) digit CAN identifier */
        while (end_offset >= 0)
        {
            buf[end_offset--] = hex_asc_upper_lo(id);
            id >>= 4;
        }
    }

    /* CAN DLC to real data length conversion helpers especially for CAN FD */
    /* get data length from can_dlc with sanitized can_dlc */
    unsigned char can_dlc2len(unsigned char can_dlc);
    /* map the sanitized data length to an appropriate data length code */
    unsigned char can_len2dlc(unsigned char len);
    unsigned char asc2nibble(char c);
    /*
     * Returns the decimal value of a given ASCII hex character.
     *
     * While 0..9, a..f, A..F are valid ASCII hex characters.
     * On invalid characters the value 16 is returned for error handling.
     */
    int hexstring2data(char *arg, unsigned char *data, int maxdlen);
    /*
     * Converts a given ASCII hex string to a (binary) byte string.
     *
     * A valid ASCII hex string consists of an even number of up to 16 chars.
     * Leading zeros '00' in the ASCII hex string are interpreted.
     *
     * Examples:
     *
     * "1234"   => data[0] = 0x12, data[1] = 0x34
     * "001234" => data[0] = 0x00, data[1] = 0x12, data[2] = 0x34
     *
     * Return values:
     * 0 = success
     * 1 = error (in length or the given characters are no ASCII hex characters)
     *
     * Remark: The not written data[] elements are initialized with zero.
     *
     */
    int parse_canframe(char *cs, struct canfd_frame *cf);
    /*
     * Transfers a valid ASCII string describing a CAN frame into struct canfd_frame.
     *
     * CAN 2.0 frames
     * - string layout <can_id>#{R{len}|data}
     * - {data} has 0 to 8 hex-values that can (optionally) be separated by '.'
     * - {len} can take values from 0 to 8 and can be omitted if zero
     * - return value on successful parsing: CAN_MTU
     *
     * CAN FD frames
     * - string layout <can_id>##<flags>{data}
     * - <flags> a single ASCII Hex value (0 .. F) which defines canfd_frame.flags
     * - {data} has 0 to 64 hex-values that can (optionally) be separated by '.'
     * - return value on successful parsing: CANFD_MTU
     *
     * Return value on detected problems: 0
     *
     * <can_id> can have 3 (standard frame format) or 8 (extended frame format)
     * hexadecimal chars
     *
     *
     * Examples:
     *
     * 123# -> standard CAN-Id = 0x123, len = 0
     * 12345678# -> extended CAN-Id = 0x12345678, len = 0
     * 123#R -> standard CAN-Id = 0x123, len = 0, RTR-frame
     * 123#R0 -> standard CAN-Id = 0x123, len = 0, RTR-frame
     * 123#R7 -> standard CAN-Id = 0x123, len = 7, RTR-frame
     * 7A1#r -> standard CAN-Id = 0x7A1, len = 0, RTR-frame
     *
     * 123#00 -> standard CAN-Id = 0x123, len = 1, data[0] = 0x00
     * 123#1122334455667788 -> standard CAN-Id = 0x123, len = 8
     * 123#11.22.33.44.55.66.77.88 -> standard CAN-Id = 0x123, len = 8
     * 123#11.2233.44556677.88 -> standard CAN-Id = 0x123, len = 8
     * 32345678#112233 -> error frame with CAN_ERR_FLAG (0x2000000) set
     *
     * 123##0112233 -> CAN FD frame standard CAN-Id = 0x123, flags = 0, len = 3
     * 123##1112233 -> CAN FD frame, flags = CANFD_BRS, len = 3
     * 123##2112233 -> CAN FD frame, flags = CANFD_ESI, len = 3
     * 123##3 -> CAN FD frame, flags = (CANFD_ESI | CANFD_BRS), len = 0
     *     ^^
     *     CAN FD extension to handle the canfd_frame.flags content
     *
     * Simple facts on this compact ASCII CAN frame representation:
     *
     * - 3 digits: standard frame format
     * - 8 digits: extendend frame format OR error frame
     * - 8 digits with CAN_ERR_FLAG (0x2000000) set: error frame
     * - an error frame is never a RTR frame
     * - CAN FD frames do not have a RTR bit
     */
    void fprint_canframe(FILE *stream, struct canfd_frame *cf, char *eol, int sep, int maxdlen);
    void sprint_canframe(char *buf, struct canfd_frame *cf, int sep, int maxdlen);
    /*
     * Creates a CAN frame hexadecimal output in compact format.
     * The CAN data[] is separated by '.' when sep != 0.
     *
     * The type of the CAN frame (CAN 2.0 / CAN FD) is specified by maxdlen:
     * maxdlen = 8 -> CAN2.0 frame
     * maxdlen = 64 -> CAN FD frame
     *
     * 12345678#112233 -> extended CAN-Id = 0x12345678, len = 3, data, sep = 0
     * 12345678#R -> extended CAN-Id = 0x12345678, RTR, len = 0
     * 12345678#R5 -> extended CAN-Id = 0x12345678, RTR, len = 5
     * 123#11.22.33.44.55.66.77.88 -> standard CAN-Id = 0x123, dlc = 8, sep = 1
     * 32345678#112233 -> error frame with CAN_ERR_FLAG (0x2000000) set
     * 123##0112233 -> CAN FD frame standard CAN-Id = 0x123, flags = 0, len = 3
     * 123##2112233 -> CAN FD frame, flags = CANFD_ESI, len = 3
     *
     * Examples:
     *
     * fprint_canframe(stdout, &frame, "\n", 0); // with eol to STDOUT
     * fprint_canframe(stderr, &frame, NULL, 0); // no eol to STDERR
     *
     */
    void fprint_long_canframe(FILE *stream, struct canfd_frame *cf, char *eol, int view, int maxdlen);
    void sprint_long_canframe(char *buf, struct canfd_frame *cf, int view, int maxdlen);
    /*
     * Creates a CAN frame hexadecimal output in user readable format.
     *
     * The type of the CAN frame (CAN 2.0 / CAN FD) is specified by maxdlen:
     * maxdlen = 8 -> CAN2.0 frame
     * maxdlen = 64 -> CAN FD frame
     *
     * 12345678   [3]  11 22 33 -> extended CAN-Id = 0x12345678, dlc = 3, data
     * 12345678   [0]  remote request -> extended CAN-Id = 0x12345678, RTR
     * 14B0DC51   [8]  4A 94 E8 2A EC 58 55 62   'J..*.XUb' -> (with ASCII output)
     * 20001111   [7]  C6 23 7B 32 69 98 3C      ERRORFRAME -> (CAN_ERR_FLAG set)
     * 12345678  [03]  11 22 33 -> CAN FD with extended CAN-Id = 0x12345678, dlc = 3
     *
     * 123   [3]  11 22 33         -> CANLIB_VIEW_INDENT_SFF == 0
     *      123   [3]  11 22 33    -> CANLIB_VIEW_INDENT_SFF == set
     *
     * Examples:
     *
     * // CAN FD frame with eol to STDOUT
     * fprint_long_canframe(stdout, &frame, "\n", 0, CANFD_MAX_DLEN);
     *
     * // CAN 2.0 frame without eol to STDERR
     * fprint_long_canframe(stderr, &frame, NULL, 0, CAN_MAX_DLEN);
     *
     */
    static constexpr uint32_t SAMPLE_RATE{100}; // samples per second (10ms)
    static constexpr size_t TAIL_BYTE_START_OF_TRANSFER{128};

    perf_counter_t _comms_error{perf_alloc(PC_COUNT, MODULE_NAME ": comms_error")};
    perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": sample")};

#pragma endregion Can send functions

#pragma region Can dump functions

    int idx2dindex(int ifidx, int socket);

#pragma endregion Can dump functions

#pragma region Currawong functions

    // write frame on CAN bus, returns true on success
    int write_frame(int can_port, int real_number_devices, canfd_frame *out_frame, uint64_t timeout);
    // read frame on CAN bus, returns true on succses
    int read_frame(int can_port, int real_number_devices, canfd_frame *recv_frame, uint64_t timeout);
    // send ECU commands over CAN
    void send_ecu_messages(float throttle_percent);
    // interpret an ECU message received over CAN
    bool handle_ecu_message(canfd_frame *frame);
    // send ESC commands over CAN
    void send_esc_messages(void);
    // interpret an ESC message received over CAN
    bool handle_esc_message(canfd_frame *frame);
    // send servo commands over CAN
    void send_servo_messages(void);
    // interpret a servo message received over CAN
    bool handle_servo_message(canfd_frame *frame);

#pragma endregion Currawong functions
};
