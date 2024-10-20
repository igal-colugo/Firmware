// ECUSettings.h was generated by ProtoGen version 3.2.a

/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */


#ifndef _ECUSETTINGS_H
#define _ECUSETTINGS_H

// Language target is C, C++ compilers: don't mangle us
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file
 */

#include <stdbool.h>
#include "ECUProtocol.hpp"
#include "ECUDefines.hpp"
/*!
 * ECU settings packets (not available on CAN or RS232 interfaces)
 */
typedef enum
{
    PKT_ECU_SETTINGS_THROTTLE = 0xF0,    //!< Throttle settings
    PKT_ECU_SETTINGS_PUMP = 0xF1,        //!< Throttle settings
    PKT_ECU_SETTINGS_GOVERNOR = 0xF2,    //!< Throttle settings
    PKT_ECU_SETTINGS_TELEMETRY = 0xF3,   //!< Throttle settings
    PKT_ECU_SETTINGS_FUEL_USED = 0xF4,   //!< Throttle settings
    PKT_ECU_SETTINGS_ENGINE_TIME = 0xF5, //!< Engine run time
    PKT_ECU_SETTINGS_USER = 0xFA         //!< User data
} ECUSettingsPackets;

typedef struct
{
    unsigned reserved : 8; //!< Reserved for future use
}ECU_PumpOptionBits_t;

//! return the minimum encoded length for the ECU_PumpOptionBits_t structure
#define getMinLengthOfECU_PumpOptionBits_t() (1)

//! return the maximum encoded length for the ECU_PumpOptionBits_t structure
#define getMaxLengthOfECU_PumpOptionBits_t() (1)

//! Encode a ECU_PumpOptionBits_t into a byte array
void encodeECU_PumpOptionBits_t(uint8_t* data, int* bytecount, const ECU_PumpOptionBits_t* user);

//! Decode a ECU_PumpOptionBits_t from a byte array
int decodeECU_PumpOptionBits_t(const uint8_t* data, int* bytecount, ECU_PumpOptionBits_t* user);

/*!
 * Throttle settings
 */
typedef struct
{
    uint16_t                      pulseOpen;        //!< Throttle open PWM value
    uint16_t                      pulseClosed;      //!< Throttle closed PWM value
    uint16_t                      pulseInputOpen;   //!< Throttle input open PWM value
    uint16_t                      pulseInputClosed; //!< Throttle input closed PWM value
    uint8_t                       delay;            //!< Throttle delay, constant value
    uint8_t                       minDelay;         //!< Throttle delay, minimum value
    uint8_t                       maxDelay;         //!< Throttle delay, minimum value
    ECU_ThrottleDelayConfigBits_t delayConfig;
    uint8_t                       softLimit;        //!< Throttle dashpot soft limit value
    uint8_t                       hardLimit;        //!< Throttle dashpot hard limit value
    uint8_t                       falloffRate;      //!< Throttle dashpot falloff rate
    uint8_t                       curve[11];        //!< Throttle curve lookup table elements
    ECU_ThrottleCurveConfigBits_t curveConfig;      //!< Throttle curve config bits
    ECU_ThrottleConfigBits_t      config;
    uint8_t                       analogSpan1;
    uint16_t                      analogSpan2;
    uint8_t                       throttleTarget;
}ECU_ThrottleSettings_t;

//! Create the ECU_ThrottleSettings packet
void encodeECU_ThrottleSettingsPacketStructure(void* pkt, const ECU_ThrottleSettings_t* user);

//! Decode the ECU_ThrottleSettings packet
int decodeECU_ThrottleSettingsPacketStructure(const void* pkt, ECU_ThrottleSettings_t* user);

//! return the packet ID for the ECU_ThrottleSettings packet
#define getECU_ThrottleSettingsPacketID() (PKT_ECU_SETTINGS_THROTTLE)

//! return the minimum encoded length for the ECU_ThrottleSettings packet
#define getECU_ThrottleSettingsMinDataLength() (31)

//! return the maximum encoded length for the ECU_ThrottleSettings packet
#define getECU_ThrottleSettingsMaxDataLength() (31)

/*!
 * Throttle settings
 */
typedef struct
{
    uint8_t  resetOnStartup;
    uint16_t divisor;
    uint16_t offset;
}ECU_FuelUsedSettings_t;

//! Create the ECU_FuelUsedSettings packet
void encodeECU_FuelUsedSettingsPacketStructure(void* pkt, const ECU_FuelUsedSettings_t* user);

//! Decode the ECU_FuelUsedSettings packet
int decodeECU_FuelUsedSettingsPacketStructure(const void* pkt, ECU_FuelUsedSettings_t* user);

//! return the packet ID for the ECU_FuelUsedSettings packet
#define getECU_FuelUsedSettingsPacketID() (PKT_ECU_SETTINGS_FUEL_USED)

//! return the minimum encoded length for the ECU_FuelUsedSettings packet
#define getECU_FuelUsedSettingsMinDataLength() (5)

//! return the maximum encoded length for the ECU_FuelUsedSettings packet
#define getECU_FuelUsedSettingsMaxDataLength() (5)

/*!
 * Throttle settings
 */
typedef struct
{
    float   pGain;
    float   iGain;
    float   dGain;
    float   scalePower;
    float   maxRPM;
    float   minRPM;
    uint8_t mode;
}ECU_GovernorSettings_t;

//! Create the ECU_GovernorSettings packet
void encodeECU_GovernorSettingsPacketStructure(void* pkt, const ECU_GovernorSettings_t* user);

//! Decode the ECU_GovernorSettings packet
int decodeECU_GovernorSettingsPacketStructure(const void* pkt, ECU_GovernorSettings_t* user);

//! return the packet ID for the ECU_GovernorSettings packet
#define getECU_GovernorSettingsPacketID() (PKT_ECU_SETTINGS_GOVERNOR)

//! return the minimum encoded length for the ECU_GovernorSettings packet
#define getECU_GovernorSettingsMinDataLength() (19)

//! return the maximum encoded length for the ECU_GovernorSettings packet
#define getECU_GovernorSettingsMaxDataLength() (19)

/*!
 * Throttle settings
 */
typedef struct
{
    float                kp;                 //!< Pump proportional gain
    float                ki;                 //!< Pump integral gain
    float                km;                 //!< Pump IMC (internal model) gain
    float                pressureLowerLimit; //!< Pump lower pressure limit (PSI)
    float                pressureUpperLimit; //!< Pump upper pressure limit (PSI)
    float                pressureSetpoint;   //!< Fuel pressure setpoint
    uint8_t              minimumPWM;
    uint8_t              maximumPWM;
    uint8_t              rampRate;           //!< Pump duty cycle ramp rate
    ECU_PumpOptionBits_t options;            //!< Pump control system options
    uint8_t              reservedA;          //!< Reserved for future use
    uint8_t              reservedB;          //!< Reserved for future use
}ECU_PumpSettings_t;

//! Create the ECU_PumpSettings packet
void encodeECU_PumpSettingsPacketStructure(void* pkt, const ECU_PumpSettings_t* user);

//! Decode the ECU_PumpSettings packet
int decodeECU_PumpSettingsPacketStructure(const void* pkt, ECU_PumpSettings_t* user);

//! return the packet ID for the ECU_PumpSettings packet
#define getECU_PumpSettingsPacketID() (PKT_ECU_SETTINGS_PUMP)

//! return the minimum encoded length for the ECU_PumpSettings packet
#define getECU_PumpSettingsMinDataLength() (30)

//! return the maximum encoded length for the ECU_PumpSettings packet
#define getECU_PumpSettingsMaxDataLength() (30)

/*!
 * User data
 */
typedef struct
{
    uint16_t powerCycles;
    uint32_t engineTime;
    uint32_t engineTimeTotal;
    uint16_t fuelUsedOverflows;
    uint8_t  userValues[8];
}ECU_ECUData_t;

//! Create the ECU_ECUData packet
void encodeECU_ECUDataPacketStructure(void* pkt, const ECU_ECUData_t* user);

//! Decode the ECU_ECUData packet
int decodeECU_ECUDataPacketStructure(const void* pkt, ECU_ECUData_t* user);

//! return the packet ID for the ECU_ECUData packet
#define getECU_ECUDataPacketID() (PKT_ECU_SETTINGS_USER)

//! return the minimum encoded length for the ECU_ECUData packet
#define getECU_ECUDataMinDataLength() (20)

//! return the maximum encoded length for the ECU_ECUData packet
#define getECU_ECUDataMaxDataLength() (20)

#ifdef __cplusplus
}
#endif
#endif // _ECUSETTINGS_H
