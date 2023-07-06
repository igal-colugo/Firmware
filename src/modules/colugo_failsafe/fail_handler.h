#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>

#define M_PI (3.141592653589793)
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
// Centi-degrees to radians
#define DEGX100 5729.57795f

#define CLASS_BASE_HANDLER 1
#define CLASS_GPS_HANDLER 2
#define CLASS_AHRS_HANDLER 3
#define CLASS_BARO_HANDLER 4
#define CLASS_COMPASS_HANDLER 5
#define CLASS_PROP_HANDLER 6
#define CLASS_ROLL_HANDLER 7
#define CLASS_PITCH_HANDLER 8
#define CLASS_COMMUNICATION_HANDLER 9

#define GPS_REAL 0
#define GPS_ASIO 1

/// @class Fail handler base class
class FailHandler
{
  public:
    double handler_parameters[10];
    struct sensor_gps_s sensor_gps;
    bool is_handled = false;

    FailHandler()
    {
    }
    FailHandler(uint32_t id)
    {
        _id = id;
        _error_code = 0;
    }

    virtual int classType()
    {
        return CLASS_BASE_HANDLER;
    }
    virtual bool set_parameters(double *parameters) = 0;
    virtual bool is_fail() = 0;
    virtual bool set_taking_care(uint8_t taking_care) = 0;
    virtual uint8_t get_taking_care() = 0;

    uint32_t get_id()
    {
        return _id;
    }
    uint32_t get_error_code()
    {
        return _error_code;
    }

    static inline constexpr float radians(float deg)
    {
        return deg * DEG_TO_RAD;
    }

    // radians -> degrees
    static inline constexpr float degrees(float rad)
    {
        return rad * RAD_TO_DEG;
    }

  protected:
    uint32_t _id;
    uint8_t _taking_care;
    uint32_t _error_code;
    uint8_t _counter_fails;
};
/// @class GPS fail handler class
class GPSFailHandler : public FailHandler
{
  public:
    enum class EErrorCode : uint32_t
    {
        GPS_HEALTHY_ERROR = 1U << 0U,
        GPS_SATELLITES_ERROR = 1U << 1U,
        GPS_VELOCITY_VARIANCE_ERROR = 1U << 2U,
        GPS_DISTANCE_ERROR = 1U << 3U,
        GPS_HDOP_ERROR = 1U << 4U,
        GPS_STATUS_ERROR = 1U << 5U
    };
    GPSFailHandler(uint32_t id)
    {
        _id = id;
        _error_code = 0;
        memset(&handler_parameters, 0, sizeof(handler_parameters));
    }
    virtual int classType()
    {
        return CLASS_GPS_HANDLER;
    }
    bool is_fail() override;
    /*
    _params[0]-GPS grace time
    _params[1]-GPS fail grace time
    _params[2]-GPS minimum amount satellites
    _params[3]-GPS distance variance
    _params[4]-GPS velocity variance
    _params[5]-GPS hdop
    _params[6]-GPS status
    _params[7]-GPS reserved
    _params[8]-GPS reserved
    _params[9]-GPS reserved
    */
    bool set_parameters(double *parameters) override;
    bool set_taking_care(uint8_t taking_care) override;
    uint8_t get_taking_care() override;

  private:
    uint8_t check();
    bool check_healthy(sensor_gps_s sensor_gps);
    bool check_satellites(sensor_gps_s sensor_gps);
    bool check_hdop(sensor_gps_s sensor_gps);
    bool check_status(sensor_gps_s sensor_gps);
    bool check_distance(sensor_gps_s sensor_gps);
    bool check_velocity_variance(sensor_gps_s sensor_gps);
};
