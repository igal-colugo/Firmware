#include "fail_handler.h"

#pragma region GPS handler

bool GPSFailHandler::set_parameters(double *parameters)
{
    bool return_value = true;

    memcpy(&handler_parameters, parameters, sizeof(handler_parameters));

    return return_value;
}

bool GPSFailHandler::set_taking_care(uint8_t taking_care)
{
    _taking_care = taking_care;
}

uint8_t GPSFailHandler::get_taking_care()
{
    return _taking_care;
}

bool GPSFailHandler::is_fail()
{
    static bool return_value = true;

    static uint32_t actual_time_ms = 0;
    static uint32_t previous_time_ms = 0;
    uint8_t counter_fails = 0;
    static uint32_t good_condition_time_accumulator = 0;
    static uint32_t bad_condition_time_accumulator = 0;
    static bool actual_state = false;
    static bool previous_state = !((check() > 0) ? true : false); // assume it was opposite to actual state
    static bool is_can_run_good_condition_stopwatch = false;
    static bool is_can_run_bad_condition_stopwatch = false;
    static bool init_condition = false;

    if (init_condition == false)
    {
        if (previous_state == actual_state)
        {
            return_value = false;
        }

        init_condition = true;
    }

    // 1-get actual time
    actual_time_ms = hrt_absolute_time();
    // 2-check gps
    counter_fails = check();
    // 3-set actual state
    actual_state = (counter_fails > 0) ? true : false;

    if (actual_state != previous_state)
    {
        if (actual_state == true)
        {
            good_condition_time_accumulator = 0;
            is_can_run_good_condition_stopwatch = false;
            is_can_run_bad_condition_stopwatch = true;
        }
        else
        {
            bad_condition_time_accumulator = 0;
            is_can_run_good_condition_stopwatch = true;
            is_can_run_bad_condition_stopwatch = false;
        }
    }
    if (is_can_run_good_condition_stopwatch == true)
    {
        // 6- accumulate time and return previous state
        actual_time_ms = hrt_absolute_time();
        good_condition_time_accumulator += abs((int32_t) (actual_time_ms - previous_time_ms));
    }
    if (is_can_run_bad_condition_stopwatch == true)
    {
        // 6- accumulate time and return previous state
        actual_time_ms = hrt_absolute_time();
        bad_condition_time_accumulator += abs((int32_t) (actual_time_ms - previous_time_ms));
    }
    if (good_condition_time_accumulator >= handler_parameters[0] && good_condition_time_accumulator > 0)
    {
        // 8-time out and update previous state
        is_can_run_good_condition_stopwatch = false;
        good_condition_time_accumulator = 0;
        return_value = actual_state;
    }
    if (bad_condition_time_accumulator >= handler_parameters[1] && bad_condition_time_accumulator > 0)
    {
        // 8-time out and update previous state
        is_can_run_bad_condition_stopwatch = false;
        bad_condition_time_accumulator = 0;
        return_value = actual_state;
    }

    previous_state = actual_state;

    // 9-update previous time
    previous_time_ms = hrt_absolute_time();

    return return_value;
}

uint8_t GPSFailHandler::check()
{
    int gps_sub = orb_subscribe(ORB_ID(sensor_gps));

    px4_pollfd_struct_t fds[1];

    fds[0].fd = gps_sub;
    fds[0].events = POLLIN;

    // get data from gps sensor
    orb_copy(ORB_ID(sensor_gps), gps_sub, &sensor_gps);

    bool check_result = false;
    uint8_t counter_fails = 0;

    check_result = check_healthy(sensor_gps);
    counter_fails = (check_result == false) ? counter_fails + 1 : counter_fails;
    check_result = check_satellites(sensor_gps);
    counter_fails = (check_result == false) ? counter_fails + 1 : counter_fails;
    check_result = check_velocity_variance(sensor_gps);
    counter_fails = (check_result == false) ? counter_fails + 1 : counter_fails;
    check_result = check_distance(sensor_gps);
    counter_fails = (check_result == false) ? counter_fails + 1 : counter_fails;
    check_result = check_hdop(sensor_gps);
    counter_fails = (check_result == false) ? counter_fails + 1 : counter_fails;
    check_result = check_status(sensor_gps);
    counter_fails = (check_result == false) ? counter_fails + 1 : counter_fails;

    orb_unsubscribe(gps_sub);

    return counter_fails;
}

bool GPSFailHandler::check_healthy(sensor_gps_s sensor_gps)
{
    bool return_value = true;

    // if (sensor_gps..is_healthy(GPS_REAL) == false)
    // {
    //     return_value = false;
    //     _error_code = _error_code | (uint32_t) EErrorCode::GPS_HEALTHY_ERROR;
    // }
    // else
    // {
    //     _error_code = _error_code & (~((uint32_t) EErrorCode::GPS_HEALTHY_ERROR));
    // }

    return return_value;
}

bool GPSFailHandler::check_satellites(sensor_gps_s sensor_gps)
{
    bool return_value = true;

    if (sensor_gps.satellites_used < handler_parameters[2])
    {
        return_value = false;
        _error_code = _error_code | (uint32_t) EErrorCode::GPS_SATELLITES_ERROR;
    }
    else
    {
        _error_code = _error_code & (~((uint32_t) EErrorCode::GPS_SATELLITES_ERROR));
    }

    return return_value;
}

bool GPSFailHandler::check_hdop(sensor_gps_s sensor_gps)
{
    bool return_value = true;

    if (sensor_gps.hdop > handler_parameters[5])
    {
        return_value = false;
        _error_code = _error_code | (uint32_t) EErrorCode::GPS_HDOP_ERROR;
    }
    else
    {
        _error_code = _error_code & (~((uint32_t) EErrorCode::GPS_HDOP_ERROR));
    }

    return return_value;
}

bool GPSFailHandler::check_status(sensor_gps_s sensor_gps)
{
    bool return_value = true;

    if (sensor_gps.fix_type < handler_parameters[6])
    {
        return_value = false;
        _error_code = _error_code | (uint32_t) EErrorCode::GPS_STATUS_ERROR;
    }
    else
    {
        _error_code = _error_code & (~((uint32_t) EErrorCode::GPS_STATUS_ERROR));
    }

    return return_value;
}

bool GPSFailHandler::check_distance(sensor_gps_s sensor_gps)
{
    bool return_value = true;

    // Location& actual_location;
    static int32_t previous_location_lat = 0;
    static int32_t previous_location_lon = 0;
    static float actual_distance = 0.0; // units km

    int32_t actual_location_lat = sensor_gps.lat;
    int32_t actual_location_lon = sensor_gps.lon;

    float actual_longitude = radians(actual_location_lon / 10000000.0);
    float actual_latitude = radians(actual_location_lat / 10000000.0);
    float previous_longitude = radians(previous_location_lon / 10000000.0);
    float previous_latitude = radians(previous_location_lat / 10000000.0);

    float sin_act_lat = sinf(actual_latitude);
    float sin_prev_lat = sinf(previous_latitude);

    float cos_act_lat = cosf(actual_latitude);
    float cos_prev_lat = cosf(previous_latitude);
    float cos_dif_prev_act_lon = cosf(previous_longitude - actual_longitude);
    float acos_content = sin_act_lat * sin_prev_lat + cos_act_lat * cos_prev_lat * cos_dif_prev_act_lon;

    if (!isnan(acos_content))
    {
        /* The acos() function returns the arccosine of x. The value of x must be between -1 and 1 inclusive.
         If x is less than -1 or greater than 1, acos() sets errno to EDOM and returns 0.*/
        if (acos_content >= -1.0 && acos_content <= 1.0)
        {
            actual_distance = acosf(acos_content) * 6371; // 6371 is Earth radius in km
        }
    }

    if (actual_distance > handler_parameters[3])
    {
        return_value = false;
        _error_code = _error_code | (uint32_t) EErrorCode::GPS_DISTANCE_ERROR;
    }
    else
    {
        _error_code = _error_code & (~((uint32_t) EErrorCode::GPS_DISTANCE_ERROR));
    }

    previous_location_lat = actual_location_lat;
    previous_location_lon = actual_location_lon;

    return return_value;
}

bool GPSFailHandler::check_velocity_variance(sensor_gps_s sensor_gps)
{
    bool return_value = true;

    // float vel_variance = 0.0;
    // float pos_variance = 0.0;
    // float hgt_variance = 0.0;
    // float tas_variance = 0.0;
    // Vector3f mag_variance;
    // Vector2f offset;

    // if (&ahrs != nullptr)
    // {
    //     bool is_variance_available = ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance);
    //     if (is_variance_available == true)
    //     {
    //         if (!isnan(vel_variance))
    //         {
    //             if (vel_variance >= handler_parameters[4])
    //             {
    //                 return_value = false;
    //                 _error_code = _error_code | (uint32_t) EErrorCode::GPS_VELOCITY_VARIANCE_ERROR;
    //             }
    //             else
    //             {
    //                 _error_code = _error_code & (~((uint32_t) EErrorCode::GPS_VELOCITY_VARIANCE_ERROR));
    //             }
    //         }
    //     }
    // }
    // else
    // {
    //     return_value = false;
    // }

    return return_value;
}

#pragma endregion
