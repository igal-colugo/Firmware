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

#include <limits.h>

#include <mixer_module/output_functions.hpp>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/colugo_actuator.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/lights.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/strobe.h>
#include <uORB/topics/vehicle_command.h>

#define MAV_CMD_LIGHTS (51001)
#define MAV_CMD_STROBE (51002)

class FunctionProviderBase
{
  public:
    struct Context
    {
        px4::WorkItem &work_item;
        const float &thrust_factor;
    };

    FunctionProviderBase() = default;
    virtual ~FunctionProviderBase() = default;

    virtual void update() = 0;

    /**
     * Get the current output value for a given function
     * @return NAN (=disarmed) or value in range [-1, 1]
     */
    virtual float value(OutputFunction func) = 0;

    virtual float defaultFailsafeValue(OutputFunction func) const
    {
        return NAN;
    }
    virtual bool allowPrearmControl() const
    {
        return true;
    }

    virtual uORB::SubscriptionCallbackWorkItem *subscriptionCallback()
    {
        return nullptr;
    }

    virtual bool getLatestSampleTimestamp(hrt_abstime &t) const
    {
        return false;
    }

    /**
     * Check whether the output (motor) is configured to be reversible
     */
    virtual bool reversible(OutputFunction func) const
    {
        return false;
    }
};

/**
 * Functions: Constant_Min
 */
class FunctionConstantMin : public FunctionProviderBase
{
  public:
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionConstantMin();
    }

    float value(OutputFunction func) override
    {
        return -1.f;
    }
    void update() override
    {
    }

    float defaultFailsafeValue(OutputFunction func) const override
    {
        return -1.f;
    }
};

/**
 * Functions: Constant_Max
 */
class FunctionConstantMax : public FunctionProviderBase
{
  public:
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionConstantMax();
    }

    float value(OutputFunction func) override
    {
        return 1.f;
    }
    void update() override
    {
    }

    float defaultFailsafeValue(OutputFunction func) const override
    {
        return 1.f;
    }
};

/**
 * Functions: Motor1 ... MotorMax
 */
class FunctionMotors : public FunctionProviderBase
{
  public:
    static_assert(actuator_motors_s::NUM_CONTROLS == (int) OutputFunction::MotorMax - (int) OutputFunction::Motor1 + 1, "Unexpected num motors");

    FunctionMotors(const Context &context);
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionMotors(context);
    }

    void update() override;
    float value(OutputFunction func) override
    {
        return _data.control[(int) func - (int) OutputFunction::Motor1];
    }

    bool allowPrearmControl() const override
    {
        return false;
    }

    uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override
    {
        return &_topic;
    }

    bool getLatestSampleTimestamp(hrt_abstime &t) const override
    {
        t = _data.timestamp_sample;
        return t != 0;
    }

    static inline void updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values);

    bool reversible(OutputFunction func) const override
    {
        return _data.reversible_flags & (1u << ((int) func - (int) OutputFunction::Motor1));
    }

  private:
    uORB::SubscriptionCallbackWorkItem _topic;
    actuator_motors_s _data{};
    const float &_thrust_factor;
};

void FunctionMotors::updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values)
{
    if (thrust_factor > 0.f && thrust_factor <= 1.f)
    {
        // thrust factor
        //  rel_thrust = factor * x^2 + (1-factor) * x,
        const float a = thrust_factor;
        const float b = (1.f - thrust_factor);

        // don't recompute for all values (ax^2+bx+c=0)
        const float tmp1 = b / (2.f * a);
        const float tmp2 = b * b / (4.f * a * a);

        for (int i = 0; i < num_values; ++i)
        {
            float control = values[i];

            if (control > 0.f)
            {
                values[i] = -tmp1 + sqrtf(tmp2 + (control / a));
            }
            else if (control < -0.f)
            {
                values[i] = tmp1 - sqrtf(tmp2 - (control / a));
            }
            else
            {
                values[i] = 0.f;
            }
        }
    }

    for (int i = 0; i < num_values; ++i)
    {
        if ((reversible & (1u << i)) == 0)
        {
            if (values[i] < -FLT_EPSILON)
            {
                values[i] = NAN;
            }
            else
            {
                // remap from [0, 1] to [-1, 1]
                values[i] = values[i] * 2.f - 1.f;
            }
        }
    }
}

/**
 * Functions: Servo1 ... ServoMax
 */
class FunctionServos : public FunctionProviderBase
{
  public:
    static_assert(actuator_servos_s::NUM_CONTROLS == (int) OutputFunction::ServoMax - (int) OutputFunction::Servo1 + 1, "Unexpected num servos");

    FunctionServos(const Context &context);
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionServos(context);
    }

    void update() override
    {
        _topic.update(&_data);
    }
    float value(OutputFunction func) override
    {
        return _data.control[(int) func - (int) OutputFunction::Servo1];
    }

    uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override
    {
        return &_topic;
    }

    float defaultFailsafeValue(OutputFunction func) const override
    {
        return 0.f;
    }

  private:
    uORB::SubscriptionCallbackWorkItem _topic;
    actuator_servos_s _data{};
};

/**
 * Functions: Offboard_Actuator_Set1 ... Offboard_Actuator_Set6
 */
class FunctionActuatorSet : public FunctionProviderBase
{
  public:
    FunctionActuatorSet();
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionActuatorSet();
    }

    void update() override;
    float value(OutputFunction func) override
    {
        return _data[(int) func - (int) OutputFunction::Offboard_Actuator_Set1];
    }

  private:
    static constexpr int max_num_actuators = 6;

    uORB::Subscription _topic{ORB_ID(vehicle_command)};
    float _data[max_num_actuators];
};

/**
 * Functions: Landing_Gear
 */
class FunctionLandingGear : public FunctionProviderBase
{
  public:
    FunctionLandingGear() = default;
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionLandingGear();
    }

    void update() override;
    float value(OutputFunction func) override
    {
        return _data;
    }

  private:
    uORB::Subscription _topic{ORB_ID(landing_gear)};
    float _data{-1.f};
};

/**
 * Functions: Colugo
 */
class FunctionColugoActuator : public FunctionProviderBase
{
  public:
    FunctionColugoActuator() = default;
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionColugoActuator();
    }

    void update() override;
    float value(OutputFunction func) override
    {
        return _data;
    }

  private:
    uORB::Subscription _topic{ORB_ID(colugo_actuator)};
    float _data{-1.f};
};

/**
 * Functions: Parachute
 */
class FunctionParachute : public FunctionProviderBase
{
  public:
    FunctionParachute() = default;
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionParachute();
    }

    void update() override
    {
    }
    float value(OutputFunction func) override
    {
        return -1.f;
    }
    float defaultFailsafeValue(OutputFunction func) const override
    {
        return 1.f;
    }
};

/**
 * Functions: RC_Roll .. RCAUX_Max
 */
class FunctionManualRC : public FunctionProviderBase
{
  public:
    FunctionManualRC();
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionManualRC();
    }

    void update() override;
    float value(OutputFunction func) override
    {
        return _data[(int) func - (int) OutputFunction::RC_Roll];
    }

  private:
    static constexpr int num_data_points = 11;

    static_assert(num_data_points == (int) OutputFunction::RC_AUXMax - (int) OutputFunction::RC_Roll + 1, "number of functions mismatch");

    uORB::Subscription _topic{ORB_ID(manual_control_setpoint)};
    float _data[num_data_points];
};

/**
 * Functions: Gimbal_Roll .. Gimbal_Yaw
 */
class FunctionGimbal : public FunctionProviderBase
{
  public:
    FunctionGimbal() = default;
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionGimbal();
    }

    void update() override;
    float value(OutputFunction func) override
    {
        return _data[(int) func - (int) OutputFunction::Gimbal_Roll];
    }

  private:
    uORB::Subscription _topic{ORB_ID(actuator_controls_2)};
    float _data[3]{NAN, NAN, NAN};
};

/**
 * Functions: Lights
 */
class FunctionLights : public FunctionProviderBase
{
  public:
    FunctionLights() = default;
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionLights();
    }

    void update() override;

    float value(OutputFunction func) override
    {
        return _data;
    }
    float defaultFailsafeValue(OutputFunction func) const override
    {
        return 1.f;
    }

  private:
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _lights_sub{ORB_ID(lights)};
    float _data{-1.f};
};

/**
 * Functions: Strobe
 */
class FunctionStrobe : public FunctionProviderBase
{
  public:
    FunctionStrobe() = default;
    static FunctionProviderBase *allocate(const Context &context)
    {
        return new FunctionStrobe();
    }

    void update() override;

    float value(OutputFunction func) override
    {
        return _data;
    }
    float defaultFailsafeValue(OutputFunction func) const override
    {
        return 1.f;
    }

  private:
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _strobe_sub{ORB_ID(strobe)};
    float _data{-1.f};
};
