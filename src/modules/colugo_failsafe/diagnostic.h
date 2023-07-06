#pragma once

#include <cstdint>

/// @class	Diagnostic
class Diagnostic
{

  public:
    enum EnumIndicators
    {
    };

    union DiagnosticIndicators {
        uint32_t array = 0;

        struct __attribute__((packed, aligned(1)))
        {
            uint32_t bit_0 : 1;
            uint32_t bit_1 : 1;
            uint32_t bit_2 : 1;
            uint32_t bit_3 : 1;
            uint32_t bit_4 : 1;
            uint32_t bit_5 : 1;
            uint32_t bit_6 : 1;
            uint32_t bit_7 : 1;
            uint32_t bit_8 : 1;
            uint32_t bit_9 : 1;
            uint32_t bit_10 : 1;
        } bits;
    };

    DiagnosticIndicators indicators;

    Diagnostic()
    {
        _singleton = this;
    }

    /* Do not allow copies */
    Diagnostic(const Diagnostic &other) = delete;
    Diagnostic &operator=(const Diagnostic &) = delete;

    // get singleton instance
    static Diagnostic *get_singleton()
    {
        return _singleton;
    }

  private:
    static Diagnostic *_singleton;

    int8_t _diagnostic_parameter_1;
    int8_t _diagnostic_parameter_2;
};
