/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_Thermal2_data.hpp
  @brief Thermal2 data types (constants, Refresh, conversions, Data struct)

  Kept apart from unit_Thermal2.hpp so the native (host) tests can include it
  without pulling in M5UnitComponent.
*/
#ifndef M5_UNIT_THERMO_UNIT_THERMAL2_DATA_HPP
#define M5_UNIT_THERMO_UNIT_THERMAL2_DATA_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>  // NaN

namespace m5 {
namespace unit {

/*!
  @namespace thermal2
  @brief For UnitThermal2
 */
namespace thermal2 {

///@sa m5::unit::UnitThermal2::readButtonStatus
///@name Button status
///@{
constexpr uint8_t button_is_pressed{0x01};    //!< Is pressed now
constexpr uint8_t button_was_pressed{0x02};   //!< Was pressed
constexpr uint8_t button_was_released{0x04};  //!< Was released
constexpr uint8_t button_was_clicked{0x08};   //!< Was clicked
constexpr uint8_t button_was_hold{0x10};      //!< Was held
///@}

///@sa m5::unit::UnitThermal2::readFunctionControl m5::unit::UnitThermal2::writeFunctionControl
///@name Function control
///@{
constexpr uint8_t enabled_function_buzzer{0x01};        //!< Buzzer enabled
constexpr uint8_t enabled_function_led{0x02};           //!< LED enabled
constexpr uint8_t enabled_function_auto_refresh{0x04};  //!< Auto refresh (periodic measurement) enabled
///@}

///@sa m5::unit::UnitThermal2::readAlarmEnabled m5::unit::UnitThermal2::writeAlarmEnabled
///@name Alarm enabled bits
///@{
constexpr uint8_t enabled_low_temperature_low{0x01};    //!< Low temp reached low threshold
constexpr uint8_t enabled_med_temperature_low{0x02};    //!< Med temp reached low threshold
constexpr uint8_t enabled_ave_temperature_low{0x04};    //!< Ave temp reached low threshold
constexpr uint8_t enabled_high_temperature_low{0x08};   //!< High temp reached low threshold
constexpr uint8_t enabled_low_temperature_high{0x10};   //!< Low temp reached high threshold
constexpr uint8_t enabled_med_temperature_high{0x20};   //!< Med temp reached high threshold
constexpr uint8_t enabled_ave_temperature_high{0x40};   //!< Ave temp reached high threshold
constexpr uint8_t enabled_high_temperature_high{0x80};  //!< High temp reached high threshold
///@}

/*!
  @enum Refresh
  @brief Refresh rate(Hz)
 */
enum class Refresh : uint8_t {
    Rate0_5Hz,  //!< 0.5Hz
    Rate1Hz,    //!< 1Hz
    Rate2Hz,    //!< 2Hz
    Rate4Hz,    //!< 4Hz
    Rate8Hz,    //!< 8Hz
    Rate16Hz,   //!< 16Hz
    Rate32Hz,   //!< 32Hz
    Rate64Hz,   //!< 64Hz
};

/*!
  @brief Celsius to raw temperature value
  @param f Celsius
  @return Raw value ((f + 64) * 128, clamped to 0 - 65535; NaN gives 65535)
 */
inline uint16_t celsius_to_raw(const float f)
{
    // Clamp in float before converting, so that NaN or a huge value does not overflow the integer conversion
    const float v{std::fmax(std::fmin((f + 64) * 128, 65535.f), 0.f)};
    return static_cast<uint16_t>(std::round(v));
}
/*!
  @brief Raw temperature value to celsius
  @param u16 Raw value
  @return Celsius
 */
inline float raw_to_celsius(const uint16_t u16)
{
    return u16 / 128.0f - 64;
}

#pragma pack(push)
#pragma pack(1)
/*!
  @struct Data
  @brief Measurement data group
 */
struct Data {
    uint8_t subpage{};  //!< Subpage 0:even 1:odd
    union {
        uint16_t temp[8]{};  //!< Temperature information
        struct {
            uint16_t median_temperature;     //!< Median temperature (raw)
            uint16_t average_temperature;    //!< Average temperature (raw)
            uint16_t most_diff_temperature;  //!< Most different temperature (raw)
            uint8_t most_diff_x;             //!< Most different pixel X
            uint8_t most_diff_y;             //!< Most different pixel Y
            uint16_t lowest_temperature;     //!< Lowest temperature (raw)
            uint8_t lowest_diff_x;           //!< Lowest pixel X
            uint8_t lowest_diff_y;           //!< Lowest pixel Y
            uint16_t highest_temperature;    //!< Highest temperature (raw)
            uint8_t highest_diff_x;          //!< Highest pixel X
            uint8_t highest_diff_y;          //!< Highest pixel Y
        };
    };
    uint16_t raw[384]{};  //!< Raw pixel data (1/2)

    //! @brief Gets median temperature in Celsius
    inline float medianTemperature() const
    {
        return thermal2::raw_to_celsius(temp[0]);
    }
    //! @brief Gets average temperature in Celsius
    inline float averageTemperature() const
    {
        return thermal2::raw_to_celsius(temp[1]);
    }
    //! @brief Gets most different temperature in Celsius
    inline float mostDiffTemperature() const
    {
        return thermal2::raw_to_celsius(temp[2]);
    }
    //! @brief Gets lowest temperature in Celsius
    inline float lowestTemperature() const
    {
        return thermal2::raw_to_celsius(temp[4]);
    }
    //! @brief Gets highest temperature in Celsius
    inline float highestTemperature() const
    {
        return thermal2::raw_to_celsius(temp[6]);
    }

    //! @brief Gets pixel temperature in Celsius
    //! @param idx Pixel index (0-383)
    //! @return Temperature in Celsius, or NaN if idx is out of range
    inline float temperature(const uint_fast16_t idx) const
    {
        return (idx < 384) ? thermal2::raw_to_celsius(raw[idx]) : std::numeric_limits<float>::quiet_NaN();
    }
};
#pragma pack(pop)

}  // namespace thermal2

}  // namespace unit
}  // namespace m5
#endif
