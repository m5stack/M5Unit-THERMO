/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_NCIR2_data.hpp
  @brief NCIR2 data types (Data struct)

  Kept apart from unit_NCIR2.hpp so the native (host) tests can include it
  without pulling in M5UnitComponent.
*/
#ifndef M5_UNIT_THERMO_UNIT_NCIR2_DATA_HPP
#define M5_UNIT_THERMO_UNIT_NCIR2_DATA_HPP

#include <array>
#include <cstdint>

#include <m5_utility/types.hpp>

namespace m5 {
namespace unit {

/*!
  @namespace ncir2
  @brief For UnitNCIR2
 */
namespace ncir2 {
/*!
  @struct Data
  @brief Measurement data group
  @note Valid to the second decimal place
 */
struct Data {
    std::array<uint8_t, 2> raw{0x00, 0x80};  //!< Raw [0]:low byte [1]:high byte (default 0x8000: no data)
    //! @brief Raw int16
    inline int16_t value() const
    {
        return m5::types::little_int16_t{raw[0], raw[1]}.get();
    }
    //! @brief Temperature(Celsius)
    inline float temperature() const
    {
        return value() * 0.01f;
    }
    //! @brief Celsius
    inline float celsius() const
    {
        return temperature();
    }
    //! @brief Fahrenheit
    inline float fahrenheit() const
    {
        return celsius() * 9.0f / 5.0f + 32.f;
    }
};
}  // namespace ncir2

}  // namespace unit
}  // namespace m5
#endif
