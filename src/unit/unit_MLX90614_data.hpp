/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_MLX90614_data.hpp
  @brief MLX90614 data types (enums, Data struct, register fields, conversions)

  Kept apart from unit_MLX90614.hpp so the native (host) tests can include it
  without pulling in M5UnitComponent.
*/
#ifndef M5_UNIT_THERMO_UNIT_MLX90614_DATA_HPP
#define M5_UNIT_THERMO_UNIT_MLX90614_DATA_HPP

#include <m5_utility/stl/extension.hpp>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>  // NaN

namespace m5 {
namespace unit {

/*!
  @namespace mlx90614
  @brief For mlx90614
 */
namespace mlx90614 {

/*!
  @enum Output
  @brief PWM output mode
 */
enum class Output : uint8_t {
    TA_TO1,         //!< PWM1: Ta PWM2:To1 (Ambient & Object 1)
    TA_TO2,         //!< PWM1: Ta PWM2:To2 (Ambient & Object 2)
    TO2_Undefined,  //!< PWM1: To2 PWM2:Undefined (Object 2)
    TO1_TO2,        //!< PWM1: To1 PWM2:To2 (Object 1 & 2)
};

/*!
  @enum IIR
  @brief Infinite Impulse Response
 */
enum class IIR : uint8_t {
    Filter50,   //!< 50% (a1 = 0.5, b1 = 0.5)
    Filter25,   //!< 25% (a1 = 0.25, b1 = 0.75)
    Filter17,   //!< 17% (a1 = 0.166(6), b1 = 0.833(3))
    Filter13,   //!< 13% (a1 = 0.125, b1 = 0.875)
    Filter100,  //!< 100% (a1 = 1, b1 = 0)
    Filter80,   //!< 80% (a1 = 0.8, b1 = 0.2)
    Filter67,   //!< 67% (a1 = 0.666, b1 = 0.333)
    Filter57,   //!< 57% (a1 = 0.571, b1 = 0.428)
};

/*!
  @enum FIR
  @brief Finite Impulse Response
 */
enum class FIR : uint8_t {
    Filter8,     //!< 8 Not recommended
    Filter16,    //!< 16 Not recommended
    Filter32,    //!< 32 Not recommended
    Filter64,    //!< 64 Not recommended
    Filter128,   //!< 128
    Filter256,   //!< 256
    Filter512,   //!< 512
    Filter1024,  //!< 1024

};

/*!
  @enum Gain
  @brief Amplifier gain
 */
enum class Gain : uint8_t {
    Coeff1,     //!< Bypassed
    Coeff3,     //!< 3
    Coeff6,     //!< 6
    Coeff12_5,  //!< 12.5
    Coeff25,    //!< 25
    Coeff50,    //!< 50
    Coeff100,   //!< 100
    // [7] Coeff100 (duplicate)
};

/*!
  @enum IRSensor
  @brief Infra-Red Sensor mode
 */
enum class IRSensor : uint8_t {
    Single,  //!< Single IR Sensor
    Dual     //!< Dual IR Sensor
};

/*!
  @struct Data
  @brief Measurement data group
 */
struct Data {
    std::array<uint16_t, 3> raw{};  //!< Linearized raw [0]:Ambient [1]:Object1 [2]:Object2

    //! @brief Gets ambient temperature in Kelvin
    inline float ambientKelvin() const
    {
        return ((raw[0] & 0x8000) == 0) ? raw[0] * 0.02f : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Gets ambient temperature in Celsius (alias of ambientCelsius)
    inline float ambientTemperature() const
    {
        return ambientCelsius();
    }
    //! @brief Gets ambient temperature in Celsius
    inline float ambientCelsius() const
    {
        return ambientKelvin() - 273.15f;
    }
    //! @brief Gets ambient temperature in Fahrenheit
    inline float ambientFahrenheit() const
    {
        return ambientCelsius() * 9.0f / 5.0f + 32.f;
    }

    //! @brief Gets object1 temperature in Kelvin
    inline float objectKelvin1() const
    {
        return ((raw[1] & 0x8000) == 0) ? raw[1] * 0.02f : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Gets object1 temperature in Celsius (alias of objectCelsius1)
    inline float objectTemperature1() const
    {
        return objectCelsius1();
    }
    //! @brief Gets object1 temperature in Celsius
    inline float objectCelsius1() const
    {
        return objectKelvin1() - 273.15f;
    }
    //! @brief Gets object1 temperature in Fahrenheit
    inline float objectFahrenheit1() const
    {
        return objectCelsius1() * 9.0f / 5.0f + 32.f;
    }

    //! @brief Gets object2 temperature in Kelvin (dual zone only, NaN on single zone)
    inline float objectKelvin2() const
    {
        return ((raw[2] & 0x8000) == 0) ? raw[2] * 0.02f : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Gets object2 temperature in Celsius (alias of objectCelsius2)
    inline float objectTemperature2() const
    {
        return objectCelsius2();
    }
    //! @brief Gets object2 temperature in Celsius (dual zone only, NaN on single zone)
    inline float objectCelsius2() const
    {
        return objectKelvin2() - 273.15f;
    }
    //! @brief Gets object2 temperature in Fahrenheit (dual zone only, NaN on single zone)
    inline float objectFahrenheit2() const
    {
        return objectCelsius2() * 9.0f / 5.0f + 32.f;
    }
};

/*!
  @struct EEPROM
  @brief EEPROM structure values
 */
struct EEPROM {
    uint16_t toMax{}, toMin{},  //!< Max,Min of the Object Temperature
        pwmCtrl{},              //!< Pulse Width Modulation control
        taRange{},              //!< Range of the Ambient Temperature (H/L)
        emissivity{},           //!< Emissivity
        config{},               //!< Configuration
        addr{},                 //!< I2C address(Using low byte)
        id[4]{};                //!< Unique ID
};

/*!
  @struct ConfigRegister
  @brief Config Register1 (EEPROM 0x05, command 0x25) bit fields
 */
struct ConfigRegister {
    //! @brief Constructor
    //! @param v Raw register value
    inline explicit ConfigRegister(const uint16_t v = 0) : value{v}
    {
    }
    //! @brief Gets the IIR filter (bits 2:0)
    //! @return IIR
    inline IIR iir() const
    {
        return static_cast<IIR>(value & 0x07);
    }
    //! @brief Gets the PWM output mode (bits 5:4)
    //! @return Output
    inline Output output() const
    {
        return static_cast<Output>((value >> 4) & 0x03);
    }
    //! @brief Gets the FIR filter (bits 10:8)
    //! @return FIR
    inline FIR fir() const
    {
        return static_cast<FIR>((value >> 8) & 0x07);
    }
    //! @brief Gets the amplifier gain (bits 13:11)
    //! @return Gain (field value 7 is returned as Gain::Coeff100, its duplicate)
    inline Gain gain() const
    {
        const uint8_t g = (value >> 11) & 0x07;
        return static_cast<Gain>(g > 6 ? 6 : g);
    }
    //! @brief Gets the IR sensor mode (bit 6)
    //! @return IRSensor
    inline IRSensor irSensor() const
    {
        return static_cast<IRSensor>((value >> 6) & 0x01);
    }
    //! @brief Gets the sign of Ks (bit 7)
    //! @return True if positive (bit 7 is 0)
    inline bool positiveKs() const
    {
        return !(value & (1U << 7));
    }
    //! @brief Gets the sign of Kt2 (bit 14)
    //! @return True if positive (bit 14 is 0)
    inline bool positiveKt2() const
    {
        return !(value & (1U << 14));
    }

    //! @brief Sets the IIR filter (bits 2:0)
    //! @param iir IIR
    inline void iir(const IIR iir)
    {
        value = static_cast<uint16_t>((value & ~0x07) | m5::stl::to_underlying(iir));
    }
    //! @brief Sets the PWM output mode (bits 5:4)
    //! @param o Output
    inline void output(const Output o)
    {
        value = static_cast<uint16_t>((value & ~(0x03 << 4)) | (m5::stl::to_underlying(o) << 4));
    }
    //! @brief Sets the FIR filter (bits 10:8)
    //! @param fir FIR
    inline void fir(const FIR fir)
    {
        value = static_cast<uint16_t>((value & ~(0x07 << 8)) | (m5::stl::to_underlying(fir) << 8));
    }
    //! @brief Sets the amplifier gain (bits 13:11)
    //! @param gain Gain
    inline void gain(const Gain gain)
    {
        value = static_cast<uint16_t>((value & ~(0x07 << 11)) | (m5::stl::to_underlying(gain) << 11));
    }
    //! @brief Sets the IR sensor mode (bit 6)
    //! @param irs IRSensor
    inline void irSensor(const IRSensor irs)
    {
        value = static_cast<uint16_t>((value & ~(1U << 6)) | (static_cast<unsigned>(m5::stl::to_underlying(irs)) << 6));
    }
    //! @brief Sets the sign of Ks (bit 7)
    //! @param pos Positive (bit 7 = 0) if true
    inline void positiveKs(const bool pos)
    {
        value = static_cast<uint16_t>((value & ~(1U << 7)) | (static_cast<unsigned>(!pos) << 7));
    }
    //! @brief Sets the sign of Kt2 (bit 14)
    //! @param pos Positive (bit 14 = 0) if true
    inline void positiveKt2(const bool pos)
    {
        value = static_cast<uint16_t>((value & ~(1U << 14)) | (static_cast<unsigned>(!pos) << 14));
    }
    uint16_t value{};  //!< Raw register value
};

/*!
  @brief Object temperature range raw (ToMin / ToMax) to Celsius
  @param t Raw value (0.01 K/LSB)
  @return Celsius
 */
inline float toRaw_to_celsius(const uint16_t t)
{
    return t * 0.01f - 273.15f;
}

/*!
  @brief Celsius to object temperature range raw (ToMin / ToMax)
  @param c Celsius (clamped to -273.15 - 382.2)
  @return Raw value (0.01 K/LSB)
 */
inline uint16_t celsius_to_toRaw(const float c)
{
    const float v{std::fmax(std::fmin(c, 382.2f), -273.15f)};
    return static_cast<uint16_t>(100 * (v + 0.005f + 273.15f));
}

/*!
  @brief Ambient temperature range raw (one byte of TaRange) to Celsius
  @param t Raw value (0.64 °C/LSB, offset -38.2 °C)
  @return Celsius
 */
inline float taRaw_to_celsius(const uint8_t t)
{
    return t * 64.0f / 100.f - 38.2f;
}

/*!
  @brief Celsius to ambient temperature range raw (one byte of TaRange)
  @param c Celsius (clamped to -38.2 - 125.0)
  @return Raw value (0.64 °C/LSB, offset -38.2 °C)
 */
inline uint8_t celsius_to_taRaw(const float c)
{
    const float v{std::fmax(std::fmin(c, 125.f), -38.2f)};
    return static_cast<uint8_t>(100 * (v + 0.32f + 38.2f) / 64.0f);
}

/*!
  @brief Emissivity raw to emissivity
  @param e Raw value
  @return Emissivity (0.0 - 1.0)
 */
inline float raw_to_emissivity(const uint16_t e)
{
    return e / 65535.f;
}

/*!
  @brief Emissivity to emissivity raw
  @param e Emissivity (0.0 - 1.0, not clamped; the caller must validate it)
  @return Raw value
 */
inline uint16_t emissivity_to_raw(const float e)
{
    return static_cast<uint16_t>(std::round(65535.f * e));
}

}  // namespace mlx90614

}  // namespace unit
}  // namespace m5
#endif
