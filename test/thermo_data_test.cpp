/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Tests for the data types: conversions and register fields (no hardware).
  Placed directly under test/, so they run on the host (test_native) and on the boards (test_<Unit>_<Board>)
*/
#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include <unit/unit_MLX90614_data.hpp>
#include <unit/unit_NCIR2_data.hpp>
#include <unit/unit_Thermal2_data.hpp>

using namespace m5::unit;

TEST(MLX90614Data, Conversion)
{
    using namespace m5::unit::mlx90614;

    EXPECT_FLOAT_EQ(toRaw_to_celsius(0), -273.15f);
    EXPECT_FLOAT_EQ(toRaw_to_celsius(0xFFFF), 382.2f);

    EXPECT_FLOAT_EQ(taRaw_to_celsius(0), -38.2f);
    EXPECT_FLOAT_EQ(taRaw_to_celsius(0xFF), 125.f);

    for (uint32_t i = 0; i < 65536; ++i) {
        const float c{toRaw_to_celsius(i)};
        const uint16_t to{celsius_to_toRaw(c)};
        EXPECT_EQ(to, i);
    }

    for (uint16_t i = 0; i < 256; ++i) {
        const float c{taRaw_to_celsius(i)};
        const uint8_t ta{celsius_to_taRaw(c)};
        EXPECT_EQ(ta, i);
    }

    // Out of range values are clamped
    EXPECT_EQ(celsius_to_toRaw(-300.f), 0);
    EXPECT_EQ(celsius_to_toRaw(400.f), 65535);
    EXPECT_EQ(celsius_to_taRaw(-50.f), 0);
    EXPECT_EQ(celsius_to_taRaw(130.f), 0xFF);

    // Random round trip
    // The seed is fixed, but the values still differ between standard libraries (the distribution is
    // implementation-defined), so the host and the boards may check different values. Pass/fail does not depend on them
    std::mt19937 rng{12345};
    std::uniform_real_distribution<float> dist_to{-273.15f, 382.2f};
    std::uniform_real_distribution<float> dist_ta{-38.2f, 125.f};
    uint32_t cnt{32};
    while (cnt--) {
        const float co{dist_to(rng)};
        const float ca{dist_ta(rng)};
        SCOPED_TRACE(::testing::Message() << "co:" << co << " ca:" << ca);

        const uint16_t to{celsius_to_toRaw(co)};
        const uint8_t ta{celsius_to_taRaw(ca)};
        const float tof{toRaw_to_celsius(to)};
        const float taf{taRaw_to_celsius(ta)};

        EXPECT_NEAR(tof, co, 0.01f);  // 1 LSB = 0.01°C resolution
        EXPECT_NEAR(taf, ca, 0.32f);
        EXPECT_EQ(celsius_to_toRaw(tof), to);
        EXPECT_EQ(celsius_to_taRaw(taf), ta);
    }
}

TEST(MLX90614Data, Emissivity)
{
    using namespace m5::unit::mlx90614;

    EXPECT_EQ(emissivity_to_raw(1.0f), 65535);
    EXPECT_EQ(emissivity_to_raw(0.5f), 32768);
    EXPECT_EQ(emissivity_to_raw(0.1f), 6554);
    EXPECT_FLOAT_EQ(raw_to_emissivity(65535), 1.0f);
    EXPECT_NEAR(raw_to_emissivity(6554), 0.1f, 0.00001f);
}

TEST(MLX90614Data, ConfigRegister)
{
    using namespace m5::unit::mlx90614;

    // 1001 1111 1011 0100
    const ConfigRegister c{0x9FB4};
    EXPECT_EQ(c.iir(), IIR::Filter100);
    EXPECT_EQ(c.output(), Output::TO1_TO2);
    EXPECT_EQ(c.irSensor(), IRSensor::Single);
    EXPECT_FALSE(c.positiveKs());  // bit7 = 1
    EXPECT_EQ(c.fir(), FIR::Filter1024);
    EXPECT_EQ(c.gain(), Gain::Coeff12_5);
    EXPECT_TRUE(c.positiveKf2());  // bit14 = 0

    // Gain [7] is a duplicate of Coeff100
    EXPECT_EQ(ConfigRegister{7U << 11}.gain(), Gain::Coeff100);

    // Each setter changes only its own bits
    ConfigRegister w{0x9FB4};
    w.iir(IIR::Filter57);
    EXPECT_EQ(w.value, 0x9FB7);
    w.output(Output::TA_TO1);
    EXPECT_EQ(w.value, 0x9F87);
    w.irSensor(IRSensor::Dual);
    EXPECT_EQ(w.value, 0x9FC7);
    w.positiveKs(true);
    EXPECT_EQ(w.value, 0x9F47);
    w.fir(FIR::Filter8);
    EXPECT_EQ(w.value, 0x9847);
    w.gain(Gain::Coeff1);
    EXPECT_EQ(w.value, 0x8047);
    w.positiveKf2(false);
    EXPECT_EQ(w.value, 0xC047);

    EXPECT_EQ(w.iir(), IIR::Filter57);
    EXPECT_EQ(w.output(), Output::TA_TO1);
    EXPECT_EQ(w.irSensor(), IRSensor::Dual);
    EXPECT_TRUE(w.positiveKs());
    EXPECT_EQ(w.fir(), FIR::Filter8);
    EXPECT_EQ(w.gain(), Gain::Coeff1);
    EXPECT_FALSE(w.positiveKf2());
}

TEST(MLX90614Data, Data)
{
    mlx90614::Data d{};
    d.raw[0] = 15095;   // 301.90 K
    d.raw[1] = 14658;   // 293.16 K
    d.raw[2] = 0x8000;  // Error flag

    EXPECT_NEAR(d.ambientKelvin(), 301.9f, 0.001f);
    EXPECT_NEAR(d.ambientCelsius(), 28.75f, 0.001f);
    EXPECT_NEAR(d.ambientFahrenheit(), 83.75f, 0.001f);
    EXPECT_FLOAT_EQ(d.ambientTemperature(), d.ambientCelsius());

    EXPECT_NEAR(d.objectKelvin1(), 293.16f, 0.001f);
    EXPECT_NEAR(d.objectCelsius1(), 20.01f, 0.001f);
    EXPECT_NEAR(d.objectFahrenheit1(), 68.018f, 0.001f);
    EXPECT_FLOAT_EQ(d.objectTemperature1(), d.objectCelsius1());

    EXPECT_TRUE(std::isnan(d.objectKelvin2()));
    EXPECT_TRUE(std::isnan(d.objectCelsius2()));
    EXPECT_TRUE(std::isnan(d.objectFahrenheit2()));

    d.raw[0] = 0x8000;
    d.raw[1] = 0xFFFF;
    EXPECT_TRUE(std::isnan(d.ambientKelvin()));
    EXPECT_TRUE(std::isnan(d.ambientCelsius()));
    EXPECT_TRUE(std::isnan(d.objectKelvin1()));
    EXPECT_TRUE(std::isnan(d.objectCelsius1()));
}

TEST(NCIR2Data, Data)
{
    ncir2::Data d{};
    // Default is 0x8000 (the minimum of int16_t)
    EXPECT_EQ(d.value(), -32768);
    EXPECT_FLOAT_EQ(d.temperature(), -327.68f);

    d.raw = {0xC3, 0x0A};  // 2755
    EXPECT_EQ(d.value(), 2755);
    EXPECT_FLOAT_EQ(d.temperature(), 27.55f);
    EXPECT_FLOAT_EQ(d.celsius(), 27.55f);
    EXPECT_NEAR(d.fahrenheit(), 81.59f, 0.001f);

    d.raw = {0x18, 0xFC};  // -1000
    EXPECT_EQ(d.value(), -1000);
    EXPECT_FLOAT_EQ(d.temperature(), -10.0f);
}

TEST(Thermal2Data, Conversion)
{
    using namespace m5::unit::thermal2;

    constexpr struct {
        uint16_t utemp;
        float ftemp;
        bool near;
    } temp_table[] = {
        {0, -64.f, false}, {8192, 0.f, false}, {12032, 30.f, false}, {20992, 100.f, false}, {65535, 447.99f, true},
    };

    for (auto&& t : temp_table) {
        if (t.near) {
            EXPECT_NEAR(raw_to_celsius(t.utemp), t.ftemp, 0.01f);
        } else {
            EXPECT_FLOAT_EQ(raw_to_celsius(t.utemp), t.ftemp);
        }
        EXPECT_EQ(celsius_to_raw(t.ftemp), t.utemp);
    }

    // Out of range values are clamped
    EXPECT_EQ(celsius_to_raw(-100.f), 0);
    EXPECT_EQ(celsius_to_raw(500.f), 65535);
}

TEST(Thermal2Data, Data)
{
    using namespace m5::unit::thermal2;

    Data d{};
    for (uint_fast8_t i = 0; i < 8; ++i) {
        d.temp[i] = static_cast<uint16_t>(8192 + i * 128);  // 0 + i °C
    }
    d.raw[0]   = 12032;  // 30 °C
    d.raw[383] = 20992;  // 100 °C

    EXPECT_FLOAT_EQ(d.medianTemperature(), 0.f);
    EXPECT_FLOAT_EQ(d.averageTemperature(), 1.f);
    EXPECT_FLOAT_EQ(d.mostDiffTemperature(), 2.f);
    EXPECT_FLOAT_EQ(d.lowestTemperature(), 4.f);
    EXPECT_FLOAT_EQ(d.highestTemperature(), 6.f);
    EXPECT_EQ(d.median_temperature, d.temp[0]);
    EXPECT_EQ(d.highest_temperature, d.temp[6]);

    EXPECT_FLOAT_EQ(d.temperature(0), 30.f);
    EXPECT_FLOAT_EQ(d.temperature(383), 100.f);
    EXPECT_TRUE(std::isnan(d.temperature(384)));
}
