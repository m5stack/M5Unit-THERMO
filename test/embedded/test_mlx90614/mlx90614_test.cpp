/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMLX90614BAA
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_MLX90614.hpp>
#include <m5_unit_component/adapter_i2c.hpp>
#include <cmath>
#include <esp_random.h>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::mlx90614;
using m5::unit::types::elapsed_time_t;

constexpr uint32_t STORED_SIZE{4};

class TestMLX90614BAA : public I2CComponentTestBase<UnitMLX90614BAA> {
protected:
    virtual UnitMLX90614BAA* get_instance() override
    {
        auto ptr         = new m5::unit::UnitMLX90614BAA();
        auto ccfg        = ptr->component_config();
        ccfg.stored_size = STORED_SIZE;
        ptr->component_config(ccfg);
        return ptr;
    }

    // Restore EEPROM values only (no applySettings).
    // Next test's begin() → wakeup() triggers POR which loads EEPROM→RAM.
    void restore_config()
    {
        uint16_t c{0x9FB4};  // 1001 1111 1011 0100 IIR:4, OUT:TO12 FIR:7,Gain:3,IRS:0 PosK:1 PosKf2:0
        EXPECT_TRUE(unit->writeConfig(c, false));
    }
    void restore_setting()
    {
        EXPECT_TRUE(unit->writeObjectMinMax(25315, 39315, false));
        EXPECT_TRUE(unit->writeAmbientMinMax(0x1C, 0xF7, false));
        EXPECT_TRUE(unit->writeEmissivity(0xFFFF, false));
    }
};

namespace {
constexpr Output out_table[] = {Output::TA_TO1, Output::TA_TO2, Output::TO2_Undefined, Output::TO1_TO2};
constexpr IIR iir_table[]    = {
    IIR::Filter50,  IIR::Filter25, IIR::Filter17, IIR::Filter13,
    IIR::Filter100, IIR::Filter80, IIR::Filter67, IIR::Filter57,
};
constexpr FIR fir_table[] = {
    FIR::Filter8,   FIR::Filter16,  FIR::Filter32,  FIR::Filter64,
    FIR::Filter128, FIR::Filter256, FIR::Filter512, FIR::Filter1024,
};
constexpr Gain gain_table[] = {
    Gain::Coeff1, Gain::Coeff3, Gain::Coeff6, Gain::Coeff12_5, Gain::Coeff25, Gain::Coeff50, Gain::Coeff100,
};
constexpr IRSensor irs_table[] = {IRSensor::Single, IRSensor::Dual};
constexpr bool pos_table[]     = {true, false};

constexpr uint32_t interval_tableBD[8][4] = {
    {470, 600, 840, 1330}, {1100, 1400, 2000, 3200}, {1800, 2200, 3200, 5000}, {2400, 3000, 4300, 7000},
    {60, 70, 100, 140},    {200, 240, 340, 540},     {380, 480, 670, 1100},    {420, 530, 750, 1200},
};

uint32_t get_interval(const IIR iir, const FIR fir)
{
    auto i = m5::stl::to_underlying(iir);
    auto f = m5::stl::to_underlying(fir);
    return (f < 4) ? 0 : interval_tableBD[i][f - 4];
}

inline float random_float(float min_val, float max_val)
{
    return min_val + (max_val - min_val) * (esp_random() / static_cast<float>(UINT32_MAX));
}

// From UnitMLX90614.cpp
inline float toRaw_to_celsius(const uint16_t t)
{
    return t * 0.01f - 273.15f;
}

inline uint16_t celsius_to_toRaw(const float c)
{
    float v = std::fmax(std::fmin(c, 382.2f), -273.15f);
    return 100 * (v + 0.005f + 273.15f);
}

inline float taRaw_to_celsius(const uint8_t t)
{
    return t * 64.0f / 100.f - 38.2f;
}

inline uint8_t celsius_to_taRaw(const float c)
{
    float v = std::fmax(std::fmin(c, 125.f), -38.2f);
    return 100 * (v + 0.32f + 38.2f) / 64.0f;
}

inline float raw_to_emissivity(const uint16_t e)
{
    return e / 65535.f;
}

inline uint16_t emissivity_to_raw(const float e)
{
    return std::round(65535.f * e);
}

}  // namespace

TEST_F(TestMLX90614BAA, Conversion)
{
    SCOPED_TRACE(ustr);

    EXPECT_FLOAT_EQ(toRaw_to_celsius(0), -273.15f);
    EXPECT_FLOAT_EQ(toRaw_to_celsius(0xFFFF), 382.2f);

    EXPECT_FLOAT_EQ(taRaw_to_celsius(0), -38.2f);
    EXPECT_FLOAT_EQ(taRaw_to_celsius(0xFF), 125.f);

    for (uint32_t i = 0; i < 65536; ++i) {
        float c     = toRaw_to_celsius(i);
        uint16_t to = celsius_to_toRaw(c);
        EXPECT_EQ(to, i);
    }

    for (uint16_t i = 0; i < 256; ++i) {
        float c    = taRaw_to_celsius(i);
        uint8_t ta = celsius_to_taRaw(c);
        EXPECT_EQ(ta, i);
    }

    // random
    uint32_t cnt{32};
    while (cnt--) {
        float co = random_float(-273.15f, 382.2f);
        float ca = random_float(-38.2f, 125.f);
        auto s   = m5::utility::formatString("co:%f ca:%f", co, ca);
        SCOPED_TRACE(s);

        uint16_t to  = celsius_to_toRaw(co);
        uint8_t ta   = celsius_to_taRaw(ca);
        float tof    = toRaw_to_celsius(to);
        float taf    = taRaw_to_celsius(ta);
        uint16_t to2 = celsius_to_toRaw(tof);
        uint8_t ta2  = celsius_to_taRaw(taf);

        EXPECT_NEAR(tof, co, 0.01f);  // 1 LSB = 0.01°C resolution
        EXPECT_NEAR(taf, ca, 0.32f);
        EXPECT_EQ(to2, to);
        EXPECT_EQ(ta2, ta);
    }
}

TEST_F(TestMLX90614BAA, Emissivity)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->writeEmissivity(0.1f));
    EXPECT_FALSE(unit->writeEmissivity(32768));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    uint16_t raw{};
    float e{};
    constexpr float near{0.00001f};

    // EEPROM write/read roundtrip only (apply=false to avoid sleep/wakeup per iteration).
    // apply=true (EEPROM→RAM reflection via POR) is covered by the Periodic test.

    EXPECT_TRUE(unit->writeEmissivity(0.1f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_TRUE(unit->readEmissivity(raw));
    EXPECT_NEAR(e, 0.1f, near);
    EXPECT_EQ(raw, 6554);

    EXPECT_TRUE(unit->writeEmissivity(0.5f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_TRUE(unit->readEmissivity(raw));
    EXPECT_NEAR(e, 0.5f, near);
    EXPECT_EQ(raw, 32768);

    EXPECT_TRUE(unit->writeEmissivity(1.0f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_TRUE(unit->readEmissivity(raw));
    EXPECT_NEAR(e, 1.0f, near);
    EXPECT_EQ(raw, 65535);

    EXPECT_FALSE(unit->writeEmissivity(0.09f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_NEAR(e, 1.0f, near);

    EXPECT_FALSE(unit->writeEmissivity(-1.f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_NEAR(e, 1.0f, near);

    EXPECT_FALSE(unit->writeEmissivity(1.001f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_NEAR(e, 1.0f, near);

    // Restore default emissivity
    EXPECT_TRUE(unit->writeEmissivity(0.95f, false));
    EXPECT_TRUE(unit->readEmissivity(e));
    EXPECT_TRUE(unit->readEmissivity(raw));
    EXPECT_NEAR(e, 0.95f, near);
    EXPECT_EQ(raw, 62258);
}

TEST_F(TestMLX90614BAA, Config)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());

    EXPECT_FALSE(unit->writeOutput(Output::TA_TO2));
    EXPECT_FALSE(unit->writeIIR(IIR::Filter100));
    EXPECT_FALSE(unit->writeFIR(FIR::Filter512));
    EXPECT_FALSE(unit->writeGain(Gain::Coeff1));
    EXPECT_FALSE(unit->writeIRSensor(IRSensor::Single));
    EXPECT_FALSE(unit->writePositiveKs(false));
    EXPECT_FALSE(unit->writePositiveKf2(false));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    // EEPROM write/read roundtrip only (apply=false to avoid 33 sleep/wakeup cycles).
    // apply=true (EEPROM→RAM reflection via POR) is covered by the Periodic test,
    // which calls startPeriodicMeasurement() with various IIR/FIR values.

    /// Output
    for (auto&& o : out_table) {
        EXPECT_TRUE(unit->writeOutput(o, false));
        Output v{};
        EXPECT_TRUE(unit->readOutput(v));
        EXPECT_EQ(v, o);
    }

    // IIR
    for (auto&& iir : iir_table) {
        EXPECT_TRUE(unit->writeIIR(iir, false));
        IIR v{};
        EXPECT_TRUE(unit->readIIR(v));
        EXPECT_EQ(v, iir);
    }

    // FIR
    for (auto&& fir : fir_table) {
        EXPECT_TRUE(unit->writeFIR(fir, false));
        FIR v{};
        EXPECT_TRUE(unit->readFIR(v));
        EXPECT_EQ(v, fir);
    }

    // Gain
    for (auto&& gain : gain_table) {
        EXPECT_TRUE(unit->writeGain(gain, false));
        Gain v{};
        EXPECT_TRUE(unit->readGain(v));
        EXPECT_EQ(v, gain);
    }

    // IRSensor
    for (auto&& irs : irs_table) {
        EXPECT_TRUE(unit->writeIRSensor(irs, false));
        IRSensor v{};
        EXPECT_TRUE(unit->readIRSensor(v));
        EXPECT_EQ(v, irs);
    }

    // PosK
    for (auto&& pos : pos_table) {
        EXPECT_TRUE(unit->writePositiveKs(pos, false));
        bool v{};
        EXPECT_TRUE(unit->readPositiveKs(v));
        EXPECT_EQ(v, pos);
    }

    // PosKf2
    for (auto&& pos : pos_table) {
        EXPECT_TRUE(unit->writePositiveKf2(pos, false));
        bool v{};
        EXPECT_TRUE(unit->readPositiveKf2(v));
        EXPECT_EQ(v, pos);
    }

    // Restore EEPROM only; next test's begin() → wakeup() applies via POR
    restore_config();
}

TEST_F(TestMLX90614BAA, SettingObjectTemperatureMinMax)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());

    EXPECT_FALSE(unit->writeObjectMinMax(-273.15f, -273.15f));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    float tminF{}, tmaxF{};
    uint16_t tmin{}, tmax{};

    // EEPROM write/read roundtrip only (apply=false to avoid sleep/wakeup per iteration).
    // apply=true (EEPROM→RAM reflection via POR) is covered by the Periodic test.

    // min
    EXPECT_TRUE(unit->writeObjectMinMax(-273.15f, -273.15f, false));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -273.15f);
    EXPECT_FLOAT_EQ(tmaxF, -273.15f);

    // under
    EXPECT_TRUE(unit->writeObjectMinMax(-1273.15f, -1273.15f, false));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -273.15f);
    EXPECT_FLOAT_EQ(tmaxF, -273.15f);

    // max
    EXPECT_TRUE(unit->writeObjectMinMax(382.2f, 382.2f, false));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFFFF);
    EXPECT_EQ(tmax, 0xFFFF);
    EXPECT_FLOAT_EQ(tminF, 382.2f);
    EXPECT_FLOAT_EQ(tmaxF, 382.2f);

    // over
    EXPECT_TRUE(unit->writeObjectMinMax(1382.2f, 1382.2f, false));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFFFF);
    EXPECT_EQ(tmax, 0xFFFF);
    EXPECT_FLOAT_EQ(tminF, 382.2f);
    EXPECT_FLOAT_EQ(tmaxF, 382.2f);

    // random
    uint32_t cnt{32};
    while (cnt--) {
        float toMin = random_float(-273.15f, 382.2f);
        float toMax = random_float(-273.15f, 382.2f);
        if (toMin > toMax) {
            std::swap(toMin, toMax);
        }
        auto s = m5::utility::formatString("%f/%f", toMin, toMax);
        SCOPED_TRACE(s);

        EXPECT_TRUE(unit->writeObjectMinMax(toMin, toMax, false));

        EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
        EXPECT_NEAR(tminF, toMin, 0.01f);  // 1 LSB = 0.01°C resolution
        EXPECT_NEAR(tmaxF, toMax, 0.01f);  // 1 LSB = 0.01°C resolution
    }

    // Restore EEPROM only; next test's begin() → wakeup() applies via POR
    restore_setting();
}

TEST_F(TestMLX90614BAA, SettingAmbientTemperatureMinMax)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());

    EXPECT_FALSE(unit->writeAmbientMinMax(-38.2f, 125.f));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    float tminF{}, tmaxF{};
    uint8_t tmin{}, tmax{};

    // EEPROM write/read roundtrip only (apply=false to avoid sleep/wakeup cycles).
    // apply=true (EEPROM→RAM reflection via POR) is covered by the Periodic test.

    // min
    EXPECT_TRUE(unit->writeAmbientMinMax(-38.2f, -38.2f, false));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -38.2f);
    EXPECT_FLOAT_EQ(tmaxF, -38.2f);

    // under
    EXPECT_TRUE(unit->writeAmbientMinMax(-1273.15f, -1273.15f, false));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -38.2f);
    EXPECT_FLOAT_EQ(tmaxF, -38.2f);

    // max
    EXPECT_TRUE(unit->writeAmbientMinMax(125.f, 125.f, false));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFF);
    EXPECT_EQ(tmax, 0xFF);
    EXPECT_FLOAT_EQ(tminF, 125.f);
    EXPECT_FLOAT_EQ(tmaxF, 125.f);

    // over
    EXPECT_TRUE(unit->writeAmbientMinMax(1382.2f, 1382.2f, false));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFF);
    EXPECT_EQ(tmax, 0xFF);
    EXPECT_FLOAT_EQ(tminF, 125.f);
    EXPECT_FLOAT_EQ(tmaxF, 125.f);

    // random
    uint32_t cnt{32};
    while (cnt--) {
        float toMin = random_float(-38.2f, 125.f);
        float toMax = random_float(-38.2f, 125.f);
        if (toMin > toMax) {
            std::swap(toMin, toMax);
        }
        auto s = m5::utility::formatString("%f/%f", toMin, toMax);
        SCOPED_TRACE(s);

        EXPECT_TRUE(unit->writeAmbientMinMax(toMin, toMax, false));

        EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
        EXPECT_NEAR(tminF, toMin, 0.32f);
        EXPECT_NEAR(tmaxF, toMax, 0.32f);
        // M5_LOGI("%f %f", tminF, toMin);
        // M5_LOGI("%f %f", tmaxF, toMax);
    }

    // Restore EEPROM only; next test's begin() → wakeup() applies via POR
    restore_setting();
}

TEST_F(TestMLX90614BAA, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    auto ad     = unit->asAdapter<m5::unit::AdapterI2C>(m5::unit::Adapter::Type::I2C);
    bool is_bus = ad && ad->implType() == m5::unit::AdapterI2C::ImplType::Bus;

    // Orthogonal test: all IIR with fastest valid FIR + all FIR with fastest IIR.
    // Covers each IIR and FIR value at least once (11 combinations instead of 32).
    // Also serves as the apply=true test: startPeriodicMeasurement() writes IIR/FIR
    // to EEPROM with apply=true (POR), then verifies the device measures correctly.
    struct IIR_FIR {
        IIR iir;
        FIR fir;
    };
    constexpr IIR_FIR test_pairs[] = {
        // All IIR values with FIR=128 (fastest valid FIR)
        {IIR::Filter50, FIR::Filter128},
        {IIR::Filter25, FIR::Filter128},
        {IIR::Filter17, FIR::Filter128},
        {IIR::Filter13, FIR::Filter128},
        {IIR::Filter100, FIR::Filter128},
        {IIR::Filter80, FIR::Filter128},
        {IIR::Filter67, FIR::Filter128},
        {IIR::Filter57, FIR::Filter128},
        // Remaining FIR values with IIR=100% (fastest IIR)
        {IIR::Filter100, FIR::Filter256},
        {IIR::Filter100, FIR::Filter512},
        {IIR::Filter100, FIR::Filter1024},
    };

    for (auto&& pair : test_pairs) {
        auto s = m5::utility::formatString("IIR:%u FIR:%u", pair.iir, pair.fir);
        SCOPED_TRACE(s);
        const auto iir = pair.iir;
        const auto fir = pair.fir;

        EXPECT_TRUE(unit->startPeriodicMeasurement(iir, fir, Gain::Coeff12_5, IRSensor::Dual));
        EXPECT_TRUE(unit->inPeriodic());

        auto tm        = get_interval(iir, fir);
        uint32_t cycle = std::max<uint32_t>(tm ? tm : 1, unit->interval());
        uint32_t timeout =
            is_bus ? std::max<uint32_t>(cycle, 500) * (STORED_SIZE + 1) * 4 : cycle * (STORED_SIZE + 1) * 2;
        auto r = collect_periodic_measurements(unit.get(), STORED_SIZE, timeout);

        EXPECT_TRUE(unit->stopPeriodicMeasurement());
        EXPECT_FALSE(unit->inPeriodic());

        EXPECT_FALSE(r.timed_out);
        EXPECT_EQ(r.update_count, STORED_SIZE);
        uint32_t tolerance = is_bus ? 5 : 1;
        EXPECT_LE(r.median(), static_cast<uint32_t>(cycle + tolerance));

        M5_LOGI("TM:%u IT:%u med:%u", tm, unit->interval(), r.median());

        //
        EXPECT_EQ(unit->available(), STORED_SIZE);
        EXPECT_FALSE(unit->empty());
        EXPECT_TRUE(unit->full());

        uint32_t cnt{STORED_SIZE / 2};
        while (cnt-- && unit->available()) {
            EXPECT_TRUE(std::isfinite(unit->ambientTemperature()));
            EXPECT_FLOAT_EQ(unit->ambientTemperature(), unit->oldest().ambientTemperature());
            EXPECT_TRUE(std::isfinite(unit->objectTemperature1()));
            EXPECT_FLOAT_EQ(unit->objectTemperature1(), unit->oldest().objectTemperature1());
            EXPECT_TRUE(std::isfinite(unit->objectTemperature2()));
            EXPECT_FLOAT_EQ(unit->objectTemperature2(), unit->oldest().objectTemperature2());

            EXPECT_FLOAT_EQ(unit->ambientCelsius(), unit->ambientTemperature());
            EXPECT_FLOAT_EQ(unit->objectCelsius1(), unit->objectTemperature1());
            EXPECT_FLOAT_EQ(unit->objectCelsius2(), unit->objectTemperature2());
            EXPECT_TRUE(std::isfinite(unit->ambientKelvin()));
            EXPECT_TRUE(std::isfinite(unit->ambientFahrenheit()));
            EXPECT_TRUE(std::isfinite(unit->objectKelvin1()));
            EXPECT_TRUE(std::isfinite(unit->objectFahrenheit1()));
            EXPECT_TRUE(std::isfinite(unit->objectKelvin2()));
            EXPECT_TRUE(std::isfinite(unit->objectFahrenheit2()));

            EXPECT_FALSE(unit->empty());
            unit->discard();
        }
        EXPECT_EQ(unit->available(), STORED_SIZE / 2);
        EXPECT_FALSE(unit->empty());
        EXPECT_FALSE(unit->full());

        unit->flush();
        EXPECT_EQ(unit->available(), 0);
        EXPECT_TRUE(unit->empty());
        EXPECT_FALSE(unit->full());

        EXPECT_FALSE(std::isfinite(unit->ambientTemperature()));
        EXPECT_FALSE(std::isfinite(unit->objectTemperature1()));
        EXPECT_FALSE(std::isfinite(unit->objectTemperature2()));

        EXPECT_FALSE(std::isfinite(unit->ambientCelsius()));
        EXPECT_FALSE(std::isfinite(unit->ambientKelvin()));
        EXPECT_FALSE(std::isfinite(unit->ambientFahrenheit()));
        EXPECT_FALSE(std::isfinite(unit->objectCelsius1()));
        EXPECT_FALSE(std::isfinite(unit->objectKelvin1()));
        EXPECT_FALSE(std::isfinite(unit->objectFahrenheit1()));
        EXPECT_FALSE(std::isfinite(unit->objectCelsius2()));
        EXPECT_FALSE(std::isfinite(unit->objectKelvin2()));
        EXPECT_FALSE(std::isfinite(unit->objectFahrenheit2()));
    }

    // Restore EEPROM only; next test's begin() → wakeup() applies via POR
    restore_setting();
    restore_config();
}

TEST_F(TestMLX90614BAA, BeginAppliesConfig)
{
    SCOPED_TRACE(ustr);

    // Verify that begin() started periodic measurement with default config
    EXPECT_TRUE(unit->inPeriodic());

    // Read back config register values that begin() should have applied
    mlx90614::IIR iir{};
    mlx90614::FIR fir{};
    mlx90614::Gain gain{};
    mlx90614::IRSensor irs{};
    EXPECT_TRUE(unit->readIIR(iir));
    EXPECT_TRUE(unit->readFIR(fir));
    EXPECT_TRUE(unit->readGain(gain));
    EXPECT_TRUE(unit->readIRSensor(irs));

    // Default config values
    EXPECT_EQ(iir, mlx90614::IIR::Filter100);
    EXPECT_EQ(fir, mlx90614::FIR::Filter1024);
    EXPECT_EQ(gain, mlx90614::Gain::Coeff12_5);
    EXPECT_EQ(irs, mlx90614::IRSensor::Single);

    // Emissivity: default is 1.0f (raw 65535)
    float emiss{};
    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_NEAR(emiss, 1.0f, 0.001f);
}

TEST_F(TestMLX90614BAA, ChangeAddress)
{
    SCOPED_TRACE(ustr);

    uint8_t addr{};
    uint16_t emiss_org{}, emiss{};

    EXPECT_TRUE(unit->readEmissivity(emiss_org));

    EXPECT_FALSE(unit->changeI2CAddress(0x07));  // Invalid
    EXPECT_FALSE(unit->changeI2CAddress(0x78));  // Invalid

    // Change to 0x10
    EXPECT_TRUE(unit->changeI2CAddress(0x10));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x10);
    EXPECT_EQ(unit->address(), 0x10);

    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_EQ(emiss, emiss_org);

    // Change to 0x77
    EXPECT_TRUE(unit->changeI2CAddress(0x77));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x77);
    EXPECT_EQ(unit->address(), 0x77);

    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_EQ(emiss, emiss_org);

    // Change to 0x52
    EXPECT_TRUE(unit->changeI2CAddress(0x52));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x52);
    EXPECT_EQ(unit->address(), 0x52);

    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_EQ(emiss, emiss_org);

    // Change to default
    EXPECT_TRUE(unit->changeI2CAddress(UnitMLX90614BAA::DEFAULT_ADDRESS));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitMLX90614BAA::DEFAULT_ADDRESS);
    EXPECT_EQ(unit->address(), +UnitMLX90614BAA::DEFAULT_ADDRESS);

    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_EQ(emiss, emiss_org);
}
