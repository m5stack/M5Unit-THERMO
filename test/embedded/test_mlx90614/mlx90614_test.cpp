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
#include <cmath>
#include <random>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::mlx90614;
using m5::unit::types::elapsed_time_t;

const ::testing::Environment* global_fixture = ::testing::AddGlobalTestEnvironment(new GlobalFixture<100000U>());

constexpr uint32_t STORED_SIZE{4};

class TestMLX90614BAA : public ComponentTestBase<UnitMLX90614BAA, bool> {
protected:
    virtual UnitMLX90614BAA* get_instance() override
    {
        auto ptr         = new m5::unit::UnitMLX90614BAA();
        auto ccfg        = ptr->component_config();
        ccfg.stored_size = STORED_SIZE;
        ptr->component_config(ccfg);
        return ptr;

        return ptr;
    }
    virtual bool is_using_hal() const override
    {
        return GetParam();
    };

    void restore_config()
    {
        // M5_LOGW("Restore config");
        uint16_t c{0x9FB4};  // 1001 1111 1011 0100 IIR:4, OUT:TO12 FIR:7,Gain:3,IRS:0 PosK:1 PosKf2:0
        EXPECT_TRUE(unit->writeConfig(c));
    }
    void restore_setting()
    {
        // M5_LOGW("Restore setting");
        EXPECT_TRUE(unit->writeObjectMinMax(25315, 39315, false));
        EXPECT_TRUE(unit->writeAmbientMinMax(0x1C, 0xF7, false));
        EXPECT_TRUE(unit->writeEmissivity(0xFFFF, false));
        EXPECT_TRUE(unit->applySettings());
    }
};

// INSTANTIATE_TEST_SUITE_P(ParamValues, TestMLX90614BAA, ::testing::Values(false, true));
// INSTANTIATE_TEST_SUITE_P(ParamValues, TestMLX90614BAA, ::testing::Values(true));
INSTANTIATE_TEST_SUITE_P(ParamValues, TestMLX90614BAA, ::testing::Values(false));

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

template <class U>
elapsed_time_t test_periodic(U* unit, const uint32_t times, const uint32_t measure_duration = 0)
{
    auto tm         = unit->interval();
    auto timeout_at = m5::utility::millis() + 10 * 1000;

    do {
        unit->update();
        if (unit->updated()) {
            break;
        }
        std::this_thread::yield();
    } while (!unit->updated() && m5::utility::millis() <= timeout_at);
    // timeout
    if (!unit->updated()) {
        return 0;
    }

    //
    uint32_t measured{};
    auto start_at = m5::utility::millis();
    timeout_at    = start_at + (times * (tm + measure_duration) * 2);

    do {
        unit->update();
        measured += unit->updated() ? 1 : 0;
        if (measured >= times) {
            break;
        }
        std::this_thread::yield();

    } while (measured < times && m5::utility::millis() <= timeout_at);
    return (measured == times) ? m5::utility::millis() - start_at : 0;
    //   return (measured == times) ? unit->updatedMillis() - start_at : 0;
}

auto rng = std::default_random_engine{};
std::uniform_real_distribution<> dist_to(-273.15f, 382.2f);
std::uniform_real_distribution<> dist_ta(-38.2f, 125.f);

// From UnitMLX90416.cpp
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

TEST_P(TestMLX90614BAA, Conversion)
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
        float co     = dist_to(rng);
        float ca     = dist_ta(rng);
        uint16_t to  = celsius_to_toRaw(co);
        uint8_t ta   = celsius_to_taRaw(ca);
        float tof    = toRaw_to_celsius(to);
        float taf    = taRaw_to_celsius(ta);
        uint16_t to2 = celsius_to_toRaw(tof);
        uint8_t ta2  = celsius_to_taRaw(taf);

        EXPECT_NEAR(tof, co, 0.005f);
        EXPECT_NEAR(taf, ca, 0.32f);
        EXPECT_EQ(to2, to);
        EXPECT_EQ(ta2, ta);
    }
}

TEST_P(TestMLX90614BAA, Config)
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

    /// Output
    for (auto&& o : out_table) {
        EXPECT_TRUE(unit->writeOutput(o));
        Output v{};
        EXPECT_TRUE(unit->readOutput(v));
        EXPECT_EQ(v, o);
    }

    // IIR
    for (auto&& iir : iir_table) {
        EXPECT_TRUE(unit->writeIIR(iir));
        IIR v{};
        EXPECT_TRUE(unit->readIIR(v));
        EXPECT_EQ(v, iir);
    }

    // FIR
    for (auto&& fir : fir_table) {
        EXPECT_TRUE(unit->writeFIR(fir));
        FIR v{};
        EXPECT_TRUE(unit->readFIR(v));
        EXPECT_EQ(v, fir);
    }

    // Gain
    for (auto&& gain : gain_table) {
        EXPECT_TRUE(unit->writeGain(gain));
        Gain v{};
        EXPECT_TRUE(unit->readGain(v));
        EXPECT_EQ(v, gain);
    }

    // IRSensor
    for (auto&& irs : irs_table) {
        EXPECT_TRUE(unit->writeIRSensor(irs));
        IRSensor v{};
        EXPECT_TRUE(unit->readIRSensor(v));
        EXPECT_EQ(v, irs);
    }

    // PosK
    for (auto&& pos : pos_table) {
        EXPECT_TRUE(unit->writePositiveKs(pos));
        bool v{};
        EXPECT_TRUE(unit->readPositiveKs(v));
        EXPECT_EQ(v, pos);
    }

    // PosKf2
    for (auto&& pos : pos_table) {
        EXPECT_TRUE(unit->writePositiveKf2(pos));
        bool v{};
        EXPECT_TRUE(unit->readPositiveKf2(v));
        EXPECT_EQ(v, pos);
    }

    //
    restore_config();
}

TEST_P(TestMLX90614BAA, SettingObjectTemperatureMinMax)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());

    EXPECT_FALSE(unit->writeObjectMinMax(-273.15f, -273.15f));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    float tminF{}, tmaxF{};
    uint16_t tmin{}, tmax{};

    // min
    EXPECT_TRUE(unit->writeObjectMinMax(-273.15f, -273.15f));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -273.15f);
    EXPECT_FLOAT_EQ(tmaxF, -273.15f);

    // under
    EXPECT_TRUE(unit->writeObjectMinMax(-1273.15f, -1273.15f));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -273.15f);
    EXPECT_FLOAT_EQ(tmaxF, -273.15f);

    // max
    EXPECT_TRUE(unit->writeObjectMinMax(382.2f, 382.2f));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFFFF);
    EXPECT_EQ(tmax, 0xFFFF);
    EXPECT_FLOAT_EQ(tminF, 382.2f);
    EXPECT_FLOAT_EQ(tmaxF, 382.2f);

    // over
    EXPECT_TRUE(unit->writeObjectMinMax(1382.2f, 1382.2f));

    EXPECT_TRUE(unit->readObjectMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFFFF);
    EXPECT_EQ(tmax, 0xFFFF);
    EXPECT_FLOAT_EQ(tminF, 382.2f);
    EXPECT_FLOAT_EQ(tmaxF, 382.2f);

    // random
    uint32_t cnt{32};
    while (cnt--) {
        float toMin = dist_to(rng);
        float toMax = dist_to(rng);
        if (toMin > toMax) {
            std::swap(toMin, toMax);
        }
        auto s = m5::utility::formatString("%f/%f", toMin, toMax);
        SCOPED_TRACE(s);

        EXPECT_TRUE(unit->writeObjectMinMax(toMin, toMax));

        EXPECT_TRUE(unit->readObjectMinMax(tminF, tmaxF));
        EXPECT_NEAR(tminF, toMin, 0.005f);
        EXPECT_NEAR(tmaxF, toMax, 0.005f);

        // M5_LOGI("%f %f", tminF, toMin);
        // M5_LOGI("%f %f", tmaxF, toMax);
    }

    //
    restore_setting();
}

TEST_P(TestMLX90614BAA, SettingAmbientTemperatureMinMax)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());

    EXPECT_FALSE(unit->writeAmbientMinMax(-38.2f, 125.f));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    float tminF{}, tmaxF{};
    uint8_t tmin{}, tmax{};

    // min
    EXPECT_TRUE(unit->writeAmbientMinMax(-38.2f, -38.2f));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -38.2f);
    EXPECT_FLOAT_EQ(tmaxF, -38.2f);

    // under
    EXPECT_TRUE(unit->writeAmbientMinMax(-1273.15f, -1273.15f));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0);
    EXPECT_EQ(tmax, 0);
    EXPECT_FLOAT_EQ(tminF, -38.2f);
    EXPECT_FLOAT_EQ(tmaxF, -38.2f);

    // max
    EXPECT_TRUE(unit->writeAmbientMinMax(125.f, 125.f));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFF);
    EXPECT_EQ(tmax, 0xFF);
    EXPECT_FLOAT_EQ(tminF, 125.f);
    EXPECT_FLOAT_EQ(tmaxF, 125.f);

    // over
    EXPECT_TRUE(unit->writeAmbientMinMax(1382.2f, 1382.2f));

    EXPECT_TRUE(unit->readAmbientMinMax(tmin, tmax));
    EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
    EXPECT_EQ(tmin, 0xFF);
    EXPECT_EQ(tmax, 0xFF);
    EXPECT_FLOAT_EQ(tminF, 125.f);
    EXPECT_FLOAT_EQ(tmaxF, 125.f);

    // random
    uint32_t cnt{32};
    while (cnt--) {
        float toMin = dist_ta(rng);
        float toMax = dist_ta(rng);
        if (toMin > toMax) {
            std::swap(toMin, toMax);
        }
        auto s = m5::utility::formatString("%f/%f", toMin, toMax);
        SCOPED_TRACE(s);

        EXPECT_TRUE(unit->writeAmbientMinMax(toMin, toMax));

        EXPECT_TRUE(unit->readAmbientMinMax(tminF, tmaxF));
        EXPECT_NEAR(tminF, toMin, 0.32f);
        EXPECT_NEAR(tmaxF, toMax, 0.32f);
        // M5_LOGI("%f %f", tminF, toMin);
        // M5_LOGI("%f %f", tmaxF, toMax);
    }

    //
    restore_setting();
}

TEST_P(TestMLX90614BAA, ChangeAddress)
{
    SCOPED_TRACE(ustr);

    uint8_t addr{};
    uint16_t emiss_org{}, emiss{};

    EXPECT_TRUE(unit->readEmissivity(emiss_org));

    EXPECT_FALSE(unit->changeI2CAddress(0x07));  // Invalid
    EXPECT_FALSE(unit->changeI2CAddress(0x78));  // Invalid

    // Change to 0x08
    EXPECT_TRUE(unit->changeI2CAddress(0x08));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, 0x08);
    EXPECT_EQ(unit->address(), 0x08);

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

TEST_P(TestMLX90614BAA, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    for (auto&& iir : iir_table) {
        for (auto&& fir : fir_table) {
            if (m5::stl::to_underlying(fir) < 4) {
                continue;
            }

            auto s = m5::utility::formatString("IIR:%u FIR:%u", iir, fir);
            SCOPED_TRACE(s);

            //                EXPECT_TRUE(unit->startPeriodicMeasurement(iir, fir, Gain::Coeff12_5, irs));
            EXPECT_TRUE(unit->startPeriodicMeasurement(iir, fir, Gain::Coeff12_5, IRSensor::Dual));
            EXPECT_TRUE(unit->inPeriodic());

            auto tm      = get_interval(iir, fir);
            auto elapsed = test_periodic(unit.get(), STORED_SIZE, tm ? tm : 1);

            EXPECT_TRUE(unit->stopPeriodicMeasurement());
            EXPECT_FALSE(unit->inPeriodic());

            EXPECT_NE(elapsed, 0);
            EXPECT_GE(elapsed + 2, STORED_SIZE * (tm ? tm : 1));

            M5_LOGI("TM:%u IT:%u e:%ld", tm, unit->interval(), elapsed);

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
        }
    }

    restore_setting();
    restore_config();
}
