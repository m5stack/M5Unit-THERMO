/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitMLX90614
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

constexpr uint32_t STORED_SIZE{8};

class TestMLX90614 : public ComponentTestBase<UnitMLX90614, bool> {
protected:
    virtual UnitMLX90614* get_instance() override
    {
        auto ptr         = new m5::unit::UnitMLX90614();
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
        M5_LOGW("Restore config");
        uint16_t c{0x9FB4};  // 1001 1111 1011 0100 IIR:4,FIR:7,Gain:3,IRS:0 PosK:1 PosKf2:0
        EXPECT_TRUE(unit->writeConfig(c));
    }
    void restore_setting()
    {
        M5_LOGW("Restore setting");
        EXPECT_TRUE(unit->writeObjectMinMax(25315, 39315));
        EXPECT_TRUE(unit->writeAmbientMinMax(0x1C, 0xF7));
        EXPECT_TRUE(unit->writeEmissivity(0xFFFF));
    }
};

// INSTANTIATE_TEST_SUITE_P(ParamValues, TestMLX90614, ::testing::Values(false, true));
// INSTANTIATE_TEST_SUITE_P(ParamValues, TestMLX90614, ::testing::Values(true));
INSTANTIATE_TEST_SUITE_P(ParamValues, TestMLX90614, ::testing::Values(false));

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

auto rng = std::default_random_engine{};
std::uniform_real_distribution<> dist_to(-273.15f, 382.2f);

}  // namespace

TEST_P(TestMLX90614, Config)
{
    SCOPED_TRACE(ustr);

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

#if 0
TEST_P(TestMLX90614, SettingTemperature)
{
    SCOPED_TRACE(ustr);

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

        //M5_LOGI("%f %f", tminF, toMin);
        //M5_LOGI("%f %f", tmaxF, toMax);
    }

    //
    restore_setting();
}
#endif

TEST_P(TestMLX90614, Reset)
{
    SCOPED_TRACE(ustr);
}
