/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitThermal2
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_Thermal2.hpp>
#include <cmath>
#include <random>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::thermal2;
using m5::unit::types::elapsed_time_t;

const ::testing::Environment* global_fixture = ::testing::AddGlobalTestEnvironment(new GlobalFixture<400000U>());

constexpr uint32_t STORED_SIZE{4};

class TestThermal2 : public ComponentTestBase<UnitThermal2, bool> {
protected:
    virtual UnitThermal2* get_instance() override
    {
        auto ptr         = new m5::unit::UnitThermal2();
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
};

// INSTANTIATE_TEST_SUITE_P(ParamValues, TestThermal2, ::testing::Values(false, true));
// INSTANTIATE_TEST_SUITE_P(ParamValues, TestThermal2, ::testing::Values(true));
INSTANTIATE_TEST_SUITE_P(ParamValues, TestThermal2, ::testing::Values(false));

namespace {

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

constexpr Refresh rate_table[] = {
    Refresh::Rate0_5Hz, Refresh::Rate1Hz,  Refresh::Rate2Hz,  Refresh::Rate4Hz,
    Refresh::Rate8Hz,   Refresh::Rate16Hz, Refresh::Rate32Hz, Refresh::Rate64Hz,
};
constexpr bool hl_table[] = {false, true};
constexpr struct {
    uint16_t utemp;
    float ftemp;
    bool near;
} temp_table[] = {
    {0, -64.f, false}, {8192, 0.f, false}, {12032, 30.f, false}, {20992, 100.f, false}, {65535, 447.99f, true},
};

}  // namespace

TEST_P(TestThermal2, Conversion)
{
    SCOPED_TRACE(ustr);

    for (auto&& t : temp_table) {
        if (t.near) {
            EXPECT_NEAR(raw_to_celsius(t.utemp), t.ftemp, 0.01f);
        } else {
            EXPECT_FLOAT_EQ(raw_to_celsius(t.utemp), t.ftemp);
        }
        EXPECT_EQ(celsius_to_raw(t.ftemp), t.utemp);
    }
}

TEST_P(TestThermal2, Settings)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());

    uint8_t prev_fc{};
    Refresh prev_rate{};
    EXPECT_TRUE(unit->readFunctionControl(prev_fc));
    EXPECT_TRUE(unit->readRefreshRate(prev_rate));

    // Failed
    for (int8_t fc = 7; fc >= 0; --fc) {
        uint8_t v{};
        EXPECT_FALSE(unit->writeFunctionControl((uint8_t)fc));
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, prev_fc);
    }
    for (auto&& r : rate_table) {
        Refresh rr{};
        EXPECT_FALSE(unit->writeRefreshRate(r));
        EXPECT_TRUE(unit->readRefreshRate(rr));
        EXPECT_EQ(rr, prev_rate);
    }

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    // Function control
    {
        uint8_t v{};
        for (int8_t fc = 7; fc >= 0; --fc) {
            EXPECT_TRUE(unit->writeFunctionControl((uint8_t)fc));
            EXPECT_TRUE(unit->readFunctionControl(v));
            EXPECT_EQ(v, fc);
        }
        EXPECT_TRUE(unit->writeFunctionControl(255U));
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, 7);

        EXPECT_TRUE(unit->writeFunctionControl(enabled_function_auto_refresh));
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, enabled_function_auto_refresh);

        bool enabled{};
        EXPECT_TRUE(unit->writeBuzzerEnabled(true));
        EXPECT_TRUE(unit->readBuzzerEnabled(enabled));
        EXPECT_TRUE(enabled);
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, enabled_function_auto_refresh | enabled_function_buzzer);

        EXPECT_TRUE(unit->writeBuzzerEnabled(false));
        EXPECT_TRUE(unit->readBuzzerEnabled(enabled));
        EXPECT_FALSE(enabled);
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, enabled_function_auto_refresh);

        EXPECT_TRUE(unit->writeLEDEnabled(true));
        EXPECT_TRUE(unit->readLEDEnabled(enabled));
        EXPECT_TRUE(enabled);
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, enabled_function_auto_refresh | enabled_function_led);

        EXPECT_TRUE(unit->writeLEDEnabled(false));
        EXPECT_TRUE(unit->readLEDEnabled(enabled));
        EXPECT_FALSE(enabled);
        EXPECT_TRUE(unit->readFunctionControl(v));
        EXPECT_EQ(v, enabled_function_auto_refresh);
    }
    // Refresh rate
    {
        for (auto&& r : rate_table) {
            Refresh rr{};
            EXPECT_TRUE(unit->writeRefreshRate(r));
            EXPECT_TRUE(unit->readRefreshRate(rr));
            EXPECT_EQ(rr, r) << (int)r;
        }
    }
    // Noice filter
    {
        for (uint8_t lv = 0; lv < 16; ++lv) {
            uint8_t v{};
            EXPECT_TRUE(unit->writeNoiseFilterLevel(lv));
            EXPECT_TRUE(unit->readNoiseFilterLevel(v));
            EXPECT_EQ(v, lv) << lv;
        }
        uint8_t prev_lv{}, lv{};
        EXPECT_TRUE(unit->readNoiseFilterLevel(prev_lv));

        EXPECT_FALSE(unit->writeNoiseFilterLevel(16));
        EXPECT_TRUE(unit->readNoiseFilterLevel(lv));
        EXPECT_EQ(lv, prev_lv);

        EXPECT_FALSE(unit->writeNoiseFilterLevel(255));
        EXPECT_TRUE(unit->readNoiseFilterLevel(lv));
        EXPECT_EQ(lv, prev_lv);
    }
    // Monitor size
    {
        uint8_t w{}, h{}, prev_w{}, prev_h{};
        for (uint8_t ww = 0; ww < 16; ++ww) {
            for (uint8_t hh = 0; hh < 12; ++hh) {
                EXPECT_TRUE(unit->writeTemeratureMonitorSize(ww, hh));
                EXPECT_TRUE(unit->readTemeratureMonitorSize(w, h));
                EXPECT_EQ(w, ww);
                EXPECT_EQ(h, hh);
            }
        }
        prev_w = w;
        prev_h = h;

        EXPECT_FALSE(unit->writeTemeratureMonitorSize(16, h));
        EXPECT_TRUE(unit->readTemeratureMonitorSize(w, h));
        EXPECT_EQ(w, prev_w);
        EXPECT_EQ(h, prev_h);
        EXPECT_FALSE(unit->writeTemeratureMonitorSize(w, 12));
        EXPECT_TRUE(unit->readTemeratureMonitorSize(w, h));
        EXPECT_EQ(w, prev_w);
        EXPECT_EQ(h, prev_h);
        EXPECT_FALSE(unit->writeTemeratureMonitorSize(16, 12));
        EXPECT_TRUE(unit->readTemeratureMonitorSize(w, h));
        EXPECT_EQ(w, prev_w);
        EXPECT_EQ(h, prev_h);
        EXPECT_FALSE(unit->writeTemeratureMonitorSize(255, 255));
        EXPECT_TRUE(unit->readTemeratureMonitorSize(w, h));
        EXPECT_EQ(w, prev_w);
        EXPECT_EQ(h, prev_h);
    }
}

TEST_P(TestThermal2, Alarm)
{
    SCOPED_TRACE(ustr);

    // Temp
    {
        for (auto&& hl : hl_table) {
            auto s = m5::utility::formatString("HL:%u", hl);
            SCOPED_TRACE(s);

            for (auto&& t : temp_table) {
                uint16_t _temp{};
                float _ftemp{};
                EXPECT_TRUE(unit->writeAlarmTemperature(hl, t.utemp));
                EXPECT_TRUE(unit->readAlarmTemperature(hl, _temp));
                EXPECT_TRUE(unit->readAlarmTemperature(hl, _ftemp));
                EXPECT_EQ(_temp, t.utemp);
                if (t.near) {
                    EXPECT_NEAR(_ftemp, t.ftemp, 0.01f);
                } else {
                    EXPECT_FLOAT_EQ(_ftemp, t.ftemp);
                }

                EXPECT_TRUE(unit->writeAlarmTemperature(hl, t.ftemp));
                EXPECT_TRUE(unit->readAlarmTemperature(hl, _temp));
                EXPECT_TRUE(unit->readAlarmTemperature(hl, _ftemp));
                EXPECT_EQ(_temp, t.utemp);
                if (t.near) {
                    EXPECT_NEAR(_ftemp, t.ftemp, 0.01f);
                } else {
                    EXPECT_FLOAT_EQ(_ftemp, t.ftemp);
                }
            }
        }
    }

    // LED
    {
        uint32_t count{8};
        while (count--) {
            for (auto&& hl : hl_table) {
                auto s = m5::utility::formatString("HL:%u", hl);
                SCOPED_TRACE(s);

                uint8_t r = rng() & 0xFF;
                uint8_t g = rng() & 0xFF;
                uint8_t b = rng() & 0xFF;
                uint32_t rgb{};
                EXPECT_TRUE(unit->writeAlarmLED(hl, r, g, b));
                EXPECT_TRUE(unit->readAlarmLED(hl, rgb));
                EXPECT_EQ((rgb >> 16) & 0xFF, r);
                EXPECT_EQ((rgb >> 8) & 0xFF, g);
                EXPECT_EQ((rgb >> 0) & 0xFF, b);

                uint32_t rgb24 = rng() & 0x00FFFFFF;
                EXPECT_TRUE(unit->writeAlarmLED(hl, rgb24));
                EXPECT_TRUE(unit->readAlarmLED(hl, rgb));
                EXPECT_EQ(rgb, rgb24);
            }
        }
    }

    // Buzzer
    {
        for (auto&& hl : hl_table) {
            uint16_t f{};
            uint8_t d{};
            auto s = m5::utility::formatString("HL:%u", hl);
            SCOPED_TRACE(s);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 0, 5));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(d, 5);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 65535, 255));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, d));
            EXPECT_EQ(f, 65535);
            EXPECT_EQ(d, 255);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 32768, 127));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, d));
            EXPECT_EQ(f, 32768);
            EXPECT_EQ(d, 127);

            // Failed
            EXPECT_FALSE(unit->writeAlarmBuzzer(hl, 1234, 0 /* Invalid */));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, d));
            EXPECT_EQ(f, 32768);
            EXPECT_EQ(d, 127);

            EXPECT_FALSE(unit->writeAlarmBuzzer(hl, 1234, 4 /* Invalid */));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, d));
            EXPECT_EQ(f, 32768);
            EXPECT_EQ(d, 127);
        }
    }

    // Enabled
    {
        uint8_t bits{};
        EXPECT_TRUE(unit->writeAlarmEnabled(255));
        EXPECT_TRUE(unit->readAlarmEnabled(bits));
        EXPECT_EQ(bits, 255);

        uint32_t count{16};
        while (count--) {
            uint8_t eb = rng() & 0xFF;
            EXPECT_TRUE(unit->writeAlarmEnabled(eb));
            EXPECT_TRUE(unit->readAlarmEnabled(bits));
            EXPECT_EQ(bits, eb);
        }

        EXPECT_TRUE(unit->writeAlarmEnabled(0));
        EXPECT_TRUE(unit->readAlarmEnabled(bits));
        EXPECT_EQ(bits, 0);
    }
}

TEST_P(TestThermal2, Buzzer)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->writeAlarmEnabled(0));

    bool enabled{};

    EXPECT_TRUE(unit->writeBuzzerControl(true));
    EXPECT_TRUE(unit->readBuzzerControl(enabled));
    EXPECT_TRUE(enabled);

    uint16_t f{};
    uint8_t d{};

    EXPECT_TRUE(unit->writeBuzzer(0, 0));
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 0);
    EXPECT_EQ(d, 0);

    EXPECT_TRUE(unit->writeBuzzer(65535, 255));
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 65535);
    EXPECT_EQ(d, 255);

    EXPECT_TRUE(unit->writeBuzzer(32767, 127));
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 32767);
    EXPECT_EQ(d, 127);

    EXPECT_TRUE(unit->writeBuzzerControl(false));
    EXPECT_TRUE(unit->readBuzzerControl(enabled));
    EXPECT_FALSE(enabled);
}

TEST_P(TestThermal2, LED)
{
    SCOPED_TRACE(ustr);
    EXPECT_TRUE(unit->writeAlarmEnabled(0));

    uint32_t count{8};
    while (count--) {
        uint8_t r = rng() & 0xFF;
        uint8_t g = rng() & 0xFF;
        uint8_t b = rng() & 0xFF;
        uint32_t rgb{};
        EXPECT_TRUE(unit->writeLED(r, g, b));
        EXPECT_TRUE(unit->readLED(rgb));
        EXPECT_EQ((rgb >> 16) & 0xFF, r);
        EXPECT_EQ((rgb >> 8) & 0xFF, g);
        EXPECT_EQ((rgb >> 0) & 0xFF, b);

        delay(100);

        uint32_t rgb24 = rng() & 0x00FFFFFF;
        EXPECT_TRUE(unit->writeLED(rgb24));
        EXPECT_TRUE(unit->readLED(rgb));
        EXPECT_EQ(rgb, rgb24);

        delay(100);
    }
}

TEST_P(TestThermal2, Button)
{
    SCOPED_TRACE(ustr);

    uint8_t status{};
    EXPECT_TRUE(unit->readButtonStatus(status));
    EXPECT_FALSE(status);

    unit->update();
    unit->update();

    EXPECT_FALSE(unit->isPressed());
    EXPECT_FALSE(unit->wasPressed());
    EXPECT_FALSE(unit->wasReleased());
    EXPECT_FALSE(unit->wasHold());
    EXPECT_FALSE(unit->isHolding());
}

TEST_P(TestThermal2, Firmware)
{
    SCOPED_TRACE(ustr);

    uint16_t ver{};
    EXPECT_TRUE(unit->readFirmwareVersion(ver));
    EXPECT_NE(ver, 0);
}

TEST_P(TestThermal2, Single)
{
    Data page0{}, page1{};
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->measureSingleshot(page0, page1));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    uint32_t count{8};
    while (count--) {
        Data page0{}, page1{};
        EXPECT_TRUE(unit->measureSingleshot(page0, page1));
        EXPECT_EQ(page0.subpage, 0);
        EXPECT_EQ(page1.subpage, 1);

        EXPECT_TRUE(std::any_of(std::begin(page0.temp), std::end(page0.temp), [](const uint16_t v) { return v != 0; }));
        EXPECT_TRUE(std::any_of(std::begin(page0.raw), std::end(page0.raw), [](const uint16_t v) { return v != 0; }));
        EXPECT_TRUE(std::any_of(std::begin(page1.temp), std::end(page1.temp), [](const uint16_t v) { return v != 0; }));
        EXPECT_TRUE(std::any_of(std::begin(page1.raw), std::end(page1.raw), [](const uint16_t v) { return v != 0; }));
    }
}

TEST_P(TestThermal2, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    EXPECT_TRUE(unit->startPeriodicMeasurement(Refresh::Rate16Hz));
    EXPECT_TRUE(unit->inPeriodic());

    auto elapsed = test_periodic(unit.get(), STORED_SIZE);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    EXPECT_NE(elapsed, 0);
    EXPECT_GE(elapsed, unit->interval() * STORED_SIZE);

    EXPECT_EQ(unit->available(), STORED_SIZE);
    EXPECT_FALSE(unit->empty());
    EXPECT_TRUE(unit->full());

    uint32_t cnt{STORED_SIZE / 2};
    while (cnt-- && unit->available()) {
        auto d = unit->oldest();
        EXPECT_TRUE(std::any_of(std::begin(d.temp), std::end(d.temp), [](const uint16_t v) { return v != 0; }));
        EXPECT_TRUE(std::any_of(std::begin(d.raw), std::end(d.raw), [](const uint16_t v) { return v != 0; }));

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
}

TEST_P(TestThermal2, I2CAddress)
{
    SCOPED_TRACE(ustr);

    EXPECT_FALSE(unit->changeI2CAddress(0x07));  // Invalid
    EXPECT_FALSE(unit->changeI2CAddress(0x78));  // Invalid

    // I2C address change requires a reset, so not tested here
    // EXPECT_TRUE(unit->changeI2CAddress(0x10));
    // EXPECT_TRUE(unit->changeI2CAddress(0x32));
}
