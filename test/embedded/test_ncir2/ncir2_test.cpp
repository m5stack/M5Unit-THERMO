/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  UnitTest for UnitNCIR2
*/
#include <gtest/gtest.h>
#include <Wire.h>
#include <M5Unified.h>
#include <M5UnitUnified.hpp>
#include <googletest/test_template.hpp>
#include <googletest/test_helper.hpp>
#include <unit/unit_NCIR2.hpp>
#include <m5_unit_component/adapter_i2c.hpp>
#include <cmath>
#include <esp_random.h>

using namespace m5::unit::googletest;
using namespace m5::unit;
using namespace m5::unit::ncir2;
using m5::unit::types::elapsed_time_t;

constexpr uint32_t STORED_SIZE{4};

class TestNCIR2 : public I2CComponentTestBase<UnitNCIR2> {
protected:
    virtual UnitNCIR2* get_instance() override
    {
        auto ptr         = new m5::unit::UnitNCIR2();
        auto ccfg        = ptr->component_config();
        ccfg.stored_size = STORED_SIZE;
        ptr->component_config(ccfg);
        return ptr;
    }
};

namespace {

constexpr bool hl_table[] = {false, true};

}  // namespace

TEST_F(TestNCIR2, Emissivity)
{
    SCOPED_TRACE(ustr);

    {
        uint16_t raw{};
        float e{};
        constexpr float near{0.00001f};

        EXPECT_TRUE(unit->writeEmissivity(0.1f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_TRUE(unit->readEmissivity(raw));
        EXPECT_NEAR(e, 0.1f, near);
        EXPECT_EQ(raw, 6554);

        EXPECT_TRUE(unit->writeEmissivity(0.5f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_TRUE(unit->readEmissivity(raw));
        EXPECT_NEAR(e, 0.5f, near);
        EXPECT_EQ(raw, 32768);

        EXPECT_TRUE(unit->writeEmissivity(1.0f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_TRUE(unit->readEmissivity(raw));
        EXPECT_NEAR(e, 1.0f, near);
        EXPECT_EQ(raw, 65535);

        // false
        EXPECT_FALSE(unit->writeEmissivity(0.09f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_NEAR(e, 1.0f, near);

        EXPECT_FALSE(unit->writeEmissivity(-1.f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_NEAR(e, 1.0f, near);

        EXPECT_FALSE(unit->writeEmissivity(1.001f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_NEAR(e, 1.0f, near);

        // default
        EXPECT_TRUE(unit->writeEmissivity(0.95f));
        EXPECT_TRUE(unit->readEmissivity(e));
        EXPECT_TRUE(unit->readEmissivity(raw));
        EXPECT_NEAR(e, 0.95f, near);
        EXPECT_EQ(raw, 62258);  // datasheet
    }
}

TEST_F(TestNCIR2, Alarm)
{
    SCOPED_TRACE(ustr);

    // Temp
    {
        for (auto&& hl : hl_table) {
            auto s = m5::utility::formatString("HL:%u", hl);
            SCOPED_TRACE(s);

            int16_t temp{};
            EXPECT_TRUE(unit->writeAlarmTemperature(hl, -32768));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_EQ(temp, -32768);

            EXPECT_TRUE(unit->writeAlarmTemperature(hl, 0));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_EQ(temp, 0);

            EXPECT_TRUE(unit->writeAlarmTemperature(hl, 32767));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_EQ(temp, 32767);
        }

        for (auto&& hl : hl_table) {
            auto s = m5::utility::formatString("HL:%u", hl);
            SCOPED_TRACE(s);

            float temp{};
            EXPECT_TRUE(unit->writeAlarmTemperature(hl, -327.68f));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_FLOAT_EQ(temp, -327.68f);

            EXPECT_TRUE(unit->writeAlarmTemperature(hl, 0.0f));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_FLOAT_EQ(temp, 0.0f);

            EXPECT_TRUE(unit->writeAlarmTemperature(hl, 327.67f));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_FLOAT_EQ(temp, 327.67f);

            // false
            EXPECT_FALSE(unit->writeAlarmTemperature(hl, -327.69f));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_FLOAT_EQ(temp, 327.67f);

            EXPECT_FALSE(unit->writeAlarmTemperature(hl, 327.68f));
            EXPECT_TRUE(unit->readAlarmTemperature(hl, temp));
            EXPECT_FLOAT_EQ(temp, 327.67f);
        }

        // set minimum to low and maximum to high
        EXPECT_TRUE(unit->writeAlarmTemperature(false, -32768));
        EXPECT_TRUE(unit->writeAlarmTemperature(true, 32767));
    }

    // LED
    {
        uint32_t count{8};
        while (count--) {
            for (auto&& hl : hl_table) {
                auto s = m5::utility::formatString("HL:%u", hl);
                SCOPED_TRACE(s);

                uint8_t r = esp_random() & 0xFF;
                uint8_t g = esp_random() & 0xFF;
                uint8_t b = esp_random() & 0xFF;
                uint32_t rgb{};
                EXPECT_TRUE(unit->writeAlarmLED(hl, r, g, b));
                EXPECT_TRUE(unit->readAlarmLED(hl, rgb));
                EXPECT_EQ((rgb >> 16) & 0xFF, r);
                EXPECT_EQ((rgb >> 8) & 0xFF, g);
                EXPECT_EQ((rgb >> 0) & 0xFF, b);

                uint32_t rgb24 = esp_random() & 0x00FFFFFF;
                EXPECT_TRUE(unit->writeAlarmLED(hl, rgb24));
                EXPECT_TRUE(unit->readAlarmLED(hl, rgb));
                EXPECT_EQ(rgb, rgb24);
            }
        }
    }

    // Buzzer
    {
        for (auto&& hl : hl_table) {
            auto s = m5::utility::formatString("HL:%u", hl);
            SCOPED_TRACE(s);

            uint16_t f{}, i{};
            uint8_t d{};
            float fd{};

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 0, 1, 0));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_EQ(d, 0);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 65535, 5000, 255));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 65535);
            EXPECT_EQ(i, 5000);
            EXPECT_EQ(d, 255);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 32768, 2500, 127));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 32768);
            EXPECT_EQ(i, 2500);
            EXPECT_EQ(d, 127);

            // false
            EXPECT_FALSE(unit->writeAlarmBuzzer(hl, 1234, 0 /* invalid */, 56));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 32768);
            EXPECT_EQ(i, 2500);
            EXPECT_EQ(d, 127);

            EXPECT_FALSE(unit->writeAlarmBuzzer(hl, 1234, 5001 /* invalid */, 56));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 32768);
            EXPECT_EQ(i, 2500);
            EXPECT_EQ(d, 127);

            // float duty
            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 0, 1, 0.0f));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, fd));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_FLOAT_EQ(fd, 0.0f);
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_EQ(d, 0);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 0, 1, 1.0f));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, fd));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_FLOAT_EQ(fd, 1.0f);
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_EQ(d, 255);

            EXPECT_TRUE(unit->writeAlarmBuzzer(hl, 0, 1, 0.5f));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, fd));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_FLOAT_EQ(fd, 0.5f);
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_EQ(d, 127);  // datasheet

            // false
            EXPECT_FALSE(unit->writeAlarmBuzzer(hl, 0, 1, -0.0001f));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_EQ(d, 127);

            EXPECT_FALSE(unit->writeAlarmBuzzer(hl, 0, 1, 1.0001f));
            EXPECT_TRUE(unit->readAlarmBuzzer(hl, f, i, d));
            EXPECT_EQ(f, 0);
            EXPECT_EQ(i, 1);
            EXPECT_EQ(d, 127);
        }
    }
}

TEST_F(TestNCIR2, Buzzer)
{
    SCOPED_TRACE(ustr);

    bool enabled{};

    EXPECT_TRUE(unit->writeBuzzerControl(true));
    EXPECT_TRUE(unit->readBuzzerControl(enabled));
    EXPECT_TRUE(enabled);

    uint16_t f{};
    uint8_t d{};
    float fd{};

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

    // float duty
    EXPECT_TRUE(unit->writeBuzzer(0, 0.0f));
    EXPECT_TRUE(unit->readBuzzer(f, fd));
    EXPECT_EQ(f, 0);
    EXPECT_FLOAT_EQ(fd, 0.0f);
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 0);
    EXPECT_EQ(d, 0);

    EXPECT_TRUE(unit->writeBuzzer(0, 1.0f));
    EXPECT_TRUE(unit->readBuzzer(f, fd));
    EXPECT_EQ(f, 0);
    EXPECT_FLOAT_EQ(fd, 1.0f);
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 0);
    EXPECT_EQ(d, 255);

    EXPECT_TRUE(unit->writeBuzzer(0, 0.5f));
    EXPECT_TRUE(unit->readBuzzer(f, fd));
    EXPECT_EQ(f, 0);
    EXPECT_FLOAT_EQ(fd, 0.5f);
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 0);
    EXPECT_EQ(d, 127);

    // false
    EXPECT_FALSE(unit->writeBuzzer(0, -0.0001f));
    EXPECT_TRUE(unit->readBuzzer(f, fd));
    EXPECT_EQ(f, 0);
    EXPECT_FLOAT_EQ(fd, 0.5f);
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 0);
    EXPECT_EQ(d, 127);

    EXPECT_FALSE(unit->writeBuzzer(0, 1.0001f));
    EXPECT_TRUE(unit->readBuzzer(f, fd));
    EXPECT_EQ(f, 0);
    EXPECT_FLOAT_EQ(fd, 0.5f);
    EXPECT_TRUE(unit->readBuzzer(f, d));
    EXPECT_EQ(f, 0);
    EXPECT_EQ(d, 127);

    // disable
    EXPECT_TRUE(unit->writeBuzzerControl(false));
    EXPECT_TRUE(unit->readBuzzerControl(enabled));
    EXPECT_FALSE(enabled);
}

TEST_F(TestNCIR2, LED)
{
    SCOPED_TRACE(ustr);

    uint32_t count{8};
    while (count--) {
        uint8_t r = esp_random() & 0xFF;
        uint8_t g = esp_random() & 0xFF;
        uint8_t b = esp_random() & 0xFF;
        uint32_t rgb{};
        EXPECT_TRUE(unit->writeLED(r, g, b));
        EXPECT_TRUE(unit->readLED(rgb));
        EXPECT_EQ((rgb >> 16) & 0xFF, r);
        EXPECT_EQ((rgb >> 8) & 0xFF, g);
        EXPECT_EQ((rgb >> 0) & 0xFF, b);

        uint32_t rgb24 = esp_random() & 0x00FFFFFF;
        EXPECT_TRUE(unit->writeLED(rgb24));
        EXPECT_TRUE(unit->readLED(rgb));
        EXPECT_EQ(rgb, rgb24);
    }
}

TEST_F(TestNCIR2, Button)
{
    SCOPED_TRACE(ustr);

    bool press{};
    EXPECT_TRUE(unit->readButtonStatus(press));
    EXPECT_FALSE(press);

    unit->update();
    unit->update();

    EXPECT_FALSE(unit->isPressed());
    EXPECT_FALSE(unit->wasPressed());
    EXPECT_FALSE(unit->wasReleased());
}

TEST_F(TestNCIR2, Firmware)
{
    SCOPED_TRACE(ustr);

    uint8_t ver{};
    EXPECT_TRUE(unit->readFirmwareVersion(ver));
    EXPECT_NE(ver, 0);
}

TEST_F(TestNCIR2, SingleAndChip)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    Data d{};
    EXPECT_FALSE(unit->measureSingleshot(d));
    EXPECT_TRUE(unit->readChipTemperature(d));

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    uint32_t count{8};
    while (count--) {
        Data ds{}, dc{};
        float temp{}, ctemp{};
        EXPECT_TRUE(unit->measureSingleshot(ds));
        EXPECT_TRUE(unit->readChipTemperature(dc));

        temp  = ds.temperature();
        ctemp = dc.temperature();
        EXPECT_NE(temp, 0.0f);
        EXPECT_TRUE(std::isfinite(temp));
        EXPECT_TRUE(std::isfinite(ctemp));
        EXPECT_TRUE(std::isfinite(ds.fahrenheit()));
        EXPECT_TRUE(std::isfinite(dc.fahrenheit()));

        EXPECT_NE(temp, ctemp);

        // M5_LOGI("Single:%f Chip:%f", temp, ctemp);
        m5::utility::delay(100);
    }
}

TEST_F(TestNCIR2, Periodic)
{
    SCOPED_TRACE(ustr);

    EXPECT_TRUE(unit->inPeriodic());
    EXPECT_FALSE(unit->startPeriodicMeasurement());
    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    EXPECT_TRUE(unit->startPeriodicMeasurement(100));
    EXPECT_TRUE(unit->inPeriodic());

    auto ad          = unit->asAdapter<m5::unit::AdapterI2C>(m5::unit::Adapter::Type::I2C);
    bool is_bus      = ad && ad->implType() == m5::unit::AdapterI2C::ImplType::Bus;
    uint32_t timeout = is_bus ? std::max<uint32_t>(unit->interval(), 500) * (STORED_SIZE + 1) * 4
                              : unit->interval() * (STORED_SIZE + 1) * 2;
    auto r           = collect_periodic_measurements(unit.get(), STORED_SIZE, timeout);

    EXPECT_TRUE(unit->stopPeriodicMeasurement());
    EXPECT_FALSE(unit->inPeriodic());

    EXPECT_FALSE(r.timed_out);
    EXPECT_EQ(r.update_count, STORED_SIZE);

    EXPECT_EQ(unit->available(), STORED_SIZE);
    EXPECT_FALSE(unit->empty());
    EXPECT_TRUE(unit->full());

    uint32_t cnt{STORED_SIZE / 2};
    while (cnt-- && unit->available()) {
        EXPECT_TRUE(std::isfinite(unit->temperature()));
        EXPECT_TRUE(std::isfinite(unit->fahrenheit()));
        EXPECT_FLOAT_EQ(unit->temperature(), unit->oldest().temperature());
        EXPECT_FLOAT_EQ(unit->fahrenheit(), unit->oldest().fahrenheit());

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

    EXPECT_FALSE(std::isfinite(unit->temperature()));
    EXPECT_FALSE(std::isfinite(unit->fahrenheit()));
}

/*
  WARNING: writeConfig() persists ALL settings (Emissivity, Alarm, LED) to flash.
  If restore fails, flash retains test values. Original values are logged via M5_LOGI.
  Known factory default — Emissivity: 0.95 (raw 62258), others: undocumented.
*/
TEST_F(TestNCIR2, WriteConfig)
{
    SCOPED_TRACE(ustr);

    // Save all flash-persistent settings (ASSERT to abort if read fails)
    uint16_t orig_emiss{};
    int16_t orig_alarm_lo{}, orig_alarm_hi{};
    uint32_t orig_alarm_led_lo{}, orig_alarm_led_hi{};
    uint16_t orig_abuz_lo_f{}, orig_abuz_lo_i{}, orig_abuz_hi_f{}, orig_abuz_hi_i{};
    uint8_t orig_abuz_lo_d{}, orig_abuz_hi_d{};
    uint32_t orig_led{};
    ASSERT_TRUE(unit->readEmissivity(orig_emiss));
    ASSERT_TRUE(unit->readAlarmTemperature(false, orig_alarm_lo));
    ASSERT_TRUE(unit->readAlarmTemperature(true, orig_alarm_hi));
    ASSERT_TRUE(unit->readAlarmLED(false, orig_alarm_led_lo));
    ASSERT_TRUE(unit->readAlarmLED(true, orig_alarm_led_hi));
    ASSERT_TRUE(unit->readAlarmBuzzer(false, orig_abuz_lo_f, orig_abuz_lo_i, orig_abuz_lo_d));
    ASSERT_TRUE(unit->readAlarmBuzzer(true, orig_abuz_hi_f, orig_abuz_hi_i, orig_abuz_hi_d));
    ASSERT_TRUE(unit->readLED(orig_led));

    M5_LOGI("Original emiss:%u alarm_lo:%d alarm_hi:%d", orig_emiss, orig_alarm_lo, orig_alarm_hi);
    M5_LOGI("Original alarm_led_lo:0x%06X alarm_led_hi:0x%06X led:0x%06X", orig_alarm_led_lo, orig_alarm_led_hi,
            orig_led);
    M5_LOGI("Original abuz_lo f:%u i:%u d:%u  abuz_hi f:%u i:%u d:%u", orig_abuz_lo_f, orig_abuz_lo_i, orig_abuz_lo_d,
            orig_abuz_hi_f, orig_abuz_hi_i, orig_abuz_hi_d);

    // Modify settings
    EXPECT_TRUE(unit->writeEmissivity(0.5f));
    EXPECT_TRUE(unit->writeAlarmTemperature(false, (int16_t)-1000));
    EXPECT_TRUE(unit->writeAlarmTemperature(true, (int16_t)5000));
    EXPECT_TRUE(unit->writeAlarmLED(false, (uint32_t)0x112233));
    EXPECT_TRUE(unit->writeAlarmLED(true, (uint32_t)0x445566));
    EXPECT_TRUE(unit->writeLED((uint32_t)0x778899));

    // Persist to flash
    EXPECT_TRUE(unit->writeConfig());

    // Verify settings after save
    uint16_t emiss{};
    int16_t alarm_lo{}, alarm_hi{};
    uint32_t alarm_led_lo{}, alarm_led_hi{}, led{};
    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_TRUE(unit->readAlarmTemperature(false, alarm_lo));
    EXPECT_TRUE(unit->readAlarmTemperature(true, alarm_hi));
    EXPECT_TRUE(unit->readAlarmLED(false, alarm_led_lo));
    EXPECT_TRUE(unit->readAlarmLED(true, alarm_led_hi));
    EXPECT_TRUE(unit->readLED(led));
    EXPECT_EQ(emiss, 32768);
    EXPECT_EQ(alarm_lo, -1000);
    EXPECT_EQ(alarm_hi, 5000);
    EXPECT_EQ(alarm_led_lo, (uint32_t)0x112233);
    EXPECT_EQ(alarm_led_hi, (uint32_t)0x445566);
    EXPECT_EQ(led, (uint32_t)0x778899);

    // Restore all original values and save back
    ASSERT_TRUE(unit->writeEmissivity(orig_emiss)) << "Flash may retain test emissivity (32768)";
    ASSERT_TRUE(unit->writeAlarmTemperature(false, orig_alarm_lo)) << "Flash may retain alarm_lo (-1000)";
    ASSERT_TRUE(unit->writeAlarmTemperature(true, orig_alarm_hi)) << "Flash may retain alarm_hi (5000)";
    ASSERT_TRUE(unit->writeAlarmLED(false, orig_alarm_led_lo)) << "Flash may retain alarm_led_lo (0x112233)";
    ASSERT_TRUE(unit->writeAlarmLED(true, orig_alarm_led_hi)) << "Flash may retain alarm_led_hi (0x445566)";
    ASSERT_TRUE(unit->writeAlarmBuzzer(false, orig_abuz_lo_f, orig_abuz_lo_i, orig_abuz_lo_d));
    ASSERT_TRUE(unit->writeAlarmBuzzer(true, orig_abuz_hi_f, orig_abuz_hi_i, orig_abuz_hi_d));
    ASSERT_TRUE(unit->writeLED(orig_led)) << "Flash may retain LED (0x778899)";
    ASSERT_TRUE(unit->writeConfig()) << "Failed to persist restored values";
}

/*
  WARNING!!
  Failure of this test will result in an unexpected I2C address being set!
*/
TEST_F(TestNCIR2, I2CAddress)
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
    EXPECT_TRUE(unit->changeI2CAddress(UnitNCIR2::DEFAULT_ADDRESS));
    EXPECT_TRUE(unit->readI2CAddress(addr));
    EXPECT_EQ(addr, +UnitNCIR2::DEFAULT_ADDRESS);
    EXPECT_EQ(unit->address(), +UnitNCIR2::DEFAULT_ADDRESS);

    EXPECT_TRUE(unit->readEmissivity(emiss));
    EXPECT_EQ(emiss, emiss_org);
}
