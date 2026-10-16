/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitThermal2
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTHERMO.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

using namespace m5::unit::thermal2;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitThermal2 unit;

void ring_buzzer(const uint16_t freq, const uint8_t duty, const uint16_t count = 1, const uint32_t ms = 100,
                 const uint32_t interval = 50)
{
    unit.writeBuzzerControl(false);
    for (uint16_t i = 0; i < count; ++i) {
        unit.writeBuzzer(freq, duty);
        unit.writeBuzzerControl(true);
        m5::utility::delay(ms);
        unit.writeBuzzerControl(false);
        if (i + 1 != count) {
            m5::utility::delay(interval);
        }
    }
    unit.writeBuzzerControl(false);
}

constexpr float low_alarm_temp{10.0f};
constexpr float high_alarm_temp{30.0f};

void dump(const Data& d)
{
    M5.Log.printf("Subpage:%u\n", d.subpage);
    M5.Log.printf("Med:%.2f Avg:%.2f High:%.2f Low:%.2f\n", d.medianTemperature(), d.averageTemperature(),
                  d.highestTemperature(), d.lowestTemperature());

    M5.Log.printf("MostDiff(%u,%u) lowest{%u,%u} highest<%u,%u>\n", d.most_diff_x, d.most_diff_y, d.lowest_diff_x,
                  d.lowest_diff_y, d.highest_diff_x, d.highest_diff_y);

    M5.Log.printf(
        "    0000   0001   0002   0003   0004   0005   0006   0007   0008   0009   0010   0011   0012   0013   0014   "
        "0015   "
        "0016   0017   0018   0019   0020   0021   0022   0023   0024   0025   0026   0027   0028   0029   0030   "
        "0031\n");
    M5.Log.printf(
        "--------------------------------------------------------------------------------------------------------------"
        "------"
        "-------------------------------------------------------------------------------------------------------------"
        "\n");
    for (int y = 0; y < 24; ++y) {
        M5.Log.printf("%02d:%s", y, ((y & 1) != d.subpage) ? "       " : "");
        for (int x = 0; x < 16; ++x) {
            auto val = d.raw[y * 16 + x];
            int xx   = x * 2 + ((y & 1) != d.subpage);
            if (xx == d.most_diff_x && y == d.most_diff_y) {
                M5.Log.printf("(%04X) ", val);
            } else if (xx == d.lowest_diff_x && y == d.lowest_diff_y) {
                M5.Log.printf("{%04X} ", val);
            } else if (xx == d.highest_diff_x && y == d.highest_diff_y) {
                M5.Log.printf("<%04X> ", val);
            } else {
                M5.Log.printf(" %04X  ", val);
            }
            if (x != 15) {
                M5.Log.printf("       ");
            }
        }
        M5.Log.printf("\n");
    }
}

void draw_values(const Data& d)
{
    lcd.startWrite();
    lcd.fillRect(8, 8, 8 * 10, 16 * 3, TFT_BLACK);
    lcd.setCursor(8, 8 + 16 * 0);
    lcd.printf("Avg:%.2f", d.averageTemperature());
    lcd.setCursor(8, 8 + 16 * 1);
    lcd.printf("Hi:%.2f", d.highestTemperature());
    lcd.setCursor(8, 8 + 16 * 2);
    lcd.printf("Lo:%.2f", d.lowestTemperature());
    lcd.endWrite();
}

}  // namespace

void setup()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    auto cfg = unit.config();
    cfg.rate = Refresh::Rate0_5Hz;  // Measure per 2 second
    unit.config(cfg);

    // Board-aware connection: NessoN1 -> SoftwareI2C on GROVE, NanoC6/NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, unit) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    unit.writeAlarmEnabled(0);
    unit.writeBuzzerControl(false);
    unit.writeLED(2, 2, 10);

    lcd.setFont(&fonts::AsciiFont8x16);
    lcd.fillScreen(TFT_DARKGREEN);
    lcd.fillRect(8, 8, 8 * 10, 16 * 3, TFT_BLACK);
}

void loop()
{
    M5.update();
    Units.update();

    // Periodic
    if (unit.updated()) {
        dump(unit.oldest());
        draw_values(unit.oldest());
    }

    // Button on UnitThermal2 (toggle  periodic <-> single)
    if (unit.wasClicked()) {
        static bool single{};
        single = !single;

        if (single) {
            unit.stopPeriodicMeasurement();

            unit.writeLED(10, 2, 10);
            ring_buzzer(2000, 64, 2);

            Data page0{}, page1{};
            if (unit.measureSingleshot(page0, page1)) {
                ring_buzzer(2000, 64);
                unit.writeLED(2, 10, 2);
                dump(page0);
                dump(page1);
                draw_values(page1);
            }
        } else {
            unit.writeLED(2, 2, 10);
            ring_buzzer(4000, 64);
            unit.startPeriodicMeasurement();
        }
    }
}

#if !defined(ARDUINO)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#if CONFIG_FREERTOS_UNICORE
// Single-core SoCs: run loop() back-to-back, but every ~2 s yield a 5 ms slice so the IDLE task
// runs and feeds the task watchdog (default 5 s).
static inline void feedIdleTaskPeriodically(void)
{
    constexpr uint32_t FEED_INTERVAL_MS{2000};
    constexpr TickType_t FEED_SLEEP_TICKS{pdMS_TO_TICKS(5)};
    static uint32_t s_last_feed_ms{};
    const uint32_t now_ms{static_cast<uint32_t>(esp_timer_get_time() / 1000)};
    if (now_ms - s_last_feed_ms >= FEED_INTERVAL_MS) {
        s_last_feed_ms = now_ms;
        vTaskDelay(FEED_SLEEP_TICKS);
    }
}
#endif

extern "C" void app_main(void)
{
    setup();
    for (;;) {
#if CONFIG_FREERTOS_UNICORE
        feedIdleTaskPeriodically();
#endif
        loop();
    }
}
#endif
