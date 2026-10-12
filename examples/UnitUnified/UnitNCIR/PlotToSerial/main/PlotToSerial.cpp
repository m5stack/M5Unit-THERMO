/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitNCIR
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTHERMO.h>
#include <M5Utility.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

using namespace m5::unit::mlx90614;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitNCIR unit;
}  // namespace

void setup()
{
    M5.begin();
    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    // Board-aware connection: NessoN1 -> SoftwareI2C on GROVE, NanoC6/NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, unit) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        m5::unit::wiring::failStop();
    }
    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());

    lcd.setFont(&fonts::AsciiFont8x16);
    lcd.fillScreen(TFT_DARKGREEN);
    lcd.fillRect(8, 8, 8 * 10, 16 * 3, TFT_BLACK);
}

void loop()
{
    M5.update();

    // Periodic
    Units.update();
    if (unit.updated()) {
        M5.Log.printf(">Amb:%f\n>Obj1:%f\n>Obj2:%f\n", unit.ambientTemperature(), unit.objectTemperature1(),
                      unit.objectTemperature2());

        lcd.startWrite();
        lcd.fillRect(8, 8, 8 * 10, 16 * 3, TFT_BLACK);
        lcd.setCursor(8, 8 + 16 * 0);
        lcd.printf("A:%.2f", unit.ambientTemperature());
        lcd.setCursor(8, 8 + 16 * 1);
        lcd.printf("1:%.2f", unit.objectTemperature1());
        lcd.setCursor(8, 8 + 16 * 2);
        lcd.printf("2:%.2f", unit.objectTemperature2());
        lcd.endWrite();
    }

    // Change measure target
    if (M5.BtnA.wasClicked()) {
        unit.stopPeriodicMeasurement();

        static bool b{};
        b = !b;
        if (b) {
            M5_LOGI("Enable object2");
            unit.writeIRSensor(IRSensor::Dual, false);  // Enable object 2 measuring
            unit.writeEmissivity(0.25f, false);
            unit.applySettings();
        } else {
            M5_LOGI("Disable object2");
            unit.writeIRSensor(IRSensor::Single, false);  // Disable object 2 measuring
            unit.writeEmissivity(1.0f, false);
            unit.applySettings();
        }
        unit.startPeriodicMeasurement();
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
