/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example of using the thermo units via UnitPaHub (PaHub2)

  Device ---> PaHub2 (0x70)
                |- ch:0 UnitNCIR     (0x5A)
                |- ch:1 UnitNCIR2    (0x5A)
                `- ch:2 UnitThermal2 (0x32)

  Each PaHub channel is an independent I2C bus, so UnitNCIR and UnitNCIR2 can be used together
  even though both have the same address (0x5A).
*/
#include <algorithm>
#include <cstdio>

#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedHUB.h>
#include <M5UnitUnifiedTHERMO.h>
#include <wiring/m5_unit_unified_wiring.hpp>  // Board-aware connection helpers (include last)

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;

// The default address of PaHub2 is 0x70 (changeable with the A0-A2 pins (v2.0) or the DIP switch (v2.1))
// https://docs.m5stack.com/en/unit/pahub2
m5::unit::UnitPaHub2 hub{};

m5::unit::UnitNCIR ncir{};
m5::unit::UnitNCIR2 ncir2{};
m5::unit::UnitThermal2 thermal2{};

// PaHub channels used by each unit
constexpr uint8_t CH_NCIR{0};
constexpr uint8_t CH_NCIR2{1};
constexpr uint8_t CH_THERMAL2{2};

// Text layout: up to ROWS lines of up to COLUMNS characters, scaled to fit the screen
// (AtomS3 128x128, StickC 160x80, Core 320x240, Tab5 1280x720, ...)
constexpr int32_t ROWS{6};
constexpr int32_t COLUMNS{14};

bool has_display()
{
    return lcd.width() > 0 && lcd.height() > 0;
}

void setup_text()
{
    if (!has_display()) {
        return;
    }
    lcd.setFont(&fonts::Font0);  // 6x8
    const int32_t sx{lcd.width() / (COLUMNS * 6)};
    const int32_t sy{lcd.height() / (ROWS * 8)};
    lcd.setTextSize(std::max<int32_t>(1, std::min(sx, sy)));
    lcd.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    lcd.fillScreen(TFT_DARKGREEN);
}

void draw_row(const int32_t row, const char* text)
{
    if (!has_display()) {
        return;
    }
    const int32_t h{lcd.fontHeight()};
    lcd.fillRect(0, row * h, lcd.width(), h, TFT_DARKGREEN);
    lcd.drawString(text, 0, row * h);
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

    // Thermal2 refresh rate (must be set before begin)
    auto cfg = thermal2.config();
    cfg.rate = m5::unit::thermal2::Refresh::Rate2Hz;
    thermal2.config(cfg);

    if (!hub.add(ncir, CH_NCIR) || !hub.add(ncir2, CH_NCIR2) || !hub.add(thermal2, CH_THERMAL2)) {
        M5_LOGE("Failed to add");
        m5::unit::wiring::failStop();
    }

    // Run the PaHub at the lowest clock of the units behind it: its channel select is sent at that clock, and a
    // read on its own runs at the clock left by the last transfer
    auto hcfg = hub.component_config();
    hcfg.clock =
        std::min(hcfg.clock, std::min(ncir.component_config().clock,
                                      std::min(ncir2.component_config().clock, thermal2.component_config().clock)));
    hub.component_config(hcfg);

    // The hub is the I2C device on the port
    // NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, hub) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        M5_LOGW("%s", Units.debugInfo().c_str());
        m5::unit::wiring::failStop();
    }

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());
    setup_text();
}

void loop()
{
    M5.update();
    Units.update();

    char buf[64]{};
    if (ncir.updated()) {
        M5.Log.printf(">NCIR_Amb:%f\n>NCIR_Obj:%f\n", ncir.ambientTemperature(), ncir.objectTemperature1());
        snprintf(buf, sizeof(buf), "NCIR  A:%.2f", ncir.ambientTemperature());
        draw_row(0, buf);
        snprintf(buf, sizeof(buf), "      O:%.2f", ncir.objectTemperature1());
        draw_row(1, buf);
    }
    if (ncir2.updated()) {
        M5.Log.printf(">NCIR2:%f\n", ncir2.temperature());
        snprintf(buf, sizeof(buf), "NCIR2   %.2f", ncir2.temperature());
        draw_row(2, buf);
    }
    if (thermal2.updated()) {
        const auto& d = thermal2.oldest();
        M5.Log.printf(">Thermal2_Avg:%f\n>Thermal2_High:%f\n>Thermal2_Low:%f\n", d.averageTemperature(),
                      d.highestTemperature(), d.lowestTemperature());
        snprintf(buf, sizeof(buf), "T2 Avg:%.2f", d.averageTemperature());
        draw_row(3, buf);
        snprintf(buf, sizeof(buf), "   Hi:%.2f", d.highestTemperature());
        draw_row(4, buf);
        snprintf(buf, sizeof(buf), "   Lo:%.2f", d.lowestTemperature());
        draw_row(5, buf);
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
