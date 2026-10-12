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

    // The hub is the I2C device on the port
    // NessoN1 -> SoftwareI2C (M5HAL), NanoC6 / NanoH2 -> M5.Ex_I2C, others -> Wire
    if (!m5::unit::wiring::addI2C(Units, hub) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        M5_LOGW("%s", Units.debugInfo().c_str());
        m5::unit::wiring::failStop();
    }

    M5_LOGI("M5UnitUnified initialized");
    M5_LOGI("%s", Units.debugInfo().c_str());
    lcd.fillScreen(TFT_DARKGREEN);
}

void loop()
{
    M5.update();
    Units.update();

    if (ncir.updated()) {
        M5.Log.printf(">NCIR_Amb:%f\n>NCIR_Obj:%f\n", ncir.ambientTemperature(), ncir.objectTemperature1());
    }
    if (ncir2.updated()) {
        M5.Log.printf(">NCIR2:%f\n", ncir2.temperature());
    }
    if (thermal2.updated()) {
        const auto& d = thermal2.oldest();
        M5.Log.printf(">Thermal2_Avg:%f\n>Thermal2_High:%f\n>Thermal2_Low:%f\n", d.averageTemperature(),
                      d.highestTemperature(), d.lowestTemperature());
    }
}
