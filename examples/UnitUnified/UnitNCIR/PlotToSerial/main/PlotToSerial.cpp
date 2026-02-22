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
#include <M5HAL.hpp>

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

    auto board       = M5.getBoard();
    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    if (board == m5::board_t::board_ArduinoNessoN1) {
        pin_num_sda = M5.getPin(m5::pin_name_t::port_b_out);
        pin_num_scl = M5.getPin(m5::pin_name_t::port_b_in);
        M5_LOGI("getPin(NessoN1): SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        m5::hal::bus::I2CBusConfig i2c_cfg;
        i2c_cfg.pin_sda = m5::hal::gpio::getPin(pin_num_sda);
        i2c_cfg.pin_scl = m5::hal::gpio::getPin(pin_num_scl);
        auto i2c_bus    = m5::hal::bus::i2c::getBus(i2c_cfg);
        if (!Units.add(unit, i2c_bus ? i2c_bus.value() : nullptr) || !Units.begin()) {
            M5_LOGE("Failed to begin");
            lcd.fillScreen(TFT_RED);
            while (true) {
                m5::utility::delay(10000);
            }
        }
    } else {
        M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        Wire.end();
        Wire.begin(pin_num_sda, pin_num_scl, 100 * 1000U);
        if (!Units.add(unit, Wire) || !Units.begin()) {
            M5_LOGE("Failed to begin");
            lcd.fillScreen(TFT_RED);
            while (true) {
                m5::utility::delay(10000);
            }
        }
    }
    M5_LOGI("M5UnitUnified has been begun");
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
