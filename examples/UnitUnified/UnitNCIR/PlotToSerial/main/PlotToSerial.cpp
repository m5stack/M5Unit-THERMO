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

using namespace m5::unit::mlx90614;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitNCIR unit;

uint32_t idx{};
constexpr Output out_table[] = {
    Output::TA_TO1,
    Output::TA_TO2,
    Output::TO2_Undefined,
    Output::TO1_TO2,
};
const char* os_table[] = {"TA_TO1", "TA_To2", "TO2", "TO1/2"};

}  // namespace

void setup()
{
    M5.begin();

    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 100 * 1000U);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        lcd.clear(TFT_RED);
        while (true) {
            m5::utility::delay(10000);
        }
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    {
        unit.stopPeriodicMeasurement();

        unit.writeIRSensor(IRSensor::Dual);  // To enable measuring To2
        unit.writeOutput(out_table[idx]);

        unit.startPeriodicMeasurement();
    }

    lcd.setFont(&fonts::AsciiFont8x16);
    lcd.clear(TFT_DARKGREEN);
}

void loop()
{
    M5.update();
    auto touch = M5.Touch.getDetail();

    // Periodic
    Units.update();
    if (unit.updated()) {
        M5_LOGI("\n>Amb:%f\n>Obj1:%f\n>Obj2:%f", unit.ambientTemperature(), unit.objectTemperature1(),
                unit.objectTemperature2());

        lcd.fillRect(0, 16, lcd.width(), 16 * 4, TFT_DARKGREEN);
        lcd.setCursor(8, 16 * 1);
        lcd.printf("%s:", os_table[idx]);
        lcd.setCursor(8, 16 * 2);
        lcd.printf("A:%.2f", unit.ambientTemperature());
        lcd.setCursor(8, 16 * 3);
        lcd.printf("1:%.2f", unit.objectTemperature1());
        lcd.setCursor(8, 16 * 4);
        lcd.printf("2:%.2f", unit.objectTemperature2());
    }

    // Change measure target
    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        ++idx;
        idx &= 0x03;
        unit.stopPeriodicMeasurement();
        unit.writeOutput(out_table[idx]);
        unit.startPeriodicMeasurement();
    }

#if 0
    // Single
    if (M5.BtnA.wasClicked() || touch.wasClicked()) {
        unit.stopPeriodicMeasurement();
        Data d{};
        if (unit.measureSingleshot(d)) {
            M5_LOGI("Single: %d/%f", d.differentialValue(), d.differentialVoltage());
        }
        unit.startPeriodicMeasurement();
    }
#endif
    //    m5::utility::delay(1000);
}
