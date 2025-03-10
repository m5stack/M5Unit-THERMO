/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitNCIR2
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTHERMO.h>
#include <M5Utility.h>

using namespace m5::unit::ncir2;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitNCIR2 unit;

constexpr float min_temp{0.0f};
constexpr float max_temp{100.0f};

void ring_buzzer(const uint16_t freq, const uint8_t duty, const uint32_t count = 1, const uint32_t interval = 50)
{
    for (uint16_t i = 0; i < count; ++i) {
        unit.writeBuzzer(freq, duty);
        unit.writeBuzzerControl(true);
        m5::utility::delay(interval);
        unit.writeBuzzerControl(false);
    }
}

uint32_t HSV_to_RGB(const float h, const float s, const float v)
{
    float c = v * s;
    float x = c * (1 - std::fabs(std::fmod(h / 60.0f, 2) - 1));
    float m = v - c;
    float r{}, g{}, b{};

    if (h < 60) {
        r = c, g = x, b = 0;
    } else if (h < 120) {
        r = x, g = c, b = 0;
    } else if (h < 180) {
        r = 0, g = c, b = x;
    } else if (h < 240) {
        r = 0, g = x, b = c;
    } else if (h < 300) {
        r = x, g = 0, b = c;
    } else {
        r = c, g = 0, b = x;
    }
    return (static_cast<uint8_t>((r + m) * 255) << 16) | (static_cast<uint8_t>((g + m) * 255) << 8) |
           static_cast<uint8_t>((b + m) * 255);
}

}  // namespace

void setup()
{
    M5.begin();

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

    float e{};
    unit.readEmissivity(e);
    M5.Log.printf("Emissivity:%.2f\n", e);

    unit.writeLED(0, 0, 0);
    unit.writeAlarmBuzzer(false, 0, 5000, 0);
    unit.writeAlarmBuzzer(true, 0, 5000, 0);
    unit.writeConfig();

    ring_buzzer(4000, 204);
}

void loop()
{
    static float ptemp{}, temp{};

    M5.update();
    Units.update();

    // Periodic
    if (unit.updated()) {
        temp = unit.temperature();
        Data ctemp{};
        unit.readChipTemperature(ctemp);
        M5.Log.printf(">Temp:%.2f\n>Chip:%.2f\n", temp, ctemp.temperature());
    }

    // Button
    if (unit.wasPressed()) {
        M5.Log.printf("Button pressed\n");
    }
    if (unit.wasReleased()) {
        M5.Log.printf("Button released\n");
    }

    // Toggle between periodic and single (Use the button on UnitNCIR2)
    if (unit.wasReleased()) {
        static bool single{};
        single = !single;

        if (single) {
            ring_buzzer(2000, 204);

            unit.writeLED(32, 8, 32);
            unit.stopPeriodicMeasurement();

            Data d{};
            unit.measureSingleshot(d);
            temp = d.temperature();
            M5.Log.printf("Single:%.2f\n", temp);
        } else {
            ring_buzzer(2000, 204);

            unit.startPeriodicMeasurement();
            ptemp = min_temp;
        }
    }

    // Change LED color min_tenp:Blue <- temp -> max_temp:Red
    if ((int32_t)(ptemp * 100) != (int32_t)(temp * 100)) {
        static uint32_t prgb{};
        ptemp        = temp;
        auto ratio   = temp / (max_temp - min_temp);
        ratio        = std::fmax(std::fmin(1.0f, ratio), 0.0f);
        auto h       = 240.f * (1.0f - ratio);
        uint32_t rgb = HSV_to_RGB(h, 1.0f, 1.0f);
        if (prgb != rgb) {
            prgb = rgb;
            unit.writeLED(rgb);
        }
    }
}
