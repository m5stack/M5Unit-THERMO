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
#include <M5HAL.hpp>

using namespace m5::unit::ncir2;

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitNCIR2 unit;

// Temperature range for LED color mapping (Blue: min_temp -> Red: max_temp)
// Adjust these values to suit your measurement environment
constexpr float min_temp{20.0f};  // Lower bound (Celsius): LED turns Blue
constexpr float max_temp{50.0f};  // Upper bound (Celsius): LED turns Red

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
    M5.setTouchButtonHeightByRatio(100);
    // The screen shall be in landscape mode
    if (lcd.height() > lcd.width()) {
        lcd.setRotation(1);
    }

    auto board = M5.getBoard();

    // NessoN1: Arduino Wire (I2C_NUM_0) cannot be used for GROVE port.
    //   Wire is used by M5Unified In_I2C for internal devices (IOExpander etc.).
    //   Wire1 exists but is reserved for HatPort — cannot be used for GROVE.
    //   Reconfiguring Wire to GROVE pins breaks In_I2C, causing ESP_ERR_INVALID_STATE in M5.update().
    //   Solution: Use SoftwareI2C via M5HAL (bit-banging) for the GROVE port.
    // NanoC6: Wire.begin() on GROVE pins conflicts with m5::I2C_Class registered by Ex_I2C.setPort()
    //   on the same I2C_NUM_0, causing sporadic NACK errors.
    //   Solution: Use M5.Ex_I2C (m5::I2C_Class) directly instead of Arduino Wire.
    bool unit_ready{};
    if (board == m5::board_t::board_ArduinoNessoN1) {
        // NessoN1: GROVE is on port_b (GPIO 5/4), not port_a (which maps to Wire pins 8/10)
        auto pin_num_sda = M5.getPin(m5::pin_name_t::port_b_out);
        auto pin_num_scl = M5.getPin(m5::pin_name_t::port_b_in);
        M5_LOGI("getPin(M5HAL): SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        m5::hal::bus::I2CBusConfig i2c_cfg;
        i2c_cfg.pin_sda = m5::hal::gpio::getPin(pin_num_sda);
        i2c_cfg.pin_scl = m5::hal::gpio::getPin(pin_num_scl);
        auto i2c_bus    = m5::hal::bus::i2c::getBus(i2c_cfg);
        M5_LOGI("Bus:%d", i2c_bus.has_value());
        unit_ready = Units.add(unit, i2c_bus ? i2c_bus.value() : nullptr) && Units.begin();
    } else if (board == m5::board_t::board_M5NanoC6) {
        // NanoC6: Use M5.Ex_I2C (m5::I2C_Class, not Arduino Wire)
        M5_LOGI("Using M5.Ex_I2C");
        unit_ready = Units.add(unit, M5.Ex_I2C) && Units.begin();
    } else {
        auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
        auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
        M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
        Wire.end();
        Wire.begin(pin_num_sda, pin_num_scl, 100 * 1000U);
        unit_ready = Units.add(unit, Wire) && Units.begin();
    }
    if (!unit_ready) {
        M5_LOGE("Failed to begin");
        lcd.fillScreen(TFT_RED);
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

    lcd.fillScreen(TFT_DARKGREEN);
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
        M5.Log.printf("NCIR2 Button pressed\n");
    }
    if (unit.wasReleased()) {
        M5.Log.printf("NCIR2 Button released\n");
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

    // Change LED color min_temp:Blue <- temp -> max_temp:Red (updates every 1°C)
    if ((int32_t)ptemp != (int32_t)temp) {
        static uint32_t prgb{};
        ptemp        = temp;
        auto ratio   = (temp - min_temp) / (max_temp - min_temp);
        ratio        = std::fmax(std::fmin(1.0f, ratio), 0.0f);
        auto h       = 240.f * (1.0f - ratio);
        uint32_t rgb = HSV_to_RGB(h, 1.0f, 1.0f);
        if (prgb != rgb) {
            prgb = rgb;
            // M5_LOGI("LED %08X", rgb);
            unit.writeLED(rgb);
        }
    }
}
