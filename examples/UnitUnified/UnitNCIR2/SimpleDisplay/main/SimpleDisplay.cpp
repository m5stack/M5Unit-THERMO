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
#ifdef _min
#undef _min
#endif

using namespace m5::unit::ncir2;

constexpr float min_temp{-10.0f};
constexpr float max_temp{80.0f};
constexpr float low_temp{10.0f};
constexpr float high_temp{60.0f};

class View {
public:
    View(const float min_t, const float max_t, const float low, const float high, const LovyanGFX& lcd)
        : _min(min_t), _max{max_t}, _low{low}, _high{high}
    {
        _sprite.setPsram(false);
        _sprite.setColorDepth(4);  // 16 colors
        _sprite.createSprite(lcd.width(), 128);
        _sprite.setFont(&fonts::AsciiFont8x16);
        _sprite.setTextColor(7, 0);

        constexpr RGBColor palettes[16] = {
            RGBColor(0, 0, 0),       RGBColor(0, 80, 255),  RGBColor(0, 200, 0),   RGBColor(0, 255, 255),
            RGBColor(255, 40, 0),    RGBColor(255, 0, 255), RGBColor(255, 255, 0), RGBColor(255, 255, 255),
            RGBColor(160, 160, 160), RGBColor(48, 48, 48),
        };
        auto pal = _sprite.getPalette();
        for (auto&& p : palettes) {
            *pal++ = p;
        }
    }

    bool update()
    {
        if (!_counter) {
            return false;
        }
        if (--_counter) {
            _now += _add;
        } else {
            _now = _to;
        }

        _sprite.clear();

        const int32_t w       = _sprite.width();
        const int32_t margin  = 8;
        const int32_t bar_x   = margin;
        const int32_t bar_w   = w - margin * 2;
        const int32_t bar_y   = 56;
        const int32_t bar_h   = 14;
        const int32_t seg_w   = 4;
        const int32_t seg_gap = 1;
        const float range     = _max - _min;

        // Title
        _sprite.setTextDatum(top_left);
        auto s = m5::utility::formatString("NCIR2 %s", _periodic ? "(IP)" : "");
        _sprite.drawString(s.c_str(), margin, 0);

        // Temperature value (centered)
        _sprite.setTextDatum(top_center);
        s = m5::utility::formatString("%.2f C", _now);
        _sprite.drawString(s.c_str(), w >> 1, 20);

        // Triangle marker above bar
        float now_ratio = (_now - _min) / range;
        now_ratio       = std::fmax(std::fmin(1.0f, now_ratio), 0.0f);
        int32_t mx      = bar_x + static_cast<int32_t>(bar_w * now_ratio);
        _sprite.fillTriangle(mx - 4, bar_y - 8, mx + 4, bar_y - 8, mx, bar_y - 1, 7);

        // Segmented gauge bar
        float low_ratio  = (_low - _min) / range;
        float high_ratio = (_high - _min) / range;

        for (int32_t x = bar_x; x < bar_x + bar_w; x += seg_w + seg_gap) {
            int32_t sw      = std::min(seg_w, bar_x + bar_w - x);
            float seg_ratio = static_cast<float>(x - bar_x) / bar_w;

            uint16_t clr;
            if (seg_ratio <= now_ratio) {
                // Filled: color by zone
                if (seg_ratio < low_ratio) {
                    clr = 1;  // Blue (below Low)
                } else if (seg_ratio > high_ratio) {
                    clr = 4;  // Red (above High)
                } else {
                    clr = 2;  // Green (normal)
                }
            } else {
                clr = 9;  // Dark gray (empty)
            }
            _sprite.fillRect(x, bar_y, sw, bar_h, clr);
        }

        // Threshold tick marks
        int32_t low_x  = bar_x + static_cast<int32_t>(bar_w * std::fmax(std::fmin(1.0f, low_ratio), 0.0f));
        int32_t high_x = bar_x + static_cast<int32_t>(bar_w * std::fmax(std::fmin(1.0f, high_ratio), 0.0f));
        _sprite.drawFastVLine(low_x, bar_y + bar_h + 1, 6, 8);
        _sprite.drawFastVLine(high_x, bar_y + bar_h + 1, 6, 8);

        // Labels: min/max on first row, L/H on second row
        int32_t label_y = bar_y + bar_h + 8;
        _sprite.setTextDatum(top_left);
        s = m5::utility::formatString("%.0f", _min);
        _sprite.drawString(s.c_str(), bar_x, label_y);

        _sprite.setTextDatum(top_right);
        s = m5::utility::formatString("%.0f", _max);
        _sprite.drawString(s.c_str(), bar_x + bar_w, label_y);

        _sprite.setTextDatum(top_center);
        s = m5::utility::formatString("L:%.0f  H:%.0f", _low, _high);
        _sprite.drawString(s.c_str(), w >> 1, label_y + 16);

        _sprite.setTextDatum(top_left);
        return true;
    }

    void setPeriodic(const bool periodic)
    {
        _periodic = periodic;
    }

    void setTemp(const float temp)
    {
        _to      = temp;
        _counter = 8;
        _add     = (_to - _now) / _counter;
    }
    void push(LovyanGFX* dst, const int16_t x = 0, const int16_t y = 0)
    {
        _sprite.pushSprite(dst, x, y);
    }

private:
    uint32_t _counter{};
    float _min{}, _max{}, _low{}, _high{};
    float _now{}, _to{}, _add{};
    LGFX_Sprite _sprite{};
    bool _periodic{true};
};

namespace {
auto& lcd = M5.Display;
m5::unit::UnitUnified Units;
m5::unit::UnitNCIR2 unit;
View* view{};

void ring_buzzer(const uint16_t freq, const uint8_t duty, const uint32_t count = 1, const uint32_t interval = 50)
{
    for (uint16_t i = 0; i < count; ++i) {
        unit.writeBuzzer(freq, duty);
        unit.writeBuzzerControl(true);
        m5::utility::delay(interval);
        unit.writeBuzzerControl(false);
    }
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

    // No LCD or display device?
    if (lcd.width() == 0 || lcd.height() == 0 || lcd.isEPD()) {
        M5_LOGE("The core must be equipped with LCD");
        while (true) {
            m5::utility::delay(10000);
        }
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

    unit.writeLED(8, 32, 8);

    unit.writeAlarmTemperature(false, low_temp);
    unit.writeAlarmLED(false, 8, 8, 32);
    unit.writeAlarmBuzzer(false, 1000, 200, 204);

    unit.writeAlarmTemperature(true, high_temp);
    unit.writeAlarmLED(true, 32, 8, 8);
    unit.writeAlarmBuzzer(true, 2000, 100, 204);

    unit.writeConfig();

    //
    view = new View(min_temp, max_temp, low_temp, high_temp, lcd);
    assert(view);

    lcd.setFont(&fonts::AsciiFont8x16);
    lcd.startWrite();
    lcd.fillScreen(TFT_BLACK);
}

void loop()
{
    static float temp{}, ptemp{};

    M5.update();
    Units.update();

    // Periodic
    if (unit.updated()) {
        temp = unit.temperature();
        M5.Log.printf(">Temp:%.2f\n", temp);
    }

    // Toggle between periodic and single (Use the button on UnitNCIR2)
    if (unit.wasReleased()) {
        static bool single{};
        single = !single;
        view->setPeriodic(!single);
        view->push(&lcd);

        if (single) {
            ring_buzzer(2000, 204);
            unit.writeLED(32, 8, 32);
            unit.stopPeriodicMeasurement();

            Data d{};
            unit.measureSingleshot(d);
            M5.Log.printf("Single:%.2f\n", d.celsius());
        } else {
            ring_buzzer(2000, 204);
            unit.writeLED(8, 32, 8);
            unit.startPeriodicMeasurement();
        }
    }

    if ((int32_t)(temp * 100) != (int32_t)(ptemp * 100)) {
        ptemp = temp;
        view->setTemp(temp);
    }
    if (view->update()) {
        view->push(&lcd);
    }
}
