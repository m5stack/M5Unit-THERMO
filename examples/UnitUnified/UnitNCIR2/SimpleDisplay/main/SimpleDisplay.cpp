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
            RGBColor(0, 0, 0),       RGBColor(0, 0, 255),   RGBColor(0, 255, 0),   RGBColor(0, 255, 255),
            RGBColor(255, 0, 0),     RGBColor(255, 0, 255), RGBColor(255, 255, 0), RGBColor(255, 255, 255),
            RGBColor(128, 128, 128), RGBColor(64, 64, 64),
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

        auto ratio   = _now / (_max - _min);
        ratio        = std::fmax(std::fmin(1.0f, ratio), 0.0f);
        int32_t bwid = _sprite.width() * ratio;

        uint16_t clr = (_now < _low) ? 1 : (_now > _high) ? 4 : 2;
        _sprite.fillRect(0, 32, bwid, 24, clr);
        _sprite.fillRect(bwid, 32, _sprite.width() - bwid, 24, 9);
        _sprite.drawRect(0, 32, _sprite.width(), 24, 7);

        ratio = _low / (_max - _min);
        ratio = std::fmax(std::fmin(1.0f, ratio), 0.0f);
        bwid  = _sprite.width() * ratio;
        _sprite.drawFastVLine(bwid, 32, 56, 7);
        _sprite.setCursor(0, 88);
        _sprite.printf("L:%.2f", _low);

        ratio = _high / (_max - _min);
        ratio = std::fmax(std::fmin(1.0f, ratio), 0.0f);
        bwid  = _sprite.width() * ratio;
        _sprite.drawFastVLine(bwid, 32, 56, 7);
        _sprite.setTextDatum(top_right);
        _sprite.setCursor(bwid - 32, 88);
        auto s = m5::utility::formatString("H:%.2f", _high);
        _sprite.setTextDatum(top_right);
        _sprite.drawString(s.c_str(), _sprite.width(), 104);

        _sprite.setTextDatum(top_left);
        s = m5::utility::formatString("NCIR2 %s", _periodic ? "(IP)" : "");
        _sprite.drawString(s.c_str(), 0, 0);
        _sprite.setCursor(0, 16);
        _sprite.printf("%.2f", _min);

        s = m5::utility::formatString("%.2f", _max);
        _sprite.setTextDatum(top_right);
        _sprite.drawString(s.c_str(), _sprite.width(), 16);

        _sprite.setTextDatum(top_center);
        s = m5::utility::formatString("%.2f C", _now);
        _sprite.drawString(s.c_str(), _sprite.width() >> 1, 64);

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
    lcd.clear();
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
