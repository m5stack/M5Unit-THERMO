/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Simple display example using M5UnitUnified for UnitThermal2
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTHERMO.h>
#include <M5Utility.h>
#include <cmath>

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

// Rainbow 256 paletts
constexpr const uint32_t color_table[256] = {
    0x0000FFu, 0x0003FFu, 0x0006FFu, 0x0009FFu, 0x000CFFu, 0x000FFFu, 0x0012FFu, 0x0016FFu,  // 0
    0x0019FEu, 0x001CFEu, 0x001FFEu, 0x0022FDu, 0x0025FDu, 0x0028FCu, 0x002BFCu, 0x002FFBu,  //
    0x0032FBu, 0x0035FAu, 0x0038F9u, 0x003BF9u, 0x003EF8u, 0x0041F7u, 0x0044F6u, 0x0047F6u,  //
    0x004AF5u, 0x004DF4u, 0x0050F3u, 0x0053F2u, 0x0056F1u, 0x0059F0u, 0x005CEFu, 0x005FEEu,  //
    0x0062ECu, 0x0065EBu, 0x0068EAu, 0x006AE9u, 0x006DE7u, 0x0070E6u, 0x0073E5u, 0x0076E3u,  //
    0x0079E2u, 0x007BE0u, 0x007EDFu, 0x0081DDu, 0x0084DCu, 0x0086DAu, 0x0089D8u, 0x008CD7u,  //
    0x008ED5u, 0x0091D3u, 0x0093D1u, 0x0096CFu, 0x0098CEu, 0x009BCCu, 0x009DCAu, 0x00A0C8u,  //
    0x00A2C6u, 0x00A5C4u, 0x00A7C2u, 0x00AAC0u, 0x00ACBEu, 0x00AEBCu, 0x00B1B9u, 0x00B3B7u,  //
    0x00B5B5u, 0x00B7B3u, 0x00B9B1u, 0x00BCAEu, 0x00BEACu, 0x00C0AAu, 0x00C2A7u, 0x00C4A5u,  //
    0x00C6A2u, 0x00C8A0u, 0x00CA9Du, 0x00CC9Bu, 0x00CE98u, 0x00CF96u, 0x00D193u, 0x00D391u,  //
    0x00D58Eu, 0x00D78Cu, 0x00D889u, 0x00DA86u, 0x00DC84u, 0x00DD81u, 0x00DF7Eu, 0x00E07Bu,  // 80
    0x00E279u, 0x00E376u, 0x00E573u, 0x00E670u, 0x00E76Du, 0x00E96Au, 0x00EA68u, 0x00EB65u,  //
    0x00EC62u, 0x00EE5Fu, 0x00EF5Cu, 0x00F059u, 0x00F156u, 0x00F253u, 0x00F350u, 0x00F44Du,  //
    0x00F54Au, 0x00F647u, 0x00F644u, 0x00F741u, 0x00F83Eu, 0x00F93Bu, 0x00F938u, 0x00FA35u,  //
    0x00FB32u, 0x00FB2Fu, 0x00FC2Bu, 0x00FC28u, 0x00FD25u, 0x00FD22u, 0x00FE1Fu, 0x00FE1Cu,  //
    0x00FE19u, 0x00FF16u, 0x00FF12u, 0x00FF0Fu, 0x00FF0Cu, 0x00FF09u, 0x00FF06u, 0x00FF03u,  //
    0x03FF00u, 0x06FF00u, 0x09FF00u, 0x0CFF00u, 0x0FFF00u, 0x12FF00u, 0x16FF00u, 0x19FE00u,  //
    0x1CFE00u, 0x1FFE00u, 0x22FD00u, 0x25FD00u, 0x28FC00u, 0x2BFC00u, 0x2FFB00u, 0x32FB00u,  //
    0x35FA00u, 0x38F900u, 0x3BF900u, 0x3EF800u, 0x41F700u, 0x44F600u, 0x47F600u, 0x4AF500u,  //
    0x4DF400u, 0x50F300u, 0x53F200u, 0x56F100u, 0x59F000u, 0x5CEF00u, 0x5FEE00u, 0x62EC00u,  //
    0x65EB00u, 0x68EA00u, 0x6AE900u, 0x6DE700u, 0x70E600u, 0x73E500u, 0x76E300u, 0x79E200u,  // 160
    0x7BE000u, 0x7EDF00u, 0x81DD00u, 0x84DC00u, 0x86DA00u, 0x89D800u, 0x8CD700u, 0x8ED500u,  //
    0x91D300u, 0x93D100u, 0x96CF00u, 0x98CE00u, 0x9BCC00u, 0x9DCA00u, 0xA0C800u, 0xA2C600u,  //
    0xA5C400u, 0xA7C200u, 0xAAC000u, 0xACBE00u, 0xAEBC00u, 0xB1B900u, 0xB3B700u, 0xB5B500u,  //
    0xB7B300u, 0xB9B100u, 0xBCAE00u, 0xBEAC00u, 0xC0AA00u, 0xC2A700u, 0xC4A500u, 0xC6A200u,  //
    0xC8A000u, 0xCA9D00u, 0xCC9B00u, 0xCE9800u, 0xCF9600u, 0xD19300u, 0xD39100u, 0xD58E00u,  //
    0xD78C00u, 0xD88900u, 0xDA8600u, 0xDC8400u, 0xDD8100u, 0xDF7E00u, 0xE07B00u, 0xE27900u,  //
    0xE37600u, 0xE57300u, 0xE67000u, 0xE76D00u, 0xE96A00u, 0xEA6800u, 0xEB6500u, 0xEC6200u,  //
    0xEE5F00u, 0xEF5C00u, 0xF05900u, 0xF15600u, 0xF25300u, 0xF35000u, 0xF44D00u, 0xF54A00u,  //
    0xF64700u, 0xF64400u, 0xF74100u, 0xF83E00u, 0xF93B00u, 0xF93800u, 0xFA3500u, 0xFB3200u,  //
    0xFB2F00u, 0xFC2B00u, 0xFC2800u, 0xFD2500u, 0xFD2200u, 0xFE1F00u, 0xFE1C00u, 0xFE1900u,  // 240
    0xFF1600u, 0xFF1200u, 0xFF0F00u, 0xFF0C00u, 0xFF0900u, 0xFF0600u, 0xFF0300u, 0xFF0000u,  //
};

class HeatmapView {
public:
    HeatmapView(const uint32_t wid, const uint32_t hgt) : _wid{wid}, _hgt{hgt}
    {
        _rwid = _wid / 32;
        _rhgt = _hgt / 24;
        // M5_LOGI("  <%d,%d>", _rwid, _rhgt);

        assert(_rwid > 3 && _rhgt > 3);
        _sprite.setPsram(false);
        _sprite.setColorDepth(8);  // 256 colors
        auto r = _sprite.createSprite(_wid, _hgt);
        assert(r);
        _sprite.createPalette(color_table, m5::stl::size(color_table));
    }
    uint32_t width() const
    {
        return _wid;
    }
    uint32_t height() const
    {
        return _hgt;
    }
    void apply(const Data& d)
    {
        float lowest_temp  = d.lowestTemperature();
        float highest_temp = d.highestTemperature();
        float temp_diff    = highest_temp - lowest_temp;

        for (int idx = 0; idx < 384; ++idx) {
            int y     = idx >> 4;
            int x     = ((idx & 15) << 1) + ((y & 1) != d.subpage);
            float t   = d.temperature(idx) - lowest_temp;
            int level = t * 256 / temp_diff;
            level     = std::max(std::min(255, level), 0);

            _sprite.fillRect(x * _rwid, y * _rhgt, _rwid, _rhgt, level /* palette number */);
        }
        _sprite.drawRect(d.most_diff_x * _rwid + 1, d.most_diff_y * _rhgt + 1, _rwid - 2, _rhgt - 2, 128);
        _sprite.drawRect(d.highest_diff_x * _rwid, d.highest_diff_y * _rhgt, _rwid, _rhgt, 0);
        _sprite.drawRect(d.lowest_diff_x * _rwid, d.lowest_diff_y * _rhgt, _rwid, _rhgt, 255);
    }

    void clear()
    {
        _sprite.clear();
    }

    void push(LovyanGFX* dst, const int16_t x = 0, const int16_t y = 0)
    {
        _sprite.pushSprite(dst, x, y);
    }

private:
    LGFX_Sprite _sprite{};
    uint32_t _wid{}, _hgt{};
    uint32_t _rwid{}, _rhgt{};
};

HeatmapView* view{};
uint32_t text_x{}, text_y{};

void calculate_4_3(uint32_t& ow, uint32_t& oh, const uint32_t mw, const uint32_t mh)
{
    uint32_t width  = mw;
    uint32_t height = (width * 3) >> 2;
    if (height > mh) {
        height = mh;
        width  = (height << 2) / 3;
        if (width > mw) {
            width  = mw;
            height = (width * 3) / 4;
        }
    }
    ow = width;
    oh = height;
}

// Calculate maximum heatmap size and text display position
bool calculate_heatmap_size(uint32_t& ow, uint32_t& oh, const uint32_t w, const uint32_t h, const uint32_t fw = 8,
                            const uint32_t fh = 16)
{
    const uint32_t ewid{fw * 12};
    const uint32_t ehgt{fh * 4};
    uint32_t heatW0{}, heatH0{};
    uint32_t heatW1{}, heatH1{};

    calculate_4_3(heatW0, heatH0, w - ewid, h);
    calculate_4_3(heatW1, heatH1, w, h - ehgt);

    heatW0 = (heatW0 >> 5) << 5;
    heatH0 = (heatW0 * 3) >> 2;
    heatW1 = (heatW1 >> 5) << 5;
    heatH1 = (heatW1 * 3) >> 2;

    auto area0 = heatW0 * heatH0;
    auto area1 = heatW1 * heatH1;
    if (area0 >= area1) {
        ow = heatW0;
        oh = heatH0;
        return true;  // text area is right
    }
    ow = heatW1;
    oh = heatH1;
    return false;  // text area is bottom
}

constexpr float low_alarm_temp{0.0f};
constexpr float high_alarm_temp{50.0f};

static uint32_t text_color_table[] = {0x00000000, 0x00808080u, 0x00008000u, 0x00FFCF00u, 0x0000CFFFu};
LGFX_Sprite text{};
};  // namespace

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

    //
    unit.writeBuzzer(2000, 64);
    unit.writeBuzzerControl(false);
    unit.writeLED(2, 2, 10);

    unit.writeAlarmTemperature(false, low_alarm_temp);
    unit.writeAlarmLED(false, 2, 4, 20);
    unit.writeAlarmBuzzer(false, 4000, 20);
    unit.writeAlarmTemperature(true, high_alarm_temp);
    unit.writeAlarmLED(true, 20, 4, 2);
    unit.writeAlarmBuzzer(true, 2000, 10);
    unit.writeAlarmEnabled(enabled_ave_temperature_low | enabled_ave_temperature_high);

    // Sprites
    if (lcd.width() >= 240) {
        lcd.setFont(&fonts::AsciiFont8x16);
    }
    uint32_t w{}, h{};
    auto right_text = calculate_heatmap_size(w, h, lcd.width(), lcd.height(), lcd.fontWidth(), lcd.fontHeight());
    M5_LOGI("WH:%u,%u %u,%u %d", w, h, text_x, text_y, right_text);
    text_x = right_text ? w : 0;
    text_y = right_text ? 0 : h;
    view   = new HeatmapView(w, h);

    text.setPsram(false);
    text.setColorDepth(4);  // 16 colors
    auto r = text.createSprite(lcd.fontWidth() * 12, lcd.fontHeight() * 4);
    assert(r);
    text.createPalette(text_color_table, m5::stl::size(text_color_table));
    text.setFont(lcd.getFont());

    lcd.startWrite();
}

void loop()
{
    M5.update();
    Units.update();

    // Periodic
    if (unit.updated()) {
        static const char* tag[4] = {"Med ", "Avg ", "High", "Low "};

        auto d          = unit.oldest();
        float values[4] = {d.medianTemperature(), d.averageTemperature(), d.highestTemperature(),
                           d.lowestTemperature()};
        text.clear(0);
        for (int i = 0; i < 4; ++i) {
            text.setCursor(0, text.fontHeight() * i);
            text.setTextColor(i + 1);
            text.printf("%s:%.2f", tag[i], values[i]);
        }
        view->apply(d);
        view->push(&lcd);
        text.pushSprite(&lcd, text_x, text_y);
    }

    // Button (toggle  periodic <-> single)
    if (unit.wasPressed()) {
        static bool single{};
        single = !single;

        if (single) {
            view->clear();
            view->push(&lcd);

            unit.stopPeriodicMeasurement();
            unit.writeAlarmEnabled(0x00);

            unit.writeLED(10, 2, 10);
            ring_buzzer(2000, 64, 2);

            Data page0{}, page1{};
            if (unit.measureSingleshot(page0, page1)) {
                ring_buzzer(2000, 64);
                unit.writeLED(2, 10, 2);

                view->apply(page0);
                view->apply(page1);
                view->push(&lcd);
            } else {
                M5_LOGE("error");
            }
        } else {
            unit.writeLED(2, 2, 10);
            ring_buzzer(4000, 64);
            unit.writeAlarmEnabled(enabled_ave_temperature_low | enabled_ave_temperature_high);
            unit.startPeriodicMeasurement();
        }
    }
}
