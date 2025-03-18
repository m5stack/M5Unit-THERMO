/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
  Example using M5UnitUnified for UnitThermal2
*/
#include <M5Unified.h>
#include <M5UnitUnified.h>
#include <M5UnitUnifiedTHERMO.h>
#include <M5Utility.h>

using namespace m5::unit::thermal2;

namespace {
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

constexpr float low_alarm_temp{10.0f};
constexpr float high_alarm_temp{30.0f};

void dump(const Data& d)
{
    M5.Log.printf("Subpage:%u\n", d.subpage);
    M5.Log.printf("Med:%.2f Avg:%.2f High:%.2f Low:%.2f\n", d.medianTemperature(), d.averageTemperature(),
                  d.highestTemperature(), d.lowestTemperature());

    M5.Log.printf("MostDiff(%u,%u) lowest{%u,%u} highest<%u,%u>\n", d.most_diff_x, d.most_diff_y, d.lowest_diff_x,
                  d.lowest_diff_y, d.highest_diff_x, d.highest_diff_y);

    M5.Log.printf(
        "    0000   0001   0002   0003   0004   0005   0006   0007   0008   0009   0010   0011   0012   0013   0014   "
        "0015   "
        "0016   0017   0018   0019   0020   0021   0022   0023   0024   0025   0026   0027   0028   0029   0030   "
        "0031\n");
    M5.Log.printf(
        "--------------------------------------------------------------------------------------------------------------"
        "------"
        "-------------------------------------------------------------------------------------------------------------"
        "\n");
    for (int y = 0; y < 24; ++y) {
        M5.Log.printf("%02d:%s", y, ((y & 1) != d.subpage) ? "       " : "");
        for (int x = 0; x < 16; ++x) {
            auto val = d.raw[y * 16 + x];
            int xx   = x * 2 + ((y & 1) != d.subpage);
            if (xx == d.most_diff_x && y == d.most_diff_y) {
                M5.Log.printf("(%04X) ", val);
            } else if (xx == d.lowest_diff_x && y == d.lowest_diff_y) {
                M5.Log.printf("{%04X} ", val);
            } else if (xx == d.highest_diff_x && y == d.highest_diff_y) {
                M5.Log.printf("<%04X> ", val);
            } else {
                M5.Log.printf(" %04X  ", val);
            }
            if (x != 15) {
                M5.Log.printf("       ");
            }
        }
        M5.Log.printf("\n");
    }
}

};  // namespace

void setup()
{
    M5.begin();

    auto pin_num_sda = M5.getPin(m5::pin_name_t::port_a_sda);
    auto pin_num_scl = M5.getPin(m5::pin_name_t::port_a_scl);
    M5_LOGI("getPin: SDA:%u SCL:%u", pin_num_sda, pin_num_scl);
    Wire.begin(pin_num_sda, pin_num_scl, 100 * 1000U);

    auto cfg = unit.config();
    cfg.rate = Refresh::Rate0_5Hz;  // Measure per 2 second
    unit.config(cfg);

    if (!Units.add(unit, Wire) || !Units.begin()) {
        M5_LOGE("Failed to begin");
        while (true) {
            m5::utility::delay(10000);
        }
    }
    M5_LOGI("M5UnitUnified has been begun");
    M5_LOGI("%s", Units.debugInfo().c_str());

    unit.writeAlarmEnabled(0);
    unit.writeBuzzerControl(false);
    unit.writeLED(2, 2, 10);
}

void loop()
{
    M5.update();
    Units.update();

    // Periodic
    if (unit.updated()) {
        dump(unit.oldest());
    }

    // Button on UnitThermal2 (toggle  periodic <-> single)
    if (unit.wasPressed()) {
        static bool single{};
        single = !single;

        if (single) {
            unit.stopPeriodicMeasurement();

            unit.writeLED(10, 2, 10);
            ring_buzzer(2000, 64, 2);

            Data page0{}, page1{};
            if (unit.measureSingleshot(page0, page1)) {
                ring_buzzer(2000, 64);
                unit.writeLED(2, 10, 2);
                dump(page0);
                dump(page1);
            }
        } else {
            unit.writeLED(2, 2, 10);
            ring_buzzer(4000, 64);
            unit.startPeriodicMeasurement();
        }
    }
}
