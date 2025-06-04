/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_NCIR2.cpp
  @brief NCIR2 Unit for M5UnitUnified
*/
#include "unit_NCIR2.hpp"
#include <M5Utility.hpp>
#include <array>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::ncir2;
using namespace m5::unit::ncir2::command;

namespace {

inline float raw_to_duty(const uint8_t x)
{
    return (x < 128) ? x / 127.0f * 0.5f : ((x - 127) / 128.0f) * 0.5f + 0.5f;
}

inline uint8_t duty_to_raw(const float f)
{
    return f <= 0.5f ? static_cast<uint8_t>(f * 255.0f) : static_cast<float>(127 + 128.0f * (2 * (f - 0.5f)));
}

}  // namespace

namespace m5 {
namespace unit {
// class UnitNCIR2
const char UnitNCIR2::name[] = "UnitNCIR2";
const types::uid_t UnitNCIR2::uid{"UnitNCIR2"_mmh3};
const types::attr_t UnitNCIR2::attr{attribute::AccessI2C};

bool UnitNCIR2::begin()
{
    auto ssize = stored_size();
    assert(ssize && "stored_size must be greater than zero");
    if (ssize != _data->capacity()) {
        _data.reset(new m5::container::CircularBuffer<Data>(ssize));
        if (!_data) {
            M5_LIB_LOGE("Failed to allocate");
            return false;
        }
    }

    uint8_t ver{};
    if (!readFirmwareVersion(ver) || ver == 0) {
        M5_LIB_LOGE("Cannot detect NCIR2 %02X", ver);
        return false;
    }

    _button_interval = _cfg.button_interval;
    return _cfg.start_periodic ? startPeriodicMeasurement(_cfg.interval) : true;
}

void UnitNCIR2::update(const bool force)
{
    _updated = false;
    elapsed_time_t at{m5::utility::millis()};

    if (inPeriodic()) {
        if (force || !_latest || at >= _latest + _interval) {
            Data d{};
            _updated = read_temperature(TEMPERATURE_REG, d.raw.data());
            if (_updated) {
                _latest = at;
                _data->push_back(d);
            }
        }
    }

    if (force || !_latest_button || at >= _latest_button + _button_interval) {
        _prev_button = _button;
        if (readButtonStatus(_button)) {
            _latest_button = at;
        }
    }
}

bool UnitNCIR2::start_periodic_measurement(const uint32_t interval)
{
    if (inPeriodic()) {
        return false;
    }
    _periodic = true;
    _interval = interval;
    _latest   = 0;
    return true;
}

bool UnitNCIR2::start_periodic_measurement()
{
    return start_periodic_measurement(_interval);
}

bool UnitNCIR2::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitNCIR2::measureSingleshot(ncir2::Data& d)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    return read_temperature(TEMPERATURE_REG, d.raw.data());
}

bool UnitNCIR2::readEmissivity(uint16_t& raw)
{
    raw = 0;
    return readRegister16LE(EMISSIVITY_REG, raw, 0);
}

bool UnitNCIR2::readEmissivity(float& e)
{
    e = std::numeric_limits<float>::quiet_NaN();

    uint16_t raw{};
    if (readEmissivity(raw)) {
        e = raw / 65535.f;
        return true;
    }
    return false;
}

bool UnitNCIR2::writeEmissivity(const uint16_t raw)
{
    return writeRegister16LE(EMISSIVITY_REG, raw);
}

bool UnitNCIR2::write_emissivity(const float e)
{
    if (e < 0.1f || e > 1.0f) {
        M5_LIB_LOGE("Emissivity must be between 0.1 - 1.0");
        return false;
    }
    uint16_t tmp16 = std::round(e * 65535.f);
    return writeEmissivity(tmp16);
}

bool UnitNCIR2::readAlarmTemperature(const bool highlow, int16_t& raw)
{
    raw = std::numeric_limits<int16_t>::min();

    const uint8_t reg = ALARM_TEMPERATURE_REG + highlow * 2;
    uint16_t tmp{};

    if (readRegister16LE(reg, tmp, 0)) {
        raw = static_cast<int16_t>(tmp);
        return true;
    }
    return false;
}

bool UnitNCIR2::readAlarmTemperature(const bool highlow, float& celsius)
{
    celsius = std::numeric_limits<float>::quiet_NaN();
    int16_t v{};
    if (readAlarmTemperature(highlow, v)) {
        celsius = v * 0.01f;
        return true;
    }
    return false;
}

bool UnitNCIR2::writeAlarmTemperature(const bool highlow, const int16_t raw)
{
    const uint8_t reg = ALARM_TEMPERATURE_REG + highlow * 2;
    return writeRegister16LE(reg, static_cast<uint16_t>(raw));
}

bool UnitNCIR2::write_alarm_temperature(const bool highlow, const float celsius)
{
    constexpr float min16 = std::numeric_limits<int16_t>::min();
    constexpr float max16 = std::numeric_limits<int16_t>::max();

    float val = std::round(celsius * 100.f);
    if (val < min16 || val > max16) {
        M5_LIB_LOGE("celsius must be between %.2f to %.2f (%.2f)", min16 * 0.01f, max16 * 0.01f, celsius);
        return false;
    }
    return writeAlarmTemperature(highlow, static_cast<int16_t>(val));
}

bool UnitNCIR2::readAlarmLED(const bool highlow, uint32_t& rgb)
{
    rgb = 0;

    const uint8_t reg = ALARM_LED_REG + 3 * highlow;
    uint8_t v[3]{};
    if (readRegister(reg, v, 3, 0)) {
        rgb = (v[0] << 16) | (v[1] << 8) | v[2];
        return true;
    }
    return false;
}

bool UnitNCIR2::writeAlarmLED(const bool highlow, const uint8_t r, const uint8_t g, const uint8_t b)
{
    const uint8_t reg = ALARM_LED_REG + 3 * highlow;

    uint8_t v[3]{r, g, b};
    return writeRegister(reg, v, 3);
}

bool UnitNCIR2::readAlarmBuzzer(const bool highlow, uint16_t& freq, uint16_t& interval, uint8_t& rawDuty)
{
    freq = interval = rawDuty = 0;

    // Reg 0x40 can write continuously, but not read continuously... (due to Firmware)
    uint8_t reg = ALARM_BUZZER_LOW_FREQ_REG + 5 * highlow;
    return readRegister16LE(reg, freq, 0) && readRegister16LE(static_cast<uint8_t>(reg + 2), interval, 0) &&
           readRegister8(static_cast<uint8_t>(reg + 4), rawDuty, 0);
}

bool UnitNCIR2::readAlarmBuzzer(const bool highlow, uint16_t& freq, uint16_t& interval, float& duty)
{
    duty = std::numeric_limits<float>::quiet_NaN();
    uint8_t raw{};
    if (readAlarmBuzzer(highlow, freq, interval, raw)) {
        duty = raw_to_duty(raw);
        return true;
    }
    return false;
}

bool UnitNCIR2::writeAlarmBuzzer(const bool highlow, const uint16_t freq, const uint16_t interval,
                                 const uint8_t rawDuty)
{
    if (interval < 1 || interval > 5000) {
        M5_LIB_LOGE("Interval must be between 1 - 5000");
        return false;
    }

    const uint8_t reg = ALARM_BUZZER_REG + 5 * highlow;
    uint8_t v[5]{};
    v[0] = freq & 0xFF;
    v[1] = (freq >> 8) & 0xFF;
    v[2] = interval & 0xFF;
    v[3] = (interval >> 8) & 0xFF;
    v[4] = rawDuty;
    return writeRegister(reg, v, 5);
}

bool UnitNCIR2::write_alarm_buzzer(const bool highlow, const uint16_t freq, const uint16_t interval, const float duty)
{
    if (duty < 0.0f || duty > 1.0f) {
        M5_LIB_LOGE("Duty must be between 0.0 - 1.0");
        return false;
    }
    return writeAlarmBuzzer(highlow, freq, interval, duty_to_raw(duty));
}

bool UnitNCIR2::readBuzzer(uint16_t& freq, uint8_t& rawDuty)
{
    freq = rawDuty = 0;
    // Reg 0x50 can write continuously, but not read continuously... (due to Firmware)
    return readRegister16LE(BUZZER_FREQ_REG, freq, 0) && readRegister8(BUZZER_DUTY_REG, rawDuty, 0);
}

bool UnitNCIR2::readBuzzer(uint16_t& freq, float& duty)
{
    duty = std::numeric_limits<float>::quiet_NaN();
    uint8_t raw{};
    if (readBuzzer(freq, raw)) {
        duty = raw_to_duty(raw);
        return true;
    }
    return false;
}

bool UnitNCIR2::writeBuzzer(const uint16_t freq, const uint8_t rawDuty)
{
    uint8_t v[3]{};
    v[0] = freq & 0xFF;
    v[1] = (freq >> 8) & 0xFF;
    v[2] = rawDuty;
    return writeRegister(BUZZER_REG, v, 3);
}

bool UnitNCIR2::write_buzzer(const uint16_t freq, const float duty)
{
    if (duty < 0.0f || duty > 1.0f) {
        M5_LIB_LOGE("Duty must be between 0.0 - 1.0");
        return false;
    }
    return writeBuzzer(freq, duty_to_raw(duty));
}

bool UnitNCIR2::readBuzzerControl(bool& enabled)
{
    enabled = false;
    uint8_t v{};
    if (readRegister8((uint8_t)(BUZZER_REG + 3), v, 0)) {
        enabled = v;
        return true;
    }
    return false;
}

bool UnitNCIR2::writeBuzzerControl(const bool enabled)
{
    return writeRegister8(BUZZER_CONTROL_REG, (uint8_t)enabled);
}

bool UnitNCIR2::readLED(uint32_t& rgb)
{
    rgb = 0;
    uint8_t v[3]{};
    if (readRegister(LED_REG, v, 3, 0)) {
        rgb = (v[0] << 16) | (v[1] << 8) | v[2];
        return true;
    }
    return false;
}

bool UnitNCIR2::writeLED(const uint8_t r, const uint8_t g, const uint8_t b)
{
    uint8_t v[3]{r, g, b};
    return writeRegister(LED_REG, v, 3);
}

bool UnitNCIR2::readChipTemperature(Data& d)
{
    return read_temperature(CHIP_TEMPERATURE_REG, d.raw.data());
}

bool UnitNCIR2::writeConfig()
{
    return writeRegister8(SAVE_CONFIG_REG, 1);
}

bool UnitNCIR2::readButtonStatus(bool& press)
{
    press = false;
    uint8_t v{};
    if (readRegister8(BUTTON_REG, v, 0)) {
        press = !v;  // 0:press, 1:release
        return true;
    }
    return false;
}

bool UnitNCIR2::readFirmwareVersion(uint8_t& ver)
{
    return readRegister8(FIRMWARE_VERSION_REG, ver, 1);
}

bool UnitNCIR2::changeI2CAddress(const uint8_t i2c_address)
{
    if (!m5::utility::isValidI2CAddress(i2c_address)) {
        M5_LIB_LOGE("Invalid address : %02X", i2c_address);
        return false;
    }
    if (writeRegister8(I2C_ADDRESS_REG, i2c_address) && changeAddress(i2c_address)) {
        // Wait wakeup
        auto timeout_at = m5::utility::millis() + 1000;
        do {
            uint8_t v{};
            if (readRegister8(I2C_ADDRESS_REG, v, 0) && v == i2c_address) {
                return true;
            }
            m5::utility::delay(1);
        } while (m5::utility::millis() <= timeout_at);
    }
    return false;
}

bool UnitNCIR2::readI2CAddress(uint8_t& i2c_address)
{
    i2c_address = 0;
    return readRegister8(I2C_ADDRESS_REG, i2c_address, 0);
}

//

bool UnitNCIR2::read_temperature(const uint8_t reg, uint8_t v[2])
{
    v[0] = 0x80;
    v[1] = 0x00;
    return readRegister(reg, v, 2, 0);
}

}  // namespace unit
}  // namespace m5

#if 0
#include <iostream>
#include <cmath>
#include <cstdio>
#include <cstdint>

constexpr float EPSILON = -2.32e-10f;
constexpr float INV_255 = 1.0f / 255.0f;

float toFloat(uint8_t x) {
    //return (x + 0.5f) / 255.0f;
    return x * INV_255;

}

uint8_t toUint8(float f) {
    //return static_cast<uint8_t>(f * 255.0f);
 //return static_cast<uint8_t>(std::round((f + EPSILON) * 255.0f));
 return static_cast<uint8_t>(std::floor(f * 255.0f + 0.5f)); 
}


int main(int argc, char **argv) {
    for(uint16_t i=  0; i<256; ++i)
        {
        printf("%u => %f => %u\n", i, toFloat(i), toUint8(toFloat(i)));
        }


}
#endif
