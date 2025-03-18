/*
:n * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_Thermal2.cpp
  @brief Thermal2 Unit for M5UnitUnified
*/
#include "unit_Thermal2.hpp"
#include <M5Utility.hpp>
#include <array>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::thermal2;
using namespace m5::unit::thermal2::command;

namespace {
constexpr uint16_t DEVICE_ID{0x9064};

#if defined(ARDUINO)
#if defined(I2C_BUFFER_LENGTH)
constexpr uint32_t read_buffer_length{I2C_BUFFER_LENGTH};
#else
constexpr uint32_t read_buffer_length{32};
#endif
#else
//! @TODO for M5HAL
constexpr uint32_t read_buffer_length{32};
#endif

constexpr uint16_t interval_table[] = {
    2000, 1000, 1000 / 2, 1000 / 4, 1000 / 8, 1000 / 16, 1000 / 32, 1000 / 64,
};

}  // namespace

namespace m5 {
namespace unit {
// class UnitThermal2
const char UnitThermal2::name[] = "UnitThermal2";
const types::uid_t UnitThermal2::uid{"UnitThermal2"_mmh3};
const types::attr_t UnitThermal2::attr{0};

bool UnitThermal2::begin()
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

    // Detect
    uint16_t id{};
    uint16_t ver{};
    if (!read_register16BE(DEVICE_ID_REG, id) || !readFirmwareVersion(ver) || (id != DEVICE_ID) || ver == 0) {
        M5_LIB_LOGE("Cannot detect %s %04X,%04X", deviceName(), id, ver);
        return false;
    }

    _button_interval = _cfg.button_interval;

    return writeRegister8(BUTTON_STATUS_REG, 1) && writeFunctionControl(_cfg.function_control) && writeBuzzer(0, 0) &&
           writeLED(0, 0, 0) && writeTemeratureMonitorSize(_cfg.monitor_width, _cfg.monitor_height) &&
           (_cfg.start_periodic ? startPeriodicMeasurement(_cfg.rate) : true);
}

void UnitThermal2::update(const bool force)
{
    _updated = false;
    elapsed_time_t at{m5::utility::millis()};

    // Data
    if (inPeriodic()) {
        if (force || !_latest || at >= _latest + _interval) {
            Data d{};
            uint8_t ds[2]{};
            _updated = read_data_status(ds) && ds[0] && read_data(d);
            if (_updated) {
                _latest   = m5::utility::millis();
                d.subpage = ds[1];
                _data->push_back(d);
            }
        }
    }

    // Button
    if (force || !_latest_button || at >= _latest_button + _button_interval) {
        if (readButtonStatus(_button)) {
            _latest_button = at;
            if (wasReleased()) {
                _holding = 0;
            }
            if (wasHold()) {
                _holding = 1;
            }
        }
    }
}

bool UnitThermal2::start_periodic_measurement(const thermal2::Refresh rate)
{
    if (inPeriodic()) {
        return false;
    }
    _periodic = write_function_control_bit(enabled_function_auto_refresh, true) && writeRefreshRate(rate);
    if (_periodic) {
        _latest   = 0;
        _interval = interval_table[m5::stl::to_underlying(rate)];
    }
    return _periodic;
}

bool UnitThermal2::start_periodic_measurement()
{
    Refresh rate{};
    return readRefreshRate(rate) && start_periodic_measurement(rate);
}

bool UnitThermal2::stop_periodic_measurement()
{
    _periodic = false;
    return write_function_control_bit(enabled_function_auto_refresh, false);
}

bool UnitThermal2::measureSingleshot(thermal2::Data& page0, thermal2::Data& page1)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    Refresh rate{};
    if (!readRefreshRate(rate)) {
        return false;
    }

    page0.subpage = 0;
    page1.subpage = 1;

    if (request_data()) {
        auto timeout_at = m5::utility::millis() + 2500 * 2;
        uint8_t ds[2]{};
        uint8_t done{};
        m5::utility::delay(interval_table[m5::stl::to_underlying(rate)]);
        do {
            if (read_data_status(ds) && ds[0]) {
                if (read_data(ds[1] ? page1 : page0)) {
                    if (!done) {
                        request_data();
                        m5::utility::delay(interval_table[m5::stl::to_underlying(rate)]);
                    }
                    ++done;
                }
            }
        } while (done < 2 && m5::utility::millis() <= timeout_at);
        return (done == 2);
    }
    return false;
}

bool UnitThermal2::readFunctionControl(uint8_t& value)
{
    value = 0;
    return read_register8(FUNCTION_CONTROL_REG, value);
}

bool UnitThermal2::writeFunctionControl(uint8_t value)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    return writeRegister8(FUNCTION_CONTROL_REG, value & 0x07);
}

bool UnitThermal2::read_function_control_bit(const uint8_t bit, bool& enabled)
{
    uint8_t fc{};
    if (readFunctionControl(fc)) {
        enabled = (fc & bit) == bit;
        return true;
    }
    return false;
}

bool UnitThermal2::write_function_control_bit(const uint8_t bit, const bool enabled)
{
    uint8_t fc{};
    if (readFunctionControl(fc)) {
        fc = (fc & ~bit) | (enabled ? bit : 0);
        return writeRegister8(FUNCTION_CONTROL_REG, fc & 0x07);
    }
    return false;
}

bool UnitThermal2::readRefreshRate(Refresh& rate)
{
    rate = Refresh::Rate0_5Hz;
    uint8_t v{};
    if (read_register8(REFRESH_RATE_CONFIG_REG, v)) {
        rate = static_cast<Refresh>(v & 0x07);
        return true;
    }
    return false;
}

bool UnitThermal2::writeRefreshRate(const Refresh rate)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    return writeRegister8(REFRESH_RATE_CONFIG_REG, m5::stl::to_underlying(rate));
}

bool UnitThermal2::readNoiseFilterLevel(uint8_t& level)
{
    return read_register8(NOISE_FILTER_CONFIG_REG, level);
}

bool UnitThermal2::writeNoiseFilterLevel(const uint8_t level)
{
    if (level > 15) {
        M5_LIB_LOGE("Level must be between 0 and 15 (%u)", level);
        return false;
    }
    return writeRegister8(NOISE_FILTER_CONFIG_REG, level);
}

bool UnitThermal2::readTemeratureMonitorSize(uint8_t& wid, uint8_t& hgt)
{
    wid = hgt = 0;
    uint8_t v{};
    if (read_register8(TEMPERATURE_MONITOR_SIZE_REG, v)) {
        wid = v & 0x0F;
        hgt = (v >> 4) & 0x0F;
        return true;
    }
    return false;
}

bool UnitThermal2::writeTemeratureMonitorSize(const uint8_t wid, const uint8_t hgt)
{
    if (wid > 15 || hgt > 11) {
        M5_LIB_LOGE("wid must be between 0 - 15, hgt muset be between 0 - 11 (%u,%u)", wid, hgt);
        return false;
    }
    uint8_t v = (hgt << 4) | wid;
    return writeRegister8(TEMPERATURE_MONITOR_SIZE_REG, v);
}

bool UnitThermal2::readAlarmEnabled(uint8_t& enabled_bits)
{
    return read_register8(ENABLE_TEMPERATURE_ALARM_REG, enabled_bits);
}

bool UnitThermal2::writeAlarmEnabled(const uint8_t enabled_bits)
{
    return writeRegister8(ENABLE_TEMPERATURE_ALARM_REG, enabled_bits);
}

bool UnitThermal2::readAlarmTemperature(const bool highlow, uint16_t& raw)
{
    raw = 0;

    const uint8_t reg = LOW_ALARM_THERSHOLD_REG + 0x10 * highlow;
    return read_register16LE(reg, raw);
}

bool UnitThermal2::readAlarmTemperature(const bool highlow, float& celsius)
{
    celsius = std::numeric_limits<float>::quiet_NaN();
    uint16_t raw{};
    if (readAlarmTemperature(highlow, raw)) {
        celsius = raw_to_celsius(raw);
        return true;
    }
    return false;
}

bool UnitThermal2::writeAlarmTemperature(const bool highlow, const uint16_t raw)
{
    const uint8_t reg = LOW_ALARM_THERSHOLD_REG + 0x10 * highlow;
    return writeRegister16LE(reg, static_cast<uint16_t>(raw));
}

bool UnitThermal2::readAlarmLED(const bool highlow, uint32_t& rgb)
{
    rgb = 0;

    const uint8_t reg = LOW_ALARM_LED_REG + 0x10 * highlow;
    uint8_t v[3]{};
    if (read_register(reg, v, 3)) {
        rgb = (v[0] << 16) | (v[1] << 8) | v[2];
        return true;
    }
    return false;
}

bool UnitThermal2::writeAlarmLED(const bool highlow, const uint8_t r, const uint8_t g, const uint8_t b)
{
    const uint8_t reg = LOW_ALARM_LED_REG + 0x10 * highlow;

    uint8_t v[3]{r, g, b};
    return writeRegister(reg, v, 3);
}

bool UnitThermal2::readAlarmBuzzer(const bool highlow, uint16_t& freq, uint8_t& interval)
{
    freq = interval = 0;

    const uint8_t reg = LOW_ALARM_BUZZER_FREQ_REG + 0x10 * highlow;
    uint8_t v[3]{};
    if (read_register(reg, v, 3)) {
        freq     = v[0] | (v[1] << 8);
        interval = v[2];
        return true;
    }
    return false;
}

bool UnitThermal2::writeAlarmBuzzer(const bool highlow, const uint16_t freq, const uint8_t interval)
{
    if (interval < 5) {
        M5_LIB_LOGE("intrval must be between 5 - 255 (%u)", interval);
        return false;
    }

    const uint8_t reg = LOW_ALARM_BUZZER_FREQ_REG + 0x10 * highlow;
    uint8_t v[3]{};
    v[0] = freq & 0xFF;
    v[1] = freq >> 8;
    v[2] = interval;
    return writeRegister(reg, v, 3);
}

bool UnitThermal2::readBuzzer(uint16_t& freq, uint8_t& duty)
{
    freq = duty = 0;

    uint8_t v[3]{};
    if (read_register(BUZZER_FREQ_REG, v, 3)) {
        freq = v[0] | (v[1] << 8);
        duty = v[2];
        return true;
    }
    return false;
}

bool UnitThermal2::writeBuzzer(const uint16_t freq, const uint8_t duty)
{
    uint8_t v[3]{};
    v[0] = freq & 0xFF;
    v[1] = freq >> 8;
    v[2] = duty;
    return writeRegister(BUZZER_FREQ_REG, v, 3);
}

bool UnitThermal2::writeBuzzerDuty(const uint8_t duty)
{
    return writeRegister8(BUZZER_DUTY_REG, duty);
}

bool UnitThermal2::readLED(uint32_t& rgb)
{
    rgb = 0;
    uint8_t v[3]{};
    if (read_register(LED_REG, v, 3)) {
        rgb = (v[0] << 16) | (v[1] << 8) | v[2];
        M5_LIB_LOGE("R>>>>:%X (%02x,%02x,%02x)", rgb, v[0], v[1], v[2]);
        return true;
    }
    return false;
}

bool UnitThermal2::writeLED(const uint8_t r, const uint8_t g, const uint8_t b)
{
    M5_LIB_LOGE(">>>> %02X,%02X,%02X", r, g, b);

    uint8_t v[3]{r, g, b};
    return writeRegister(LED_REG, v, 3);
}

bool UnitThermal2::readButtonStatus(uint8_t& bs)
{
    bs = 0;
    // Write-back of read data to firmware is required
    // See also firmware
    // https://github.com/m5stack/M5Unit-Thermal2-Internal-FW/blob/main/src/command_processor.cpp
    return read_register8(BUTTON_STATUS_REG, bs) && writeRegister8(BUTTON_STATUS_REG, bs);
}

bool UnitThermal2::readFirmwareVersion(uint16_t& ver)
{
    ver = 0;
    return read_register16BE(FIRMWARE_VERSION_REG, ver);
}

bool UnitThermal2::changeI2CAddress(const uint8_t i2c_address)
{
    if (!m5::utility::isValidI2CAddress(i2c_address)) {
        M5_LIB_LOGE("Invalid address : %02X", i2c_address);
        return false;
    }

    uint8_t v[2]{};
    v[0] = i2c_address;
    v[1] = ~i2c_address;
    return writeRegister(I2C_ADDRESS_REG, v, 2) && changeAddress(0x10);
}

bool UnitThermal2::readI2CAddress(uint8_t& i2c_address)
{
    uint8_t v[2]{};  // [0]:addr, [1]:bit invtert addr
    if (read_register(I2C_ADDRESS_REG, v, 2)) {
        if (v[0] != ~v[1]) {
            M5_LIB_LOGE("Invalid data %02X/%02X", v[0], v[1]);
            return false;
        }
        i2c_address = v[0];
        return true;
    }
    return false;
}

bool UnitThermal2::read_data_status(uint8_t s[2])
{
    return read_register(DATA_REFRESH_CONTROL_REG, s, 2);
}

bool UnitThermal2::request_data()
{
    return writeRegister8(DATA_REFRESH_CONTROL_REG, 0);
}

bool UnitThermal2::read_data(thermal2::Data& data)
{
    // batch read
    uint8_t reg{MEDIAN_TEPERATURE_REG};
    if (writeWithTransaction(&reg, 1) != m5::hal::error::error_t::OK) {
        return false;
    }

    auto wptr    = (uint8_t*)data.temp;
    int32_t left = (384 + 8) * sizeof(uint16_t);
    // M5_LIB_LOGD("Read:[%02X] %u", reg, left);

    while (left > 0) {
        uint32_t batch_len = (left > read_buffer_length) ? read_buffer_length - (read_buffer_length % 4) : left;
        // M5_LIB_LOGD("    batch:%u", batch_len);
        if (readWithTransaction(wptr, batch_len) != m5::hal::error::error_t::OK) {
            return false;
        }
        left -= batch_len;
        wptr += batch_len;
    }
    return true;
}

}  // namespace unit
}  // namespace m5
