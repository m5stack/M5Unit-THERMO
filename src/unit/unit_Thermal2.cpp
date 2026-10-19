/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_Thermal2.cpp
  @brief Thermal2 Unit for M5UnitUnified
*/
#include "unit_Thermal2.hpp"
#include <M5Utility.hpp>
#include <algorithm>
#include <array>
#include <cstddef>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::thermal2;
using namespace m5::unit::thermal2::command;

namespace {
constexpr uint16_t DEVICE_ID{0x9064};
// Writing a bit back to BUTTON_STATUS_REG clears it
constexpr uint8_t CLEAR_BUTTON_FLAGS{m5::unit::thermal2::button_was_pressed | m5::unit::thermal2::button_was_released |
                                     m5::unit::thermal2::button_was_clicked | m5::unit::thermal2::button_was_hold};
// If the host was reset in the middle of a read, the unit returns the rest of that data first
// and recovers after about 0.5 s, so the detection is retried for up to about 1 s
constexpr uint_fast8_t DETECT_RETRY{10};
constexpr uint32_t DETECT_RETRY_INTERVAL_MS{100};

// read_data() reads the 16 bytes of temp[] and the 768 bytes of raw[] in one go
using ThermalData = m5::unit::thermal2::Data;
static_assert(offsetof(ThermalData, raw) == offsetof(ThermalData, temp) + sizeof(ThermalData::temp),
              "raw[] must follow temp[] directly");

#if defined(ARDUINO)
#if defined(I2C_BUFFER_LENGTH)
constexpr uint32_t read_buffer_length{I2C_BUFFER_LENGTH};
#else
constexpr uint32_t read_buffer_length{32};
#endif
#else
// The read buffer length on non-Arduino frameworks is not known, so 32 bytes is assumed
constexpr uint32_t read_buffer_length{32};
#endif

constexpr uint16_t interval_table[] = {
    2000, 1000, 1000 / 2, 1000 / 4, 1000 / 8, 1000 / 16, 1000 / 32, 1000 / 64,
};

#if 0
// From firmware
// Registers without this bit set will not be reflected immediately
constexpr uint8_t REGISTER_DIRECT_WRITE_MAP[] = {
    0x00, 0xD8,  // 0x00 - 0x0F
    0xC0, 0x00,  // 0x10 - 0x1F
    0xFF, 0x00,  // 0x20 - 0x2F
    0xFF, 0x00,  // 0x30 - 0x3F
    0x00, 0x00,  // 0x40 - 0x4F
    0x00, 0x00,  // 0x50 - 0x5F
    0x00, 0x02,  // 0x60 - 0x6E
};
bool is_immediately(const uint8_t reg)
{
    return ((reg >> 3) < m5::stl::size(REGISTER_DIRECT_WRITE_MAP))
               ? REGISTER_DIRECT_WRITE_MAP[reg >> 3] & (0x80 >> (reg & 0x07))
               : false;
}
#endif
}  // namespace

namespace m5 {
namespace unit {
// class UnitThermal2
const char UnitThermal2::name[] = "UnitThermal2";
const types::uid_t UnitThermal2::uid{"UnitThermal2"_mmh3};
const types::attr_t UnitThermal2::attr{attribute::AccessI2C};

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

    // Detect (see DETECT_RETRY)
    uint16_t id{};
    uint16_t ver{};
    bool detected{};
    for (uint_fast8_t retry = 0; retry < DETECT_RETRY; ++retry) {
        id = ver = 0;
        if (read_register16BE(DEVICE_ID_REG, id) && readFirmwareVersion(ver) && id == DEVICE_ID && ver != 0) {
            detected = true;
            break;
        }
        M5_LIB_LOGW("Detect retry %u (%04X,%04X)", retry, id, ver);
        if (retry + 1 < DETECT_RETRY) {
            m5::utility::delay(DETECT_RETRY_INTERVAL_MS);
        }
    }
    if (!detected) {
        M5_LIB_LOGE("Cannot detect %s %04X,%04X", deviceName(), id, ver);
        return false;
    }

    _button_interval = _cfg.button_interval;

    // Clear the latched button flags (was pressed / released / clicked / hold) left from before begin
    return writeRegister8(BUTTON_STATUS_REG, CLEAR_BUTTON_FLAGS) && writeFunctionControl(_cfg.function_control) &&
           writeBuzzer(0, 0) && writeLED(0, 0, 0) &&
           writeTemperatureMonitorSize(_cfg.monitor_width, _cfg.monitor_height) &&
           (_cfg.start_periodic ? startPeriodicMeasurement(_cfg.rate) : true);
}

void UnitThermal2::update(const bool force)
{
    _updated = false;
    elapsed_time_t at{m5::utility::millis()};

    // Data
    if (inPeriodic()) {
        if (force || !_latest || m5::utility::hasElapsed(_latest, _interval, at)) {
            Data d{};
            uint8_t ds[2]{};
            _updated = read_data_status(ds) && ds[0] && read_data(d);
            // Zero median and average (-64 C, out of the -40 to 300 C range) is not a measurement. The unit returns
            // zeros for a read past the end of its registers, which can follow a host reset during a read. Drop it and
            // request the next frame (best effort)
            if (_updated && d.median_temperature == 0 && d.average_temperature == 0) {
                M5_LIB_LOGW("Dropped an all zero frame");
                request_data();
                _updated = false;
            }
            if (_updated) {
                _latest   = m5::utility::millis();
                d.subpage = ds[1];
                _data->push_back(d);
            }
        }
    }

    // Button
    if (force || !_latest_button || m5::utility::hasElapsed(_latest_button, _button_interval, at)) {
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
    if (write_function_control_bit(enabled_function_auto_refresh, false)) {
        _periodic = false;
        return true;
    }
    return false;
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

    if (!request_data()) {
        return false;
    }
    const auto start_at = m5::utility::millis();
    const uint32_t wait{interval_table[m5::stl::to_underlying(rate)]};
    // Long enough for a repeated subpage even at the slowest rate
    const uint32_t timeout{std::max<uint32_t>(5000, wait * 4)};
    uint8_t received{};  // bit0: subpage 0, bit1: subpage 1
    uint8_t ds[2]{};
    m5::utility::delay(wait);
    do {
        if (read_data_status(ds) && ds[0]) {
            const uint8_t sp = ds[1] ? 1 : 0;
            if (read_data(sp ? page1 : page0)) {
                received |= (1U << sp);
                if (received != 0x03) {
                    if (!request_data()) {
                        return false;
                    }
                    m5::utility::delay(wait);
                    continue;
                }
            }
        }
        if (received != 0x03) {
            m5::utility::delay(1);
        }
    } while (received != 0x03 && !m5::utility::hasElapsed(start_at, timeout));
    return received == 0x03;
}

bool UnitThermal2::readFunctionControl(uint8_t& value)
{
    value = 0;
    return read_register8(FUNCTION_CONTROL_REG, value);
}

bool UnitThermal2::writeFunctionControl(const uint8_t value, const bool verify)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }
    return write_function_control(value, verify);
}

bool UnitThermal2::write_function_control(const uint8_t value, const bool verify)
{
    const uint8_t v = value & 0x07;
    if (writeRegister8(FUNCTION_CONTROL_REG, v)) {
        const auto start_at = m5::utility::millis();
        do {
            uint8_t fc{};
            if (!verify || (readFunctionControl(fc) && fc == v)) {
                return true;
            }
            m5::utility::delay(1);
        } while (!m5::utility::hasElapsed(start_at, 100));
    }
    return false;
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
        // Not writeFunctionControl(), which refuses while periodic measurement is running
        return write_function_control(static_cast<uint8_t>((fc & ~bit) | (enabled ? bit : 0)), true);
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

bool UnitThermal2::readTemperatureMonitorSize(uint8_t& wid, uint8_t& hgt)
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

bool UnitThermal2::writeTemperatureMonitorSize(const uint8_t wid, const uint8_t hgt)
{
    if (wid > 15 || hgt > 11) {
        M5_LIB_LOGE("wid must be between 0 - 15, hgt must be between 0 - 11 (%u,%u)", wid, hgt);
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

    const uint8_t reg = LOW_ALARM_THRESHOLD_REG + 0x10 * highlow;
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
    const uint8_t reg = LOW_ALARM_THRESHOLD_REG + 0x10 * highlow;
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
        freq     = m5::types::little_uint16_t{v[0], v[1]}.get();
        interval = v[2];
        return true;
    }
    return false;
}

bool UnitThermal2::writeAlarmBuzzer(const bool highlow, const uint16_t freq, const uint8_t interval)
{
    if (interval < 5) {
        M5_LIB_LOGE("interval must be between 5 - 255 (%u)", interval);
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
        freq = m5::types::little_uint16_t{v[0], v[1]}.get();
        duty = v[2];
        return true;
    }
    return false;
}

bool UnitThermal2::writeBuzzer(const uint16_t freq, const uint8_t duty, const bool verify)
{
    uint8_t v[3]{};
    v[0] = freq & 0xFF;
    v[1] = freq >> 8;
    v[2] = duty;
    if (writeRegister(BUZZER_FREQ_REG, v, 3)) {
        const auto start_at = m5::utility::millis();
        do {
            uint16_t f{};
            uint8_t d{};
            if (!verify || (readBuzzer(f, d) && f == freq && d == duty)) {
                return true;
            }
            m5::utility::delay(1);
        } while (!m5::utility::hasElapsed(start_at, 100));
    }
    return false;
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
        return true;
    }
    return false;
}

bool UnitThermal2::writeLED(const uint8_t r, const uint8_t g, const uint8_t b, const bool verify)
{
    uint8_t v[3]{r, g, b};
    if (writeRegister(LED_REG, v, 3)) {
        const auto start_at = m5::utility::millis();
        do {
            uint8_t rv[3]{};
            if (!verify || (read_register(LED_REG, rv, 3) && rv[0] == r && rv[1] == g && rv[2] == b)) {
                return true;
            }
            m5::utility::delay(1);
        } while (!m5::utility::hasElapsed(start_at, 100));
    }
    return false;
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
    return writeRegister(I2C_ADDRESS_REG, v, 2) && changeAddress(i2c_address);
}

bool UnitThermal2::readI2CAddress(uint8_t& i2c_address)
{
    uint8_t v[2]{};  // [0]:addr, [1]:bit inverted addr
    if (read_register(I2C_ADDRESS_REG, v, 2)) {
        if (v[0] != static_cast<uint8_t>(~v[1])) {
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
    uint8_t reg{MEDIAN_TEMPERATURE_REG};
    if (writeWithTransaction(&reg, 1) != m5::hal::error::error_t::OK) {
        return false;
    }

    auto wptr    = reinterpret_cast<uint8_t*>(data.temp);
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
