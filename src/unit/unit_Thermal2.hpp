/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_Thermal2.hpp
  @brief Thermal2 Unit for M5UnitUnified
*/
#ifndef M5_UNIT_THERMO_UNIT_THERMAL2_HPP
#define M5_UNIT_THERMO_UNIT_THERMAL2_HPP
#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <limits>  // NaN
#include <cmath>
#include <array>

namespace m5 {
namespace unit {
/*!
  @namespace thermal2
  @brief For UnitThermal2
 */
namespace thermal2 {

///@sa m5::unit::UnitThermal2::readButtonStatus
///@name Button status
///@{
constexpr uint8_t button_is_pressed{0x01};
constexpr uint8_t button_was_pressed{0x02};
constexpr uint8_t button_was_released{0x04};
constexpr uint8_t button_was_clicked{0x08};
constexpr uint8_t button_was_hold{0x10};
///@}

///@sa m5::unit::UnitThermal2::readFunctionControl m5::unit::UnitThermal2::writeFunctionControl
///@name Function control
///@{
constexpr uint8_t enabled_function_buzzer{0x01};
constexpr uint8_t enabled_function_led{0x02};
constexpr uint8_t enabled_function_auto_refresh{0x04};
///@}

///@sa m5::unit::UnitThermal2::readEnabledAlarm m5::unit::UnitThermal2::writeEnabledAlarm
///@name Alarm enabled bits
///@{
constexpr uint8_t enabled_low_temperature_low{0x01};    //!< Low temp reached low threshold
constexpr uint8_t enabled_med_temperature_low{0x02};    //!< Med temp reached low threshold
constexpr uint8_t enabled_ave_temperature_low{0x04};    //!< Ave temp reached low threshold
constexpr uint8_t enabled_high_temperature_low{0x08};   //!< High temp reached low threshold
constexpr uint8_t enabled_low_temperature_high{0x10};   //!< Low temp reached high threshold
constexpr uint8_t enabled_med_temperature_high{0x20};   //!< Med temp reached high threshold
constexpr uint8_t enabled_ave_temperature_high{0x40};   //!< Ave temp reached high threshold
constexpr uint8_t enabled_high_temperature_high{0x80};  //!< High temp reached high threshold
///@}

/*!
  @enum Refresh
  @brief Refresh rate(Hz)
 */
enum class Refresh : uint8_t {
    Rate0_5Hz,  //!< 0.5Hz
    Rate1Hz,    //!< 1Hz
    Rate2Hz,    //!< 2Hz
    Rate4Hz,    //!< 4Hz
    Rate8Hz,    //!< 8Hz
    Rate16Hz,   //!< 16Hz
    Rate32Hz,   //!< 32Hz
    Rate64Hz,   //!< 64Hz
};

//! @brief Celsius to raw temperature value
inline static uint16_t celsius_to_raw(const float f)
{
    int i = std::round((f + 64) * 128);
    return static_cast<uint16_t>(std::max(std::min(i, (int)std::numeric_limits<uint16_t>::max()), 0));
}
//! @brief Raw temperature value to celsius
inline static float raw_to_celsius(const uint16_t u16)
{
    return u16 / 128.0f - 64;
}

#pragma pack(push)
#pragma pack(1)
/*!
  @struct Data
  @brief Measurement data group
 */
struct Data {
    uint8_t subpage{};  // Subpage 0:even 1:odd
    union {
        uint16_t temp[8]{};  // Temperature information
        struct {
            uint16_t median_temperature;
            uint16_t average_temperature;
            uint16_t most_diff_temperature;
            uint8_t most_diff_x;
            uint8_t most_diff_y;
            uint16_t lowest_temperature;
            uint8_t lowest_diff_x;
            uint8_t lowest_diff_y;
            uint16_t highest_temperature;
            uint8_t highest_diff_x;
            uint8_t highest_diff_y;
        };
    };
    uint16_t raw[384]{};  // Raw pixel data (1/2)

    // temperture information
    inline float medianTemperature() const
    {
        return thermal2::raw_to_celsius(temp[0]);
    }
    inline float averageTemperature() const
    {
        return thermal2::raw_to_celsius(temp[1]);
    }
    inline float mostDiffTemperature() const
    {
        return thermal2::raw_to_celsius(temp[2]);
    }
    inline float lowestTemperature() const
    {
        return thermal2::raw_to_celsius(temp[4]);
    }
    inline float highestTemperature() const
    {
        return thermal2::raw_to_celsius(temp[6]);
    }

    // piexl temperature
    inline float temperature(const uint_fast16_t idx) const
    {
        return (idx < 384) ? thermal2::raw_to_celsius(raw[idx]) : std::numeric_limits<float>::quiet_NaN();
    }
};
#pragma pack(pop)

}  // namespace thermal2

/*!
  @class m5::unit::UnitThermal2
  @brief Unit Thermal2
*/
class UnitThermal2 : public Component, public PeriodicMeasurementAdapter<UnitThermal2, thermal2::Data> {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitThermal2, 0x32);

public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! Start periodic measurement on begin?
        bool start_periodic{true};
        //! Refresh rate if start on begin
        thermal2::Refresh rate{thermal2::Refresh::Rate16Hz};
        //! Temperature monitor width
        uint8_t monitor_width{15};
        //! Temperature monitor height
        uint8_t monitor_height{11};
        //! Function control bits
        uint8_t function_control{thermal2::enabled_function_led};
        //! Button status update interval(ms)
        uint32_t button_interval{20};
    };

    explicit UnitThermal2(const uint8_t addr = DEFAULT_ADDRESS)
        : Component(addr), _data{new m5::container::CircularBuffer<thermal2::Data>(1)}
    {
        auto ccfg  = component_config();
        ccfg.clock = 400 * 1000U;
        component_config(ccfg);
    }
    virtual ~UnitThermal2()
    {
    }

    virtual bool begin() override;
    virtual void update(const bool force = false) override;

    ///@name Settings for begin
    ///@{
    /*! @brief Gets the configration */
    inline config_t config()
    {
        return _cfg;
    }
    //! @brief Set the configration
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement
      @param rate Refresh rate
      @return True if successful
    */
    inline bool startPeriodicMeasurement(const thermal2::Refresh rate)
    {
        return PeriodicMeasurementAdapter<UnitThermal2, thermal2::Data>::startPeriodicMeasurement(rate);
    }
    //! @brief Start periodic measurement in the current settings
    inline bool startPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitThermal2, thermal2::Data>::startPeriodicMeasurement();
    }
    /*!
      @brief Stop periodic measurement
      @return True if successful
    */
    inline bool stopPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitThermal2, thermal2::Data>::stopPeriodicMeasurement();
    }
    ///@}

    ///@name Single shot measurement
    ///@{
    /*!
      @brief Measurement single shot
      @param[out] page0  Measuerd data subpage 0
      @param[out] page1  Measuerd data subpage 1
      @return True if successful
      @note Pixel temperature data is retrieved half at a time
      @note Measure based on the current refresh rate
      @warning During periodic detection runs, an error is returned
      @warning Blocked until end of measurement
    */
    bool measureSingleshot(thermal2::Data& page0, thermal2::Data& page1);
    ///@}

    ///@name Settings
    ///@{
    /*!
      @brief Read the function control
      @param[out] value Function control value
      @return True if successful
      @note bit0: buzzer enable. / bit1: neopixel enable. / bit2: auto refresh enable
     */
    bool readFunctionControl(uint8_t& value);
    //! @brief Read the buzzer enabled
    inline bool readBuzzerEnabled(bool& enabled)
    {
        return read_function_control_bit(thermal2::enabled_function_buzzer, enabled);
    }
    //! @brief Reads the LED enabled
    inline bool readLEDEnabled(bool& enabled)
    {
        return read_function_control_bit(thermal2::enabled_function_led, enabled);
    }
    /*!
      @brief Write the function control
      @param value Function control value
      @param verify Verify the value is written if true z(it is not reflected immediately)
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writeFunctionControl(const uint8_t value, const bool verify = true);
    //! @brief Write the buzzer enable status
    inline bool writeBuzzerEnabled(const bool enabled)
    {
        return write_function_control_bit(thermal2::enabled_function_buzzer, enabled);
    }
    //! @brief Write the LED enable status
    inline bool writeLEDEnabled(const bool enabled)
    {
        return write_function_control_bit(thermal2::enabled_function_led, enabled);
    }

    /*!
      @brief Read the refresh rate
      @param[out] rate Refresh rate
      @return True if successful
     */
    bool readRefreshRate(thermal2 ::Refresh& rate);
    /*!
      @brief Write the refresh rate
      @param rate Refresh rate
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writeRefreshRate(const thermal2::Refresh rate);

    /*!
      @brief Read the noise filter level
      @param[out] level Filter level
      @return True if successful
      @note 0:off - 15:maximum
     */
    bool readNoiseFilterLevel(uint8_t& level);
    /*!
      @brief Write the noise filter level
      @param level Filter level
      @return True if successful
      @note 0:off - 15:maximum
     */
    bool writeNoiseFilterLevel(const uint8_t level);

    /*!
      @brief Read the temperature monitor size
      @param[out] wid Width (0-15)
      @param[out] hgt Height (0-11)
      @return True if successful
     */
    bool readTemeratureMonitorSize(uint8_t& wid, uint8_t& hgt);
    /*!
      @brief Write the temperature monitor size
      @param wid Width (0-15)
      @param hgt Heigh (0-11)
      @return True if successful
     */
    bool writeTemeratureMonitorSize(const uint8_t wid, const uint8_t hgt);
    ///@}

    ///@name Alarm
    ///@{
    /*!
      @brief Read the alarm control
      @param[out] enabled_bits Enabled temperature alarm bits
      @return True if successful
    */
    bool readAlarmEnabled(uint8_t& enabled_bits);
    /*!
      @brief Read the alarm control
      @param[out] enabled_bits Enabled temperature alarm bits
      @return True if successful
    */
    bool writeAlarmEnabled(const uint8_t enabled_bits);

    /*!
      @brief Read the alarm temperature threshold
      @param highlow Target False:low True:high
      @param[out] raw Temperature (raw)
      @return True if successful
     */
    bool readAlarmTemperature(const bool highlow, uint16_t& raw);
    /*!
      @brief Read the alarm temperature threshold
      @param highlow Target False:low True:high
      @param[out] celsius Temperature
      @return True if successful
     */
    bool readAlarmTemperature(const bool highlow, float& celsius);
    /*!
      @brief Write the alarm temperature threshold
      @param highlow Target False:low True:high
      @param raw Temperature (raw)
      @return True if successful
      @note Temperature value conversion formula: (celsius + 64) * 128 = value.
      @note 0x0000:-64.0f - 0xFFFF:+447.99f
     */
    bool writeAlarmTemperature(const bool highlow, const uint16_t raw);
    /*!
      @brief Write the alarm temperature threshold
      @param highlow Target False:low True:high
      @param celsius Temperature
      @return True if successful
     */
    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeAlarmTemperature(const bool highlow, const T celsius)
    {
        return writeAlarmTemperature(highlow, thermal2::celsius_to_raw(static_cast<float>(celsius)));
    }

    /*!
      @brief Read the alarm LED color
      @param highlow Target False:low True:high
      @param[out] rgb RGB24 color
      @return True if successful
     */
    bool readAlarmLED(const bool highlow, uint32_t& rgb);
    /*!
      @brief Write the alarm LED color
      @param highlow Target False:low True:high
      @param rgb RGB24 color
      @return True if successful
     */
    inline bool writeAlarmLED(const bool highlow, const uint32_t rgb)
    {
        return writeAlarmLED(highlow, rgb >> 16, rgb >> 8, rgb & 0xFF);
    }
    /*!
      @brief Write the alarm LED color
      @param highlow Target False:low True:high
      @param r Red
      @param g Green
      @param b Blue
      @return True if successful
     */
    bool writeAlarmLED(const bool highlow, const uint8_t r, const uint8_t g, const uint8_t b);

    /*!
      @brief Read the alarm buzzer settings
      @param highlow Target False:low True:high
      @param[out] freq Frequency
      @param[out] interval Interval (10ms)
      @return True if successful
      @note The interval valid range between 5 and 255 (50ms - 2550ms)
     */
    bool readAlarmBuzzer(const bool highlow, uint16_t& freq, uint8_t& interval);
    /*!
      @brief Write the alarm buzzer settings
      @param highlow Target False:low True:high
      @param freq Frequency
      @param interval Interval (10ms)
      @return True if successful
      @note The interval valid range between 5 and 255 (50ms - 2550ms)
     */
    bool writeAlarmBuzzer(const bool highlow, const uint16_t freq, const uint8_t interval);
    ///@}

    ///@warning Value setting is invalid while buzzer is controlled by alarms
    ///@name Buzzer
    ///@{
    /*!
      @brief Read the buzzer settings
      @param[out] freq Frequency
      @param[out] duty 0 - 255
      @return True if successful
      @note buzzer duty. 0~255 (default:128 : The loudest sound setting; the further away from 128, the quieter the
      sound)
     */
    bool readBuzzer(uint16_t& freq, uint8_t& duty);
    /*!
      @brief Write the buzzer settings
      @param freq Frequency
      @param duty Duty 0 - 255
      @param verify Verify the value is written if true z(it is not reflected immediately)
      @return True if successful
      @note buzzer duty. 0~255 (default:128 : The loudest sound setting; the further away from 128, the quieter the
      sound)
     */
    bool writeBuzzer(const uint16_t freq, const uint8_t duty, const bool verify = true);
    /*!
      @brief Write the buzzer duty settings
      @param duty Duty 0 - 255
      @return True if successful
      @note buzzer duty. 0~255 (default:128 : The loudest sound setting; the further away from 128, the quieter the
      sound)
     */
    bool writeBuzzerDuty(const uint8_t duty);

    /*!
      @brief Read the Buzzer control
      @param[out] enabled True:enabled False;disabled
      @return True if successful
     */
    inline bool readBuzzerControl(bool& enabled)
    {
        return readBuzzerEnabled(enabled);
    }
    /*!
      @brief Write the Buzzer control
      @param enabled True:enabled False;disabled
      @return True if successful
     */
    inline bool writeBuzzerControl(const bool enabled)
    {
        return writeBuzzerEnabled(enabled);
    }
    ///@}

    ///@warning Value setting is invalid while LEDs are controlled by alarms
    ///@name LED
    ///@{
    /*!
      @brief Read the LED color
      @param[out] rgb RGB24 color
      @return True if successful
     */
    bool readLED(uint32_t& rgb);
    /*!
      @brief Write the LED color
      @param rgb RGB24 color
      @param verify Verify the value is written if true z(it is not reflected immediately)
      @return True if successful
     */
    inline bool writeLED(const uint32_t rgb, const bool verify = true)
    {
        return writeLED(rgb >> 16, rgb >> 8, rgb & 0xFF);
    }
    /*!
      @brief Write the LED color
      @param r Red
      @param g Green
      @param b Blue
      @param verify Verify the value is written if true z(it is not reflected immediately)
      @return True if successful
     */
    bool writeLED(const uint8_t r, const uint8_t g, const uint8_t b, const bool verify = true);
    ///@}

    ///@name Button
    ///@{
    /*!
      @brief Read the button status
      @param[out] bs Button status
      @return True if successful
     */
    bool readButtonStatus(uint8_t& bs);

    /*!
      @brief Is button pressed?
      @return True if pressed
      @note The state is managed by update
    */
    inline bool isPressed() const
    {
        return _button & thermal2::button_is_pressed;
    }
    /*!
      @brief Was button pressed?
      @return True if pressed
      @note The state is managed by update
     */
    inline bool wasPressed()
    {
        return _button & thermal2::button_was_pressed;
    }
    /*!
      @brief Was button released?
      @return True if released
      @note The state is managed by update
     */
    inline bool wasReleased()
    {
        return _button & thermal2::button_was_released;
    }
    /*!
      @brief Was button clicked?
      @return True if released
      @note The state is managed by update
     */
    inline bool wasClicked()
    {
        return _button & thermal2::button_was_clicked;
    }
    /*!
      @brief Was button hold?
      @return True if released
      @note The state is managed by update
     */
    inline bool wasHold()
    {
        return _button & thermal2::button_was_hold;
    }
    /*!
      @brief Is button holding?
      @return True if released
      @note The state is managed by update
     */
    inline bool isHolding()
    {
        return isPressed() && _holding;
    }
    ///@}

    /*!
      @brief Read the firmware version
      @param[out] ver Version high:major low:minor
      @return True if successful
     */
    bool readFirmwareVersion(uint16_t& ver);

    ///@warning Handling warning
    ///@name I2C Address
    ///@{
    /*!
      @brief Change device I2C address
      @param i2c_address I2C address
      @return True if successful
      @warning When the power is turned back on, it will operate at the new address
    */
    bool changeI2CAddress(const uint8_t i2c_address);
    /*!
      @brief Read device I2C address
      @param[out] i2c_address I2C address
      @return True if successful
    */
    bool readI2CAddress(uint8_t& i2c_address);
    ///@}

protected:
    bool read_function_control_bit(const uint8_t bit, bool& enabled);
    bool write_function_control_bit(const uint8_t bit, const bool enabled);

    bool request_data();
    bool read_data_status(uint8_t s[2]);  // [0]:data refresh ctrl, [1]subpage information
    bool read_data(thermal2::Data& data);

    bool start_periodic_measurement(const thermal2::Refresh rate);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();

    inline bool read_register(const uint8_t reg, uint8_t* v, const uint32_t len)
    {
        return readRegister(reg, v, len, 0, false /* stopbit false */);
    }

    inline bool read_register8(const uint8_t reg, uint8_t& v)
    {
        return readRegister8(reg, v, 0, false /* stopbit false */);
    }

    inline bool read_register16LE(const uint8_t reg, uint16_t& v)
    {
        return readRegister16LE(reg, v, 0, false /* stopbit false */);
    }

    inline bool read_register16BE(const uint8_t reg, uint16_t& v)
    {
        return readRegister16BE(reg, v, 0, false /* stopbit false */);
    }

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitThermal2, thermal2::Data);

private:
    std::unique_ptr<m5::container::CircularBuffer<thermal2::Data>> _data{};
    uint8_t _button{}, _holding{};
    uint32_t _button_interval{20};
    types::elapsed_time_t _latest_button{};
    config_t _cfg{};
};

namespace thermal2 {
///@cond
namespace command {
constexpr uint8_t BUTTON_STATUS_REG{0x00};             // R/W
constexpr uint8_t TEMPERATURE_ALARM_STATUS_REG{0x01};  // R
// constexpr uint8_t DEVICE_STATUS_REG{0x02};             // R 2
constexpr uint8_t DEVICE_ID_REG{0x04};            // R 2
constexpr uint8_t FIRMWARE_VERSION_REG{0x06};     // R 2
constexpr uint8_t I2C_ADDRESS_REG{0x08};          // R/W 2 (0x09:bit inverted) I
constexpr uint8_t FUNCTION_CONTROL_REG{0x0A};     // R/W
constexpr uint8_t REFRESH_RATE_CONFIG_REG{0x0B};  // R/W I
constexpr uint8_t NOISE_FILTER_CONFIG_REG{0x0C};  // R/W I

constexpr uint8_t TEMPERATURE_MONITOR_SIZE_REG{0x10};  // R/W I
constexpr uint8_t ENABLE_TEMPERATURE_ALARM_REG{0x11};  // R/W I
constexpr uint8_t BUZZER_FREQ_REG{0x12};               // R/W 2
constexpr uint8_t BUZZER_DUTY_REG{0x14};               // R/W
constexpr uint8_t LED_REG{0x15};                       // R/W 3(R,G,B)

constexpr uint8_t LOW_ALARM_THERSHOLD_REG{0x20};    // R/W 2 I
constexpr uint8_t LOW_ALARM_BUZZER_FREQ_REG{0x22};  // R/W 2 I
constexpr uint8_t LOW_ALARM_INTERVAL_REG{0x24};     // R/W I
constexpr uint8_t LOW_ALARM_LED_REG{0x25};          // R/W 3 I

constexpr uint8_t HIGH_ALARM_THERSHOLD_REG{0x30};    // R/W 2 I
constexpr uint8_t HIGH_ALARM_BUZZER_FREQ_REG{0x32};  // R/W 2 I
constexpr uint8_t HIGH_ALARM_INTERVAL_REG{0x34};     // R/W I
constexpr uint8_t HIGH_ALARM_LED_REG{0x35};          // R/W 3 I

constexpr uint8_t DATA_REFRESH_CONTROL_REG{0x6E};  // R/W I
constexpr uint8_t SUB_PAGE_INFORMATION_REG{0x6F};  // R

constexpr uint8_t MEDIAN_TEPERATURE_REG{0x70};     // R 2
constexpr uint8_t AVERAGE_TEPERATURE_REG{0x72};    // R 2
constexpr uint8_t MOST_DIFF_TEPERATURE_REG{0x74};  // R 2
constexpr uint8_t MOST_DIFF_X_POS_REG{0x76};       // R
constexpr uint8_t MOST_DIFF_Y_POS_REG{0x77};       // R
constexpr uint8_t LOWEST_TEPERATURE_REG{0x78};     // R 2
constexpr uint8_t LOWEST_DIFF_X_POS_REG{0x7A};     // R
constexpr uint8_t LOWEST_DIFF_Y_POS_REG{0x7B};     // R
constexpr uint8_t HIGHEST_TEPERATURE_REG{0x7C};    // R 2
constexpr uint8_t HIGHEST_DIFF_X_POS_REG{0x7E};    // R
constexpr uint8_t HIGHEST_DIFF_Y_POS_REG{0x7F};    // R

constexpr uint8_t TEMPERATURE_DATA_REG{0x80};  // R 768
}  // namespace command
///@endcond
}  // namespace thermal2

}  // namespace unit
}  // namespace m5
#endif
