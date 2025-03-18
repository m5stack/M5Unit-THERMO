/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_NCIR2.hpp
  @brief NCIR2 Unit for M5UnitUnified
*/
#ifndef M5_UNIT_THERMO_UNIT_NCIR2_HPP
#define M5_UNIT_THERMO_UNIT_NCIR2_HPP

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <limits>  // NaN
#include <array>

namespace m5 {
namespace unit {

/*!
  @namespace ncir2
  @brief For UnitNCIR2
 */
namespace ncir2 {
/*!
  @struct Data
  @brief Measurement data group
  @note Valid to the second decimal place
 */
struct Data {
    std::array<uint8_t, 2> raw{0x00, 0x80};  // Raw [0]:low byte [1]:high byte
    //! @brief Raw int16
    inline int16_t value() const
    {
        return (raw[1] << 8 | raw[0]);
    }
    //! @brief Temperature(Celsius)
    inline float temperature() const
    {
        return value() * 0.01f;
    }
    //! @brief Celsius
    inline float celsius() const
    {
        return temperature();
    }
    //! @brief Fahrenheit
    inline float fahrenheit() const
    {
        return celsius() * 9.0f / 5.0f + 32.f;
    }
};
}  // namespace ncir2

/*!
  @class m5::unit::UnitNCIR2
  @brief Unit NCIR2
*/
class UnitNCIR2 : public Component, public PeriodicMeasurementAdapter<UnitNCIR2, ncir2::Data> {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitNCIR2, 0x5A);

public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! Start periodic measurement on begin?
        bool start_periodic{true};
        //! Measurement interval if start on begin (ms)
        uint32_t interval{250};
        //! Button status update interval(ms)
        uint32_t button_interval{20};
    };

    explicit UnitNCIR2(const uint8_t addr = DEFAULT_ADDRESS)
        : Component(addr), _data{new m5::container::CircularBuffer<ncir2::Data>(1)}
    {
        auto ccfg  = component_config();
        ccfg.clock = 100 * 1000U;
        component_config(ccfg);
    }
    virtual ~UnitNCIR2()
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

    ///@name Measurement data by periodic
    ///@{
    /*!
      @brief Oldest temperature
      @note Valid to the second decimal place
     */
    inline float temperature() const
    {
        return !empty() ? oldest().temperature() : std::numeric_limits<float>::quiet_NaN();
    }
    /*!
      @brief Oldest celsius
      @note Valid to the second decimal place
    */
    inline float celsius() const
    {
        return !empty() ? oldest().celsius() : std::numeric_limits<float>::quiet_NaN();
    }
    /*!
      @brief Oldest fahrenheit
      @note Valid to the second decimal place
     */
    inline float fahrenheit() const
    {
        return !empty() ? oldest().fahrenheit() : std::numeric_limits<float>::quiet_NaN();
    }
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement
      @param interval Measurement interval time (ms)
      @return True if successful
    */
    inline bool startPeriodicMeasurement(const uint32_t interval)
    {
        return PeriodicMeasurementAdapter<UnitNCIR2, ncir2::Data>::startPeriodicMeasurement(interval);
    }
    //! @brief Start periodic measurement in the current settings
    inline bool startPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitNCIR2, ncir2::Data>::startPeriodicMeasurement();
    }
    /*!
      @brief Stop periodic measurement
      @return True if successful
    */
    inline bool stopPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitNCIR2, ncir2::Data>::stopPeriodicMeasurement();
    }
    ///@}

    ///@name Single shot measurement
    ///@{
    /*!
      @brief Measurement single shot
      @param[out] d Measuerd data
      @return True if successful
      @note Valid to the second decimal place
      @warning During periodic detection runs, an error is returned
    */
    bool measureSingleshot(ncir2::Data& d);
    ///@}

    ///@name Settings
    ///@{
    /*!
      @brief Read the emissivity
      @param[out] raw Raw emissivity
      @return True if successful
    */
    bool readEmissivity(uint16_t& raw);
    /*!
      @brief Read the emissivity
      @param[out] e Emissivity
      @return True if successful
    */
    bool readEmissivity(float& e);
    /*!
      @brief Write the emissivity
      @param raw Raw emissivity
      @return True if successful
    */
    bool writeEmissivity(const uint16_t raw);
    /*!
      @brief Write the emissivity
      @param e Emissivity
      @return True if successful
    */
    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeEmissivity(const T e)
    {
        return write_emissivity(static_cast<float>(e));
    }
    ///@}

    ///@name Alarm
    ///@{
    /*!
      @brief Read the alarm temperature threshold
      @param highlow Target False:low True:high
      @param[out] raw Temperature (raw)
      @return True if successful
     */
    bool readAlarmTemperature(const bool highlow, int16_t& raw);
    /*!
      @brief Read the alarm temperature threshold
      @param highlow Target False:low True:high
      @param[out] celsius Temperature
      @return True if successful
      @note Valid to the second decimal place
     */
    bool readAlarmTemperature(const bool highlow, float& celsius);
    /*!
      @brief Write the alarm temperature threshold
      @param highlow Target False:low True:high
      @param raw Temperature (raw)
      @return True if successful
     */
    bool writeAlarmTemperature(const bool highlow, const int16_t raw);
    /*!
      @brief Write the alarm temperature threshold
      @param highlow Target False:low True:high
      @param celsius Temperature
      @return True if successful
      @note Valid to the second decimal place
     */
    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeAlarmTemperature(const bool highlow, const T celsius)
    {
        return write_alarm_temperature(highlow, static_cast<float>(celsius));
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
      @param[out] interval Interval(ms) [1 - 5000]
      @param[out] rawDuty Duty (0:0.0f 255:1.0f)
      @return True if successful
     */
    bool readAlarmBuzzer(const bool highlow, uint16_t& freq, uint16_t& interval, uint8_t& rawDuty);
    /*!
      @brief Read the alarm buzzer settings
      @param highlow Target False:low True:high
      @param[out] freq Frequency
      @param[out] interval Interval(ms) [1 - 5000]
      @param[out] duty Duty
      @return True if successful
     */
    bool readAlarmBuzzer(const bool highlow, uint16_t& freq, uint16_t& interval, float& duty);
    /*!
      @brief Write the alarm buzzer settings
      @param highlow Target False:low True:high
      @param freq Frequency
      @param interval Interval(ms) [1 - 5000]
      @param rawDuty Duty (0:0.0f 255:1.0f)
      @return True if successful
     */
    bool writeAlarmBuzzer(const bool highlow, const uint16_t freq, const uint16_t interval, const uint8_t rawDuty);
    /*!
      @brief Write the alarm buzzer settings
      @param highlow Target False:low True:high
      @param freq Frequency
      @param interval Interval(ms) [1 - 5000]
      @param duty Duty
      @return True if successful
     */
    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeAlarmBuzzer(const bool highlow, const uint16_t freq, const uint16_t interval, const T duty)
    {
        return write_alarm_buzzer(highlow, freq, interval, static_cast<float>(duty));
    }
    ///@}

    ///@name Buzzer
    ///@{
    /*!
      @brief Read the buzzer settings
      @param[out] freq Frequency
      @param[out] rawDuty Duty (0:0.0f 255:1.0f)
      @return True if successful
     */
    bool readBuzzer(uint16_t& freq, uint8_t& rawDuty);
    /*!
      @brief Write the buzzer settings
      @param[out] freq Frequency
      @param[out] duty Duty
      @return True if successful
     */
    bool readBuzzer(uint16_t& freq, float& duty);

    /*!
      @brief Write the buzzer settings
      @param freq Frequency
      @param rawDuty Duty (0:0.0f 255:1.0f)
      @return True if successful
     */
    bool writeBuzzer(const uint16_t freq, const uint8_t rawDuty);
    /*!
      @brief Write the buzzer settings
      @param freq Frequency
      @param duty Duty
      @return True if successful
     */
    template <typename T, typename std::enable_if<std::is_floating_point<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeBuzzer(const uint16_t freq, const T duty)
    {
        return write_buzzer(freq, static_cast<float>(duty));
    }

    /*!
      @brief Read the Buzzer control
      @param[out] enabled True:enabled False;disabled
      @return True if successful
     */
    bool readBuzzerControl(bool& enabled);
    /*!
      @brief Write the Buzzer control
      @param enabled True:enabled False;disabled
      @return True if successful
     */
    bool writeBuzzerControl(const bool enabled);
    ///@}

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
      @return True if successful
     */
    inline bool writeLED(const uint32_t rgb)
    {
        return writeLED(rgb >> 16, rgb >> 8, rgb & 0xFF);
    }
    /*!
      @brief Write the LED color
      @param r Red
      @param g Green
      @param b Blue
      @return True if successful
     */
    bool writeLED(const uint8_t r, const uint8_t g, const uint8_t b);
    ///@}

    ///@name Chip temperature
    ///@{
    /*!
      @brief Read the Chip temperature
      @param[out] d Measuerd data
      @return True if successful
      @note Valid to the second decimal place
    */
    bool readChipTemperature(ncir2::Data& d);
    ///@}

    ///@name Flash
    ///@{
    /*!
      @brief Write configuration to inner flash
      @details Write Emissivity,Alarm,LED settings to flash
      @return True if successful
     */
    bool writeConfig();
    ///@}

    ////@name Button
    ///@{
    /*!
      @brief Read the button status
      @param[out] press Press if true
      @return True if successful
     */
    bool readButtonStatus(bool& press);
    /*!
      @brief Is button pressed?
      @return True if pressed
      @note The state is managed by update
     */
    inline bool isPressed() const
    {
        return _button;
    }
    /*!
      @brief Was button pressed?
      @return True if pressed
      @note The state is managed by update
     */
    inline bool wasPressed() const
    {
        return _button && (_button != _prev_button);
    }
    /*!
      @brief Was button released?
      @return True if released
      @note The state is managed by update
     */
    inline bool wasReleased()
    {
        return !_button && (_button != _prev_button);
    }
    ///@}

    /*!
      @brief Read the firmware version
      @param[out] ver Version
      @return True if released
     */
    bool readFirmwareVersion(uint8_t& ver);

    ///@warning Handling warning
    ///@name I2C Address
    ///@{
    /*!
      @brief Change device I2C address
      @param i2c_address I2C address
      @return True if successful

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
    bool start_periodic_measurement(const uint32_t interval);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();

    bool read_temperature(const uint8_t reg, uint8_t v[2]);

    bool write_emissivity(const float e);
    bool write_alarm_temperature(const bool highlow, const float celsius);
    bool write_alarm_buzzer(const bool highlow, const uint16_t freq, const uint16_t interval, const float duty);
    bool write_buzzer(const uint16_t freq, const float duty);

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitNCIR2, ncir2::Data);

private:
    std::unique_ptr<m5::container::CircularBuffer<ncir2::Data>> _data{};
    bool _button{}, _prev_button{};
    uint32_t _button_interval{20};
    types::elapsed_time_t _latest_button{};
    config_t _cfg{};
};

namespace ncir2 {
///@cond
namespace command {
constexpr uint8_t TEMPERATURE_REG{0x00};                 // R
constexpr uint8_t EMISSIVITY_REG{0x10};                  // R/W
constexpr uint8_t ALARM_TEMPERATURE_REG{0x20};           // R/W
constexpr uint8_t ALARM_LED_REG{0x30};                   // R/W
constexpr uint8_t ALARM_BUZZER_REG{0x40};                // W
constexpr uint8_t ALARM_BUZZER_LOW_FREQ_REG{0x40};       // R
constexpr uint8_t ALARM_BUZZER_LOW_INTERVAL_REG{0x42};   // R
constexpr uint8_t ALARM_BUZZER_LOW_DUTY_REG{0x44};       // R
constexpr uint8_t ALARM_BUZZER_HIGH_FREQ_REG{0x45};      // R
constexpr uint8_t ALARM_BUZZER_HIGH_INTERVAL_REG{0x47};  // R
constexpr uint8_t ALARM_BUZZER_HIGH_DUTY_REG{0x48};      // R
constexpr uint8_t BUZZER_REG{0x50};                      // W
constexpr uint8_t BUZZER_FREQ_REG{0x50};                 // R
constexpr uint8_t BUZZER_DUTY_REG{0x52};                 // R
constexpr uint8_t BUZZER_CONTROL_REG{0x53};              // R/W
constexpr uint8_t LED_REG{0x60};                         // R/W
constexpr uint8_t BUTTON_REG{0x70};                      // R
constexpr uint8_t SAVE_CONFIG_REG{0x80};                 // W
constexpr uint8_t CHIP_TEMPERATURE_REG{0x90};            // R
constexpr uint8_t FIRMWARE_VERSION_REG{0xFE};            // R
constexpr uint8_t I2C_ADDRESS_REG{0xFF};                 // R/W
}  // namespace command
///@endcond
}  // namespace ncir2

}  // namespace unit
}  // namespace m5

#endif
