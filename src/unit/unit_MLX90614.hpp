/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_MLX90614.hpp
  @brief MLX90614 Unit for M5UnitUnified
*/
#ifndef M5_UNIT_THERMO_UNIT_MLX90614_HPP
#define M5_UNIT_THERMO_UNIT_MLX90614_HPP

#include "unit_MLX90614_data.hpp"
#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <limits>  // NaN

namespace m5 {
namespace unit {

/*!
  @class m5::unit::UnitMLX90614
  @brief Base class of the UnitMLX90614 series
  @details It can be used to measure the surface temperature of a human body or other object.
  Only SMBus mode is supported, so the settings that only affect the PWM output (writeOutput(),
  writeObjectMinMax(), writeAmbientMinMax() and the PWMCTRL register) do not change the SMBus readings
  @todo In the future, PWM mode will be supported to allow various configurations
*/
class UnitMLX90614 : public Component, public PeriodicMeasurementAdapter<UnitMLX90614, mlx90614::Data> {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMLX90614, 0x5A);

public:
    /*!
      @struct config_t
      @brief Settings for begin
     */
    struct config_t {
        //! Start periodic measurement on begin?
        bool start_periodic{true};
        //! IIR filter if start on begin
        mlx90614::IIR iir{mlx90614::IIR::Filter100};
        //! FIR filter if start on begin
        mlx90614::FIR fir{mlx90614::FIR::Filter1024};
        //! Gain if start on begin
        mlx90614::Gain gain{mlx90614::Gain::Coeff12_5};
        //! IRSensor if start on begin
        mlx90614::IRSensor irs{mlx90614::IRSensor::Single};
        //! Emissivity if start on begin
        float emissivity{1.0f};
    };

    explicit UnitMLX90614(const uint8_t addr = DEFAULT_ADDRESS)
        : Component(addr), _data{new m5::container::CircularBuffer<mlx90614::Data>(1)}
    {
        auto ccfg  = component_config();
        ccfg.clock = 100 * 1000U;
        component_config(ccfg);
    }
    virtual ~UnitMLX90614()
    {
    }

    virtual bool begin() override;
    virtual void update(const bool force = false) override;

    ///@name Settings for begin
    ///@{
    /*! @brief Gets the configuration */
    inline config_t config()
    {
        return _cfg;
    }
    //! @brief Sets the configuration
    inline void config(const config_t& cfg)
    {
        _cfg = cfg;
    }
    ///@}

    //! @brief Gets the EEPROM structure
    const mlx90614::EEPROM& eeprom() const
    {
        return _eeprom;
    }

    ///@name Measurement data by periodic
    ///@{
    //! @brief Oldest ambient kelvin
    inline float ambientKelvin() const
    {
        return !empty() ? oldest().ambientKelvin() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest ambient temperature (Celsius)
    inline float ambientTemperature() const
    {
        return !empty() ? oldest().ambientTemperature() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest ambient temperature (Celsius)
    inline float ambientCelsius() const
    {
        return !empty() ? oldest().ambientCelsius() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest ambient temperature (Fahrenheit)
    inline float ambientFahrenheit() const
    {
        return !empty() ? oldest().ambientFahrenheit() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 1 kelvin
    inline float objectKelvin1() const
    {
        return !empty() ? oldest().objectKelvin1() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 1 temperature (Celsius)
    inline float objectTemperature1() const
    {
        return !empty() ? oldest().objectTemperature1() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 1 temperature (Celsius)
    inline float objectCelsius1() const
    {
        return !empty() ? oldest().objectCelsius1() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 1 temperature (Fahrenheit)
    inline float objectFahrenheit1() const
    {
        return !empty() ? oldest().objectFahrenheit1() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 2 kelvin
    inline float objectKelvin2() const
    {
        return !empty() ? oldest().objectKelvin2() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 2 temperature (Celsius)
    inline float objectTemperature2() const
    {
        return !empty() ? oldest().objectTemperature2() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 2 temperature (Celsius)
    inline float objectCelsius2() const
    {
        return !empty() ? oldest().objectCelsius2() : std::numeric_limits<float>::quiet_NaN();
    }
    //! @brief Oldest object 2 temperature (Fahrenheit)
    inline float objectFahrenheit2() const
    {
        return !empty() ? oldest().objectFahrenheit2() : std::numeric_limits<float>::quiet_NaN();
    }
    ///@}

    ///@name Periodic measurement
    ///@{
    /*!
      @brief Start periodic measurement
      @param iir  Infinite Impulse Response
      @param fir  Finite Impulse Response
      @param gain Amplifier gain
      @param irs  Infra-Red Sensor mode
      @return True if successful
    */
    inline bool startPeriodicMeasurement(const mlx90614::IIR iir, const mlx90614::FIR fir, const mlx90614::Gain gain,
                                         const mlx90614::IRSensor irs)
    {
        return PeriodicMeasurementAdapter<UnitMLX90614, mlx90614::Data>::startPeriodicMeasurement(iir, fir, gain, irs);
    }
    /*!
      @brief Start periodic measurement in the current settings
      @return True if successful
    */
    inline bool startPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitMLX90614, mlx90614::Data>::startPeriodicMeasurement();
    }
    /*!
      @brief Stop periodic measurement
      @return True if successful
    */
    inline bool stopPeriodicMeasurement()
    {
        return PeriodicMeasurementAdapter<UnitMLX90614, mlx90614::Data>::stopPeriodicMeasurement();
    }
    ///@}

    ///@note If apply is false, a POR or call applySettings() is required to enable the setting
    ///@warning Some settings are writable in SMBus mode, but not reflected in operation
    ///@warning When apply is true, the I2C bus is temporarily released and reinitializes (see applySettings())
    ///@name Settings(Config)
    ///@{
    /*!
      @brief Read the configuration
      @param[out] v Configuration value
      @return True if successful
     */
    bool readConfig(uint16_t& v);
    /*!
      @brief Write the configuration
      @param v Configuration value
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writeConfig(const uint16_t v, const bool apply = true);
    /*!
      @brief Read the Output
      @param[out] o Output
      @return True if successful
     */
    bool readOutput(mlx90614::Output& o);
    /*!
      @brief Write the Output
      @param o Output
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning IRSensor must be set to Dual mode to obtain TO2 output
      @warning If only one sensor is installed, the output setting to TO2 is ignored
      @warning During periodic detection runs, an error is returned
     */
    bool writeOutput(const mlx90614::Output o, const bool apply = true);
    /*!
      @brief Read the IIR
      @param[out] iir IIR
      @return True if successful
     */
    bool readIIR(mlx90614::IIR& iir);
    /*!
      @brief Write the IIR
      @param iir IIR
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writeIIR(const mlx90614::IIR iir, const bool apply = true);
    /*!
      @brief Read the FIR
      @param[out] fir FIR
      @return True if successful
     */
    bool readFIR(mlx90614::FIR& fir);
    /*!
      @brief Write the FIR
      @param fir FIR
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writeFIR(const mlx90614::FIR fir, const bool apply = true);
    /*!
      @brief Read the Gain
      @param[out] gain Gain
      @return True if successful
     */
    bool readGain(mlx90614::Gain& gain);
    /*!
      @brief Write the Gain
      @param gain Gain
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning This is a factory calibration value. Altering it invalidates calibration.
      @warning During periodic detection runs, an error is returned
     */
    bool writeGain(const mlx90614::Gain gain, const bool apply = true);
    /*!
      @brief Read the IRSensor mode
      @param[out] irs IRSensor
      @return True if successful
     */
    bool readIRSensor(mlx90614::IRSensor& irs);
    /*!
      @brief Write the IRSensor mode
      @param irs IRSensor
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning If only one sensor is installed, the IRSensor setting to Dual is ignored
      @warning During periodic detection runs, an error is returned
     */
    bool writeIRSensor(const mlx90614::IRSensor irs, const bool apply = true);
    /*!
      @brief Read the sign of Ks (Config Register1 bit7)
      @param[out] pos Positive if true
      @return True if successful
      @warning This is a factory calibration value. Altering it invalidates calibration.
     */
    bool readPositiveKs(bool& pos);
    /*!
      @brief Write the sign of Ks (Config Register1 bit7)
      @param pos Positive if true
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning This is a factory calibration value. Altering it invalidates calibration.
      @warning During periodic detection runs, an error is returned
     */
    bool writePositiveKs(const bool pos, const bool apply = true);
    /*!
      @brief Read the sign of Kt2 (Config Register1 bit14)
      @param[out] pos Positive if true
      @return True if successful
      @warning This is a factory calibration value. Altering it invalidates calibration.
     */
    bool readPositiveKt2(bool& pos);
    /*!
      @brief Write the sign of Kt2 (Config Register1 bit14)
      @param pos Positive if true
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning This is a factory calibration value. Altering it invalidates calibration.
      @warning During periodic detection runs, an error is returned
     */
    bool writePositiveKt2(const bool pos, const bool apply = true);

    //! @deprecated Use readPositiveKt2 instead
    [[deprecated("Use readPositiveKt2")]] inline bool readPositiveKf2(bool& pos)
    {
        return readPositiveKt2(pos);
    }
    //! @deprecated Use writePositiveKt2 instead
    [[deprecated("Use writePositiveKt2")]] inline bool writePositiveKf2(const bool pos, const bool apply = true)
    {
        return writePositiveKt2(pos, apply);
    }
    ///@}

    ///@note If apply is false, a POR or call applySettings() is required to enable the setting
    ///@warning Some settings are writable in SMBus mode, but not reflected in operation
    ///@warning When apply is true, the I2C bus is temporarily released and reinitializes (see applySettings())
    ///@name Settings(Temperature range)
    ///@{
    /*!
      @brief Read the minimum and maximum temperatures of the measurement for the object
      @param[out] toMin Minimum raw value
      @param[out] toMax Maximum raw value
      @return True if successful
     */
    bool readObjectMinMax(uint16_t& toMin, uint16_t& toMax);
    /*!
      @brief Read the minimum and maximum temperatures of the measurement for the object
      @param[out] toMin Minimum celsius
      @param[out] toMax Maximum celsius
      @return True if successful
     */
    bool readObjectMinMax(float& toMin, float& toMax);
    /*!
      @brief Write the minimum and maximum temperatures of the measurement for the object
      @param toMin Minimum raw value
      @param toMax Maximum raw value
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    template <typename T, typename std::enable_if<std::is_integral<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeObjectMinMax(const T toMin, const T toMax, const bool apply = true)
    {
        return write_object_minmax((uint16_t)toMin, (uint16_t)toMax, apply);
    }
    /*!
      @brief Write the minimum and maximum temperatures of the measurement for the object
      @param toMin Minimum celsius
      @param toMax Maximum celsius
      @param apply Settings take effect immediately if true
      @return True if successful
      @note Valid range between -273.15f and 382.2f
      @warning During periodic detection runs, an error is returned
     */
    bool writeObjectMinMax(const float toMin, const float toMax, const bool apply = true);
    /*!
      @brief Read the minimum and maximum temperatures of the measurement for the ambient
      @param[out] taMin Minimum raw value
      @param[out] taMax Maximum raw value
      @return True if successful
     */
    bool readAmbientMinMax(uint8_t& taMin, uint8_t& taMax);
    /*!
      @brief Read the minimum and maximum temperatures of the measurement for the ambient
      @param[out] taMin Minimum celsius
      @param[out] taMax Maximum celsius
      @return True if successful
     */
    bool readAmbientMinMax(float& taMin, float& taMax);
    /*!
      @brief Write the minimum and maximum temperatures of the measurement for the ambient
      @param taMin Minimum raw value
      @param taMax Maximum raw value
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    template <typename T, typename std::enable_if<std::is_integral<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeAmbientMinMax(const T taMin, const T taMax, const bool apply = true)
    {
        return write_ambient_minmax((uint8_t)taMin, (uint8_t)taMax, apply);
    }
    /*!
      @brief Write the minimum and maximum temperatures of the measurement for the ambient
      @param taMin Minimum celsius
      @param taMax Maximum celsius
      @param apply Settings take effect immediately if true
      @return True if successful
      @note Valid range between -38.2f and 125.0f
      @warning During periodic detection runs, an error is returned
     */
    bool writeAmbientMinMax(const float taMin, const float taMax, const bool apply = true);
    ///@}

    ///@note If apply is false, a POR or call applySettings() is required to enable the setting
    ///@warning When apply is true, the I2C bus is temporarily released and reinitializes (see applySettings())
    ///@name Settings (Emissivity)
    ///@{
    /*!
      @brief Read the emissivity
      @param[out] emiss Raw emissivity value
      @return True if successful
     */
    bool readEmissivity(uint16_t& emiss);
    /*!
      @brief Read the emissivity
      @param[out] emiss emissivity (0.1f - 1.0f)
      @return True if successful
     */
    bool readEmissivity(float& emiss);
    /*!
      @brief Write the emissivity
      @param emiss Raw emissivity value
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    template <typename T, typename std::enable_if<std::is_integral<T>::value, std::nullptr_t>::type = nullptr>
    inline bool writeEmissivity(const T emiss, const bool apply = true)
    {
        return write_emissivity((uint16_t)emiss, apply);
    }
    /*!
      @brief Write the emissivity
      @param emiss emissivity (0.1f - 1.0f)
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writeEmissivity(const float emiss, const bool apply = true);
    ///@}

    ///@name I2C Address
    ///@{
    /*!
      @brief Change device I2C address
      @param i2c_address I2C address
      @return True if successful
      @warning Internally calls applySettings() (sleep + wakeup), which temporarily releases and reinitializes
      the I2C bus. All devices sharing the same bus are affected during this period
    */
    bool changeI2CAddress(const uint8_t i2c_address);
    /*!
      @brief Read device I2C address
      @param[out] i2c_address I2C address
      @return True if successful
    */
    bool readI2CAddress(uint8_t& i2c_address);
    ///@}

    /*!
      @brief Sleep
      @return True if successful
      @warning Temporarily releases the I2C bus via end(), which affects all devices sharing the same bus
     */
    bool sleep();
    /*!
      @brief Wakeup
      @return True if successful
      @warning Temporarily releases the I2C bus via end() for GPIO manipulation (SDA low >33ms),
      then reinitializes via begin(). All devices sharing the same bus are affected during this period (~600ms)
     */
    bool wakeup();
    /*!
      @brief Apply EEPROM settings
      @return True if successful
      @note After writing to EEPROM, a POR or sleep/wakeup is required for the settings to take effect
      @warning Internally calls sleep() + wakeup(), which temporarily releases and reinitializes the I2C bus.
      All devices sharing the same bus are affected during this period
     */
    inline bool applySettings()
    {
        return sleep() && wakeup();
    }

protected:
    bool read_eeprom(mlx90614::EEPROM& e);
    bool read_register16(const uint8_t reg, uint16_t& v, const bool stopbit = false);
    bool write_register16(const uint8_t reg, const uint16_t val);
    bool write_eeprom(const uint8_t reg, const uint16_t val, const bool apply = true);
    bool write_object_minmax(const uint16_t toMin, const uint16_t toMax, const bool apply = true);
    bool write_ambient_minmax(const uint8_t taMin, const uint8_t taMax, const bool apply = true);
    bool write_emissivity(const uint16_t emiss, const bool apply = true);
    bool read_measurement(mlx90614::Data& d);

    bool start_periodic_measurement(const mlx90614::IIR iir, const mlx90614::FIR fir, const mlx90614::Gain gain,
                                    const mlx90614::IRSensor irs);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitMLX90614, mlx90614::Data);

    // Override for dual zone (xBx / xCx) parts
    virtual uint32_t get_interval(const mlx90614::IIR iir, const mlx90614::FIR fir);
    virtual bool has_dual_sensors() const
    {
        return false;
    }

private:
    std::unique_ptr<m5::container::CircularBuffer<mlx90614::Data>> _data{};
    mlx90614::EEPROM _eeprom{};
    config_t _cfg{};
};

/*!
  @class m5::unit::UnitMLX90614BAA
  @brief MLX90614 single zone unit (NCIR, SKU:U028)
  @details MLX90614ESF-BAA: 3V supply, single zone (one thermopile), so object 2 is not measured.
  objectTemperature2() and the other object 2 accessors return NaN
 */
class UnitMLX90614BAA : public UnitMLX90614 {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMLX90614BAA, 0x5A);

public:
    explicit UnitMLX90614BAA(const uint8_t addr = DEFAULT_ADDRESS) : UnitMLX90614(addr)
    {
    }
};

///@cond
namespace mlx90614 {
namespace command {
// Category bits
constexpr uint8_t COMMAND_RAM{0x00};          // 000xxxxx
constexpr uint8_t COMMAND_EEPROM{0x20};       // 001xxxxx
constexpr uint8_t COMMAND_READ_FLAGS{0xF0};   // 11110000
constexpr uint8_t COMMAND_ENTER_SLEEP{0xFF};  // 11111111 This mode is not available for the 5V supply version
// RAM
constexpr uint8_t READ_RAW_AMBIENT{0x03};
constexpr uint8_t READ_RAW_IR1{0x04};
constexpr uint8_t READ_RAW_IR2{0x05};
constexpr uint8_t READ_TAMBIENT{0x06};
constexpr uint8_t READ_TOBJECT_1{0x07};
constexpr uint8_t READ_TOBJECT_2{0x08};
// EEPROM
constexpr uint8_t EEPROM_TO_MAX{0x20};
constexpr uint8_t EEPROM_TO_MIN{0x21};
constexpr uint8_t EEPROM_PWMCTRL{0x22};
constexpr uint8_t EEPROM_TARANGE{0x23};
constexpr uint8_t EEPROM_EMISSIVITY{0x24};
constexpr uint8_t EEPROM_CONFIG{0x25};
constexpr uint8_t EEPROM_ADDR{0x2E};
constexpr uint8_t EEPROM_ID0{0x3C};
constexpr uint8_t EEPROM_ID1{0x3D};
constexpr uint8_t EEPROM_ID2{0x3E};
constexpr uint8_t EEPROM_ID3{0x3F};

}  // namespace command
}  // namespace mlx90614
///@endcond

}  // namespace unit
}  // namespace m5

#endif
