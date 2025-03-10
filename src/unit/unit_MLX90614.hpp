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

#include <M5UnitComponent.hpp>
#include <m5_utility/container/circular_buffer.hpp>
#include <limits>  // NaN
#include <array>

namespace m5 {
namespace unit {

/*!
  @namespace mlx90614
  @brief For mlx90614
 */
namespace mlx90614 {

/*!
  @enum Output
  @brief PWM output mode
 */
enum class Output : uint8_t {
    TA_TO1,         //!< PWM1: Ta PWM2:To1 (Ambient & Object 1)
    TA_TO2,         //!< PWM1: Ta PWM2:To2 (Ambient & Object 1)
    TO2_Undefined,  //!< PWM1: To2 PWM2:Undefined (Object 2)
    TO1_TO2,        //!< PWM1: To1 PWM2:To2 (Object 1 & 2)
};

/*!
  @enum IIR
  @brief Infinite Impulse Response
 */
enum class IIR : uint8_t {
    Filter50,   //!< 50% (a1 = 0.5, b1 = 0.5)
    Filter25,   //!< 25% (a1 = 0.25, b1 = 0.75)
    Filter17,   //!< 17% (a1 = 0x166(6), b1 = 0x83(3))
    Filter13,   //!< 13% (a1 = 0.125, b1 = 0.875)
    Filter100,  //!< 100% (a1 = 1, b1 = 0)
    Filter80,   //!< 80% (a1 = 0.8, b1 = 0.2)
    Filter67,   //!< 67% (a1 = 0.666, b1 = 0.333)
    Filter57,   //!< 57% (a1 = 0.571, b1 = 0.428)
};

/*!
  @enum FIR
  @brief Finite Impulse Response
 */
enum class FIR : uint8_t {
    Filter8,     //!< 8 Not recommended
    Filter16,    //!< 16 Not recommended
    Filter32,    //!< 32 Not recommended
    Filter64,    //!< 64 Not recommended
    Filter128,   //!< 128
    Filter256,   //!< 256
    Filter512,   //!< 512
    Filter1024,  //!< 1024

};

/*!
  @enum Gain
  @brief Amplifier gain
 */
enum class Gain : uint8_t {
    Coeff1,     //!< Bypassed
    Coeff3,     //!< 3
    Coeff6,     //!< 6
    Coeff12_5,  //!< 12.5
    Coeff25,    //!< 25
    Coeff50,    //!< 50
    Coeff100,   //!< 100
};

/*!
  @enum IRSensor
  @brief Infra-Red Sensor mode
 */
enum class IRSensor : uint8_t {
    Single,  //!< Single IR Sensor
    Dual     //!< Dual IR Sensor
};

/*!
  @struct Data
  @brief Measurement data group
 */
struct Data {
    std::array<uint16_t, 3> raw{};  // linearized raw [0]:Ambient [1]:Object1 [2]:Object2

    inline float ambientKelvin() const
    {
        return ((raw[0] & 0x8000) == 0) ? raw[0] * 0.02f : std::numeric_limits<float>::quiet_NaN();
    }
    inline float ambientTemperature() const
    {
        return ambientCelsius();
    }
    inline float ambientCelsius() const
    {
        return ambientKelvin() - 273.15f;
    }
    inline float ambientFahrenheit() const
    {
        return ambientCelsius() * 9.0f / 5.0f + 32.f;
    }

    inline float objectKelvin1() const
    {
        return ((raw[1] & 0x8000) == 0) ? raw[1] * 0.02f : std::numeric_limits<float>::quiet_NaN();
    }
    inline float objectTemperature1() const
    {
        return objectCelsius1();
    }
    inline float objectCelsius1() const
    {
        return objectKelvin1() - 273.15f;
    }
    inline float objectFahrenheit1() const
    {
        return objectCelsius1() * 9.0f / 5.0f + 32.f;
    }

    inline float objectKelvin2() const
    {
        return ((raw[2] & 0x8000) == 0) ? raw[2] * 0.02f : std::numeric_limits<float>::quiet_NaN();
    }
    inline float objectTemperature2() const
    {
        return objectCelsius2();
    }
    inline float objectCelsius2() const
    {
        return objectKelvin2() - 273.15f;
    }
    inline float objectFahrenheit2() const
    {
        return objectCelsius2() * 9.0f / 5.0f + 32.f;
    }
};

/*!
  @struct EEPROM structure
  @brief EEPROM values
 */
struct EEPROM {
    uint16_t toMax{}, toMin{},  //!< Max,Min of the Object Temperature
        pwmCtrl{},              //!< Pulse With Modulation control
        taRange{},              //!< Range of the Ambient Temperature (H/L)
        emissivity{},           //!< Emissivity
        config{},               //!< Configuration
        addr{},                 //!< I2C address(Using low byte)
        id[4]{};                //!< Unique ID
};

}  // namespace mlx90614

/*!
  @class m5::unit::UnitMLX90614
  @brief Base class of the UnitMLX90614 series
  @brief It can be used to measure the surface temperature of a human body or other object
  @details Currently only SMBus mode is supported. This has limited functionality and some settings are ignored
  @todo In the future, PMW mode will be supported to allow various configurations
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
    //! @brief Start periodic measurement in the current settings
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

    ///@note If apply is false, a POR or call applySetting() is required to enable the setting
    ///@warning Some settings are writable in SMBus mode, but not reflected in operation
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
      @param[out] dir FIR
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
      @brief Read the positiveKs
      @param[out] pos Positive if true
      @return True if successful
     */
    bool readPositiveKs(bool& pos);
    /*!
      @brief Write the positiveKs
      @param pos Positive if true
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writePositiveKs(const bool pos, const bool apply = true);
    /*!
      @brief Read the positiveKf2
      @param[out] pos Positive if true
      @return True if successful
     */
    bool readPositiveKf2(bool& pos);
    /*!
      @brief Write the positiveKf2
      @param pos Positive if true
      @param apply Settings take effect immediately if true
      @return True if successful
      @warning During periodic detection runs, an error is returned
     */
    bool writePositiveKf2(const bool pos, const bool apply = true);
    ///@}

    ///@note If apply is false, a POR or call applySetting() is required to enable the setting
    ///@warning Some settings are writable in SMBus mode, but not reflected in operation
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
      @note Valid range between -38.2f and 124.8f
     */
    bool writeAmbientMinMax(const float taMin, const float taMax, const bool apply = true);
    ///@}

    ///@note If apply is false, a POR or call applySetting() is required to enable the setting
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

    /*!
      @brief Sleep
      @return True if successful
     */
    bool sleep();
    /*!
      @brief Wakeup
      @return True if successful
     */
    bool wakeup();
    /*!
      @brief Apply EEPROM settings
      @return True if successful
      @note After writing to EEPROM , a reset or sleep.wakeup is required for the settings to take effect
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
    bool read_measurement(mlx90614::Data& d, const uint16_t config);

    bool start_periodic_measurement(const mlx90614::IIR iir, const mlx90614::FIR fir, const mlx90614::Gain gain,
                                    const mlx90614::IRSensor irs);
    bool start_periodic_measurement();
    bool stop_periodic_measurement();

    M5_UNIT_COMPONENT_PERIODIC_MEASUREMENT_ADAPTER_HPP_BUILDER(UnitMLX90614, mlx90614::Data);

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
  @class UnitMLX90614BAA
  @brief For UnitMLX90614BAA (NCIR using it)
 */
class UnitMLX90614BAA : public UnitMLX90614 {
    M5_UNIT_COMPONENT_HPP_BUILDER(UnitMLX90614BAA, 0x5A);

public:
    explicit UnitMLX90614BAA(const uint8_t addr = DEFAULT_ADDRESS) : UnitMLX90614(addr)
    {
    }

protected:
    virtual uint32_t get_interval(const mlx90614::IIR iir, const mlx90614::FIR fir);
    inline virtual bool has_dual_sensors() const
    {
        return true;
    }
};

///@cond 0
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
