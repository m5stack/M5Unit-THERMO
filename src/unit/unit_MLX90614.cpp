/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*!
  @file unit_MLX90614.cpp
  @brief MLX90614 Unit for M5UnitUnified
*/
#include "unit_MLX90614.hpp"
#include <M5Utility.hpp>
#include <array>

using namespace m5::utility::mmh3;
using namespace m5::unit::types;
using namespace m5::unit::mlx90614;
using namespace m5::unit::mlx90614::command;

namespace {

struct Flag {
    // EEPROM access still in progress if true
    inline bool EEBusy() const
    {
        return value & (1U << 7);
    }
    // EEPROM double error has occurred if true
    inline bool EEDead() const
    {
        return value & (1U << 5);
    }
    // POR initialization routine is still ongoing if false
    inline bool initialized() const
    {
        return (value & (1U << 4)) == 0;
    }
    uint16_t value{};
};

struct PWMCtrl {
    inline explicit PWMCtrl(const uint16_t v = 0) : value{v}
    {
    }
    enum class Mode { Extended, Single };
    inline Mode mode() const
    {
        return static_cast<Mode>(value & 0x01);
    }
    inline bool enabled() const
    {
        return value & (1U << 1);
    }
    enum class Pin { OpenDrain, PushPull };
    inline Pin pin() const
    {
        return static_cast<Pin>((value >> 2) & 0x01);
    }
    inline bool thermalRelayMode() const
    {
        return (value >> 3) & 0x01;
    }
    inline uint16_t repetition() const
    {
        return (value >> 4) & 0x1F;
    }
    inline uint16_t period_raw() const
    {
        return (value >> 9) & 0x7F;
    }
    inline float period() const
    {
        // ms
        return 1.024f * ((mode() == Mode::Single) ? 1.f : 2.f) * (((value >> 9) & 0x7F) ? ((value >> 9) & 0x7F) : 128);
    }
    uint16_t value{};
};

struct Config {
    inline explicit Config(const uint16_t v = 0) : value{v}
    {
    }
    inline IIR iir() const
    {
        return static_cast<IIR>(value & 0x07);
    }
    inline Output output() const
    {
        return static_cast<Output>((value >> 4) & 0x03);
    }
    inline FIR fir() const
    {
        return static_cast<FIR>((value >> 8) & 0x07);
    }
    inline Gain gain() const
    {
        return static_cast<Gain>((value >> 11) & 0x07);
    }
    inline IRSensor irSensor() const
    {
        return static_cast<IRSensor>((value >> 6) & 0x01);
    }
    inline bool positiveKs() const
    {
        return value & (1U << 7);
    }
    inline bool positiveKf2() const
    {
        return value & (1U << 14);
    }
    //
    inline void iir(const IIR iir)
    {
        value = (value & ~0x07) | m5::stl::to_underlying(iir);
    }
    inline void output(const Output o)
    {
        value = (value & ~(0x03 << 4)) | (m5::stl::to_underlying(o) << 4);
    }
    inline void fir(const FIR fir)
    {
        value = (value & ~(0x07 << 8)) | (m5::stl::to_underlying(fir) << 8);
    }
    inline void gain(const Gain gain)
    {
        value = (value & ~(0x07 << 11)) | (m5::stl::to_underlying(gain) << 11);
    }
    inline void irSensor(const IRSensor irs)
    {
        value = (value & ~(1U << 6)) | (m5::stl::to_underlying(irs) << 6);
    }
    inline void positiveKs(const bool pos)
    {
        value = (value & ~(1U << 7)) | ((uint16_t)pos << 7);
    }
    inline void positiveKf2(const bool pos)
    {
        value = (value & ~(1U << 14)) | ((uint16_t)pos << 14);
    }
    uint16_t value{};
};

// [IIR][FIR]
// FIR 000...011 are NOT RECOMMENDED
// For MLX90614A Series
constexpr uint32_t interval_tableA[8][4] = {
    {300, 370, 540, 860}, {700, 880, 1300, 2000}, {1100, 1400, 2000, 3300}, {1500, 1900, 2800, 4500},
    {40, 50, 60, 100},    {120, 160, 220, 350},   {240, 300, 430, 700},     {260, 340, 480, 780},
};
// For MLX90614B,D Series
constexpr uint32_t interval_tableBD[8][4] = {
    {470, 600, 840, 1330}, {1100, 1400, 2000, 3200}, {1800, 2200, 3200, 5000}, {2400, 3000, 4300, 7000},
    {60, 70, 100, 140},    {200, 240, 340, 540},     {380, 480, 670, 1100},    {420, 530, 750, 1200},
};

inline float toRaw_to_celsius(const uint16_t t)
{
    return t * 0.01f - 273.15f;
}

inline uint16_t celsius_to_toRaw(const float c)
{
    float v = std::fmax(std::fmin(c, 382.2f), -273.15f);
    return 100 * (v + 0.005f + 273.15f);
}

inline float taRaw_to_celsius(const uint8_t t)
{
    return t * 64.0f / 100.f - 38.2f;
}

inline uint8_t celsius_to_taRaw(const float c)
{
    float v = std::fmax(std::fmin(c, 125.f), -38.2f);
    return 100 * (v + 0.32f + 38.2f) / 64.0f;
}

inline float raw_to_emissivity(const uint16_t e)
{
    return e / 65535.f;
}

inline uint16_t emissivity_to_raw(const float e)
{
    return std::round(65535.f * e);
}

}  // namespace

namespace m5 {
namespace unit {
// class UnitMLX90614
const char UnitMLX90614::name[] = "UnitMLX90614";
const types::uid_t UnitMLX90614::uid{"UnitMLX90614"_mmh3};
const types::attr_t UnitMLX90614::attr{attribute::AccessI2C};

bool UnitMLX90614::begin()
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

    // Sleep and wakeup
    applySettings();

    //
    if (!read_eeprom(_eeprom)) {
        M5_LIB_LOGE("Failed to read EEPROM");
        return false;
    }
    M5_LIB_LOGV(
        "toMax:%u(%f) toMin:%u(%f) pwm:%04X TaRange:%X(%f,%f) emmiss:%04X config:%04X\n"
        "addr:%04X ID:%04X:%04X:%04X:%04X",
        _eeprom.toMax, toRaw_to_celsius(_eeprom.toMax), _eeprom.toMin, toRaw_to_celsius(_eeprom.toMin), _eeprom.pwmCtrl,
        _eeprom.taRange, taRaw_to_celsius((_eeprom.taRange) >> 8 & 0xFF), taRaw_to_celsius(_eeprom.taRange & 0xFF),
        _eeprom.emissivity, _eeprom.config, _eeprom.addr, _eeprom.id[0], _eeprom.id[1], _eeprom.id[2], _eeprom.id[3]);

    PWMCtrl pc;
    pc.value = _eeprom.pwmCtrl;
    M5_LIB_LOGV("Mode:%u Enabled:%u Pin:%u Thermal:%u Rep:%u Period:%X/%f", pc.mode(), pc.enabled(), pc.pin(),
                pc.thermalRelayMode(), pc.repetition(), pc.period_raw(), pc.period());
    Config c{_eeprom.config};
    M5_LIB_LOGV("IIR:%u OUT:%u FIR:%u Gain:%u IRS:%u PosK:%u PosKf2:%u", c.iir(), c.output(), c.fir(), c.gain(),
                c.irSensor(), c.positiveKs(), c.positiveKf2());

    return _cfg.start_periodic ? writeEmissivity(_cfg.emissivity, false) &&
                                     startPeriodicMeasurement(_cfg.iir, _cfg.fir, _cfg.gain, _cfg.irs)
                               : true;
}

void UnitMLX90614::update(const bool force)
{
    _updated = false;
    if (inPeriodic()) {
        elapsed_time_t at{m5::utility::millis()};
        if (force || !_latest || at >= _latest + _interval) {
            Data d{};
            _updated = read_measurement(d, _eeprom.config);
            if (_updated) {
                _latest = at;
                _data->push_back(d);
            }
        }
    }
}

bool UnitMLX90614::start_periodic_measurement(const mlx90614::IIR iir, const mlx90614::FIR fir,
                                              const mlx90614::Gain gain, const mlx90614::IRSensor irs)
{
    Config c{};
    if (readConfig(c.value)) {
        c.iir(iir);
        c.fir(fir);
        c.gain(gain);
        c.irSensor(irs);
        return writeConfig(c.value) && start_periodic_measurement();
    }
    return false;
}

bool UnitMLX90614::start_periodic_measurement()
{
    if (inPeriodic()) {
        return false;
    }

    Config c(_eeprom.config);
    _interval = get_interval(c.iir(), c.fir());
    _periodic = true;
    _latest   = 0;

    // M5_LIB_LOGW("IIR:%u FIR:%u IT:%u", c.iir(), c.fir(), _interval);

    return true;
}

bool UnitMLX90614::stop_periodic_measurement()
{
    _periodic = false;
    return true;
}

bool UnitMLX90614::readConfig(uint16_t& v)
{
    return read_register16(EEPROM_CONFIG, v);
}

bool UnitMLX90614::writeConfig(const uint16_t v, const bool apply)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    if (write_eeprom(EEPROM_CONFIG, v, apply)) {
        _eeprom.config = v;
        return true;
    }
    return false;
}

bool UnitMLX90614::readOutput(mlx90614::Output& o)
{
    Config c{};
    if (readConfig(c.value)) {
        o = c.output();
        return true;
    }
    return false;
}

bool UnitMLX90614::writeOutput(const mlx90614::Output o, const bool apply)
{
    Config c{};
    if (readConfig(c.value)) {
        c.output(o);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readIIR(mlx90614::IIR& iir)
{
    Config c{};
    if (readConfig(c.value)) {
        iir = c.iir();
        return true;
    }
    return false;
}

bool UnitMLX90614::writeIIR(const mlx90614::IIR iir, const bool apply)
{
    Config c{};
    if (readConfig(c.value)) {
        c.iir(iir);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readFIR(mlx90614::FIR& fir)
{
    Config c{};
    if (readConfig(c.value)) {
        fir = c.fir();
        return true;
    }
    return false;
}

bool UnitMLX90614::writeFIR(const mlx90614::FIR fir, const bool apply)
{
    // WARNING
    // datasheet says...
    if (m5::stl::to_underlying(fir) < 4) {
        M5_LIB_LOGW("Settings below FIR::Filter64 are not recommended");
    }

    Config c{};
    if (readConfig(c.value)) {
        c.fir(fir);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readGain(mlx90614::Gain& gain)
{
    Config c{};
    if (readConfig(c.value)) {
        gain = c.gain();
        return true;
    }
    return false;
}

bool UnitMLX90614::writeGain(const mlx90614::Gain gain, const bool apply)
{
    Config c{};
    if (readConfig(c.value)) {
        c.gain(gain);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readIRSensor(mlx90614::IRSensor& irs)
{
    Config c{};
    if (readConfig(c.value)) {
        irs = c.irSensor();
        return true;
    }
    return false;
}

bool UnitMLX90614::writeIRSensor(const mlx90614::IRSensor irs, const bool apply)
{
    Config c{};
    if (readConfig(c.value)) {
        c.irSensor(irs);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readPositiveKs(bool& pos)
{
    Config c{};
    if (readConfig(c.value)) {
        pos = c.positiveKs();
        return true;
    }
    return false;
}

bool UnitMLX90614::writePositiveKs(const bool pos, const bool apply)
{
    Config c{};
    if (readConfig(c.value)) {
        c.positiveKs(pos);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readPositiveKf2(bool& pos)
{
    Config c{};
    if (readConfig(c.value)) {
        pos = c.positiveKf2();
        return true;
    }
    return false;
}

bool UnitMLX90614::writePositiveKf2(const bool pos, const bool apply)
{
    Config c{};
    if (readConfig(c.value)) {
        c.positiveKf2(pos);
        return writeConfig(c.value, apply);
    }
    return false;
}

bool UnitMLX90614::readObjectMinMax(uint16_t& toMin, uint16_t& toMax)
{
    return read_register16(EEPROM_TO_MIN, toMin) && read_register16(EEPROM_TO_MAX, toMax);
}

bool UnitMLX90614::readObjectMinMax(float& toMin, float& toMax)
{
    uint16_t tmin{}, tmax{};
    if (readObjectMinMax(tmin, tmax)) {
        toMin = toRaw_to_celsius(tmin);
        toMax = toRaw_to_celsius(tmax);
        return true;
    }
    return false;
}

bool UnitMLX90614::write_object_minmax(const uint16_t toMin, const uint16_t toMax, const bool apply)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    if (toMin > toMax) {
        M5_LIB_LOGE("Need %u <= %u", toMin, toMax);
        return false;
    }
    if (write_eeprom(EEPROM_TO_MIN, toMin, apply) && write_eeprom(EEPROM_TO_MAX, toMax, apply)) {
        _eeprom.toMax = toMax;
        _eeprom.toMin = toMin;
        return apply ? applySettings() : true;
    }
    return false;
}

bool UnitMLX90614::writeObjectMinMax(const float toMin, const float toMax, const bool apply)
{
    return write_object_minmax(celsius_to_toRaw(toMin), celsius_to_toRaw(toMax), apply);
}

bool UnitMLX90614::readAmbientMinMax(uint8_t& taMin, uint8_t& taMax)
{
    uint16_t v{};
    if (read_register16(EEPROM_TARANGE, v)) {
        taMin = v & 0xFF;
        taMax = (v >> 8) & 0xFF;
        return true;
    }
    return false;
}

bool UnitMLX90614::readAmbientMinMax(float& taMin, float& taMax)
{
    uint8_t amin{}, amax{};
    if (readAmbientMinMax(amin, amax)) {
        taMin = taRaw_to_celsius(amin);
        taMax = taRaw_to_celsius(amax);
        return true;
    }
    return false;
}

bool UnitMLX90614::write_ambient_minmax(const uint8_t taMin, const uint8_t taMax, const bool apply)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    if (taMin > taMax) {
        M5_LIB_LOGE("Need %u <= %u", taMin, taMax);
        return false;
    }
    uint16_t v{};
    v = (uint16_t)taMax << 8 | taMin;
    if (write_eeprom(EEPROM_TARANGE, v, apply)) {
        _eeprom.taRange = v;
        return apply ? applySettings() : true;
    }
    return false;
}

bool UnitMLX90614::writeAmbientMinMax(const float taMin, const float taMax, const bool apply)
{
    return write_ambient_minmax(celsius_to_taRaw(taMin), celsius_to_taRaw(taMax), apply);
}

bool UnitMLX90614::readEmissivity(uint16_t& emiss)
{
    return read_register16(EEPROM_EMISSIVITY, emiss);
}

bool UnitMLX90614::readEmissivity(float& emiss)
{
    uint16_t v{};
    if (readEmissivity(v)) {
        emiss = raw_to_emissivity(v);
        return true;
    }
    return false;
}

bool UnitMLX90614::write_emissivity(const uint16_t emiss, const bool apply)
{
    if (inPeriodic()) {
        M5_LIB_LOGD("Periodic measurements are running");
        return false;
    }

    if (write_eeprom(EEPROM_EMISSIVITY, emiss, apply)) {
        _eeprom.emissivity = emiss;
        return apply ? applySettings() : true;
    }
    return false;
}

bool UnitMLX90614::writeEmissivity(const float emiss, const bool apply)
{
    if (emiss < 0.1f || emiss > 1.0f) {
        M5_LIB_LOGE("Emissivity must be between 0.1 and 1.0 (%f)", emiss);
        return false;
    }
    return write_emissivity(emissivity_to_raw(emiss), apply);
}

bool UnitMLX90614::readI2CAddress(uint8_t& i2c_address)
{
    uint16_t a{};
    if (read_register16(EEPROM_ADDR, a)) {
        i2c_address = a & 0xFF;
        return true;
    }
    return false;
}

bool UnitMLX90614::changeI2CAddress(const uint8_t i2c_address)
{
    if (!m5::utility::isValidI2CAddress(i2c_address)) {
        M5_LIB_LOGE("Invalid address : %02X", i2c_address);
        return false;
    }
    return write_eeprom(EEPROM_ADDR, i2c_address) && changeAddress(i2c_address) &&
           read_register16(EEPROM_ADDR, _eeprom.addr);
}

bool UnitMLX90614::sleep()
{
#if defined(ARDUINO)
    auto ada = adapter();
    auto scl = ada->scl();
    if (scl < 0) {
        M5_LIB_LOGE("SCL pin cannot be detcted");
        return false;
    }

    // Slave W, reg, PEC
    uint8_t buf[3]{(uint8_t)(address() << 1), COMMAND_ENTER_SLEEP};
    m5::utility::CRC8 crc8(0x00, 0x07, false, false, 0x00);  // CRC8-SMBus
    buf[2] = crc8.update(buf, 2);
    if (writeRegister(COMMAND_ENTER_SLEEP, buf + 2, 1) && ada->end() /* Keep peripheral settings */) {
        /*
          datasheet says:
          As a result, this pin needs to be forced low in sleep mode and the pull-up on the SCL line needs to be
          disabled inorder to keep the overall power drain in sleep mode really small.
         */
        ada->pinMode(scl, OUTPUT);
        ada->digitalWrite(scl, LOW);
        ada->pinMode(scl, INPUT);
        return true;
    }
    return false;
#else
#pragma message "Implement for M5HAL not yet"
    return false;
#endif
}

bool UnitMLX90614::wakeup()
{
#if defined(ARDUINO)
    auto ada = adapter();
    auto scl = ada->scl();
    auto sda = ada->sda();
    if (scl < 0 || sda < 0) {
        M5_LIB_LOGE("SCL or SDA pin cannot be detcted %d,%d", scl, sda);
        return false;
    }
    // Wakeup request (SDA low) 33ms min
    // SCL pin high and then PWM/SDA pin low for at least tDDQ > 33ms
    ada->pinMode(scl, INPUT);  // SCL H
    ada->pinMode(sda, OUTPUT);
    ada->digitalWrite(sda, LOW);  // SDA L
    m5::utility::delay(33 * 1.5f);
    // After wake up the first data is available after 0.25 seconds (typ).
    ada->pinMode(sda, INPUT);  // SDA H
    delay(550);

    return ada->begin();  // restart Wire

#else
#pragma message "Implement for M5HAL not yet"
    return false;
#endif
}

//
bool UnitMLX90614::read_eeprom(mlx90614::EEPROM& e)
{
    return read_register16(EEPROM_TO_MAX, e.toMax) && read_register16(EEPROM_TO_MIN, e.toMin) &&
           read_register16(EEPROM_PWMCTRL, e.pwmCtrl) && read_register16(EEPROM_TARANGE, e.taRange) &&
           read_register16(EEPROM_EMISSIVITY, e.emissivity) && read_register16(EEPROM_CONFIG, e.config) &&
           read_register16(EEPROM_ADDR, e.addr) && read_register16(EEPROM_ID0, e.id[0]) &&
           read_register16(EEPROM_ID1, e.id[1]) && read_register16(EEPROM_ID2, e.id[2]) &&
           read_register16(EEPROM_ID3, e.id[3]);
}

bool UnitMLX90614::read_register16(const uint8_t reg, uint16_t& v, const bool stopbit)
{
    // Slave W, reg, Slave R (for CRC), Low, High, PEC( Packet error code)
    uint8_t rbuf[6]{(uint8_t)(address() << 1), reg, (uint8_t)((address() << 1) | 0x01)};
    m5::utility::CRC8 crc8(0x00, 0x07, false, false, 0x00);  // CRC8-SMBus

    // Read 3bytes (low,high,PEC) and check PEC
    uint8_t crc{};
    if (readRegister(reg, rbuf + 3, 3, 0, stopbit) && (crc = crc8.update(rbuf, 5)) == rbuf[5]) {
        v = ((uint16_t)rbuf[4] << 8) | rbuf[3];
        return true;
    }
    //    M5_LIB_LOGD("R:%02X SB:%u CRC8:%02X PEC:%02X", reg, stopbit, crc, rbuf[5]);
    //    M5_DUMPD(rbuf, 6);
    return false;
}

bool UnitMLX90614::write_register16(const uint8_t reg, const uint16_t val)
{
    // Slave W, reg, Low, High, PEC
    uint8_t buf[5]{
        (uint8_t)(address() << 1),
        reg,
    };
    m5::utility::CRC8 crc8(0x00, 0x07, false, false, 0x00);  // CRC8-SMBus

    buf[2] = val & 0xFF;
    buf[3] = (val >> 8) & 0xFF;
    buf[4] = crc8.update(buf, 4);
    return writeRegister(reg, buf + 2, 3);
}

bool UnitMLX90614::write_eeprom(const uint8_t reg, const uint16_t val, const bool apply)
{
    if ((reg & COMMAND_EEPROM) == 0) {
        M5_LIB_LOGE("Wrong register %02X", reg);
        return false;
    }

    // Write 0x0000 first (as erase)
    if (write_register16(reg, 0)) {
        m5::utility::delay(10);  // Delay here is required (Typ:5, Max:10)
        // Write value
        if (write_register16(reg, val)) {
            m5::utility::delay(10);
            return apply ? applySettings() : true;
        }
    }
    return false;
}

bool UnitMLX90614::read_measurement(mlx90614::Data& d, const uint16_t cfg)
{
    std::fill(d.raw.begin(), d.raw.end(), 0x8000 /* invalid value */);
    return read_register16(READ_TAMBIENT, d.raw[0]) && read_register16(READ_TOBJECT_1, d.raw[1]) &&
           (has_dual_sensors() ? read_register16(READ_TOBJECT_2, d.raw[2]) : true);
}

uint32_t UnitMLX90614::get_interval(const mlx90614::IIR iir, const mlx90614::FIR fir)
{
    auto i = m5::stl::to_underlying(iir);
    auto f = m5::stl::to_underlying(fir);
    return (f < 4) ? 0 : interval_tableA[i][f - 4];  // A series as default
}

// class UnitMLX90614BAA
const char UnitMLX90614BAA::name[] = "UnitMLX90614BAA";
const types::uid_t UnitMLX90614BAA::uid{"UnitMLX90614BAA"_mmh3};
const types::attr_t UnitMLX90614BAA::attr{attribute::AccessI2C};

uint32_t UnitMLX90614BAA::get_interval(const mlx90614::IIR iir, const mlx90614::FIR fir)
{
    auto i = m5::stl::to_underlying(iir);
    auto f = m5::stl::to_underlying(fir);
    return (f < 4) ? 0 : interval_tableBD[i][f - 4];
}

}  // namespace unit
}  // namespace m5
