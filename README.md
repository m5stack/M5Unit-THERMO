# M5Unit - THERMO

## Overview

Library for THERMO using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnified is a library for unified handling of various M5 units products.

### SKU:U028
Unit NCIR is a single-point infrared temperature measurement sensor. It features a built-in infrared sensor MLX90614, capable of measuring the surface temperature of the human body or other objects.

Unlike most contact-type sensors, this sensor detects temperature by measuring infrared light emitted from distant objects, without physical contact.

### SKU:U150
Unit NCIR2 is a non-contact infrared single-point temperature measurement unit equipped with a data processing MCU (STM32). It features a temperature measurement range of -70°C to 380°C, with functions such as high/low temperature alarms, RGB LED indicator, and a built-in buzzer.

### SKU:U149
Unit Thermal2 is a thermal imaging acquisition unit equipped with a data processing MCU. It features an MLX90640 thermal imaging sensor with an imaging resolution of 32 x 24 pixels, a field of view of 110° x 75°, and a temperature measurement range of -40°C to 300°C. The MCU adopts an ESP32, enabling functions such as high/low temperature alarms, average/maximum/minimum value reading, and data caching through data processing.


## Related Link

- [Unit NCIR & Datasheet](https://docs.m5stack.com/en/unit/ncir)
- [Unit NCIR2 & Datasheet](https://docs.m5stack.com/en/unit/NCIR2)
- [Unit Thermal2 & Datasheet](https://docs.m5stack.com/en/unit/Thermal2)

## Required Libraries
- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)

## License

- [M5Unit-THERMO - MIT](LICENSE)


## Remarks

### UnitNCIR
The MLX90614 has PWM and SMBus as operating modes.  
This library currently supports only SMBus mode (protocol derived from I2C).

## Support via [PaHub](https://docs.m5stack.com/en/unit/Unit-PaHub%20v2.1)

|Unit|Support|Note|
|---|---|---|
|UnitNCIR|OK||
|UnitNCIR2|OK||
|UnitThermal2|OK||

UnitNCIR and UnitNCIR2 use the same address (0x5A), so connect them to different PaHub channels to use them together.
See also the [ViaPaHub](examples/UnitUnified/ViaPaHub) example (UnitNCIR / UnitNCIR2 / UnitThermal2 on one PaHub) and [M5Unit-HUB](https://github.com/m5stack/M5Unit-HUB)

## Examples
See also [examples/UnitUnified](examples/UnitUnified)

### For ESP-IDF settings

> **NOTE:** The ESP-IDF native build (`idf.py`) targets ESP-IDF **5.1 or later** (5.x and 6.x).

Each example covers a fixed set of units, so the examples need no Kconfig choice. Build them directly:

```sh
cd examples/UnitUnified/UnitNCIR/PlotToSerial
idf.py set-target esp32s3               # or esp32 / esp32c5 / esp32c6 / esp32h2 / esp32p4 / ...
idf.py build flash monitor
```

## Doxygen document
[GitHub Pages](https://m5stack.github.io/M5Unit-THERMO/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required
- [Doxygen](https://www.doxygen.nl/)
- [Git](https://git-scm.com/) (Output commit hash to html)


