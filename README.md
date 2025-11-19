# M5Unit - THERMO

## Overview

Library for THERMO using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnified is a library for unified handling of various M5 units products.

### SKU:U028
NCIR featured with built-in infrared MLX90614 sensor, It can be used to measure the surface temperature of a human body or other object.

Unlike most temperature sensors, this sensor measures infrared light bouncing off of remote objects so it can sense temperature without having to touch them physically.

### SKU:U150
The NCIR 2 Thermometer Unit utilizes the MLX90614 temperature sensor, and take ambient and object temperature readings with No Contact. Great for socially-distant reality.

### SKU:U149
Unit Thermal2 is a thermal imaging acquisition unit equipped with a data processing MCU, featuring the MLX90640 sensor. It has an imaging resolution of 32 x 24 pixels, a field of view of 110° x 75°, and a temperature measurement range of -40°C to 300°C. The MCU is based on the ESP32,


## Related Link
See also examples using conventional methods here.

- [Unit NCIR & Datasheet](https://docs.m5stack.com/en/unit/ncir)
- [Unit NCIR2 & Datasheet](https://docs.m5stack.com/ja/unit/NCIR2)
- [Unit Thermal2 & Datasheet](https://docs.m5stack.com/en/unit/Thermal2)

### Required Libraries:
- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)

## License

- [M5Unit-THERMO - MIT](LICENSE)


## Remarks

### UnitNCIR
The MLX90614 has PWM and SMBus as operating modes.  
This library currently supports only SMBus mode (protocol derived from I2C).

## Examples
See also [examples/UnitUnified](examples/UnitUnified)

### Doxygen document
[GitHub Pages](https://m5stack.github.io/M5Unit-THERMO/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

#### Required
- [Doxygen](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)


