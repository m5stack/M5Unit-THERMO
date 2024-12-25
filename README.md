# M5Unit - THERMO

## Overview

Library for THERMO using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnified is a library for unified handling of various M5 units products.

### SKU:U028
NCIR featured with built-in infrared MLX90614 sensor, It can be used to measure the surface temperature of a human body or other object.

Unlike most temperature sensors, this sensor measures infrared light bouncing off of remote objects so it can sense temperature without having to touch them physically.

Simply point the sensor towards what you want to measure and it will detect the temperature by absorbing IR waves emitted. Because it doesn't have to touch the object it's measuring, it can sense a wider range of temperatures than most digital sensors! It takes the measurement over an 90-degree field of view so it can be handy for determining the average temperature of an area.
The MLX90614 is factory calibrated in wide temperature ranges: -40 to 125 ˚C for the ambient temperature and -70 to 380 ˚C for the object temperature.

## Related Link
See also examples using conventional methods here.

- [Unit NCIR & Datasheet](https://docs.m5stack.com/en/unit/ncir)

### Required Libraries:
- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified) 0.0.3 or later
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
- [Doxyegn](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)


