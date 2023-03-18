# BME68X
Library for Bosch BME68X sensors which is heavily based on [Adafruit_BME680](https://github.com/adafruit/Adafruit_BME680),
but with some optimizations.

## Changes and optimizations
- Use enumerations for `Oversampling`, `FilterSize` and `ODR` settings (Removes extra if statements)
- Add `setConfig` function for setting humidity, temperature and pressure oversampling as well as irr filter size (Code speed up by ~1second)
- Option to set a temperature calibration offset with `setTOffset` which has an influence on humidity and pressure