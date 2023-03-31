#pragma once

#include <Wire.h>

#include "bosch/bme68x.h"

/// @brief Class for reading Bosch BME68X sensors
class BME68X
{
public:
    /// @brief I2C helper class
    class I2CDevice
    {
        friend BME68X;

    public:
        /// @brief Constructor
        /// @param address I2C device address
        /// @param wire I2C interface to use
        I2CDevice(const uint8_t address = ADDRESS, TwoWire& wire = Wire) : _address(address), _wire(wire) { }

        /// @brief How many bytes can be read in a transaction
        /// @return Size of Wire receive/transmit buffer
        constexpr size_t maxBufferSize()
        {
#ifdef ARDUINO_ARCH_SAMD
            return 250; // as defined in Wire.h's RingBuffer
#elif defined(ESP32)
            return I2C_BUFFER_LENGTH;
#else
            return 32;
#endif
        }

        /// @brief  Read from I2C into a buffer from the I2C device. Cannot be more than maxBufferSize() bytes.
        /// @param buffer Pointer to buffer of data to read into
        /// @param len Number of bytes from buffer to read.
        /// @param stop Whether to send an I2C STOP signal on read
        /// @return True if read was successful, false if not.
        bool read(uint8_t* buffer, const size_t len, const bool stop = true)
        {
            size_t pos = 0;
            while (pos < len)
            {
                const size_t readLen = ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
                const bool read_stop = (pos < (len - readLen)) ? false : stop;
                if (!_read(buffer + pos, readLen, read_stop))
                {
                    return false;
                }
                pos += readLen;
            }
            return true;
        }

        /// @briefWrite a buffer or two to the I2C device. Cannot be more than maxBufferSize() bytes.
        /// @param buffer Pointer to buffer of data to write. This is const to ensure the content of this buffer doesn't
        /// change.
        /// @param len Number of bytes from buffer to write
        /// @param stop Whether to send an I2C STOP signal on write
        /// @param prefixBuffer Pointer to optional array of data to write before buffer. Cannot be more than
        /// maxBufferSize() bytes. This is const to ensure the content of this buffer doesn't change.
        /// @param prefixLen Number of bytes from prefix buffer to write
        /// @return True if write was successful, false if not.
        bool write(const uint8_t* buffer, const size_t len, const bool stop = true,
            const uint8_t* prefixBuffer = nullptr, const size_t prefixLen = 0)
        {
            if ((len + prefixLen) > maxBufferSize())
            {
                return false;
            }

            _wire.beginTransmission(_address);

            // Write the prefix data (usually an address)
            if ((prefixLen != 0) && (prefixBuffer != nullptr))
            {
                if (_wire.write(prefixBuffer, prefixLen) != prefixLen)
                {
                    return false;
                }
            }

            // Write the data itself
            if (_wire.write(buffer, len) != len)
            {
                return false;
            }

            return _wire.endTransmission(stop) == 0;
        }

        /// @brief Write some data, then read some data from I2C into another buffer. Cannot be more than
        /// maxBufferSize() bytes. The buffers can point to same/overlapping locations.
        /// @param writeBuffer Pointer to buffer of data to write from
        /// @param writeLen Number of bytes from buffer to write.
        /// @param readBuffer Pointer to buffer of data to read into.
        /// @param readLen Number of bytes from buffer to read.
        /// @param stop Whether to send an I2C STOP signal between the write and read
        /// @return True if write & read where successful, false if not.
        bool writeThenRead(const uint8_t* writeBuffer, const size_t writeLen, uint8_t* readBuffer, const size_t readLen,
            const bool stop)
        {
            return write(writeBuffer, writeLen, stop) && read(readBuffer, readLen);
        }

    protected:
        bool _read(uint8_t* buffer, const size_t len, const bool stop)
        {
#if defined(TinyWireM_h)
            const size_t recv = _wire.requestFrom((uint8_t)_address, (uint8_t)len);
#elif defined(ARDUINO_ARCH_MEGAAVR)
            const size_t recv = _wire.requestFrom(_address, len, stop);
#else
            const size_t recv = _wire.requestFrom((uint8_t)_address, (uint8_t)len, (uint8_t)stop);
#endif

            if (recv != len)
            {
                // Not enough data available to fulfill our obligation!
                return false;
            }

            for (uint16_t i = 0; i < len; ++i)
            {
                buffer[i] = _wire.read();
            }

            return true;
        }

    protected:
        /// @brief I2C device address
        uint8_t _address;
        /// @brief I2C interface
        TwoWire& _wire;
    };

    /// @brief Oversampling amounts
    enum class Oversampling
    {
        /// @brief No measurement
        NONE = BME68X_OS_NONE,
        /// @brief Perform 1 measurement
        X1 = BME68X_OS_1X,
        /// @brief Perform 2 measurements
        X2 = BME68X_OS_2X,
        /// @brief Perform 4 measurements
        X4 = BME68X_OS_4X,
        /// @brief Perform 8 measurements
        X8 = BME68X_OS_8X,
        /// @brief Perform 16 measurements
        X16 = BME68X_OS_16X,
    };

    /// @brief Filter sizes
    enum class FilterSize
    {
        /// @brief Filter off
        OFF = BME68X_FILTER_OFF,
        /// @brief Filter coefficient 2
        SIZE_1 = BME68X_FILTER_SIZE_1,
        /// @brief Filter coefficient 4
        SIZE_3 = BME68X_FILTER_SIZE_3,
        /// @brief Filter coefficient 8
        SIZE_7 = BME68X_FILTER_SIZE_7,
        /// @brief Filter coefficient 16
        SIZE_15 = BME68X_FILTER_SIZE_15,
        /// @brief Filter coefficient 32
        SIZE_31 = BME68X_FILTER_SIZE_31,
        /// @brief Filter coefficient 64
        SIZE_63 = BME68X_FILTER_SIZE_63,
        /// @brief Filter coefficient 128
        SIZE_127 = BME68X_FILTER_SIZE_127,
    };

    /// @brief Output data rates
    enum class ODR
    {
        /// @brief No standby time
        NONE = BME68X_ODR_NONE,
        /// @brief 0.59ms standby time
        MS_0_59 = BME68X_ODR_0_59_MS,
        /// @brief 10ms standby time
        MS_10 = BME68X_ODR_10_MS,
        /// @brief 20ms standby time
        MS_20 = BME68X_ODR_20_MS,
        /// @brief 62.5ms standby time
        MS_62_5 = BME68X_ODR_62_5_MS,
        /// @brief 125ms standby time
        MS_125 = BME68X_ODR_125_MS,
        /// @brief 250ms standby time
        MS_250 = BME68X_ODR_250_MS,
        /// @brief 500ms standby time
        MS_500 = BME68X_ODR_500_MS,
        /// @brief 1000ms standby time
        MS_1000 = BME68X_ODR_1000_MS,
    };

public:
    /// @brief Instantiate sensor
    /// @param wire Optional I2C interface
    BME68X(TwoWire& wire = Wire) : _wire(wire) { }

    /// @brief Set a temperature offset in °C
    /// @note Should be called before a sensor reading is requested
    /// @param tOffset Offset in °C
    void setTOffset(const float tOffset = 0.0f)
    {
        _device.calib.t_fine_offset
            = tOffset == 0 ? 0 : std::copysign(((int32_t(std::abs(tOffset) * 100) << 8) - 128) / 5, tOffset);
    }

    /// @brief Initialize the sensor and read its calibration data
    /// @param address I2C address
    /// @return True on success, false if not
    bool begin(const uint8_t address = ADDRESS)
    {
        _i2c._address = address;
        _i2c._wire = _wire;

        _device.chip_id = address;
        _device.intf = BME68X_I2C_INTF;
        _device.intf_ptr = &_i2c;
        _device.read = &read;
        _device.write = &write;

        // The ambient temperature in deg C is used for defining the heater temperature
        _device.amb_temp = 25;
        _device.delay_us = delayUS;

        // Make sure offset is 0
        _device.calib.t_fine_offset = 0;

        int8_t result = bme68x_init(&_device);
        if (result != BME68X_OK)
        {
            return false;
        }

        setGasHeater(0, 0);

        // don't do anything till we request a reading
        result = bme68x_set_op_mode(BME68X_FORCED_MODE, &_device);
        if (result != BME68X_OK)
        {
            return false;
        }
        return true;
    }

    /// @brief Set multiple configurations at once to save time
    /// @param humidity Humidity oversampling
    /// @param temperature Temperature oversampling
    /// @param pressure Pressure oversampling
    /// @param iir IIR filter size
    /// @param odr Output data rate
    /// @return True on success, false if not
    bool setConfig(const Oversampling humidity, const Oversampling temperature, const Oversampling pressure,
        const FilterSize iir, const ODR odr = ODR::NONE)
    {
        _config.os_hum = static_cast<uint8_t>(humidity);
        _config.os_temp = static_cast<uint8_t>(temperature);
        _config.os_pres = static_cast<uint8_t>(pressure);
        _config.filter = static_cast<uint8_t>(iir);
        _config.odr = static_cast<uint8_t>(odr);
        return BME68X_OK == bme68x_set_conf(&_config, &_device);
    }
    /// @brief Set humidity oversampling amount
    /// @param os Amount of oversampling
    /// @return True on success, false if not
    bool setHumidityOversampling(const Oversampling os)
    {
        _config.os_hum = static_cast<uint8_t>(os);
        return BME68X_OK == bme68x_set_conf(&_config, &_device);
    }
    /// @brief Set temperature oversampling amount
    /// @param os Amount of oversampling
    /// @return True on success, false if not
    bool setTemperatureOversampling(const Oversampling os)
    {
        _config.os_temp = static_cast<uint8_t>(os);
        return BME68X_OK == bme68x_set_conf(&_config, &_device);
    }
    /// @brief Set pressure oversampling amount
    /// @param os Amount of oversampling
    /// @return True on success, false if not
    bool setPressureOversampling(const Oversampling os)
    {
        _config.os_pres = static_cast<uint8_t>(os);
        return BME68X_OK == bme68x_set_conf(&_config, &_device);
    }
    /// @brief Set IIR filter size
    /// @param fs Filter size
    /// @return True on success, false if not
    bool setIIRFilterSize(const FilterSize fs)
    {
        _config.filter = static_cast<uint8_t>(fs);
        return BME68X_OK == bme68x_set_conf(&_config, &_device);
    }
    /// @brief Set Output Data Rate
    /// @param odr Rate
    /// @return True on success, false if not
    bool setODR(const ODR odr)
    {
        _config.odr = static_cast<uint8_t>(odr);
        return BME68X_OK == bme68x_set_conf(&_config, &_device);
    }
    /// @brief Enable and configure gas reading and heater
    /// @param heaterTemp Desired temperature in °C
    /// @param heaterTime Time to keep heater on in milliseconds
    /// @return True on success, false if not
    bool setGasHeater(const uint16_t heaterTemp, const uint16_t heaterTime)
    {
        if ((heaterTemp == 0) || (heaterTime == 0))
        {
            _heaterConfig.enable = BME68X_DISABLE;
            _heaterConfig.heatr_dur = 0;
        }
        else
        {
            _heaterConfig.enable = BME68X_ENABLE;
            _heaterConfig.heatr_temp = heaterTemp;
            _heaterConfig.heatr_dur = heaterTime;
        }
        return BME68X_OK == bme68x_set_heatr_conf(BME68X_FORCED_MODE, &_heaterConfig, &_device);
    }

    /// @brief Performs a full reading of all 4 sensors in the BME680.
    /// @note Assigns the internal BME68X::temperature, BME68X::pressure, BME68X::humidity and BME68X::gasResistance
    /// member variables
    /// @return True on success, false if not
    bool performReading() { return endReading(); }

    /// @brief Begin an asynchronous reading.
    /// @return When the reading would be ready as absolute time in milliseconds.
    uint64_t beginReading()
    {
        if (_measStart != 0)
        {
            // A measurement is already in progress
            return _measStart + _measPeriod;
        }

        if (bme68x_set_op_mode(BME68X_FORCED_MODE, &_device) != BME68X_OK)
        {
            return false;
        }

        // Calculate delay period in milliseconds
        _measStart = millis();
        _measPeriod = bme68x_get_meas_dur(BME68X_FORCED_MODE, &_config, &_device) / 1000 + _heaterConfig.heatr_dur;

        return _measStart + _measPeriod;
    }

    /// @brief End an asynchronous reading
    /// @note If the asynchronous reading is still in progress, block until it ends.
    /// If no asynchronous reading has started, this is equivalent to performReading().
    /// @return True on success, false if not
    bool endReading()
    {
        if (beginReading() == 0)
        {
            return false;
        }

        const int remaining_millis = remainingReadingMillis();
        if (remaining_millis > 0)
        {
            // Delay till the measurement is ready
            // Serial.printf("d%d\n", remaining_millis);
            // delay(remaining_millis * 2);
            delay(remaining_millis);
        }
        // Allow new measurement to begin
        _measStart = 0;
        _measPeriod = 0;

        bme68x_data data;
        uint8_t nFields;
        if (bme68x_get_data(BME68X_FORCED_MODE, &data, &nFields, &_device) != BME68X_OK)
        {
            return false;
        }

        if (nFields)
        {
            temperature = data.temperature;
            humidity = data.humidity;
            pressure = data.pressure;

            if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK))
            {
                gasResistance = data.gas_resistance;
            }
            else
            {
                gasResistance = 0;
            }
        }
        return true;
    }

    /// @brief Get remaining time for an asynchronous reading in milliseconds
    /// @note  If the asynchronous reading is still in progress, the time until completion in ms is returned.
    /// If the asynchronous reading is completed, 0 is returned.
    /// If no asynchronous reading has been started, -1 is returned.
    /// Does not block.
    /// @return Time unitl completion in ms, READING_COMPLETE (0) if complete and READING_NOT_STARTED (-1) if reading
    /// has not been started
    int64_t remainingReadingMillis() const
    {
        if (_measStart > 0)
        {
            // Measurement is already in progress
            const int64_t remaining_time = _measPeriod - (millis() - _measStart);
            return remaining_time < 0 ? READING_COMPLETE : remaining_time;
        }
        return READING_NOT_STARTED;
    }

private:
    /// @brief I2C reading callback
    /// @param reg_addr
    /// @param reg_data
    /// @param len
    /// @param intf
    /// @return
    static int8_t read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf)
    {
        return ((I2CDevice*)intf)->writeThenRead(&reg_addr, 1, reg_data, len, true) ? 0 : -1;
    }

    /// @brief I2C writing callback
    /// @param reg_addr
    /// @param reg_data
    /// @param len
    /// @param intf
    /// @return
    static int8_t write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf)
    {
        return ((I2CDevice*)intf)->write((uint8_t*)reg_data, len, true, &reg_addr, 1) ? 0 : -1;
    }

    /// @brief Delay callback
    /// @param us Delay in microseconds
    /// @param
    static void delayUS(uint32_t us, void*)
    {
        delayMicroseconds(us);
        yield();
    }

public:
    /// @brief Default device address
    static constexpr uint8_t ADDRESS = 0x77;
    /// @brief Indicates no asynchronous reading has been initiated by beginReading.
    static constexpr int READING_NOT_STARTED = -1;
    /// @brief Indicates asynchronous reading is complete and calling endReading will not block
    static constexpr int READING_COMPLETE = 0;

    /// @brief Temperature in °C
    float temperature;
    /// @brief Pressure in Pascals
    uint32_t pressure;
    /// @brief Relative humidity in %
    float humidity;
    /// @brief Gas resistance in Ohms
    uint32_t gasResistance;

private:
    /// @brief I2C interface
    TwoWire& _wire;
    /// @brief Helper for I2C commmunication
    I2CDevice _i2c;
    /// @brief Bosch BME68X device data
    bme68x_dev _device;
    /// @brief Bosch BME68X device config
    bme68x_conf _config;
    /// @brief Bosch BME68X device heater config
    bme68x_heatr_conf _heaterConfig;

    /// @brief Start time of measurement in ms
    uint64_t _measStart = 0;
    /// @brief Measure period in ms
    uint16_t _measPeriod = 0;
};
