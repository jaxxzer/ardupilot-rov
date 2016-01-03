/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_MS5803_H__
#define __AP_BARO_MS5803_H__

#include <AP_HAL.h>
#include "AP_Baro.h"

#if CONFIG_HAL_BOARD != HAL_BOARD_APM2 && CONFIG_HAL_BOARD != HAL_BOARD_APM1
#define MS5803_WITH_I2C 1
#else
#define MS5803_WITH_I2C 0
#endif


/** Abstract serial device driver for MS5803. */
class AP_Baro_MS5803_Serial
{
public:
    /** Initialize the driver. */
    virtual void init() = 0;

    /** Read a 16-bit value from register "reg". */
    virtual uint16_t read_16bits(uint8_t reg) = 0;

    /** Read a 24-bit value from the ADC. */
    virtual uint32_t read_adc() = 0;

    /** Write a single byte command. */
    virtual void write(uint8_t reg) = 0;

    /** Acquire the internal semaphore for this device.
     * take_nonblocking should be used from the timer process,
     * take_blocking from synchronous code (i.e. init) */
    virtual bool sem_take_nonblocking() { return true; }
    virtual bool sem_take_blocking() { return true; }

    /** Release the internal semaphore for this device. */
    virtual void sem_give() {}
};

/** SPI serial device. */
class AP_Baro_MS5803_SPI : public AP_Baro_MS5803_Serial
{
public:
    virtual void init();
    virtual uint16_t read_16bits(uint8_t reg);
    virtual uint32_t read_adc();
    virtual void write(uint8_t reg);
    virtual bool sem_take_nonblocking();
    virtual bool sem_take_blocking();
    virtual void sem_give();

private:
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};

#if MS5803_WITH_I2C
/** I2C serial device. */
class AP_Baro_MS5803_I2C : public AP_Baro_MS5803_Serial
{
public:
    virtual void init();
    virtual uint16_t read_16bits(uint8_t reg);
    virtual uint32_t read_adc();
    virtual void write(uint8_t reg);
    virtual bool sem_take_nonblocking();
    virtual bool sem_take_blocking();
    virtual void sem_give();

private:
    AP_HAL::Semaphore *_i2c_sem;
};
#endif // MS5803_WITH_I2C

class AP_Baro_MS5803 : public AP_Baro
{
public:
    AP_Baro_MS5803(AP_Baro_MS5803_Serial *serial)
    {
        _serial = serial;
    }

    /* AP_Baro public interface: */
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees


    /* Serial port drivers to pass to "init". */
    static AP_Baro_MS5803_SPI spi;
#if MS5803_WITH_I2C
    static AP_Baro_MS5803_I2C i2c;
#endif

private:
    void            _calculate();
    /* Asynchronous handler functions: */
    void                            _update();

#if CONFIG_HAL_BOARD != HAL_BOARD_APM2
    bool check_crc(void);
#endif

    /* Asynchronous state: */
    static volatile bool            _updated;
    static volatile uint8_t         _d1_count;
    static volatile uint8_t         _d2_count;
    static volatile uint32_t        _s_D1, _s_D2;
    static uint8_t                  _state;
    static uint32_t                 _timer;
    static AP_Baro_MS5803_Serial   *_serial;
    /* Gates access to asynchronous state: */
    static bool                     _sync_access;

    float                           Temp;
    float                           Press;

    int32_t                         _raw_press;
    int32_t                         _raw_temp;
    // Internal calibration registers
    uint16_t                        C1,C2,C3,C4,C5,C6;
    float                           D1,D2;

};

#endif //  __AP_BARO_MS5803_H__
