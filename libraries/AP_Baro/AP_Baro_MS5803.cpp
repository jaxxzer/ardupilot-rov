/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       APM_MS5803.cpp - Arduino Library for MS5803-01BA01 absolute pressure sensor
 *       Code by Jose Julio, Pat Hickey and Jordi Muñoz. DIYDrones.com
 *
 *       Sensor is conected to standard SPI port
 *       Chip Select pin: Analog2 (provisional until Jordi defines the pin)!!
 *
 *       Variables:
 *               Temp : Calculated temperature (in Celsius degrees)
 *               Press : Calculated pressure   (in mbar units * 100)
 *
 *
 *       Methods:
 *               init() : Initialization and sensor reset
 *               read() : Read sensor data and _calculate Temperature, Pressure
 *                        This function is optimized so the main host don´t need to wait
 *                                You can call this function in your main loop
 *                                Maximum data output frequency 100Hz - this allows maximum oversampling in the chip ADC
 *                                It returns a 1 if there are new data.
 *               get_pressure() : return pressure in mbar*100 units
 *               get_temperature() : return temperature in celsius degrees*100 units
 *
 *       Internal functions:
 *               _calculate() : Calculate Temperature and Pressure (temperature compensated) in real units
 *
 *
 */

#include <AP_HAL.h>
#include "AP_Baro_MS5803.h"

extern const AP_HAL::HAL& hal;

#define CMD_MS5803_RESET 0x1E
#define CMD_MS5803_PROM_Setup 0xA0//Parameters (coefficients) factory calibration
#define CMD_MS5803_PROM_C1 0xA2
#define CMD_MS5803_PROM_C2 0xA4
#define CMD_MS5803_PROM_C3 0xA6
#define CMD_MS5803_PROM_C4 0xA8
#define CMD_MS5803_PROM_C5 0xAA
#define CMD_MS5803_PROM_C6 0xAC
#define CMD_MS5803_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

uint32_t volatile AP_Baro_MS5803::_s_D1;
uint32_t volatile AP_Baro_MS5803::_s_D2;
uint8_t volatile AP_Baro_MS5803::_d1_count;
uint8_t volatile AP_Baro_MS5803::_d2_count;
uint8_t AP_Baro_MS5803::_state;
uint32_t AP_Baro_MS5803::_timer;
bool volatile AP_Baro_MS5803::_updated;

AP_Baro_MS5803_Serial* AP_Baro_MS5803::_serial = NULL;
//AP_Baro_MS5803_SPI AP_Baro_MS5803::spi;
#if MS5803_WITH_I2C
AP_Baro_MS5803_I2C AP_Baro_MS5803::i2c;
#endif

// SPI Device //////////////////////////////////////////////////////////////////
/*

void AP_Baro_MS5803_SPI::init()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MS5803);
    if (_spi == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5803 did not get "
                    "valid SPI device driver!"));
        return;  never reached
    }
    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5803 did not get "
                    "valid SPI semaphroe!"));
        return;  never reached

    }

    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
}

uint16_t AP_Baro_MS5803_SPI::read_16bits(uint8_t reg)
{
    uint8_t tx[3];
    uint8_t rx[3];
    tx[0] = reg; tx[1] = 0; tx[2] = 0;
    _spi->transaction(tx, rx, 3);
    return ((uint16_t) rx[1] << 8 ) | ( rx[2] );
}

uint32_t AP_Baro_MS5803_SPI::read_adc()
{
    uint8_t tx[4];
    uint8_t rx[4];
    memset(tx, 0, 4);  first byte is addr = 0
    _spi->transaction(tx, rx, 4);
    return (((uint32_t)rx[1])<<16) | (((uint32_t)rx[2])<<8) | ((uint32_t)rx[3]);
}


void AP_Baro_MS5803_SPI::write(uint8_t reg)
{
    uint8_t tx[1];
    tx[0] = reg;
    _spi->transaction(tx, NULL, 1);
}

bool AP_Baro_MS5803_SPI::sem_take_blocking() {
    return _spi_sem->take(10);
}

bool AP_Baro_MS5803_SPI::sem_take_nonblocking()
{
    *
     * Take nonblocking from a TimerProcess context &
     * monitor for bad failures

    static int semfail_ctr = 0;
    bool got = _spi_sem->take_nonblocking();
    if (!got) {
        if (!hal.scheduler->system_initializing()) {
            semfail_ctr++;
            if (semfail_ctr > 100) {
                hal.scheduler->panic(PSTR("PANIC: failed to take _spi_sem "
                                          "100 times in a row, in "
                                          "AP_Baro_MS5803::_update"));
            }
        }
        return false;  never reached
    } else {
        semfail_ctr = 0;
    }
    return got;
}

void AP_Baro_MS5803_SPI::sem_give()
{
    _spi_sem->give();
}
*/

// I2C Device //////////////////////////////////////////////////////////////////
#if MS5803_WITH_I2C


//External baro for water pressure, I2C address is 0x76.
//MS5803 can have either 0x77 or 0x76, the lowest bit of the address is the compliment of the signal on the CSB pin.
//Set CSB pin high for 0x76
//Low for 0x77
//Do not leave CSB floating
//See ArduCopter.pde
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
#define MS5803_ADDR 0x77
#else
#define MS5803_ADDR 0x76 /** I2C address of the MS5803 on the PX4 board. */
#endif

void AP_Baro_MS5803_I2C::init()
{
    _i2c_sem = hal.i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5803 did not get "
                                  "valid I2C semaphroe!"));
        return; /* never reached */
    }
}

uint16_t AP_Baro_MS5803_I2C::read_16bits(uint8_t reg)
{
    uint8_t buf[2];

    if (hal.i2c->readRegisters(MS5803_ADDR, reg, sizeof(buf), buf) == 0)
        return (((uint16_t)(buf[0]) << 8) | buf[1]);

    return 0;
}

uint32_t AP_Baro_MS5803_I2C::read_adc()
{
    uint8_t buf[3];

    if (hal.i2c->readRegisters(MS5803_ADDR, 0x00, sizeof(buf), buf) == 0)
        return (((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) | buf[2];

    return 0;
}

void AP_Baro_MS5803_I2C::write(uint8_t reg)
{
    hal.i2c->write(MS5803_ADDR, 1, &reg);
}

bool AP_Baro_MS5803_I2C::sem_take_blocking() {
    return _i2c_sem->take(10);
}

bool AP_Baro_MS5803_I2C::sem_take_nonblocking()
{
    /**
     * Take nonblocking from a TimerProcess context &
     * monitor for bad failures
     */
    static int semfail_ctr = 0;
    bool got = _i2c_sem->take_nonblocking();
    if (!got) {
        if (!hal.scheduler->system_initializing()) {
            semfail_ctr++;
            if (semfail_ctr > 100) {
                hal.scheduler->panic(PSTR("PANIC: failed to take _i2c_sem "
                                          "100 times in a row, in "
                                          "AP_Baro_MS5803::_update"));
            }
        }
        return false; /* never reached */
    } else {
        semfail_ctr = 0;
    }
    return got;
}

void AP_Baro_MS5803_I2C::sem_give()
{
    _i2c_sem->give();
}
#endif // MS5803_WITH_I2C

// Public Methods //////////////////////////////////////////////////////////////

#if CONFIG_HAL_BOARD != HAL_BOARD_APM2
/**
 * MS5803 crc4 method based on PX4Firmware code
 */
bool AP_Baro_MS5803::check_crc(void)
{
    int16_t cnt;
    uint16_t n_rem;
    uint16_t crc_read;
    uint8_t n_bit;
    uint16_t n_prom[8] = { _serial->read_16bits(CMD_MS5803_PROM_Setup),
                           C1, C2, C3, C4, C5, C6,
                           _serial->read_16bits(CMD_MS5803_PROM_CRC) };
    n_rem = 0x00;

    /* save the read crc */
    crc_read = n_prom[7];

    /* remove CRC byte */
    n_prom[7] = (0xFF00 & (n_prom[7]));

    for (cnt = 0; cnt < 16; cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

        } else {
            n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;

            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    /* final 4 bit remainder is CRC value */
    n_rem = (0x000F & (n_rem >> 12));
    n_prom[7] = crc_read;

    /* return true if CRCs match */
    return (0x000F & crc_read) == (n_rem ^ 0x00);
}
#endif

// SPI should be initialized externally
bool AP_Baro_MS5803::init()
{
    if (_serial == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5803: NULL serial driver"));
        return false; /* never reached */
    }

    _serial->init();
    if (!_serial->sem_take_blocking()){
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5803: failed to take "
                    "serial semaphore for init"));
        return false; /* never reached */
    }

    _serial->write(CMD_MS5803_RESET);
    hal.scheduler->delay(4);

    // We read the factory calibration
    // The on-chip CRC is not used
    C1 = _serial->read_16bits(CMD_MS5803_PROM_C1);
    C2 = _serial->read_16bits(CMD_MS5803_PROM_C2);
    C3 = _serial->read_16bits(CMD_MS5803_PROM_C3);
    C4 = _serial->read_16bits(CMD_MS5803_PROM_C4);
    C5 = _serial->read_16bits(CMD_MS5803_PROM_C5);
    C6 = _serial->read_16bits(CMD_MS5803_PROM_C6);

    // if not on APM2 then check CRC
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
    if (!check_crc()) {
        hal.scheduler->panic("Bad CRC on MS5803");
    }
#endif

    //Send a command to read Temp first
    _serial->write(CMD_CONVERT_D2_OSR4096);
    _timer = hal.scheduler->micros();
    _state = 0;
    Temp=0;
    Press=0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

    hal.scheduler->register_timer_process( AP_HAL_MEMBERPROC(&AP_Baro_MS5803::_update));
    _serial->sem_give();

    // wait for at least one value to be read
    uint32_t tstart = hal.scheduler->millis();
    while (!_updated) {
        hal.scheduler->delay(10);
        if (hal.scheduler->millis() - tstart > 1000) {
            hal.scheduler->panic(PSTR("PANIC: AP_Baro_MS5803 took more than "
                        "1000ms to initialize"));
            _flags.healthy = false;
            return false;
        }
    }

    _flags.healthy = true;
    return true;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
void AP_Baro_MS5803::_update(void)
{
    uint32_t tnow = hal.scheduler->micros();
    // Throttle read rate to 100hz maximum.
    if (tnow - _timer < 10000) {
        return;
    }

    if (!_serial->sem_take_nonblocking()) {
        return;
    }
    _timer = tnow;

    if (_state == 0) {
        // On state 0 we read temp
        uint32_t d2 = _serial->read_adc();
        if (d2 != 0) {
            _s_D2 += d2;
            _d2_count++;
            if (_d2_count == 32) {
                // we have summed 32 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D2 >>= 1;
                _d2_count = 16;
            }
        }
        _state++;
        _serial->write(CMD_CONVERT_D1_OSR4096);      // Command to read pressure
    } else {
        uint32_t d1 = _serial->read_adc();;
        if (d1 != 0) {
            // occasional zero values have been seen on the PXF
            // board. These may be SPI errors, but safest to ignore
            _s_D1 += d1;
            _d1_count++;
            if (_d1_count == 128) {
                // we have summed 128 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D1 >>= 1;
                _d1_count = 64;
            }
            // Now a new reading exists
            _updated = true;
        }
        _state++;
        if (_state == 5) {
            _serial->write(CMD_CONVERT_D2_OSR4096); // Command to read temperature
            _state = 0;
        } else {
            _serial->write(CMD_CONVERT_D1_OSR4096); // Command to read pressure
        }
    }

    _serial->sem_give();
}

uint8_t AP_Baro_MS5803::read()
{
    bool updated = _updated;
    if (updated) {
        uint32_t sD1, sD2;
        uint8_t d1count, d2count;

        // Suspend timer procs because these variables are written to
        // in "_update".
        hal.scheduler->suspend_timer_procs();
        sD1 = _s_D1; _s_D1 = 0;
        sD2 = _s_D2; _s_D2 = 0;
        d1count = _d1_count; _d1_count = 0;
        d2count = _d2_count; _d2_count = 0;
        _updated = false;
        hal.scheduler->resume_timer_procs();

        if (d1count != 0) {
            D1 = ((float)sD1) / d1count;
        }
        if (d2count != 0) {
            D2 = ((float)sD2) / d2count;
        }
        _pressure_samples = d1count;
        _raw_press = D1;//in mbar
        _raw_temp = D2;//in degrees C*100
    }
    _calculate();
    if (updated) {
        _last_update = hal.scheduler->millis();
    }
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5803::_calculate()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;
    float P;

    // Formulas from manufacturer datasheet
    // sub -20c temperature compensation is not included

    // we do the calculations using floating point
    // as this is much faster on an AVR2560, and also allows
    // us to take advantage of the averaging of D1 and D1 over
    // multiple samples, giving us more precision
    dT = D2-(((uint32_t)C5)<<8);
    TEMP = (dT * C6)/8388608;
    OFF = C2 * 65536.0f + (C4 * dT) / 128;
    SENS = C1 * 32768.0f + (C3 * dT) / 256;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 2.5f*Aux;
        float SENS2 = 1.25f*Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    P = (D1*SENS/2097152 - OFF)/32768;
    Temp = (TEMP + 2000) * 0.01f;//2473.0 = 24.73 degrees c = degrees C *100
    Press = P;//1000009.0 = 100000.9mbar = mbar*10
}

float AP_Baro_MS5803::get_pressure()
{
    return Press*10; //In mbar*10
}

float AP_Baro_MS5803::get_temperature()
{
    // temperature in degrees C units
    return Temp;// in degreeC*100
}

float AP_Baro_MS5803::get_altitude(void)
{
    if (_ground_pressure == 0) {
        // called before initialisation
        return 0;
    }

    if (_last_altitude_t == _last_update) {
        // no new information
        return _altitude + _alt_offset;
    }

    float pressure = get_pressure();
    //float alt = get_altitude_difference(_ground_pressure, pressure);

    float alt = (_ground_pressure - pressure) / 10052.0f; //101325Pa is sea level air pressure, 10052 Pascal/ m depth in water. No temperature or depth compensation for density of water.

    // record that we have consumed latest data
    _last_altitude_t = _last_update;

    // sanity check altitude
    if (isnan(alt) || isinf(alt)) {
        _flags.alt_ok = false;
    } else {
        _altitude = alt;
        _flags.alt_ok = true;
    }

    // ensure the climb rate filter is updated
    _climb_rate_filter.update(_altitude, _last_update);

    return (_altitude + _alt_offset);//multiply by 10 here because the ms5803 reports to .1 mbar and ms5611 reports to .01 mbar
    //return alt;//    10052Pa/m depth.
}
