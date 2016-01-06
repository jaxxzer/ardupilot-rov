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

#ifndef __AP_NOTIFY_H__
#define __AP_NOTIFY_H__

#include <AP_Common.h>
#include <AP_BoardLED.h>
#include <ToshibaLED.h>
#include <ToshibaLED_I2C.h>
#include <ToshibaLED_PX4.h>
#include <ToneAlarm_PX4.h>
#include <ExternalLED.h>
#include <Buzzer.h>

class AP_Notify
{
public:
    /// notify_type - bitmask of notification types
    struct notify_type {
        uint16_t initialising       : 1;    // 1 if initialising and copter should not be moved
        uint16_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
        uint16_t gps_glitching      : 1;    // 1 if gps position is not good
        uint16_t baro_glitching     : 1;    // 1 if baro altitude is not good
        uint16_t armed              : 1;    // 0 = disarmed, 1 = armed
        uint16_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
        uint16_t pre_arm_gps_check  : 1;    // 0 = failing pre-arm GPS checks, 1 = passed
        uint16_t save_trim          : 1;    // 1 if gathering trim data
        uint16_t esc_calibration    : 1;    // 1 if calibrating escs
        uint16_t failsafe_radio     : 1;    // 1 if radio failsafe
        uint16_t failsafe_battery   : 1;    // 1 if battery failsafe
        uint16_t failsafe_gps       : 1;    // 1 if gps failsafe
        uint16_t arming_failed      : 1;    // 1 if copter failed to arm after user input
        uint16_t failsafe_breach    : 1;	// 1 if internal pressure breach failsafe
        uint16_t parachute_release  : 1;    // 1 if parachute is being released
        uint16_t ekf_bad            : 1;    // 1 if ekf is reporting problems

        // additional flags
        uint16_t external_leds      : 1;    // 1 if external LEDs are enabled (normally only used for copter)
    };

    // the notify flags are static to allow direct class access
    // without declaring the object
    static struct notify_type flags;

    // initialisation
    void init(bool enable_external_leds);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

private:
    // individual drivers
    AP_BoardLED boardled;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    ToshibaLED_PX4 toshibaled;
    ToneAlarm_PX4 tonealarm;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2 
    ExternalLED externalled;
    Buzzer buzzer;
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ToshibaLED_I2C toshibaled;
    ExternalLED externalled;
    Buzzer buzzer;
#else
    ToshibaLED_I2C toshibaled;
#endif
};

#endif	// __AP_NOTIFY_H__
