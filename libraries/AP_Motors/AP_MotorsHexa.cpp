// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_MotorsHexa.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_MotorsHexa.h"

// setup_motors - configures the motors for a hexa
void AP_MotorsHexa::setup_motors()
{
    // call parent
    AP_MotorsMatrix::setup_motors();
    float
		//Front left downward facing
		MOT_1_ROLL_FACTOR = -0.5,
		MOT_1_PITCH_FACTOR = 0.5,
		MOT_1_YAW_FACTOR = 0,

		//Front right downward facing
		MOT_2_ROLL_FACTOR = 0.5,
		MOT_2_PITCH_FACTOR = 0.5,
		MOT_2_YAW_FACTOR = 0,

		//Mid left backward facing
		MOT_3_ROLL_FACTOR = 0,
		MOT_3_PITCH_FACTOR = 0,
		MOT_3_YAW_FACTOR = 1.0,

		//Mid right backward facing
		MOT_4_ROLL_FACTOR = 0,
		MOT_4_PITCH_FACTOR = 0,
		MOT_4_YAW_FACTOR = -1.0,

		//Rear motor downward facing
		MOT_5_ROLL_FACTOR = 0,
		MOT_5_PITCH_FACTOR = -1.0,
		MOT_5_YAW_FACTOR = 0.0,

		//Bottom motor facing left
		MOT_6_ROLL_FACTOR = 0.3,
		MOT_6_PITCH_FACTOR = 0.5,
		MOT_6_YAW_FACTOR = 0.0;


	add_motor_raw(AP_MOTORS_MOT_1, MOT_1_ROLL_FACTOR, MOT_1_PITCH_FACTOR, MOT_1_YAW_FACTOR, 1);
	add_motor_raw(AP_MOTORS_MOT_2, MOT_2_ROLL_FACTOR, MOT_2_PITCH_FACTOR, MOT_2_YAW_FACTOR, 2);
	add_motor_raw(AP_MOTORS_MOT_3, MOT_3_ROLL_FACTOR, MOT_3_PITCH_FACTOR, MOT_3_YAW_FACTOR, 3);
	add_motor_raw(AP_MOTORS_MOT_4, MOT_4_ROLL_FACTOR, MOT_4_PITCH_FACTOR, MOT_4_YAW_FACTOR, 4);
	add_motor_raw(AP_MOTORS_MOT_5, MOT_5_ROLL_FACTOR, MOT_5_PITCH_FACTOR, MOT_5_YAW_FACTOR, 5);
	add_motor_raw(AP_MOTORS_MOT_6, MOT_6_ROLL_FACTOR, MOT_6_PITCH_FACTOR, MOT_6_YAW_FACTOR, 6);

}
