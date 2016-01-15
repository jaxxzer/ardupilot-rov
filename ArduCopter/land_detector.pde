/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// counter to verify landings
static uint16_t land_detector = LAND_DETECTOR_TRIGGER;  // we assume we are landed

// land_complete_maybe - return true if we may have landed (used to reset loiter targets during landing)
static bool land_complete_maybe()
{
    return (ap.land_complete || ap.land_complete_maybe);
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at 50hz
static void update_land_detector()
{
//    bool climb_rate_low = (abs(climb_rate) < LAND_DETECTOR_CLIMBRATE_MAX) && (abs(baro_climbrate) < LAND_DETECTOR_BARO_CLIMBRATE_MAX);
//    bool target_climb_rate_low = !pos_control.is_active_z() || (pos_control.get_desired_velocity().z <= LAND_DETECTOR_DESIRED_CLIMBRATE_MAX);
//    bool motor_at_lower_limit = motors.limit.throttle_lower;
//    bool throttle_low = (FRAME_CONFIG == HELI_FRAME) || (motors.get_throttle_out() < get_non_takeoff_throttle());
//    bool not_rotating_fast = (ahrs.get_gyro().length() < LAND_DETECTOR_ROTATION_MAX);
//
//    if (climb_rate_low && target_climb_rate_low && motor_at_lower_limit && throttle_low && not_rotating_fast) {
//        if (!ap.land_complete) {
//            // increase counter until we hit the trigger then set land complete flag
//            if( land_detector < LAND_DETECTOR_TRIGGER) {
//                land_detector++;
//            }else{
//                set_land_complete(true);
//                land_detector = LAND_DETECTOR_TRIGGER;
//            }
//        }
//    } else {
//        // we've sensed movement up or down so reset land_detector
//        land_detector = 0;
//        // if throttle output is high then clear landing flag
//        if (motors.get_throttle_out() > get_non_takeoff_throttle()) {
//            set_land_complete(false);
//        }
//    }
//
//    // set land maybe flag
	//50Hz call
//    set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER)

//#ifndef SURFACE_END_DEPTH
//#define SURFACE_END_DEPTH 1500.0f //depth at which we exit the surface maneuver and disarm the motors
//#endif
//	if(barometer.get_pressure() < barometer.get_ground_pressure() + SURFACE_END_DEPTH)//~15cm here arbitrary number, calculate this, match to something reasonable in events.pde
//		set_land_complete(true);
//	else
//		set_land_complete(false);
	if(barometer.get_altitude() > -0.04f) //if our depth is < 2cm we have landed
		set_land_complete(true);
	else
		set_land_complete(false);

	// gcs_send_text_fmt(PSTR("\naltitude= %f"), barometer.get_altitude());
	//gcs_send_text_fmt(PSTR("\nthrottle_out= %d"), motors.get_throttle_out());
//	if(ap.land_complete && motors.get_throttle_out() < 100)
//		set_land_complete(false);

	//gcs_send_text_fmt(PSTR("\nland_complete= %d"), ap.land_complete);


}
