// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            if (ap.throttle_zero || ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            }else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if far from home then RTL
            }else if(home_distance > wp_nav.get_wp_radius()) {
                if (!set_mode(RTL)) {
                    set_mode_land_with_pause();
                }

            // We have no GPS or are very close to home so we will land
            }else{
                set_mode_land_with_pause();
            }
            break;

        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            if (!ap.auto_armed && ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            } else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_RTL do RTL
            } else if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                }else{
                    // We are very close to home so we will land
                    set_mode_land_with_pause();
                }
            }
            // failsafe_throttle must be FS_THR_ENABLED_CONTINUE_MISSION so no need to do anything
            break;

        case LAND:
            // continue to land if battery failsafe is also active otherwise fall through to default handling
            if (g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
                break;
            }
            // no break
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, OF_Loiter, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            if (ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            } else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if far from home then RTL
            }else if(home_distance > wp_nav.get_wp_radius()) {
                if (!set_mode(RTL)){
                    // if RTL fails because of no GPS, then LAND
                    set_mode_land_with_pause();
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
                // if throttle is zero OR vehicle is landed disarm motors
                if (ap.throttle_zero || ap.land_complete) {
                    init_disarm_motors();
                }else{
                    // set mode to RTL or LAND
                    if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_wp_radius()) {
                        if (!set_mode(RTL)) {
                            set_mode_land_with_pause();
                        }
                    }else{
                        set_mode_land_with_pause();
                    }
                }
                break;
            case AUTO:
                // if mission has not started AND vehicle is landed, disarm motors
                if (!ap.auto_armed && ap.land_complete) {
                    init_disarm_motors();

                // set mode to RTL or LAND
                } else if (home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                } else {
                    set_mode_land_with_pause();
                }
                break;
            default:
                // used for AltHold, Guided, Loiter, RTL, Circle, OF_Loiter, Drift, Sport, Flip, Autotune, PosHold
                // if landed disarm
                if (ap.land_complete) {
                    init_disarm_motors();

                // set mode to RTL or LAND
                } else if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                } else {
                    set_mode_land_with_pause();
                }
                break;
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_gps_check - check for gps failsafe
static void failsafe_gps_check()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled or we have never had GPS lock
    if (g.failsafe_gps_enabled == FS_GPS_DISABLED || !ap.home_is_set) {
        // if we have just disabled the gps failsafe, ensure the gps failsafe event is cleared
        if (failsafe.gps) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // calc time since last gps update
    last_gps_update_ms = millis() - gps_glitch.last_good_update();

    // check if all is well
    if( last_gps_update_ms < FAILSAFE_GPS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( failsafe.gps ) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( failsafe.gps || !motors.armed()) {
        return;
    }

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    set_failsafe_gps(true);
    gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode and FS_GPS_ENABLED parameter
    if (mode_requires_GPS(control_mode) || g.failsafe_gps_enabled == FS_GPS_LAND_EVEN_STABILIZE) {
        if (g.failsafe_gps_enabled == FS_GPS_ALTHOLD && !failsafe.radio) {
            set_mode(ALT_HOLD);
        }else{
            set_mode_land_with_pause();
        }
    }

    // if flight mode is LAND ensure it's not the GPS controlled LAND
    if (control_mode == LAND) {
        land_do_not_use_GPS();
    }
}

// failsafe_gps_off_event - actions to take when GPS contact is restored
static void failsafe_gps_off_event(void)
{
    // log recovery of GPS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// failsafe_gcs_check - check for ground station failsafe
static void failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs
    if( g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0 || !failsafe.rc_override_active) {
        return;
    }

    // calc time since last gcs update
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if( last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gcs failsafe
        if (failsafe.gcs) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if( failsafe.gcs || !motors.armed()) {
        return;
    }

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    init_disarm_motors(); // just cut the motors for gcs failsafe
/*
    // clear overrides so that RC control can be regained with radio.
    hal.rcin->clear_overrides();
    failsafe.rc_override_active = false;

    // This is how to handle a failsafe.
    // use the throttle failsafe setting to decide what to do
    switch(control_mode) {

        case STABILIZE:
        case ACRO:
        case SPORT:
            // if throttle is zero disarm motors
            if (ap.throttle_zero) {
                init_disarm_motors();
            }else if(home_distance > wp_nav.get_wp_radius()) {
                if (!set_mode(RTL)) {
                    set_mode_land_with_pause();
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
        case AUTO:
            // if g.failsafe_gcs is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_RTL) {
                if (home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                }else{
                    // We are very close to home so we will land
                    set_mode_land_with_pause();
                }
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;


        default:
            if(home_distance > wp_nav.get_wp_radius()) {
                if (!set_mode(RTL)) {
                    set_mode_land_with_pause();
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
    }

*/
}
static void failsafe_breach_check() {
	float last_pressure;

	float pressure_failsafe_threshold = 50.0f;
	float current_pressure = barometer.get_pressure();


	if(failsafe.breach && ap.land_complete && !motors.armed()) {
		if(current_pressure < barometer.get_pressure() + pressure_failsafe_threshold) {
			set_failsafe_breach(false);
			set_mode(STABILIZE);
			return;
		}
	}

    // do nothing if breach failsafe already triggered or motors disarmed
    if( failsafe.breach || !motors.armed()) {
        return;
    }





	if(current_pressure > barometer.get_ground_pressure() + pressure_failsafe_threshold) {
		set_failsafe_breach(true);
		set_mode(LAND);
		return;

	}

	//last_pressure =
}
// failsafe_gcs_off_event - actions to take when GCS contact is restored
static void failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

//static void failsafe_breach_on_event(void) {
//	//surface, mode land
//	set_mode(LAND);
//	//set_mode_land_with_pause();//to delay 4 seconds before final ascent, we dont want to do this we want to get out of water asap.
//
//}

static void failsafe_breach_off_event(void) {
	//disarm motors?
}

static void update_events()
{
    ServoRelayEvents.update_events();
}

