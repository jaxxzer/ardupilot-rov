/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
	static uint16_t counter_delay;

	static float last_pressure;
	static uint16_t counter;

	//init_arm_motors();
    // put your 10Hz code here
	if(counter_delay < 300) {
		counter_delay++;
	}
	else {
		float current_pressure = internal_barometer.get_pressure();

		if(current_pressure < last_pressure + 1000 && current_pressure > last_pressure - 1000)
			counter++;
		if(counter > 100) {

			if(motors.armed()) {
				init_disarm_motors();
			} else {
				init_arm_motors();
			}
	//		init_arm_motors();
		}
		last_pressure = current_pressure;
	}

}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
