#include "Copter.h"

Mode::_TakeOff Mode::takeoff;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
	takeoff_alt_cm = (float)copter.fence.get_margin() * 10;

    copter.flightmode->takeoff.start(takeoff_alt_cm);
    return true;
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Mode::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    if (!copter.motors->armed()) {
        return false;
    }
    if (!copter.ap.land_complete) {
        // can't takeoff again!
        return false;
    }
	/*
    if (!has_user_takeoff(must_navigate)) {
        // this mode doesn't support user takeoff
        return false;
    }
	*/
    if (takeoff_alt_cm <= copter.current_loc.alt) {
        // can't takeoff downwards...
        return false;
    }
    // Helicopters should return false if MAVlink takeoff command is received while the rotor is not spinning
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED && copter.ap.using_interlock) {
        return false;
    }

    if (!do_user_takeoff_start(takeoff_alt_cm)) {
        return false;
    }

    copter.set_auto_armed(true);
    return true;
}

// start takeoff to specified altitude above home in centimeters
void Mode::_TakeOff::start(float alt_cm)
{
    // indicate we are taking off
    copter.set_land_complete(false);
    // tell position controller to reset alt target and reset I terms
    copter.set_throttle_takeoff();

    // calculate climb rate
    //const float speed = MIN(copter.wp_nav->get_default_speed_up(), MAX(copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f));
    float speed = MIN(copter.wp_nav->get_default_speed_up(), MAX(copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f));

	speed = 4.0f;

    // sanity check speed and target
    if (speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

	//copter.gcs().send_text(MAV_SEVERITY_INFO,"max_speed = %4.2f %4.2f %4.2f", copter.wp_nav->get_default_speed_up(), copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f);
	copter.gcs().send_text(MAV_SEVERITY_INFO,"Takeoff start : target alt(%4.2f) max_speed(%4.2f)", alt_cm, speed);

    // initialise takeoff state
    _running = true;
    max_speed = speed;
    start_ms = millis();
    alt_delta = alt_cm;
}

// stop takeoff
void Mode::_TakeOff::stop()
{
    _running = false;
    start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void Mode::_TakeOff::get_climb_rates(float& pilot_climb_rate,
                                                  float& takeoff_climb_rate)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // acceleration of 50cm/s/s
    static constexpr float TAKEOFF_ACCEL = 5.0f;
    const float takeoff_minspeed = MIN(10.0f, max_speed);
    const float time_elapsed = (millis() - start_ms) * 1.0e-3f;
    const float speed = MIN(time_elapsed * TAKEOFF_ACCEL + takeoff_minspeed, max_speed);

    const float time_to_max_speed = (max_speed - takeoff_minspeed) / TAKEOFF_ACCEL;
    float height_gained;
    if (time_elapsed <= time_to_max_speed) {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_elapsed) + takeoff_minspeed * time_elapsed;
    } else {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_to_max_speed) + takeoff_minspeed * time_to_max_speed +
                        (time_elapsed - time_to_max_speed) * max_speed;
    }

#if 0 // YIG-CHG
    // check if the takeoff is over
    if (height_gained >= alt_delta) {
        stop();
    }
#else
	height_gained = height_gained;

	//int32_t target_alt = copter.avoid.get_margin() * 100;
	int32_t rng_alt = copter.flightmode->get_alt_above_ground_cm();

	int32_t rel_alt = copter.current_loc.alt;
	float nav_alt = copter.inertial_nav.get_altitude();

	//if (rel_alt >= alt_delta && rng_alt >= target_alt)
	if (rel_alt >= alt_delta)
	{
		stop();
		copter.save_rel_alt = rel_alt;
		copter.gcs().send_text(MAV_SEVERITY_INFO,"rel alt reached (%4.2f) (%4.2f) (%4.2f)", rel_alt*0.01f, rng_alt * 0.01f, nav_alt * 0.01f);
	}
#if 1
	else if (rng_alt >= 130)
	{
		stop();
		copter.save_rel_alt = rel_alt;
		copter.gcs().send_text(MAV_SEVERITY_INFO,"rng alt reached (%4.2f) (%4.2f) (%4.2f)", rng_alt * 0.01, rel_alt * 0.01, nav_alt * 0.01);
	}
#endif
#endif

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = speed;


	if(AP_HAL::millis() - copter.loop_time_3 > 1000)
	{
		//copter.gcs().send_text(MAV_SEVERITY_INFO,"takeoff cur_alt (%5d)", ground_cm);
		//copter.gcs().send_text(MAV_SEVERITY_INFO,"climb_rate (p=%4.0f, t=%4.0f)", pilot_climb_rate, takeoff_climb_rate);
	    copter.loop_time_3 = AP_HAL::millis();
	}


    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) {
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate;
            pilot_climb_rate = 0.0f;
        } else {
            // if overall is negative, move to pilot climb rate
            pilot_climb_rate = pilot_climb_rate + takeoff_climb_rate;
            takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (pilot_climb_rate > takeoff_climb_rate) {
            pilot_climb_rate = pilot_climb_rate - takeoff_climb_rate;
        } else {
            pilot_climb_rate = 0.0f;
        }
    }
}

void Mode::auto_takeoff_set_start_alt(void)
{
    // start with our current altitude
    auto_takeoff_no_nav_alt_cm = inertial_nav.get_altitude(); 
	// 수동모드에서 고도 상승 후 자동모드로 전환 경우, Takeoff은 현재고도에서 시작함
    
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        // we are not flying, add the wp_navalt_min
        auto_takeoff_no_nav_alt_cm += g2.wp_navalt_min * 100; 
		// 최소 고도 설정하여, 지상에서 이륙할 경우 이 고도까지는 급작스런 상승 막고(i term 누적제한), 상승이후에 본격적인 제어 수행하도록 유도함
    }
}


/*
  call attitude controller for automatic takeoff, limiting roll/pitch
  if below wp_navalt_min
 */
void Mode::auto_takeoff_attitude_run(float target_yaw_rate)
{
    float nav_roll, nav_pitch;
    
    if (g2.wp_navalt_min > 0 && inertial_nav.get_altitude() < auto_takeoff_no_nav_alt_cm) { // 최소 설정 고도에 도달하지 않았으면
        // we haven't reached the takeoff navigation altitude yet
        nav_roll = 0;
        nav_pitch = 0;
        // tell the position controller that we have limited roll/pitch demand to prevent integrator buildup
        pos_control->set_limit_accel_xy();
    } else {
        nav_roll = wp_nav->get_roll();
        nav_pitch = wp_nav->get_pitch();
    }
    
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate);
}

bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    if (copter.ap.land_complete) {
        return false;
    }
    if (takeoff.running()) {
        return true;
    }
    return false;
}
