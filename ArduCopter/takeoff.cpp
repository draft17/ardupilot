#include "Copter.h"

Mode::_TakeOff Mode::takeoff;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
	if(takeoff_alt_cm >= 300.0f) takeoff_alt_cm = 300.0f;
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
    if (!has_user_takeoff(must_navigate)) {
        // this mode doesn't support user takeoff
        return false;
    }
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
    const float speed = MIN(copter.wp_nav->get_default_speed_up(), MAX(copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f));

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
    static constexpr float TAKEOFF_ACCEL = 50.0f;
    const float takeoff_minspeed = MIN(50.0f, max_speed);
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

    // check if the takeoff is over
    if (height_gained >= alt_delta) {
        stop();
    }

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) // YIG-Jawoldo : 조종기의 쓰로틀 값만 사용
	{
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = speed;

    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) { // 조종기 값을 내렸는데
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) { // takeoff_climb_rate가 조종기 내린값보다 크면 차이만큼 climb rate를 올림
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate; // 차이만큼만 올림
            pilot_climb_rate = 0.0f;
        } else { // 조종기 내린 값이 takeoff_climb_rate 보다 크면 차이만큼 내림
            // if overall is negative, move to pilot climb rate
            pilot_climb_rate = pilot_climb_rate + takeoff_climb_rate;
            takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb // 조종기 값을 올렸다면
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (pilot_climb_rate > takeoff_climb_rate) { // 조종기 값이 takeoff_climb_rate보다 크면
            pilot_climb_rate = pilot_climb_rate - takeoff_climb_rate; // 플러스 하지말고 조종기 값과의 차이만큼만 올림
        } else { // takeoff_climb_rate가 조종기 값보다 크다면 그냥 takeoff_climb_rate 값만 적용
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
