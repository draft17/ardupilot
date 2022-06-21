#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up); // 최대 pilot_speed_up 까지만

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

#if 0
    if(AP_HAL::millis() - althold_debug_timer2 > 1000)
    {
		if(althold_state > 0)
		{
      		gcs().send_text(MAV_SEVERITY_INFO,"(%d) climb %4.1f", althold_state, target_climb_rate);
      		//gcs().send_text(MAV_SEVERITY_INFO,"(%d) alt ( %4.2f, %4.2f )", althold_state, alt_above_ground, inertial_nav.get_altitude());
			gcs().send_text(MAV_SEVERITY_INFO,"(%d) est_rt(%4.1f) rel_alt(%4.1f)", althold_state, inertial_nav.get_altitude() * 0.01f, copter.current_loc.alt * 0.01f);
			
		}
       	althold_debug_timer2 = AP_HAL::millis();
    }
#endif

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped: // 0
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Landed_Ground_Idle: // 2
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff: // 3
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Takeoff: // 1
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

#if 0
        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
#endif

#if 0
        if(AP_HAL::millis() - althold_debug_timer1 > 1000)
        {
        	//gcs().send_text(MAV_SEVERITY_INFO,"tar_cr (%4.2f) take_cr (%4.2f)", target_climb_rate, takeoff_climb_rate);
        	althold_debug_timer1 = AP_HAL::millis();
    	}
#endif

        // set position controller targets
#if 0 // YIG-CHG
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
#else
        pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, true);
#endif
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        break;

    case AltHold_Flying: // 4
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
#if 0
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif
#endif

#if 0
        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
#endif

#if 0 // YIG-CHG
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
#else // for GTB


#if 0
		if (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm >= 50.0f)                                                                            
        {
	        target_climb_rate = 0;
		}
#endif

        if(AP_HAL::millis() - althold_debug_timer1 > 2000)
        {
        	gcs().send_text(MAV_SEVERITY_INFO,"target_climb_rate (%4.1f) (%4.1f)", target_climb_rate, copter.pos_control->get_alt_target());
        	althold_debug_timer1 = AP_HAL::millis();
    	}

#if 0
		if(target_climb_rate == 0)
		{
			pos_control->set_alt_target_with_slew(copter.t float current_alt_error = copter.pos_control->get_alt_target()ave_rel_alt, G_Dt);
		}
		else
#endif
        	//pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
        	pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
#endif
        break;
    }

#if 0
	const float angle_error = attitude_control->get_att_error_angle_deg();
	if(angle_error < 2.0f)
	{
		//if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z()) < 60 && labs(ahrs.roll_sensor) < 200 && labs(ahrs.pitch_sensor) < 200) 
		if(labs(ahrs.roll_sensor) < 200)
		{
			if((target_roll * 0.01f) > 3.0f)
				target_roll = 300;
		}
		if(labs(ahrs.pitch_sensor) < 200)
		{
			if((target_pitch * 0.01f) > 3.0f)
				target_pitch = 300;
		}
	}
#endif

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

}
