#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);

   // YIG-ADD
   rangefinder_fw_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
   rangefinder_fw_state.enabled = rangefinder.has_orientation(ROTATION_NONE);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    const float tilt_correction = MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    // iterate through downward and upward facing lidar
    struct {
        RangeFinderState &state;
        enum Rotation orientation;
    //} rngfnd[2] = {{rangefinder_state, ROTATION_PITCH_270}, {rangefinder_up_state, ROTATION_PITCH_90}}; // YIG-ADD
    } rngfnd[3] = {{rangefinder_state, ROTATION_PITCH_270}, {rangefinder_up_state, ROTATION_PITCH_90}, {rangefinder_fw_state, ROTATION_NONE}};

    for (uint8_t i=0; i < ARRAY_SIZE(rngfnd); i++) 
	{
        // local variables to make accessing simpler
        RangeFinderState &rf_state = rngfnd[i].state;
        enum Rotation rf_orient = rngfnd[i].orientation;

        // update health
        rf_state.alt_healthy = ((rangefinder.status_orient(rf_orient) == RangeFinder::Status::Good) &&
                                (rangefinder.range_valid_count_orient(rf_orient) >= RANGEFINDER_HEALTH_MAX));

        // tilt corrected but unfiltered, not glitch protected alt
        rf_state.alt_cm = tilt_correction * rangefinder.distance_cm_orient(rf_orient);

        // remember inertial alt to allow us to interpolate rangefinder
        rf_state.inertial_alt_cm = inertial_nav.get_altitude();

        // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
        // are considered a glitch and glitch_count becomes non-zero
        // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
        // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
        const int32_t glitch_cm = rf_state.alt_cm - rf_state.alt_cm_glitch_protected;
        if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MAX(rf_state.glitch_count+1, 1);
        } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MIN(rf_state.glitch_count-1, -1);
        } else {
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
        }
        if (abs(rf_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
            // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
            rf_state.glitch_cleared_ms = AP_HAL::millis();
        }

        // filter rangefinder altitude
        uint32_t now = AP_HAL::millis();
        const bool timed_out = now - rf_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
        if (rf_state.alt_healthy) {
            if (timed_out) {
                // reset filter if we haven't used it within the last second
                rf_state.alt_cm_filt.reset(rf_state.alt_cm);
            } else {
                rf_state.alt_cm_filt.apply(rf_state.alt_cm, 0.05f);
            }
            rf_state.last_healthy_ms = now;
        }

#if 1 // YIG-ADD : AVOID

		// 하방 LiDAR 관련 동작
       	//if (rf_orient == ROTATION_PITCH_270 && rf_state.alt_healthy)
       	if (rf_orient == ROTATION_PITCH_270)
		{
			// Auto 모드 일 경우, 하방 LiDAR를 통한 지상 충돌 방지 & Gripper & Return to Home
			// Gripper 바로 전, 하강 미션에서만 동작
			if(copter.control_mode == Mode::Number::AUTO && copter.mode_auto.mission.get_current_nav_id() == MAV_CMD_NAV_LAND && _mission_changed == false)
			{
				AP_Mission::Mission_Command grip1_cmd;
				AP_Mission::Mission_Command grip2_cmd;
				AP_Mission::Mission_Command wp_cmd;
				uint16_t current_idx = copter.mode_auto.mission.get_current_nav_index();

				copter.mode_auto.mission.get_next_cmd(current_idx + 1, grip1_cmd, false, false);
				copter.mode_auto.mission.get_next_cmd(current_idx + 2, grip2_cmd, false, false);
				copter.mode_auto.mission.get_next_cmd(current_idx + 3, wp_cmd, false, false);

				//if(grip1_cmd.id == MAV_CMD_DO_GRIPPER && grip2_cmd.id == MAV_CMD_DO_GRIPPER && wp_cmd.id == MAV_CMD_NAV_WAYPOINT)
				if(grip1_cmd.id == MAV_CMD_DO_GRIPPER && grip2_cmd.id == MAV_CMD_DO_GRIPPER)
				{
					if(AP_HAL::millis() - down_loop_time > 1000)
					{
						gcs().send_text(MAV_SEVERITY_INFO,"[%d  %d] : Distance from Ground = (%d)", current_idx, copter.mode_auto.mission.get_current_nav_id(), rf_state.alt_cm);
						down_loop_time = AP_HAL::millis();
					}

					uint16_t down_dist = (uint16_t)copter.avoid.get_margin();

					if(down_dist == 5) // 수동
					{
						if(rf_state.alt_cm <= 1000 && rf_state.alt_cm > 500)
						{
							gcs().send_text(MAV_SEVERITY_EMERGENCY, "Brake, For Loiter Gripper  (%d)", rf_state.alt_cm);
	   						copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND);

							copter.mode_auto.mission.set_current_cmd(current_idx + 3);
						}
					}
					else if(down_dist > 5) // 자동
					{
						if(rf_state.alt_cm < ((down_dist * 100) - 200))
						{
							_mission_changed = true;

							gcs().send_text(MAV_SEVERITY_EMERGENCY, "Brake, For Auto Gripper (%d)", rf_state.alt_cm);
	   						copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND);

							AP_Gripper *gripper = AP::gripper();
							if (gripper == nullptr) gcs().send_text(MAV_SEVERITY_WARNING, "Gripper not setup");
							if(!gripper->enabled()) gcs().send_text(MAV_SEVERITY_WARNING, "Gripper not enabled");
							else {
								gripper->release(); gripper->release(); gripper->release();
								gcs().send_text(MAV_SEVERITY_EMERGENCY, "Gripper releasing");
							}

							AP_Mission::Mission_Command _cmd;
							copter.mode_auto.mission.get_next_cmd(current_idx + 3, _cmd, false, false);
							if(_cmd.id == MAV_CMD_NAV_WAYPOINT || _cmd.id == MAV_CMD_NAV_LAND)
							{
								gcs().send_text(MAV_SEVERITY_WARNING, "Good Next Mission %d", current_idx + 3);
							}

							copter.mode_auto.mission.set_current_cmd(current_idx + 3);
							uint16_t curr_idx = copter.mode_auto.mission.get_current_nav_index();
							gcs().send_text(MAV_SEVERITY_WARNING, "Mission Changed = %d", curr_idx);

							if (copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) 
							{
						    	copter.set_auto_armed(true);
						    	if (copter.mode_auto.mission.state() != AP_Mission::MISSION_RUNNING) 
								{
							    	copter.mode_auto.mission.start_or_resume();
						    	}
								gcs().send_text(MAV_SEVERITY_EMERGENCY, "AUTO, Return to Home");
							}
							else
								gcs().send_text(MAV_SEVERITY_EMERGENCY, "Brake Holding");
						}
					}

				}
			}
		}

        //if (rf_orient == ROTATION_NONE && rf_state.alt_healthy)
        if (rf_orient == ROTATION_NONE)
		{
			if(copter.avoid.fence_margin() >= 1500.0f && copter.inertial_nav.get_altitude() >= 250.0f)
			{
				if(AP_HAL::millis() - front_loop_time > 2000)
				{
					gcs().send_text(MAV_SEVERITY_INFO,"Front Obstacle %d", rf_state.alt_cm);
					front_loop_time = AP_HAL::millis();
				}

				// Auto 모드일 경우, 전방 LiDAR를 통한 장애물 감지 시 Brake
				if(copter.control_mode == Mode::Number::AUTO)
				{
					// WP 비행 중에만 동작하도록 : Takeoff/RTL 시, 전방장애물이 있는 좁은 지역의 경우 불필요한 감지 방지
					if(mode_auto.mission.get_current_nav_id() == MAV_CMD_NAV_WAYPOINT 
						&& mode_auto.mission.curr_nav_idx() > 2
						//&& copter.fence.get_action() == 3 // 검수를 위한 방안
		   				//|| mode_auto.mission.get_current_nav_id() == MAV_CMD_NAV_RETURN_TO_LAUNCH
					)
					{
						if(rf_state.alt_cm < copter.avoid.fence_margin()) 
						{
							AP_Notify::flags.parachute_release = 1; // beep alarm
	   						copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND);
							gcs().send_text(MAV_SEVERITY_CRITICAL, "Obstacle (%d) : AUTO to BRAKE !!", rf_state.alt_cm);
						}
					}
				}

				// Loiter 모드일 경우, 전방 LiDAR를 통한 장애물 감지 시 Brake
				if(copter.control_mode == Mode::Number::LOITER)
				{
					if(rf_state.alt_cm < copter.avoid.fence_margin())
					{
						AP_Notify::flags.parachute_release = 1; // beep alarm
			    		copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND);
						gcs().send_text(MAV_SEVERITY_CRITICAL, "Obstacle (%d) : LOITER to BRAKE !!", rf_state.alt_cm);
					}
				}
			}
		}
#endif

        // send downward facing lidar altitude and health to waypoint and circle navigation libraries
        if (rf_orient == ROTATION_PITCH_270) {
            if (rangefinder_state.alt_healthy || timed_out) {
                wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
                circle_nav->set_rangefinder_alt(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
            }
        }
    }

#else
    // downward facing rangefinder
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;

    // upward facing rangefinder
    rangefinder_up_state.enabled = false;
    rangefinder_up_state.alt_healthy = false;
    rangefinder_up_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok()
{
    return (rangefinder_up_state.enabled && rangefinder_up_state.alt_healthy);
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret)
{
    if (!rangefinder_alt_ok()) {
        return false;
    }

    ret = rangefinder_state.alt_cm_filt.get();
    float inertial_alt_cm = inertial_nav.get_altitude();
    ret += inertial_alt_cm - rangefinder_state.inertial_alt_cm;

#if 1
	if(AP_HAL::millis() - front_loop_time > 1000)
	{
		gcs().send_text(MAV_SEVERITY_INFO,"land rng cm %d", (uint16_t)ret);
		front_loop_time = AP_HAL::millis();
	}
#endif

    return true;
}


/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
#if RPM_ENABLED == ENABLED
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            logger.Write_RPM(rpm_sensor);
        }
    }
#endif
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // OPTFLOW == ENABLED
}

void Copter::compass_cal_update()
{
    compass.cal_update();

    if (hal.util->get_soft_armed()) {
        return;
    }

    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors->armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
#endif
}

// winch and wheel encoder initialisation
void Copter::winch_init()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.init();
    g2.winch.init(&g2.wheel_encoder);
#endif
}

// winch and wheel encoder update
void Copter::winch_update()
{
#if WINCH_ENABLED == ENABLED
    g2.wheel_encoder.update();
    g2.winch.update();
#endif
}
