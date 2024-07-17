#include "Copter.h"

//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = false;
static uint16_t failsafe_last_ticks;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void Copter::failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void Copter::failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void Copter::failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // the main loop is running, all is OK
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors->
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors->armed()) {
            motors->output_min();
        }

        AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors->armed()) {
            motors->armed(false);
            motors->output();
        }
    }
}

#if 1 // YIG-ADD
void Copter::disable_diagnosis()
{
	diagnosis_enabled = false;
}

void Copter::enable_diagnosis()
{
	// 초기화
	AP_Notify::diag_status.ov = 				 0;
	AP_Notify::diag_status.oc = 				 0;
	AP_Notify::diag_status.ot = 				 0;
	AP_Notify::diag_status.deadlock = 			 0;
	AP_Notify::diag_status.gyro_failed[0] = 	 0;
	AP_Notify::diag_status.gyro_failed[1] = 	 0;
	AP_Notify::diag_status.gyro_failed[2] = 	 0;
	AP_Notify::diag_status.accel_failed[0] = 	 0;
	AP_Notify::diag_status.accel_failed[1] = 	 0;
	AP_Notify::diag_status.accel_failed[2] = 	 0;
	AP_Notify::diag_status.compass_failed[0] =   0;
	AP_Notify::diag_status.compass_failed[1] = 	 0;
	AP_Notify::diag_status.compass_failed[2] = 	 0;
	AP_Notify::diag_status.baro_failed[0] = 	 0;
	AP_Notify::diag_status.baro_failed[1] = 	 0;
	AP_Notify::diag_status.gps_failed[0] = 		 0;
	AP_Notify::diag_status.gps_failed[1] = 		 0;
	AP_Notify::diag_status.gps_failed[2] = 		 0;
	AP_Notify::diag_status.lidar_failed[0] = 	 0;
	AP_Notify::diag_status.lidar_failed[1] = 	 0;
	AP_Notify::diag_status.lte_link_failed[0] =  0;
	AP_Notify::diag_status.lte_link_failed[1] =  0;
	AP_Notify::diag_status.rf_link_failed = 	 0;
	AP_Notify::diag_status.storage_failed[0] =   0;
	AP_Notify::diag_status.storage_failed[1] =   0; // not used :  FC#2에 동시 저장됨
	AP_Notify::diag_status.motor_status_failed = 0;
	for(int i=0;i<9;i++) AP_Notify::diag_status.motor_failed[i] = 0;

#if 1 // YIG : For Diagnosis SW Insert
	AP_Notify::diag_status.ov_insert = 				   0;
	AP_Notify::diag_status.oc_insert = 				   0;
	AP_Notify::diag_status.ot_insert = 				   0;
	AP_Notify::diag_status.deadlock_insert = 		   0;
	AP_Notify::diag_status.gyro_failed_insert[0] =     0;
	AP_Notify::diag_status.gyro_failed_insert[1] =     0;
	AP_Notify::diag_status.gyro_failed_insert[2] =     0;
	AP_Notify::diag_status.accel_failed_insert[0] =    0;
	AP_Notify::diag_status.accel_failed_insert[1] =    0;
	AP_Notify::diag_status.accel_failed_insert[2] =    0;
	AP_Notify::diag_status.compass_failed_insert[0] =  0;
	AP_Notify::diag_status.compass_failed_insert[1] =  0;
	AP_Notify::diag_status.compass_failed_insert[2] =  0;
	AP_Notify::diag_status.baro_failed_insert[0] = 	   0;
	AP_Notify::diag_status.baro_failed_insert[1] = 	   0;
	AP_Notify::diag_status.gps_failed_insert[0] = 	   0;
	AP_Notify::diag_status.gps_failed_insert[1] = 	   0;
	AP_Notify::diag_status.gps_failed_insert[2] = 	   0;
	AP_Notify::diag_status.lidar_failed_insert[0] =    0;
	AP_Notify::diag_status.lidar_failed_insert[1] =    0;
	AP_Notify::diag_status.lte_link_failed_insert[0] = 0;
	AP_Notify::diag_status.lte_link_failed_insert[1] = 0;
	AP_Notify::diag_status.storage_failed_insert[0] =  0;
	AP_Notify::diag_status.storage_failed_insert[1] =  0;
	for(int i=0;i<9;i++) AP_Notify::diag_status.motor_failed_insert[i] = 0;
#endif

	AP_Notify::diag_status.watchdog_on = false;
    AP_Notify::diag_status.fc_switch_over = false;
	failed_motor = 9;

    diagnosis_enabled = true;
}

bool Copter::check_diagnosis()
{
#if 1 // YIG-IMSI

	if(AP_Notify::events.user_mode_change)
	{
		AP_Notify::events.user_mode_change = 0;
		Log_Write_Event(DATA_GRIPPER_GRAB);
	}
	if(AP_Notify::events.user_mode_change_failed)
	{
		AP_Notify::events.user_mode_change_failed = 0;
		Log_Write_Event(DATA_GRIPPER_RELEASE);
	}


	if(AP_Notify::events.autotune_failed)
	{
		AP_Notify::events.autotune_failed = 0;
		gcs().send_text(MAV_SEVERITY_CRITICAL, "Avoidance Event");
		Log_Write_Event(DATA_FLIP_START);
	}
	if(AP_Notify::flags.parachute_release)
	{
		AP_Notify::flags.parachute_release = 0;
		Log_Write_Event(DATA_FLIP_END);
	}

	if(AP_Notify::diag_status.ot) // deadlock imsi
	{
		AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_OCCURRED);
		AP_Notify::diag_status.ot = false;
		//AP::internalerror().error(AP_InternalError::error_t::main_loop_stuck);

		Log_Write_Event(DATA_SWITCH_OVER_RELEASED);
#if 0
		msc.switch_over(0);
#else // YIG-IMSI for Jawol 평가
		msc.switch_over_run(1);
#endif
		gcs().send_text(MAV_SEVERITY_CRITICAL, "Switch-Over to FC#2");
		AP_Notify::diag_status.oc = true; // failsafe 에서 활용

		// YIG-IMSI
		//set_mode(Mode::Number::LAND, ModeReason::FAILSAFE);
		//AP::logger().StopLogging();
		loop_time_1 = AP_HAL::millis(); // failsafe 에서 활용
		//
	}
#endif

    //if( AP_Notify::diag_status.ov || AP_Notify::diag_status.oc || AP_Notify::diag_status.ot || AP_Notify::diag_status.deadlock 			  ||
    if( AP_Notify::diag_status.ov || AP_Notify::diag_status.oc || AP_Notify::diag_status.deadlock 			  ||
	   (AP_Notify::diag_status.baro_failed[0]    && AP_Notify::diag_status.baro_failed[1]) 												  ||
	   (AP_Notify::diag_status.gyro_failed[0]    && AP_Notify::diag_status.gyro_failed[1]    && AP_Notify::diag_status.gyro_failed[2])    ||
	   (AP_Notify::diag_status.accel_failed[0]   && AP_Notify::diag_status.accel_failed[1]   && AP_Notify::diag_status.accel_failed[2])   ||
	   (AP_Notify::diag_status.compass_failed[0] && AP_Notify::diag_status.compass_failed[1] && AP_Notify::diag_status.compass_failed[2]) ||
	   (AP_Notify::diag_status.gps_failed[0]     && AP_Notify::diag_status.gps_failed[1]     && AP_Notify::diag_status.gps_failed[2])
	  )
	{
		if(!AP_Notify::diag_status.fc_switch_over)
		{
			AP_Notify::diag_status.fc_switch_over = true;
#if 0 // YIG-IMSI
			msc.switch_over(0);
			Log_Write_Event(DATA_SWITCHOVER_RELEASE);
			gcs().send_text(MAV_SEVERITY_CRITICAL, "SWITCHOVER TO FC#2");
#endif
		}
	}

	// Motor Fail
	if     (AP_Notify::diag_status.motor_failed[0] == true) failed_motor = 0;
	//else if(AP_Notify::diag_status.motor_failed[1] == true) failed_motor = 1;
	else if(AP_Notify::diag_status.motor_failed[2] == true) failed_motor = 2;
	else if(AP_Notify::diag_status.motor_failed[3] == true) failed_motor = 3;
	else if(AP_Notify::diag_status.motor_failed[4] == true) failed_motor = 4;
	else if(AP_Notify::diag_status.motor_failed[5] == true) failed_motor = 5;
	else if(AP_Notify::diag_status.motor_failed[6] == true) failed_motor = 6;
	else if(AP_Notify::diag_status.motor_failed[7] == true) failed_motor = 7;

	else if(AP_Notify::diag_status.motor_failed[8] == true) failed_motor = 8;

	if(failed_motor < 8 && AP_Notify::diag_status.motor_failed[1] == false)
	{
		//motors->set_lost_motor(failed_motor);
		//msc.motor_fail(0, (failed_motor * 10));

		uint8_t perc = (uint8_t)copter.fence.get_margin();
		msc.motor_fail(6, perc * 10);

		gcs().send_text(MAV_SEVERITY_INFO, "Motor (#7) Loss");
		AP::logger().Write_Error(LogErrorSubsystem::THRUST_LOSS_CHECK, LogErrorCode::FAILSAFE_OCCURRED);

		//motors->set_thrust_boost(true);
		AP_Notify::diag_status.motor_status_failed = true;
		//set_mode(Mode::Number::LAND, ModeReason::FAILSAFE);

		gcs().send_text(MAV_SEVERITY_INFO, "Thrust Boost Mode triggered");

		AP_Notify::diag_status.motor_failed[2] = false;
		AP_Notify::diag_status.motor_failed[1] = true;

		failed_motor = 9;
	}
	else if(failed_motor == 8 && AP_Notify::diag_status.motor_failed[1] == true)
	{
		msc.motor_success();

		gcs().send_text(MAV_SEVERITY_INFO, "motor #7");

		//motors->set_thrust_boost(false);
		AP_Notify::diag_status.motor_status_failed = false;
		AP_Notify::diag_status.motor_failed[8] = false;
		AP_Notify::diag_status.motor_failed[1] = false;

		failed_motor = 9;
	}
	//

	return true;
}

bool Copter::redundancy_transfer()
{
	if(!diagnosis_enabled) return false;

    if(AP_Notify::diag_status.fc_switch_over)
    {
	    transfer_redundancy(1);
        diagnosis_enabled = false;
    }

	return true;
}
#endif

#if ADVANCED_FAILSAFE == ENABLED
/*
  check for AFS failsafe check
*/
void Copter::afs_fs_check(void)
{
    // perform AFS failsafe checks
#if AC_FENCE
    const bool fence_breached = fence.get_breaches() != 0;
#else
    const bool fence_breached = false;
#endif
    g2.afs.check(failsafe.last_heartbeat_ms, fence_breached, last_radio_update_ms);
}
#endif
