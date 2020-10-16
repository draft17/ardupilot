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
	AP_Notify::diag_status.ov = 0;
	AP_Notify::diag_status.oc = 0;
	AP_Notify::diag_status.ot = 0;
	AP_Notify::diag_status.deadlock = 0;
	AP_Notify::diag_status.gyro_failed[0] = 0;
	AP_Notify::diag_status.gyro_failed[1] = 0;
	AP_Notify::diag_status.gyro_failed[2] = 0;
	AP_Notify::diag_status.pri_gyro = 0;
	AP_Notify::diag_status.accel_failed[0] = 0;
	AP_Notify::diag_status.accel_failed[1] = 0;
	AP_Notify::diag_status.accel_failed[2] = 0;
	AP_Notify::diag_status.pri_accel = 0;
	AP_Notify::diag_status.compass_failed[0] = 0;
	AP_Notify::diag_status.compass_failed[1] = 0;
	AP_Notify::diag_status.compass_failed[2] = 0;
	AP_Notify::diag_status.pri_compass = 0;
	AP_Notify::diag_status.baro_failed[0] = 0;
	AP_Notify::diag_status.baro_failed[1] = 0;
	AP_Notify::diag_status.pri_baro = 0;
	AP_Notify::diag_status.gps_failed[0] = 0;
	AP_Notify::diag_status.gps_failed[1] = 0;
	AP_Notify::diag_status.gps_failed[2] = 0;
	AP_Notify::diag_status.pri_gps = 0;
	AP_Notify::diag_status.lidar_failed[0] = 0;
	AP_Notify::diag_status.lidar_failed[1] = 0;
	AP_Notify::diag_status.pri_lidar = 0;
	AP_Notify::diag_status.lte_link_failed[0] = 0;
	AP_Notify::diag_status.lte_link_failed[1] = 0;
	AP_Notify::diag_status.pri_lte = 0;
	AP_Notify::diag_status.rf_link_failed = 0;
	AP_Notify::diag_status.storage_failed[0] = 0;
	AP_Notify::diag_status.storage_failed[1] = 0; // not used :  FC#2에 동시 저장됨
	AP_Notify::diag_status.pri_storage = 0; // not used
    AP_Notify::diag_status.watchdog_on = 0;
    fc_switch_over = false;

    diagnosis_enabled = true;
}

bool Copter::check_diagnosis()
{
	if(!diagnosis_enabled) return false;

    if(AP_Notify::diag_status.ov || AP_Notify::diag_status.oc ||
	   AP_Notify::diag_status.ot || AP_Notify::diag_status.deadlock)
		fc_switch_over = true;

	else if(AP_Notify::diag_status.gyro_failed[0] && AP_Notify::diag_status.gyro_failed[1] && AP_Notify::diag_status.gyro_failed[2])
		fc_switch_over = true;

	else if(AP_Notify::diag_status.accel_failed[0] && AP_Notify::diag_status.accel_failed[1] && AP_Notify::diag_status.accel_failed[2])
		fc_switch_over = true;

	else if(AP_Notify::diag_status.baro_failed[0] && AP_Notify::diag_status.baro_failed[1])
		fc_switch_over = true;

	else if(AP_Notify::diag_status.compass_failed[0] && AP_Notify::diag_status.compass_failed[1] && AP_Notify::diag_status.compass_failed[2])
		fc_switch_over = true;

	else if(AP_Notify::diag_status.gps_failed[0] && AP_Notify::diag_status.gps_failed[1] && AP_Notify::diag_status.gps_failed[2])
		fc_switch_over = true;

	if(fc_switch_over) // 이중화 절체 (to F/C #2)
		printf("fc_switch_over\n\r");

	return true;
}

bool Copter::redundancy_transfer()
{
	if(!diagnosis_enabled) return false;

    if(fc_switch_over)
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
