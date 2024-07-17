/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */
#include <utility>
#include <stdio.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if 1 
//HAL_WITH_MSC
#include "AP_MSC.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define debug_msc(level_debug, fmt, args...) do { if ((level_debug) <= 3) { printf(fmt, ##args); }} while (0)

AP_MSC::AP_MSC()
{
    for (uint8_t i = 0; i < MSC_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
        _SRV_conf[i].servo_pending = false;
    }
	activeflag = 1;
	activeflag_run = 0;
	activeflag_run_cnt = 0;
	motorfailflag = - 1;
	motorfailperc = 100;
    msc_singleton = this;
    debug_msc(2, "AP_MSC constructed\n\r");
}

AP_MSC::~AP_MSC()
{
}

AP_MSC *AP_MSC::get_msc()
{
    return msc_singleton;
}

void AP_MSC::init()
{
    if (_initialized) {
        debug_msc(2, "MSC: init called more than once\n\r");
        return;
    }
    _dev = std::move(hal.spi->get_device("MSCmaster_spi5"));
    debug_msc(2, "MSC: SPI bus %d addr %d id %d\n\r", _dev->bus_num(), _dev->get_bus_address(), _dev->get_bus_id());

    spitxdata = (struct spitx *)msc_spidata_tx;
    spirxdata = (struct spirx *)msc_spidata_rx;

//    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSC::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
//        debug_msc(1, "MSC: couldn't create thread\n\r");
//        return;
//    }
	if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSC::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) 
	{
    	debug_msc(1, "MSC: couldn't create thread\n\r");
    	return;
	}

    _initialized = true;
    debug_msc(2, "MSC: init done\n\r");

	_test_timer1 = AP_HAL::millis();
	_test_timer2 = AP_HAL::millis();
	_test_timer3 = AP_HAL::millis();
}

void AP_MSC::loop(void)
{
    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
            return;
        }

        // if (_SRV_armed) {
        if (1) {
            // if we have any ESC's in bitmask
            if (MSC_ESC_BM > 0/* && !sent_servos*/) {
                SRV_send_esc();
            }

            for (uint8_t i = 0; i < MSC_SRV_NUMBER; i++) {
                _SRV_conf[i].esc_pending = false;
            }
            hal.scheduler->delay_microseconds(1000);
        }
    }
}


///// SRV output /////

void AP_MSC::SRV_send_esc(void)
{
    uint8_t active_esc_num = 0, max_esc_num = 0;

    WITH_SEMAPHORE(SRV_sem);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = 0; i < MSC_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & MSC_ESC_BM) {
            max_esc_num = i + 1;
			if (_SRV_conf[i].pulse == 500) _SRV_conf[i].pulse = 0;
			if (i == motorfailflag) _SRV_conf[i].pulse = _SRV_conf[i].pulse * motorfailperc / 100;
            if (_SRV_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {

#ifdef MSC_MULTI_X

        for (uint8_t i = 0; i < max_esc_num; i++) 
		{
            if ((((uint32_t) 1) << i) & MSC_ESC_BM) 
			{
				if(i==0)      spitxdata->rpm0 = _SRV_conf[0].pulse;
				else if(i==1) spitxdata->rpm1 = _SRV_conf[1].pulse;
				else if(i==2) spitxdata->rpm2 = _SRV_conf[2].pulse;
				else if(i==3) spitxdata->rpm3 = _SRV_conf[3].pulse;

				else if(i==4) spitxdata->rpm0 = _SRV_conf[4].pulse;
				else if(i==5) spitxdata->rpm1 = _SRV_conf[5].pulse;
				else if(i==6) spitxdata->rpm2 = _SRV_conf[6].pulse;
				else if(i==7) spitxdata->rpm3 = _SRV_conf[7].pulse;
			}

			if((i!=3) && (i!=7)) continue;

            spitxdata->active = activeflag;
            spitxdata->channel = i;
            spitxdata->reserved = 0;
            spitxdata->spiCRC = crc_crc64((uint32_t *)msc_spidata_tx, 2);

			memset(msc_spidata_rx, 0, sizeof(msc_spidata_rx));

			hal.scheduler->delay_microseconds(200);

            if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) AP_HAL::panic("PANIC: failed to take semaphore for MSC");
            _dev->transfer_fullduplex(msc_spidata_tx, msc_spidata_rx, 16);
            _dev->get_semaphore()->give();

            if(crc_crc64((uint32_t *)msc_spidata_rx, 2) == spirxdata->spiCRC) {
				if(AP_HAL::millis() - _test_timer2 > 10000) {
					gcs().send_text(MAV_SEVERITY_INFO, "MSC crc ok %02x%02x%02x%02x %02x%02x%02x%02x",
							msc_spidata_rx[0], msc_spidata_rx[1], msc_spidata_rx[2], msc_spidata_rx[3], msc_spidata_rx[4], msc_spidata_rx[5], msc_spidata_rx[6], msc_spidata_rx[7]);
					_test_timer2 = AP_HAL::millis();
				}
			} else {
				if(AP_HAL::millis() - _test_timer2 > 4000) {
					gcs().send_text(MAV_SEVERITY_INFO, "MSC crc err %02x%02x%02x%02x %02x%02x%02x%02x	%02x%02x%02x%02x %02x%02x%02x%02x", 
							msc_spidata_rx[0], msc_spidata_rx[1], msc_spidata_rx[2], msc_spidata_rx[3], msc_spidata_rx[4], msc_spidata_rx[5], msc_spidata_rx[6], msc_spidata_rx[7],
							msc_spidata_rx[8], msc_spidata_rx[9], msc_spidata_rx[10], msc_spidata_rx[11], msc_spidata_rx[12], msc_spidata_rx[13], msc_spidata_rx[14], msc_spidata_rx[15]);
					_test_timer2 = AP_HAL::millis();
				}
			}
        }

#else // MSC_UNIX_X

        for (uint8_t i = 0; i < max_esc_num; i++) 
		{
            if ((((uint32_t) 1) << i) & MSC_ESC_BM) 
			{

                spitxdata->active = activeflag;
                spitxdata->channel = i;
				spitxdata->rpm = _SRV_conf[i].pulse;
				// YIG-CHG
				if(activeflag_run) {
					if(activeflag_run_cnt < 20) {
						spitxdata->reserved = 1;
						activeflag_run_cnt++;
					}
					else {
						activeflag_run = 0; // 한번만 전송 위해
						activeflag_run_cnt = 0;
					}
				}
				else spitxdata->reserved = 0;
				//
                spitxdata->spiCRC = crc_crc32(0xFFFFFFFF, msc_spidata_tx, 4);
				memset(msc_spidata_rx, 0, sizeof(msc_spidata_rx));

#if 1 // YIG-ADD : for clear SPI crc error
				hal.scheduler->delay_microseconds(100);
#endif
                if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                    AP_HAL::panic("PANIC: failed to take semaphore for MSC");
                }

                //_dev->transfer(msc_spidata_tx, 8, msc_spidata_rx, 8);
                _dev->transfer_fullduplex(msc_spidata_tx, msc_spidata_rx, 8);

                _dev->get_semaphore()->give();

                if(crc_crc32(0xFFFFFFFF, msc_spidata_rx, 4) == spirxdata->spiCRC) {
					if(AP_HAL::millis() - _test_timer2 > 10000) {
						//gcs().send_text(MAV_SEVERITY_INFO, "%d MSC ok %02x %02x %02x %02x", AP_Notify::diag_status.deadlock_insert, msc_spidata_rx[0], msc_spidata_rx[1], msc_spidata_rx[2], msc_spidata_rx[3]);
						_test_timer2 = AP_HAL::millis();
					}
				} else {
					if(AP_HAL::millis() - _test_timer2 > 4000) {
						gcs().send_text(MAV_SEVERITY_INFO, "MSC crc err %02x%02x%02x%02x %02x%02x%02x%02x", 
								msc_spidata_rx[0], msc_spidata_rx[1], msc_spidata_rx[2], msc_spidata_rx[3], msc_spidata_rx[4], msc_spidata_rx[5], msc_spidata_rx[6], msc_spidata_rx[7]);
						_test_timer2 = AP_HAL::millis();
					}
				}
            } else {
                // esc_msg.cmd.push_back(static_cast<unsigned>(0));
            }
        }
#endif // MSC_MULTI_X
    }
}

void AP_MSC::SRV_push_servos()
{
    WITH_SEMAPHORE(SRV_sem);

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i)) {
            _SRV_conf[i].pulse = SRV_Channels::srv_channel(i)->get_output_pwm();
            _SRV_conf[i].esc_pending = true;
            _SRV_conf[i].servo_pending = true;
        }
    }

    _SRV_armed = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}

bool AP_MSC::motor_status_check(uint8_t num, uint32_t &error_code)
{
	if (_motor_status[num].err_code) {
		error_code = _motor_status[num].err_code; 
		return false;
	}

	return true;
}

uint8_t AP_MSC::is_active()
{
	return activeflag;
}

void AP_MSC::switch_over(uint8_t a)
{
	activeflag = a;
	gcs().send_text(MAV_SEVERITY_CRITICAL, "activeflag %d", activeflag);
}

void AP_MSC::switch_over_run(uint8_t a)
{
	activeflag_run = a;
	gcs().send_text(MAV_SEVERITY_CRITICAL, "activeflag_run %d", activeflag_run);
}

void AP_MSC::motor_fail(uint8_t num, uint8_t percentage)
{
	motorfailflag = num;
	motorfailperc = percentage;
}

void AP_MSC::motor_success()
{
	motorfailflag = - 1;
	motorfailperc = 100;
}

AP_MSC *AP_MSC::msc_singleton;

#endif // HAL_WITH_MSC

