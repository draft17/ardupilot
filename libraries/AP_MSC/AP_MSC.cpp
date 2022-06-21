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

    spitxdata = (struct spitx *)msc_spidata;
    spirxdata = (struct spirx *)msc_spidata;

//    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSC::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
//        debug_msc(1, "MSC: couldn't create thread\n\r");
//        return;
//    }
	if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_MSC::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
    debug_msc(1, "MSC: couldn't create thread\n\r");
    return;
}

    _initialized = true;
    debug_msc(2, "MSC: init done\n\r");

	_test_timer1 = AP_HAL::millis();
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

#if 0
	// imsi
	if(AP_HAL::millis() - _test_timer1 > 2000)
	{
        for (uint8_t i = 0; i < max_esc_num; i++) {
			gcs().send_text(MAV_SEVERITY_INFO, "%4.0d", _SRV_conf[i].pulse);
		}
		_test_timer1 = AP_HAL::millis();
	}
#endif

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
        for (uint8_t i = 0; i < max_esc_num; i++) {
            if ((((uint32_t) 1) << i) & MSC_ESC_BM) {
                // TODO: ESC negative scaling for reverse thrust and reverse rotation

                spitxdata->active = activeflag;
                spitxdata->channel = i;
                //spitxdata->rpm = static_cast<uint16_t>((_SRV_conf[i].pulse - 1100) * 2.5);
				spitxdata->rpm = _SRV_conf[i].pulse;
                spitxdata->reserved = 0;
                spitxdata->spiCRC = crc_crc32(0xFFFFFFFF, msc_spidata, 4);

                if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                    AP_HAL::panic("PANIC: AP_Baro_MS56XX: failed to take serial semaphore for init");
                }
                _dev->transfer(msc_spidata, 8, msc_spidata, 8);
                _dev->get_semaphore()->give();
                //if(crc_crc32(0xFFFFFFFF, msc_spidata, 4) == spirxdata->spiCRC)
                //    debug_msc(1, "%d %d %d\r\n", spirxdata->err, spirxdata->channel, spirxdata->rpm);
            } else {
                // esc_msg.cmd.push_back(static_cast<unsigned>(0));
            }
        }
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
void AP_MSC::switch_over(int a)
{
	activeflag = a;
}

void AP_MSC::motor_fail(uint8_t num, uint8_t percentage)
{
	motorfailflag = num;
	motorfailperc = percentage;
}

AP_MSC *AP_MSC::msc_singleton;

#endif // HAL_WITH_MSC

