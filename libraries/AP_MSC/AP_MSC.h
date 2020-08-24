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
#ifndef AP_MSC_H_
#define AP_MSC_H_

#include <AP_HAL/Semaphores.h>
// #include <AP_Param/AP_Param.h>

#ifndef MSC_SRV_NUMBER
#define MSC_SRV_NUMBER 18
#endif
#define MSC_ESC_BM 0x000000FF

class AP_MSC {
public:
    AP_MSC();
    ~AP_MSC();

    // Return MSC from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_MSC *get_msc();

    void init();

    ///// SRV output /////
    void SRV_push_servos(void);

 private:

    void loop(void);

    ///// SRV output /////
    void SRV_send_esc();

    // MSC parameters
    // AP_Int32 _servo_bm;
    // AP_Int32 _esc_bm;

    char _thread_name[13] = "msc_01";
    bool _initialized;
    static AP_MSC *msc_singleton;
    ///// SRV output /////
    struct {
        uint16_t pulse;
        bool esc_pending;
        bool servo_pending;
    } _SRV_conf[MSC_SRV_NUMBER];

    uint8_t _SRV_armed;
    HAL_Semaphore SRV_sem;

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t msc_spidata[8];
    struct spitx
    {
        uint32_t active : 1;
        uint32_t channel : 3;
        uint32_t rpm : 12;
        uint32_t reserved : 16;
        uint32_t spiCRC : 32;
    } *spitxdata;
    struct spirx
    {
        uint32_t channel : 3;
        uint32_t rpm : 12;
        uint32_t err : 16;
        uint32_t reserved : 1;
        uint32_t spiCRC : 32;
    } *spirxdata;
};

#endif /* AP_MSC_H_ */

