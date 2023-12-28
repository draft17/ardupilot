/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_DLA_ENABLED
#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

/*!
 * class definition for DLA232 ECU
 */
class AP_EFI_Serial_DLA232: public AP_EFI_Backend {
public:
    AP_EFI_Serial_DLA232(AP_EFI &_frontend);

    void update() override;

private:
    // serial port instance
    AP_HAL::UARTDriver *port;

    // periodic refresh 
    uint32_t last_request_ms;
    uint32_t last_packet_ms;
    uint32_t last_req_send_throttle_ms;
    uint32_t last_req_send_startup_ms;
    uint32_t last_req_send_flameout_ms;
    uint32_t last_req_send_status_ms;

    // raw bytes - max size
    uint8_t raw_data[256];
	uint16_t computed_checksum;

    // TRUE - Request is sent; waiting for response
    // FALSE - Response is already received
    bool waiting_response;

    // Throttle values
    uint16_t last_throttle;

    uint32_t last_fuel_integration_ms;

    // custom status for Hirth
    bool engine_temperature_sensor_status;
    bool air_temperature_sensor_status;
    bool air_pressure_sensor_status;
    bool throttle_sensor_status;

    bool CHT_1_error_excess_temperature_status;
    bool CHT_2_error_excess_temperature_status;
    bool EGT_1_error_excess_temperature_status;
    bool EGT_2_error_excess_temperature_status;
    uint32_t crc_fail_cnt;
    uint32_t uptime;
    uint32_t ack_fail_cnt;
    uint32_t dump_time;

    struct PACKED Status {                      // [1][2] : frame header
		uint32_t pkt_length;                    // [3][4][5][6]
        uint16_t reserved_78;                   // [7][8]
		uint8_t  command;                       // [9]
		uint8_t  reserved_1015[6];              // [10][11][12][13][14][15]
		uint16_t rpm;                           // [16][17]
		uint8_t  reserved_1820[3];              // [18][19][20]
		uint8_t  engine_state;                  // [21]
		uint8_t  reserved_2225[4];              // [22][23][24][25]
		uint16_t atmospheric_pressure;          // [26][27]
		uint8_t  reserved_2829[2];              // [28][29]
		uint16_t atmospheric_temperature;       // [30][31]
		uint8_t  reserved_3235[4];              // [32][33][34][35]
		uint16_t battery_voltage;               // [36][37]
		uint8_t  reserved_3889[52];             // [38]...[89]
		uint8_t  startup_success;               // [90]
        uint8_t  reserved_91;                   // [91]
		uint32_t engine_fuel;                   // [92][93][94][95]
		uint16_t cylinder_1_temp;               // [96][97]
		uint16_t cylinder_3_temp;               // [98][99]
		uint16_t cylinder_4_temp;               // [100][101]
		uint16_t cylinder_2_temp;               // [102][103]
		uint8_t  reserved_104107[4];            // [104][105][106][107]
		uint16_t rt_engine_fuel;                // [108][109]
		uint8_t  reserved_110119[10];           // [110]...[119]
		uint32_t runtime;                       // [120][121][122][123]
    };                                          // [124][125]-CRC : [126][127]-frame end
    //static_assert(sizeof(Status) == 121, "incorrect Status length");
    void hexdump(void *ptr, int buflen);    // jhkang - ADD
    void check_response();
    void send_request();
    void decode_data();
    bool send_request_status();
    bool send_request_startup();
    bool send_request_flameout();
    bool send_throttle_values(uint16_t);
    void log_status();
	uint8_t read_byte_CRC16();
    uint16_t CRC16_compute_byte(uint16_t crc, uint8_t data);
    
};

#endif // AP_EFI_SERIAL_HIRTH_ENABLED
