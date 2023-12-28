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


#include <stdio.h>          // jhkang
#include <ctype.h>          // jhkang

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_DLA_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_EFI/AP_EFI_Serial_DLA.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Math/definitions.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_Notify/AP_Notify.h>        // jhkang
#include <GCS_MAVLink/GCS.h>            // jhkang

#define DLA232_STATUS_PKT_SIZE 127

#define SERIAL_WAIT_TIMEOUT_MS 100

#define ENGINE_RUNNING 4
#define THROTTLE_POSITION_FACTOR 10
#define CRANK_SHAFT_SENSOR_OK 0x0F
#define INJECTION_TIME_RESOLUTION 0.8
#define THROTTLE_POSITION_RESOLUTION 0.1
#define VOLTAGE_RESOLUTION 0.0049       /* 5/1024 */
#define ADC_CALIBRATION (5.0/1024.0)
#define MAP_HPA_PER_VOLT_FACTOR 248.2673
#define HPA_TO_KPA 0.1
#define TPS_SCALE 0.70

extern const AP_HAL::HAL& hal;

/**
 * @brief Constructor with port initialization
 * 
 * @param _frontend 
 */
AP_EFI_Serial_DLA232::AP_EFI_Serial_DLA232(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}

/**
 * @brief checks for response from or makes requests to DLA232 ECU periodically
 * 
 */
void AP_EFI_Serial_DLA232::update()
{
    if (port == nullptr) {
        return;
    }

    // parse response from DLA232
    check_response();

    // send request
    send_request();
}

// jhkang - ADD
void AP_EFI_Serial_DLA232::hexdump(void *ptr, int buflen)
{
  unsigned char *buf = (unsigned char *)ptr;
  int i, j;
  for (i = 0; i < buflen; i += 16)
    {
      printf ("%06x: ", i);
      for (j = 0; j < 16; j++)
        if (i + j < buflen)
          printf ("%02x ", buf[i + j]);
        else
          printf ("   ");
      printf (" ");
      for (j = 0; j < 16; j++)
        if (i + j < buflen)
          printf ("%c", isprint (buf[i + j]) ? buf[i + j] : '.');
      printf ("\n");
    }
}

/**
 * @brief Checks if required bytes are available and proceeds with parsing
 * 
 */
void AP_EFI_Serial_DLA232::check_response()
{
    const uint32_t now = AP_HAL::millis();

    // waiting for response
    if (!waiting_response) { // not requested status
        return;
    }

	uint16_t checksum;
	uint8_t header[2]; uint8_t tail[2];
    const uint32_t num_bytes = port->available();

    // if already requested
    if (num_bytes >= DLA232_STATUS_PKT_SIZE) 
	{
        // read data from buffer

        header[0] = port->read();
        header[1] = port->read();

        for (int i = 0; i < (DLA232_STATUS_PKT_SIZE - 6); i++) {
            // raw_data[i] = read_byte_CRC16();
            raw_data[i] = port->read();
        }

#if 0
        if ( (dump_time == 0) || (AP_HAL::millis() - dump_time > 2000)) {
            dump_time = AP_HAL::millis();
            hexdump(raw_data, (DLA232_STATUS_PKT_SIZE - 6));    // jhkang - dump data
        }
#endif
        checksum = port->read(); 
		checksum += port->read();

        tail[0] = port->read(); tail[1] = port->read();
        
        computed_checksum = crc16_ccitt_false(raw_data, DLA232_STATUS_PKT_SIZE-6, 0);

        if (header[0] != 0xF1 || header[1] != 0x1F || tail[0] != 0xF2 || tail[1] != 0x2F) {
			ack_fail_cnt++;
            printf("ack_fail_cnt=%d\n", ack_fail_cnt);
			port->discard_input();
		}
        #if 1
		else if (checksum != computed_checksum) {
            crc_fail_cnt++;
            port->discard_input();
            #if 0
            if ( (dump_time == 0) || (AP_HAL::millis() - dump_time > 2000)) {
                dump_time = AP_HAL::millis();
            #endif
                printf("*");
                //printf("crc_fail_cnt=%d : rec_crc=0x%x : computed_crc=0x%x\n",crc_fail_cnt, checksum, computed_checksum);
                //hexdump(raw_data, DLA232_STATUS_PKT_SIZE);    // jhkang - dump data
            #if 0
            }
            #endif
        }
        #endif
         else {
            uptime = now - last_packet_ms;
            last_packet_ms = now;
            internal_state.last_updated_ms = now;
            decode_data();
            copy_to_frontend();
            port->discard_input();
        }

        waiting_response = false;

#if HAL_LOGGING_ENABLED
        //log_status();
#endif
    }

    // reset request if no response for SERIAL_WAIT_TIMEOUT_MS
    if (waiting_response &&
        now - last_request_ms > SERIAL_WAIT_TIMEOUT_MS) 
	{
        waiting_response = false;
        last_request_ms = now;

        port->discard_input();
        ack_fail_cnt++;
    }
}

/**
 * @brief Send Throttle and Telemetry requests to DLA232
 * 
 */
void AP_EFI_Serial_DLA232::send_request()
{
    if (waiting_response) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    // get new throttle value
    //const uint16_t new_throttle = internal_state.req_engine_throttle_level;
    const uint16_t new_throttle = AP_Notify::engine_cmd.engine_throttle;

#if 0   // jhkang-ORG
	if (internal_state.req_engine_startup)
	{
		if ((internal_state.ecu_index == 0) && (now - last_req_send_startup_ms > 2000)) {
			send_request_startup();
			internal_state.req_engine_startup = false;
        	last_req_send_startup_ms = now;
		}
	}
	else if (internal_state.req_engine_flameout)
	{
        bool allow_flameout = hal.util->get_soft_armed();
        if (!allow_flameout) 
		{
			send_request_flameout();
			internal_state.req_engine_flameout = false;
		}
	}
#else   // jhkang-CHG
    // if (internal_state.req_engine_startup)
    if (AP_Notify::engine_cmd.engine_start)
	{
		if ((internal_state.ecu_index == 0) && (now - last_req_send_startup_ms > 2000)) {
			send_request_startup();
			// internal_state.req_engine_startup = false;
            AP_Notify::engine_cmd.engine_start = false;
        	last_req_send_startup_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Engine Started");
            printf(">> Engine Started!\n\r");
		}
	}
	// else if (internal_state.req_engine_flameout)
	else if (AP_Notify::engine_cmd.engine_flameout)
	{
        bool allow_flameout = hal.util->get_soft_armed();
        if (!allow_flameout) {
			send_request_flameout();
			// internal_state.req_engine_flameout = false;
            AP_Notify::engine_cmd.engine_flameout = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Engine FlameOut Done");
            printf(">> Engine FlameOut Done!\n\r");
		}
	}
#endif
    // check for change or timeout for throttle value
	//else if (internal_state.ecu_index && internal_state.engine_state   )
	else if (internal_state.ecu_index)
	{
		if((new_throttle != last_throttle) || 
			(now - last_req_send_throttle_ms > 30))
		{
        	// send new throttle value, only when ARMED
        	bool allow_throttle = hal.util->get_soft_armed();
        	if (allow_throttle)
			{
				if(new_throttle != last_throttle) {
            		send_throttle_values(new_throttle);
                    printf("Engine Throttle [%d%%] Out!", new_throttle*10);
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Engine Throttle [%d%%] Out!", new_throttle*10);
                }
        		else {
            		send_throttle_values(last_throttle);
                }
        	}
		}

		if(new_throttle != last_throttle)
        	last_throttle = new_throttle;

        last_req_send_throttle_ms = now;
    }
	// send engine status request
	else if (now - last_req_send_status_ms > 150)
	{
       	send_request_status();
        waiting_response = true;
		last_req_send_status_ms = now;
	}

    last_request_ms = now;
}


/**
 * @brief sends the new throttle command to DLA232 ECU
 * 
 * @param thr - new throttle value given by SRV_Channel::k_throttle
 * @return true - if success
 * @return false - currently not implemented
 */
bool AP_EFI_Serial_DLA232::send_throttle_values(uint16_t thr)
{
	static uint8_t d[20] = {
		0xF1, 0x1F,                             // [0,1]   frame header 
		0x00, 0x00, 0x00, 0x12,                 // [2,3,4,5] packet length 
		0x00, 0x00,                             // [6,7] frame number
		0x22,                                   // [8] command word
        0x77,                                   // [9] Write the throttle opening cmd
        0x32,                                   // [10] The Low byte of register address to be written
        0x05,                                   // [11] The High byte of register address to be written
        0x02,                                   // [12] The Low byte of length to be written
        0x00,                                   // [13] The High byte of length to be written
		0x00,                                   // [14] The Low byte of throttle opening value 10 times to ECU
        0x00,                                   // [15] The High byte of throttle opening value 10 times to ECU
		0x5F, 0xFC,                             // [17,17] checkword
		0xF2, 0x2F                              // [18,19] frame tail
	};

	if(thr == 1) { d[14] = 0x64; d[15] = 0x00; d[16] = 0x98; d[17] = 0x12; }
	else if(thr == 2) { d[14] = 0xC8; d[15] = 0x00; d[16] = 0xC0; d[17] = 0x01; }
	else if(thr == 3) { d[14] = 0x2C; d[15] = 0x01; d[16] = 0x0C; d[17] = 0x56; }
	else if(thr == 4) { d[14] = 0x90; d[15] = 0x01; d[16] = 0x57; d[17] = 0x36; }
	else if(thr == 5) { d[14] = 0xF4; d[15] = 0x01; d[16] = 0x90; d[17] = 0xD8; }
	else if(thr == 6) { d[14] = 0x58; d[15] = 0x02; d[16] = 0xF8; d[17] = 0xA8; }
	else if(thr == 7) { d[14] = 0xBC; d[15] = 0x02; d[16] = 0x24; d[17] = 0xDE; }
	else if(thr == 8) { d[14] = 0x20; d[15] = 0x03; d[16] = 0x69; d[17] = 0x79; }
	else if(thr == 9) { d[14] = 0x84; d[15] = 0x03; d[16] = 0xB8; d[17] = 0xC3; }
	else if(thr == 10) { d[14] = 0xE8; d[15] = 0x03; d[16] = 0xF6; d[17] = 0x84; }

    port->write(d, sizeof(d));

    return true;
}


/**
 * @brief cyclically sends different Status requests to DLA232 ECU
 * 
 * @return true - when successful
 * @return false  - not implemented
 */
bool AP_EFI_Serial_DLA232::send_request_status() {

	static const uint8_t d[14] = {
		0xF1, 0x1F,                     // frame header
		0x00, 0x00, 0x00, 0x0C,         // packet length
		0x00, 0x00,                     // frame number
		0x22,                           // command word
        0x66,                           // status data
		0xD6, 0xF1,                     // checkword(crc)
		0xF2, 0x2F                      // frame tail
	};

    port->write(d, sizeof(d));

    return true;
}

bool AP_EFI_Serial_DLA232::send_request_startup() {

	static const uint8_t d[16] = {
		0xF1, 0x1F,                     // frame header
		0x00, 0x00, 0x00, 0x0E,         // packet length
		0x00, 0x00,                     // frame number
		0x22,                           // command word
        0x71, 0x71, 0x64,               // status data ('qqd')
		0xF1, 0xE5,                     // checkword(crc)
		0xF2, 0x2F                      // frame tail
	};

    port->write(d, sizeof(d));

    return true;
}

bool AP_EFI_Serial_DLA232::send_request_flameout() {

	static const uint8_t d[16] = {
		0xF1, 0x1F,                     // frame header
		0x00, 0x00, 0x00, 0x0E,         // packet length
		0x00, 0x00,                     // frame number
		0x22,                           // command word
        0x71, 0x71, 0x78,               // status data ('qqx')
		0x22, 0x58,                     // checkword(crc)
		0xF2, 0x2F                      // frame tail
	};

    port->write(d, sizeof(d));

    return true;
}


/**
 * @brief parses the response from DLA232 ECU and updates the internal state instance
 * 
 */
void AP_EFI_Serial_DLA232::decode_data()
{
    struct Status *status = (Status*)raw_data;

    internal_state.engine_speed_rpm 				= status->rpm;
	internal_state.engine_load_percent 				= status->engine_state;
    internal_state.intake_manifold_pressure_kpa 	= (float)status->atmospheric_pressure*0.1;
	internal_state.intake_manifold_temperature 		= (float)status->atmospheric_temperature*0.1;
	internal_state.ignition_voltage 				= (float)status->battery_voltage*0.1;
	internal_state.ecu_index 						= status->startup_success;
	internal_state.fuel_consumption_rate_cm3pm 		= status->engine_fuel;
	internal_state.coolant_temperature 				= (float)status->cylinder_1_temp*0.1;
    internal_state.oil_pressure 					= (float)status->cylinder_3_temp*0.1;
    internal_state.oil_temperature 					= (float)status->cylinder_4_temp*0.1;
    internal_state.fuel_pressure 					= (float)status->cylinder_2_temp*0.1;
    internal_state.estimated_consumed_fuel_volume_cm3 = (float)status->rt_engine_fuel*0.1;
    internal_state.spark_dwell_time_ms 				= status->runtime;

    printf("startup_s=%x\n",status->startup_success);
}

uint8_t AP_EFI_Serial_DLA232::read_byte_CRC16()
{
	    // Read a byte and update the CRC
	    uint8_t data = port->read();
		computed_checksum = CRC16_compute_byte(computed_checksum, data);
		return data;
}

uint16_t AP_EFI_Serial_DLA232::CRC16_compute_byte(uint16_t crc, uint8_t data)
{
	    //crc ^= ~0U;
		crc = crc16_ccitt_false(&data, 1, crc);
		//crc ^= ~0U;
		return crc;
}

#if HAL_LOGGING_ENABLED
void AP_EFI_Serial_DLA232::log_status(void)
{
    // @LoggerMessage: EFIS
    // @Description: Electronic Fuel Injection data - Hirth specific Status information
    // @Field: TimeUS: Time since system startup
    // @Field: ETS1: Status of EGT1 excess temperature error
    // @Field: ETS2: Status of EGT2 excess temperature error
    // @Field: CTS1: Status of CHT1 excess temperature error
    // @Field: CTS2: Status of CHT2 excess temperature error
    // @Field: ETSS: Status of Engine temperature sensor
    // @Field: ATSS: Status of Air temperature sensor
    // @Field: APSS: Status of Air pressure sensor
    // @Field: TSS: Status of Temperature sensor
    // @Field: CRCF: CRC failure count
    // @Field: AckF: ACK failure count
    // @Field: Up: Uptime between 2 messages
    // @Field: ThrO: Throttle output as received by the engine
    AP::logger().WriteStreaming("EFIS",
                                "TimeUS,ETS1,ETS2,CTS1,CTS2,ETSS,ATSS,APSS,TSS,CRCF,AckF,Up,ThrO",
                                "s------------",
                                "F------------",
                                "QBBBBBBBBIIIf",
                                AP_HAL::micros64(),
                                uint8_t(EGT_1_error_excess_temperature_status),
                                uint8_t(EGT_2_error_excess_temperature_status),
                                uint8_t(CHT_1_error_excess_temperature_status),
                                uint8_t(CHT_2_error_excess_temperature_status),
                                uint8_t(engine_temperature_sensor_status),
                                uint8_t(air_temperature_sensor_status),
                                uint8_t(air_pressure_sensor_status),
                                uint8_t(throttle_sensor_status),
                                uint32_t(crc_fail_cnt),
                                uint32_t(ack_fail_cnt),
                                uint32_t(uptime),
                                float(internal_state.throttle_out));
}
#endif // HAL_LOGGING_ENABLED

#endif // AP_EFI_SERIAL_DLA_ENABLED
