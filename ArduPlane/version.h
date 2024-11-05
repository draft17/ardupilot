#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduPlane V4.2.2.5"
/* 
 *	Version inform :
 *  SoomVi AF-BirdControl Plane V4.2.2.2 - FC log encryption, decryption 변경
 *  SoomVi AF-BirdControl Plane V4.2.2.3 - GCS에서 set_parameter를 이용한 Motor E-STOP 구현'
 *  ArduPlane V4.2.2.4 - plot.ardupilot.org 에서 FW version(ArduPlane) 확인_flight mode 이상 수정
 *  ArduPlane V4.2.2.5 - qloiter, qhover mode에서 prop 돌지 않게 수정.
 */

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,2,2,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 2
#define FW_PATCH 2
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

#include <AP_Common/AP_FWVersionDefine.h>
