/*!
 * @file rt_vectornav.h
 * @brief VectorNav IMU communication
 */

#ifndef _rt_vectornav
#define _rt_vectornav

#ifdef linux

#include <cstdint>
#include <lcm/lcm-cpp.hpp>
#include "SimUtilities/IMUTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vn/sensors.h"

#ifdef __cplusplus
}
#endif

bool init_vectornav(VectorNavData* vd_data);
bool vectornav_data_is_valid(uint64_t max_age_us);
uint64_t vectornav_last_packet_age_us();

#endif
#endif
