/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <systemlib/mavlink_log.h>
#include "ashtech.h"

GPSDriverAshtech::GPSDriverAshtech(GPSCallbackPtr callback, void *callback_user,
				   struct vehicle_gps_position_s *gps_position,
				   struct satellite_info_s *satellite_info, float heading_offset) :
	GPSHelper(callback, callback_user),
	_satellite_info(satellite_info),
	_gps_position(gps_position),	
	_mavlink_log_pub(nullptr),
	_heading_offset(heading_offset)
{
	decodeInit();
	_decode_state = NME_DECODE_UNINIT;
	_rx_buffer_bytes = 0;
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

int GPSDriverAshtech::handleMessage(int len)
{
	char *endp;	
	if (len < 7) {
		return 0;
	}

	int uiCalcComma = 0;

	for (int i = 0 ; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}

	char *bufptr = (char *)(_rx_buffer + 6);
	int ret = 0;

// 	if ((memcmp(_rx_buffer + 3, "ZDA,", 3) == 0) && (uiCalcComma == 6)) {
// 		PX4_ERR("zda");
// 		/*
// 		UTC day, month, and year, and local time zone offset
// 		An example of the ZDA message string is:

// 		$GPZDA,172809.456,12,07,1996,00,00*45

// 		ZDA message fields
// 		Field	Meaning
// 		0	Message ID $GPZDA
// 		1	UTC
// 		2	Day, ranging between 01 and 31
// 		3	Month, ranging between 01 and 12
// 		4	Year
// 		5	Local time zone offset from GMT, ranging from 00 through 13 hours
// 		6	Local time zone offset from GMT, ranging from 00 through 59 minutes
// 		7	The checksum data, always begins with *
// 		Fields 5 and 6 together yield the total offset. For example, if field 5 is -5 and field 6 is +15, local time is 5 hours and 15 minutes earlier than GMT.
// 		*/
// 		double ashtech_time = 0.0;
// 		int day = 0, month = 0, year = 0, local_time_off_hour __attribute__((unused)) = 0,
// 		    local_time_off_min __attribute__((unused)) = 0;

// 		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

// 		if (bufptr && *(++bufptr) != ',') { day = strtol(bufptr, &endp, 10); bufptr = endp; }

// 		if (bufptr && *(++bufptr) != ',') { month = strtol(bufptr, &endp, 10); bufptr = endp; }

// 		if (bufptr && *(++bufptr) != ',') { year = strtol(bufptr, &endp, 10); bufptr = endp; }

// 		if (bufptr && *(++bufptr) != ',') { local_time_off_hour = strtol(bufptr, &endp, 10); bufptr = endp; }

// 		if (bufptr && *(++bufptr) != ',') { local_time_off_min = strtol(bufptr, &endp, 10); bufptr = endp; }


// 		int ashtech_hour = static_cast<int>(ashtech_time / 10000);
// 		int ashtech_minute = static_cast<int>((ashtech_time - ashtech_hour * 10000) / 100);
// 		double ashtech_sec = static_cast<double>(ashtech_time - ashtech_hour * 10000 - ashtech_minute * 100);

// 		/*
// 		 * convert to unix timestamp
// 		 */
// 		struct tm timeinfo = {};
// 		timeinfo.tm_year = year - 1900;
// 		timeinfo.tm_mon = month - 1;
// 		timeinfo.tm_mday = day;
// 		timeinfo.tm_hour = ashtech_hour;
// 		timeinfo.tm_min = ashtech_minute;
// 		timeinfo.tm_sec = int(ashtech_sec);
// 		timeinfo.tm_isdst = 0;

// #ifndef NO_MKTIME
// 		time_t epoch = mktime(&timeinfo);

// 		if (epoch > GPS_EPOCH_SECS) {
// 			uint64_t usecs = static_cast<uint64_t>((ashtech_sec - static_cast<uint64_t>(ashtech_sec))) * 1000000;

// 			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
// 			// and control its drift. Since we rely on the HRT for our monotonic
// 			// clock, updating it from time to time is safe.

// 			timespec ts{};
// 			ts.tv_sec = epoch;
// 			ts.tv_nsec = usecs * 1000;

// 			setClock(ts);

// 			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
// 			_gps_position->time_utc_usec += usecs;

// 		} else {
// 			_gps_position->time_utc_usec = 0;
// 		}

// #else
// 		_gps_position->time_utc_usec = 0;
// #endif

// 		_last_timestamp_time = gps_absolute_time();
// 	}
	//else 
	if ((memcmp(_rx_buffer, "$GPNAV,", 6) == 0) && (uiCalcComma == 36) ) {
		//PX4_ERR("GPNAV");
		int sinan_date = 0.0;
		double sinan_utc = 0.0;
		int gps_leap_second __attribute__((unused)) = 0,
			bds_leap_second __attribute__((unused)) = 0,
			reserved1 __attribute__((unused)) = 0;

		double lat = 0.0, lon = 0.0, alt = 0.0;
		double separation __attribute__((unused)) = 0,
		trackingAngle __attribute__((unused)) = 0,
		pitch __attribute__((unused)) = 0,
		roll __attribute__((unused)) = 0,
		ve = 0.0, vn = 0.0, vu = 0.0,vg = 0.0;
		int status1 = 0,
		systemMask __attribute__((unused)) = 0;
		double baseline __attribute__((unused)) = 0;
		char svUsed[5] = {0};
		char svTracked[5] __attribute__((unused)) = {0};
		float 	heading = 0;
	 	char status2[2] = {0};

		if (bufptr && *(++bufptr) != ',') { sinan_date = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { sinan_utc = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { gps_leap_second = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { bds_leap_second = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { reserved1 = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }
		
		if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { separation = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { trackingAngle = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { heading = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { pitch = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { roll = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ve = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vn = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vu = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vg = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { status1 = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { status2[0] = *(bufptr++); }
		if (bufptr && *(++bufptr) != ',') { status2[1] = *(bufptr++); }		

		if (bufptr && *(++bufptr) != ',') { baseline = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { svUsed[0] = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { svUsed[1]  = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { svUsed[2]  = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { svUsed[3]  = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { svUsed[4]  = strtod(bufptr, &endp); bufptr = endp; }

//time set
		
		int sinan_year = static_cast<int>(sinan_date / 10000);
		int sinan_mon = static_cast<int>((sinan_date %10000) / 100);
		int sinan_day = static_cast<int>((sinan_date %100));


		
		int sinan_hour = static_cast<int>(sinan_utc / 10000);
		int sinan_minute = static_cast<int>((sinan_utc - sinan_hour * 10000) / 100);
		double sinan_sec = static_cast<double>(sinan_utc - sinan_hour * 10000 - sinan_minute * 100);

		/*
		 * convert to unix timestamp
		 */
		struct tm timeinfo = {};
		timeinfo.tm_year = sinan_year - 1900;
		timeinfo.tm_mon = sinan_mon - 1;
		timeinfo.tm_mday = sinan_day;
		timeinfo.tm_hour = sinan_hour;
		timeinfo.tm_min = sinan_minute;
		timeinfo.tm_sec = int(sinan_sec);
		timeinfo.tm_isdst = 0;

#ifndef NO_MKTIME
		time_t epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((sinan_sec - static_cast<uint64_t>(sinan_sec))) * 1000000;

			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts{};
			ts.tv_sec = epoch;
			ts.tv_nsec = usecs * 1000;

			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += usecs;

		} else {
			_gps_position->time_utc_usec = 0;
		}

#else
		_gps_position->time_utc_usec = 0;
#endif

		_last_timestamp_time = gps_absolute_time();
		_gps_position->timestamp = gps_absolute_time();
		if((status2[0] == 'N')&&(status2[1] == 'V'))
		{
			heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
			heading -= _heading_offset; // range: [-pi, 3pi]

			if (heading > M_PI_F) {
				heading -= 2.f * M_PI_F; // final range is [-pi, pi]
			}

			_gps_position->heading = heading;
		}
		else 
		{
			_gps_position->heading = NAN;
		}
		

//lat lon alt
		_gps_position->lat = static_cast<int>((lat) * 10000000);
		_gps_position->lon = static_cast<int>((lon) * 10000000);
		_gps_position->alt = static_cast<int>(alt * 1000);

		_gps_position->vel_m_s = vg;				/** GPS ground speed (m/s) */
		_gps_position->vel_n_m_s = vn;			/** GPS ground speed in m/s */
		_gps_position->vel_e_m_s = ve;			/** GPS ground speed in m/s */
		_gps_position->vel_d_m_s = static_cast<float>(-vu);				/** GPS ground speed in m/s */

//fix type
		if (status1 <= 0) {
			_gps_position->fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (status1 == 5)
			{
				status1 = 3; 
			}
			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position->fix_type = 3 + status1 - 1;
			_gps_position->vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
		}

		_gps_position->satellites_used = svUsed[0] + svUsed[1]+svUsed[2]+svUsed[3]+svUsed[4];

		_gps_position->cog_rad =
			0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		
		_gps_position->c_variance_rad = 0.1f;
		ret = 1;

	}
	// else if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (uiCalcComma == 14) && !_got_pashr_pos_message) {
	// 	PX4_ERR("gga");
	// 	/*
	// 	  Time, position, and fix related data
	// 	  An example of the GBS message string is:

	// 	  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F

	// 	  Note - The data string exceeds the ASHTECH standard length.
	// 	  GGA message fields
	// 	  Field   Meaning
	// 	  0   Message ID $GPGGA
	// 	  1   UTC of position fix
	// 	  2   Latitude
	// 	  3   Direction of latitude:
	// 	  N: North
	// 	  S: South
	// 	  4   Longitude
	// 	  5   Direction of longitude:
	// 	  E: East
	// 	  W: West
	// 	  6   GPS Quality indicator:
	// 	  0: Fix not valid
	// 	  1: GPS fix
	// 	  2: Differential GPS fix, OmniSTAR VBS
	// 	  4: Real-Time Kinematic, fixed integers
	// 	  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
	// 	  7   Number of SVs in use, range from 00 through to 24+
	// 	  8   HDOP
	// 	  9   Orthometric height (MSL reference)
	// 	  10  M: unit of measure for orthometric height is meters
	// 	  11  Geoid separation
	// 	  12  M: geoid separation measured in meters
	// 	  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
	// 	  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
	// 	  15
	// 	  The checksum data, always begins with *
	// 	  Note - If a user-defined geoid model, or an inclined
	// 	*/
	// 	double ashtech_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
	// 	int num_of_sv __attribute__((unused)) = 0, fix_quality = 0;
	// 	double hdop __attribute__((unused)) = 99.9;
	// 	char ns = '?', ew = '?';

	// 	if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

	// 	if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

	// 	if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

	// 	if (ns == 'S') {
	// 		lat = -lat;
	// 	}

	// 	if (ew == 'W') {
	// 		lon = -lon;
	// 	}

	// 	/* convert from degrees, minutes and seconds to degrees * 1e7 */
	// 	_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
	// 	_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
	// 	_gps_position->alt = static_cast<int>(alt * 1000);
	// 	_gps_position->satellites_used = num_of_sv;
	// 	_gps_position->hdop = (float)hdop ;
	// 	_rate_count_lat_lon++;
		
	// 	if (fix_quality <= 0) {
	// 		_gps_position->fix_type = 0;

	// 	} else {
	// 		/*
	// 		 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
	// 		 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
	// 		 */
	// 		if (fix_quality == 5) { fix_quality = 3; }

	// 		/*
	// 		 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
	// 		 */
	// 		_gps_position->fix_type = 3 + fix_quality - 1;
	// 	}

	// 	_gps_position->timestamp = gps_absolute_time();

	// 	_gps_position->vel_m_s = 0;                                  /**< GPS ground speed (m/s) */
	// 	_gps_position->vel_n_m_s = 0;                                /**< GPS ground speed in m/s */
	// 	_gps_position->vel_e_m_s = 0;                                /**< GPS ground speed in m/s */
	// 	_gps_position->vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
	// 	_gps_position->cog_rad =
	// 		0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
	// 	_gps_position->vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
	// 	_gps_position->c_variance_rad = 0.1f;
	// 	ret = 1;
	
	// }
	 else if ((memcmp(_rx_buffer + 3, "GST,", 3) == 0) && (uiCalcComma == 8)) {
		//PX4_ERR("gst");
		/*
		  Position error statistics
		  An example of the GST message string is:

		  $GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031*6A

		  The Talker ID ($--) will vary depending on the satellite system used for the position solution:

		  $GP - GPS only
		  $GL - GLONASS only
		  $GN - Combined
		  GST message fields
		  Field   Meaning
		  0   Message ID $GPGST
		  1   UTC of position fix
		  2   RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
		  3   Error ellipse semi-major axis 1 sigma error, in meters
		  4   Error ellipse semi-minor axis 1 sigma error, in meters
		  5   Error ellipse orientation, degrees from true north
		  6   Latitude 1 sigma error, in meters
		  7   Longitude 1 sigma error, in meters
		  8   Height 1 sigma error, in meters
		  9   The checksum data, always begins with *
		*/
		double ashtech_time __attribute__((unused)) = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
		double min_err __attribute__((unused)) = 0.0, maj_err __attribute__((unused)) = 0.0,
		deg_from_north __attribute__((unused)) = 0.0, rms_err __attribute__((unused)) = 0.0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { rms_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { maj_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { min_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { deg_from_north = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt_err = strtod(bufptr, &endp); bufptr = endp; }

		_gps_position->eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
					   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
		_gps_position->epv = static_cast<float>(alt_err);

		_gps_position->s_variance_m_s = 0;

	} else if ((memcmp(_rx_buffer + 3, "HDT,", 2) == 0)) {
		PX4_ERR("HDT");
		/*
		Heading message
		Example $GPHDT,121.2,T*35

		f1 Last computed heading value, in degrees (0-359.99)
		T “T” for “True”
		 */

		float heading = 0.f;

		if (bufptr && *(++bufptr) != ',') {
			heading = strtof(bufptr, &endp); bufptr = endp;
			
			heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
			// heading -= _heading_offset; // range: [-pi, 3pi]

			if (heading > M_PI_F) {
				heading -= 2.f * M_PI_F; // final range is [-pi, pi]
			}

			//_gps_position->heading = heading;
		}

	}

	//}else if ((memcmp(_rx_buffer + 3, "GSV,", 3) == 0)) {
	// 	PX4_ERR("gsv");
	// 	/*
	// 	  The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

	// 	  $GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

	// 	  GSV message fields
	// 	  Field   Meaning
	// 	  0   Message ID $GPGSV
	// 	  1   Total number of messages of this type in this cycle
	// 	  2   Message number
	// 	  3   Total number of SVs visible
	// 	  4   SV PRN number
	// 	  5   Elevation, in degrees, 90 maximum
	// 	  6   Azimuth, degrees from True North, 000 through 359
	// 	  7   SNR, 00 through 99 dB (null when not tracking)
	// 	  8-11    Information about second SV, same format as fields 4 through 7
	// 	  12-15   Information about third SV, same format as fields 4 through 7
	// 	  16-19   Information about fourth SV, same format as fields 4 through 7
	// 	  20  The checksum data, always begins with *
	// 	*/
	// 	/*
	// 	 * currently process only gps, because do not know what
	// 	 * Global satellite ID I should use for non GPS sats
	// 	 */
	// 	bool bGPS = false;

	// 	if (memcmp(_rx_buffer, "$GP", 3) != 0) {
	// 		return 0;

	// 	} else {
	// 		bGPS = true;
	// 	}

	// 	int all_msg_num = 0, this_msg_num = 0, tot_sv_visible = 0;
	// 	struct gsv_sat {
	// 		int svid;
	// 		int elevation;
	// 		int azimuth;
	// 		int snr;
	// 	} sat[4];
	// 	memset(sat, 0, sizeof(sat));

	// 	if (bufptr && *(++bufptr) != ',') { all_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { this_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 	if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 	if ((this_msg_num < 1) || (this_msg_num > all_msg_num)) {
	// 		return 0;
	// 	}

	// 	if (this_msg_num == 0 && bGPS && _satellite_info) {
	// 		memset(_satellite_info->svid,     0, sizeof(_satellite_info->svid));
	// 		memset(_satellite_info->used,     0, sizeof(_satellite_info->used));
	// 		memset(_satellite_info->snr,      0, sizeof(_satellite_info->snr));
	// 		memset(_satellite_info->elevation, 0, sizeof(_satellite_info->elevation));
	// 		memset(_satellite_info->azimuth,  0, sizeof(_satellite_info->azimuth));
	// 	}

	// 	int end = 4;

	// 	if (this_msg_num == all_msg_num) {
	// 		end =  tot_sv_visible - (this_msg_num - 1) * 4;
	// 		_gps_position->satellites_used = tot_sv_visible;

	// 		if (_satellite_info) {
	// 			_satellite_info->count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
	// 			_satellite_info->timestamp = gps_absolute_time();
	// 		}
	// 	}

	// 	if (_satellite_info) {
	// 		for (int y = 0 ; y < end ; y++) {
	// 			if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 			if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 			if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 			if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

	// 			_satellite_info->svid[y + (this_msg_num - 1) * 4]      = sat[y].svid;
	// 			_satellite_info->used[y + (this_msg_num - 1) * 4]      = (sat[y].snr > 0);
	// 			_satellite_info->snr[y + (this_msg_num - 1) * 4]       = sat[y].snr;
	// 			_satellite_info->elevation[y + (this_msg_num - 1) * 4] = sat[y].elevation;
	// 			_satellite_info->azimuth[y + (this_msg_num - 1) * 4]   = sat[y].azimuth;
	// 		}
	// 	}
	//}
	else 
	{		
		PX4_ERR("%c %c %c %c %c",_rx_buffer[0],_rx_buffer[1],_rx_buffer[2],_rx_buffer[3],_rx_buffer[4]);
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}


int GPSDriverAshtech::receive(unsigned timeout)
{
	{
		uint8_t receiveFlag = 0;
		uint8_t buf[GPS_READ_BUFFER_SIZE];

		/* timeout additional to poll */
		uint64_t time_started = gps_absolute_time();

		int j = 0;
		ssize_t bytes_count = 0;

		while (true) {
			receiveFlag = 0;
			/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parseChar(buf[j])) > 0) {
					//PX4_ERR("l:%d",l);
					/* return to configure during configuration or to the gps driver during normal work
					 * if a packet has arrived */
					if (handleMessage(l) > 0) {
						receiveFlag = 1;
					}
				}

				j++;
			}
			if(receiveFlag == 1)
			{
				return 1;
			}
			/* everything is read */
			j = bytes_count = 0;

			/* then poll or read for new data */
			int ret = read(buf, sizeof(buf), timeout * 2);
			//PX4_ERR("ret:%d",ret);
			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */

			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				bytes_count = ret;
			}

			/* in case we get crap from GPS or time out */
			if (time_started + timeout * 1000 * 2 < gps_absolute_time()) {
				return -1;
			}
		}
	}

}
#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int GPSDriverAshtech::parseChar(uint8_t b)
{
	int iRet = 0;

	switch (_decode_state) {
	/* First, look for sync1 */
	case NME_DECODE_UNINIT:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NME_DECODE_GOT_SYNC1:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;

		} else if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS;
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;

		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NME_DECODE_GOT_ASTERIKS:
		_rx_buffer[_rx_buffer_bytes++] = b;
		_decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
		break;

	case NME_DECODE_GOT_FIRST_CS_BYTE:
		_rx_buffer[_rx_buffer_bytes++] = b;
		uint8_t checksum = 0;
		uint8_t *buffer = _rx_buffer + 1;
		uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

		for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

		if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
		    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
			iRet = _rx_buffer_bytes;
		}

		_decode_state = NME_DECODE_UNINIT;
		_rx_buffer_bytes = 0;
		break;
	}

	return iRet;
}

void GPSDriverAshtech::decodeInit()
{

}

/*
 * ashtech board configuration script
 */
/*
const char comm[] = "$PASHS,POP,20\r\n"\
		    "$PASHS,NME,ZDA,B,ON,3\r\n"\
		    "$PASHS,NME,GGA,B,OFF\r\n"\
		    "$PASHS,NME,GST,B,ON,3\r\n"\
		    "$PASHS,NME,POS,B,ON,0.05\r\n"\
		    "$PASHS,NME,GSV,B,ON,3\r\n"\
		    "$PASHS,SPD,A,8\r\n"\
		    "$PASHS,SPD,B,9\r\n";
*/
int GPSDriverAshtech::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("ASHTECH: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}


	return setBaudrate(115200);
}
