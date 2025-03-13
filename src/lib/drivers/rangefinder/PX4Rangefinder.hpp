/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/distance_sensor.h>

class PX4Rangefinder
{
	#define SINGLE_BUFFER_SIZE 50  // max 50hz
	typedef enum {
		SONAR_FREQUENCE_10HZ = 10,
		SONAR_FREQUENCE_50HZ = 50,
	} sonar_frequence_e;

	typedef struct {
		uint8_t tail = 0;
		float max_value = -1.0f;
		float min_value = 1000.0f;
		uint8_t max_index = 0;
		uint8_t min_index = 0;
		float pld_srch_alt = 6.0f;
		sonar_frequence_e sonar_frequence_ = SONAR_FREQUENCE_10HZ;
		// bool data_collect_complete = false;
		float dn_speed_max = 2.0f;

		float buffer[SINGLE_BUFFER_SIZE] = {-1.0};

		void push(float value)
		{
			tail = (tail + 1) % sonar_frequence_;
			buffer[tail] = value;

			// // come to 0 again means buffer full
			// if (tail == 0) {
			// 	data_collect_complete = true;
			// }

			update();
		}

		void setDownSpeed(float max_vz)
		{
			dn_speed_max = max_vz + 0.5f;	// Tolerance range
			PX4_INFO("dist sensor max dn speed %.2f, precland alt %.2f", (double)dn_speed_max, (double)pld_srch_alt);
		}

		// LAND MODE reset
		void reset()
		{
			for (uint8_t i = 0; i < SINGLE_BUFFER_SIZE; ++i) {
				buffer[i] = -1.0;
			}

			max_value = -1.0f;
			min_value = 1000.0f;
			tail = 0;
			min_index = 0; // at first no satisfy
			max_index = SINGLE_BUFFER_SIZE;
			// data_collect_complete = false;
		}

		void update()
		{
			for (uint8_t i = 0; i < sonar_frequence_; ++i) {
				if (buffer[i] > 0.0f && buffer[i] > max_value) {
					max_value = buffer[i];
					max_index = i;
				}

				if (buffer[i] > 0.0f && buffer[i] < min_value) {
					min_value = buffer[i];
					min_index = i;
				}
			}
		}

		bool getValid()
		{
			bool ret = false;

			if (/*data_collect_complete && */max_value < 11.0f && min_value > pld_srch_alt && abs(max_index - min_index) < 3
							 && (fabsf(max_value - min_value) < dn_speed_max) && (fabsf(max_value - min_value) > 0.25f)) {
				ret = true;
			}

			return ret;
		}

		void resetPeak(void)
		{
			max_value = -1.0f;
			min_value = 1000.0f;
		}
		void setSrchAlt(float alt) { pld_srch_alt = alt; }
		void setSensorFreq(uint8_t hz)
		{
			if (hz > 20) {
				sonar_frequence_ = SONAR_FREQUENCE_50HZ;
			}
		}
	} single_buffer_t;

public:
	PX4Rangefinder(const uint32_t device_id,
		       const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	~PX4Rangefinder();

	// Set the MAV_DISTANCE_SENSOR type (LASER, ULTRASOUND, INFRARED, RADAR)
	void set_rangefinder_type(uint8_t rangefinder_type) { _distance_sensor_pub.get().type = rangefinder_type; };

	void set_device_id(const uint32_t device_id) { _distance_sensor_pub.get().device_id = device_id; };
	void set_device_type(const uint8_t device_type);

	void set_fov(const float fov) { set_hfov(fov); set_vfov(fov); }
	void set_hfov(const float fov) { _distance_sensor_pub.get().h_fov = fov; }
	void set_vfov(const float fov) { _distance_sensor_pub.get().v_fov = fov; }

	void set_max_distance(const float distance) { _distance_sensor_pub.get().max_distance = distance; }
	void set_min_distance(const float distance) { _distance_sensor_pub.get().min_distance = distance; }

	void set_orientation(const uint8_t device_orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	void set_mode(const uint8_t mode) { _distance_sensor_pub.get().mode = mode; }

	void update(const hrt_abstime &timestamp_sample, const float distance, const int8_t quality = -1);
	void force_push(const hrt_abstime &timestamp_sample, const float distance);

	int get_instance() { return _distance_sensor_pub.get_instance(); };

	single_buffer_t single_buffer_;
	bool data_valid_ = false;
private:
	uORB::PublicationMultiData<distance_sensor_s> _distance_sensor_pub{ORB_ID(distance_sensor)};
};
