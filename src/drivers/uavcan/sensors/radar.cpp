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

#include "radar.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanRadarBridge::NAME = "radar";

UavcanRadarBridge::UavcanRadarBridge(uavcan::INode &node)
	: UavcanSensorBridgeBase("uavcan_radar", ORB_ID(mmc_obstacle)), _sub_lidar(node)
{
}

int UavcanRadarBridge::init()
{
	int res = _sub_lidar.start(LidarCbBinder(this, &UavcanRadarBridge::Lidar_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanRadarBridge::Lidar_sub_cb(const uavcan::ReceivedDataStructure<mmc::Lidar> &msg)
{

	mmc_obstacle_s obstacle{0};
	obstacle_distance_s obstacle_fused{0};

	const uint16_t body_dist = 134;  // body_dist max  cm

	obstacle.timestamp = hrt_absolute_time();
	obstacle.fov = 360;
	obstacle.max_distance = 3000U;
	obstacle.min_distance = msg.min_distance;
	// obstacle.sectors = msg.distance.size();
	obstacle.sectors = mmc_obstacle_s::NUM_SECTORS_S20;

	for (unsigned i = 0; i < obstacle.sectors; ++i) {
		if (msg.distance[i] <= body_dist) {
			// obstacle.distance[i] = msg.max_distance;
			obstacle.distance[i] = 0x0000;

		} else {
			obstacle.distance[i] = msg.distance[i];
		}
	}

	obstacle.sensor_type = mmc_obstacle_s::LIDAR_FSS20;

	obstacle_fused.timestamp = hrt_absolute_time();
	obstacle_fused.increment = 10.0f;
	obstacle_fused.angle_offset = 0.0f;
	memcpy(obstacle_fused.distances, obstacle.distance, sizeof(obstacle.distance));
	// memset(&obstacle_fused.distances[36], 0, 36);
	obstacle_fused.max_distance = 3000U;
	obstacle_fused.min_distance = msg.min_distance;
	obstacle_fused.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;  // body

	// obstacle_fused.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_RADAR;
	if (_sub_obstacle_trigger.updated()) {
		obstacle_trigger_s ob_trig;
		_sub_obstacle_trigger.copy(&ob_trig);

		if (obstacle_fused.timestamp - ob_trig.timestamp < 200 * 1000) {
			if (ob_trig.trig_flg == 1) {
				obstacle_fused.sensor_type = 1;

			} else {
				obstacle_fused.sensor_type = 0;
			}
		}
	}

	_obstacle_distance_pub.publish(obstacle_fused);  // for used
	_mmc_obstacle_topic.publish(obstacle);
}
