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

#include "sensor_bridge.hpp"

#include <stdint.h>

#include <uORB/topics/mmc_obstacle.h>
#include <uORB/topics/obstacle_trigger.h>
#include <uORB/topics/obstacle_distance.h>
#include <mmc/Lidar.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

class UavcanRadarBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanRadarBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	void Lidar_sub_cb(const uavcan::ReceivedDataStructure<mmc::Lidar> &msg);

	typedef uavcan::MethodBinder<UavcanRadarBridge *,
		void (UavcanRadarBridge::*)(const uavcan::ReceivedDataStructure<mmc::Lidar> &)>
		LidarCbBinder;

	uavcan::Subscriber<mmc::Lidar, LidarCbBinder> _sub_lidar;

	uORB::Publication<mmc_obstacle_s> _mmc_obstacle_topic{ORB_ID(mmc_obstacle)};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{
		ORB_ID(obstacle_distance_fused)}; /**< obstacle_distance publication */

	uORB::Subscription _sub_obstacle_trigger{ORB_ID(obstacle_trigger)};
};
