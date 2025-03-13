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

#include "sensor_bridge.hpp"

#include "mmc_irlock.hpp"

#include <drivers/drv_hrt.h>

const char *const UavcanIrlockBridge::NAME = "irlock";

UavcanIrlockBridge::UavcanIrlockBridge(uavcan::INode &node)
	: UavcanSensorBridgeBase("uavcan_irlock", ORB_ID(irlock_report)), _sub_Irlock(node)
{
}

int UavcanIrlockBridge::init()
{
	int res = _sub_Irlock.start(IrlockCbBinder(this, &UavcanIrlockBridge::Irlock_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

void UavcanIrlockBridge::Irlock_sub_cb(const uavcan::ReceivedDataStructure<mmc::Irlock> &msg)
{
	irlock_report_s irlock_report_buf{0};

	irlock_report_buf.timestamp = hrt_absolute_time();
	irlock_report_buf.signature = msg.signature;	//apriltag_id
	irlock_report_buf.pos_x = (msg.pixel_x - IRLOCK_CENTER_X) * IRLOCK_TAN_ANG_PER_PIXEL_X;
	irlock_report_buf.pos_y = (msg.pixel_y - IRLOCK_CENTER_Y) * IRLOCK_TAN_ANG_PER_PIXEL_Y;

	uint16_t remainder_yaw = msg.pixel_size_x % 90;
	irlock_report_buf.size_x = (remainder_yaw > 45 ? 90 - remainder_yaw : - remainder_yaw) * 1.0f ;

	irlock_report_buf.size_y = msg.pixel_size_y * 1.0f;	// dist_z

	static uint64_t time_send = 0;

	if ((hrt_absolute_time() - time_send > 1000000) && (irlock_report_buf.signature != 0)) {
		time_send = hrt_absolute_time();
		mavlink_log_info(&_mavlink_log_pub, "dx = %d, dy = %d, dyaw = %.1f", (msg.pixel_x - IRLOCK_CENTER_X),
				 (msg.pixel_y - IRLOCK_CENTER_Y), (double)irlock_report_buf.size_x);
	}

	_uavcan_irlock_topic.publish(irlock_report_buf);
}
