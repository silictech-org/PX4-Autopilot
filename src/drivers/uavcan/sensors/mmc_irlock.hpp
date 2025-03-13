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

#include <uORB/topics/irlock_report.h>
#include <mmc/Irlock.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <systemlib/mavlink_log.h>

#define IRLOCK_RES_X 160	//QQVGA
#define IRLOCK_RES_Y 120	//QQVGA

#define IRLOCK_CENTER_X				(IRLOCK_RES_X/2)			// the x-axis center pixel position
#define IRLOCK_CENTER_Y				(IRLOCK_RES_Y/2)			// the y-axis center pixel position

#define IRLOCK_FOV_X (70.0f*M_PI_F/180.0f)
#define IRLOCK_FOV_Y (50.0f*M_PI_F/180.0f)

#define IRLOCK_TAN_HALF_FOV_X 0.6998f // tan(0.5 * 70 * pi/180)
#define IRLOCK_TAN_HALF_FOV_Y 0.466f // tan(0.5 * 50 * pi/180)

#define IRLOCK_TAN_ANG_PER_PIXEL_X	(2*IRLOCK_TAN_HALF_FOV_X/IRLOCK_RES_X)
#define IRLOCK_TAN_ANG_PER_PIXEL_Y	(2*IRLOCK_TAN_HALF_FOV_Y/IRLOCK_RES_Y)

class UavcanIrlockBridge : public UavcanSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanIrlockBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	void Irlock_sub_cb(const uavcan::ReceivedDataStructure<mmc::Irlock> &msg);

	typedef uavcan::MethodBinder<UavcanIrlockBridge *,
		void (UavcanIrlockBridge::*)(const uavcan::ReceivedDataStructure<mmc::Irlock> &)>
		IrlockCbBinder;

	uavcan::Subscriber<mmc::Irlock, IrlockCbBinder> _sub_Irlock;

	uORB::Publication<irlock_report_s> _uavcan_irlock_topic{ORB_ID(irlock_report)};

	orb_advert_t _mavlink_log_pub{nullptr}; /**< Mavlink log uORB handle */


};
