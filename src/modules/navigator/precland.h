/***************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
/**
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/distance_sensor_raw.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/precland.h>
#include <uORB/topics/vehicle_command.h>
#include <commander/px4_custom_mode.h>

#include "navigator_mode.h"
#include "mission_block.h"

#include <systemlib/mavlink_log.h>

enum class PrecLandState {
	Start, // Starting state
	HorizontalApproach, // Positioning over landing target while maintaining altitude
	DescendAboveTarget, // Stay over landing target while descending
	FinalApproach, // Final landing approach, even without landing target
	Search, // Search for landing target
	Fallback, // Fallback landing method
	Done // Done landing
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning
	Required = 2 // try to find landing target if not visible at the beginning
};

class PrecLand : public MissionBlock, public ModuleParams
{
public:
	PrecLand(Navigator *navigator);
	~PrecLand() override = default;

	void on_activation() override;
	void on_active() override;
	void on_inactivation() override;

	void set_mode(PrecLandMode mode) { _mode = mode; };

	PrecLandMode get_mode() { return _mode; };

	bool is_activated() { return _is_activated; };

private:

	void updateParams() override;
	void update_control_params(bool recover);
	void record_orin_value();
	void record_orin_dn_speed();
	void limit_dn_speed_params(bool recover);
	// run the control loop for each state
	void run_state_start();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_final_approach();
	void run_state_search();
	void run_state_fallback();

	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_start();
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_descend_above_target();
	bool switch_to_state_final_approach();
	bool switch_to_state_search();
	bool switch_to_state_fallback();
	bool switch_to_state_done();

	void print_state_switch_message(const char *state_name);

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void slewrate(float &sp_x, float &sp_y);

	landing_target_pose_s _target_pose{}; /**< precision landing target position */
	const static int _kParamNum = 2;
	const char *_param_names[_kParamNum] = {"MPC_JERK_AUTO", "MPC_ACC_HOR"};
	float _orin_value[_kParamNum];

	const char *_dn_param_name = "MPC_Z_V_AUTO_DN";
	float _orin_dn_speed{0.0f};

	uORB::Subscription _target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _distance_sensor_raw_sub{ORB_ID(distance_sensor_raw)};

	bool _target_pose_valid{false}; /**< whether we have received a landing target position message */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated */
	bool _dist_sensor_updated{false};
	bool _limit_dn_speed_enable{false};
	bool _sensor_valid_loiter{false};

	MapProjection _map_ref{}; /**< class for local/global projections */

	uint64_t _state_start_time{0}; /**< time when we entered current state */
	uint64_t _last_slewrate_time{0}; /**< time when we last limited setpoint changes */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint */
	uint64_t _sensor_valid_time{0};

	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target */
	float _approach_alt{0.0f}; /**< altitude at which to stay during horizontal approach */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	PrecLandState _state{PrecLandState::Start};

	PrecLandMode _mode{PrecLandMode::Opportunistic};

	bool _is_activated {false}; /**< indicates if precland is activated */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PLD_BTOUT>) _param_pld_btout,
		(ParamFloat<px4::params::PLD_HACC_RAD>) _param_pld_hacc_rad,
		(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_pld_fappr_alt,
		(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_pld_srch_alt,
		(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_pld_srch_tout,
		(ParamInt<px4::params::PLD_MAX_SRCH>) _param_pld_max_srch,
		(ParamFloat<px4::params::PLD_JERK_AUTO>) _param_pld_jerk_auto,
		(ParamFloat<px4::params::PLD_ACC_HOR>) _param_pld_acc_hor,
		(ParamInt<px4::params::PLD_FALL_MODE>) _param_pld_fall_mode,
		(ParamFloat<px4::params::PLD_AUTO_DN>)  _param_pld_auto_dn
	)

	// non-navigator parameters
	param_t	_handle_param_acceleration_hor{PARAM_INVALID};
	param_t	_handle_param_xy_vel_cruise{PARAM_INVALID};
	float	_param_acceleration_hor{0.0f};
	float	_param_xy_vel_cruise{0.0f};

	orb_advert_t _mavlink_log_pub{nullptr}; /**< Mavlink log uORB handle */

	precland_s _precland_buf{0};
	distance_sensor_s _dist_sensor{0};
	distance_sensor_raw_s _dist_sensor_raw{0};
	uORB::Publication<precland_s> _precland_topic{ORB_ID(precland)};
	uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};	/**< vehicle command do publication */

	void _publishVehicleCmdDoLoiter();
	void _returnSafePoint();
};
