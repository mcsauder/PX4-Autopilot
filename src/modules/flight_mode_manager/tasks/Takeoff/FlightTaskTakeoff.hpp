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

/**
 * @file FlightTaskTakeoff.hpp
 */

#pragma once

#include <cstdio>

#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

#include "FlightTask.hpp"

class FlightTaskTakeoff : public FlightTask
{
public:
	FlightTaskTakeoff() = default;
	virtual ~FlightTaskTakeoff() = default;

	bool update() override;

	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;

	void reActivate() override;

	bool send_vehicle_command(const uint32_t cmd,
				  const float param1 = NAN,
				  const float param2 = NAN,
				  const float param3 = NAN,
				  const float param4 = NAN,
				  const double param5 = static_cast<double>(NAN),
				  const double param6 = static_cast<double>(NAN),
				  const float param7 = NAN);

private:

	void arm_vehicle();

	void subscribe_to_topics();

	void takeoff();

	hrt_abstime _manuever_start_time {0};
	hrt_abstime _manuever_time_us {100000};

	float _takeoff_time {0.f};

	home_position_s                   _home_pos {};                        /**< home position for RTL */
	takeoff_status_s                  _takeoff_status {};                  /**< takeoff status */
	vehicle_constraints_s             _vehicle_constraints {};
	vehicle_control_mode_s            _vehicle_control_mode {};
	vehicle_global_position_s         _vehicle_global_pos {};              /**< global vehicle position */
	vehicle_gps_position_s            _vehicle_gps_pos {};                 /**< gps position */
	vehicle_land_detected_s           _vehicle_land_detected {};           /**< vehicle land_detected */
	vehicle_local_position_s          _vehicle_local_position {};          /**< local vehicle position */
	vehicle_local_position_setpoint_s _vehicle_local_position_setpoint {}; /**< local vehicle position setpoint */
	vehicle_status_s                  _vehicle_status {};                  /**< vehicle status */

	uORB::PublicationData<takeoff_status_s>       _takeoff_status_pub{ORB_ID(takeoff_status)};
	uORB::PublicationData<vehicle_constraints_s>  _vehicle_constraints_pub{ORB_ID(vehicle_constraints)};
	uORB::PublicationData<vehicle_control_mode_s> _vehicle_control_mode_pub{ORB_ID(vehicle_control_mode)};

	// Subscriptions
	uORB::Subscription         _home_position_sub {ORB_ID(home_position)};
	uORB::Subscription         _takeoff_status_sub {ORB_ID(takeoff_status)};
	uORB::Subscription         _vehicle_constraints_sub {ORB_ID(vehicle_constraints)};
	uORB::Subscription         _vehicle_control_mode_sub {ORB_ID(vehicle_control_mode)};
	uORB::Subscription         _vehicle_land_detected_sub {ORB_ID(vehicle_land_detected)};
	uORB::Subscription         _vehicle_local_position_sub {ORB_ID(vehicle_local_position)};
	uORB::Subscription         _vehicle_local_position_setpoint_sub {ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription         _vehicle_status_sub {ORB_ID(vehicle_status)};


	DEFINE_PARAMETERS_CUSTOM_PARENT(
		FlightTask,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max,   ///< horizontal acceleration
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max      ///< horizontal velocity
	)
};
