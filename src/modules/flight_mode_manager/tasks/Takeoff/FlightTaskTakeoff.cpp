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
 * @file FlightTaskTakeoff.cpp
 */

#include "FlightTaskTakeoff.hpp"

bool FlightTaskTakeoff::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint = _position;
	_velocity_setpoint = _velocity;
	_yaw_setpoint      = _yaw;

	PX4_INFO("activate()");

	_takeoff_time = hrt_absolute_time();

	FlightTask::_setDefaultConstraints();

	takeoff();

	return ret;
}

void FlightTaskTakeoff::arm_vehicle()
{
	_vehicle_status_sub.update(&_vehicle_status);

	if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 0.f);
		_vehicle_control_mode.flag_armed = true;
	}
}

void FlightTaskTakeoff::reActivate()
{
	PX4_INFO("Not reactivating.");
	// activate(empty_setpoint);
}

bool FlightTaskTakeoff::update()
{
	bool ret = FlightTask::update();

	arm_vehicle();

	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime time_delta = time_now - _manuever_start_time;

	if (time_delta > 1000000) {

		PX4_INFO("Position setpoint updated.");
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);
		_position_setpoint(2) = _position(2) - 0.5f;
	}

	return ret;
}

void FlightTaskTakeoff::subscribe_to_topics()
{
	_home_position_sub.subscribe();
	_takeoff_status_sub.subscribe();
	_vehicle_constraints_sub.subscribe();
	_vehicle_control_mode_sub.subscribe();
	_vehicle_land_detected_sub.subscribe();
	_vehicle_local_position_sub.subscribe();
	_vehicle_status_sub.subscribe();
}

void FlightTaskTakeoff::takeoff()
{
	PX4_INFO("takeoff()");
	_constraints.want_takeoff = true;
}

bool FlightTaskTakeoff::send_vehicle_command(const uint32_t cmd,
		const float param1,
		const float param2,
		const float param3,
		const float param4,
		const double param5,
		const double param6,
		const float param7)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}
