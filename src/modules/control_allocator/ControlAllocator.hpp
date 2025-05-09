/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.hpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once


#include <ControlAllocationUtils.hpp> // custom

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>

#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failure_detector_status.h>

#include <uORB/topics/manual_control_setpoint.h> // custom

//ㅡㅡㅡㅡㅡㅡㅡㅡㅡ25.03.18.Song Yeong Inㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ//
#include <cmath>
#include <uORB/topics/servo_angle.h>
#include <uORB/topics/thrust_command.h>
#include <uORB/topics/wrench_command.h>

class ControlAllocator : public ModuleBase<ControlAllocator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:



	ControlAllocator();

	virtual ~ControlAllocator();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

	

private:


	/**
	 * initialize some vectors/matrices from parameters
	 */
		
	void parameters_updated();

	void update_effectiveness_matrix_if_needed();

	
	void publish_control_allocator_status(int matrix_index);

	void publish_actuator_controls();

	hrt_abstime _last_effectiveness_update{0};
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// Inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< vehicle torque setpoint subscription */
	uORB::SubscriptionCallbackWorkItem _vehicle_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint)};	 /**< vehicle thrust setpoint subscription */

	uORB::Subscription _vehicle_torque_setpoint1_sub{ORB_ID(vehicle_torque_setpoint), 1};  /**< vehicle torque setpoint subscription (2. instance) */
	uORB::Subscription _vehicle_thrust_setpoint1_sub{ORB_ID(vehicle_thrust_setpoint), 1};	 /**< vehicle thrust setpoint subscription (2. instance) */

	// Outputs
	uORB::PublicationMulti<control_allocator_status_s> _control_allocator_status_pub[2] {ORB_ID(control_allocator_status), ORB_ID(control_allocator_status)};

	uORB::Publication<actuator_motors_s>	_actuator_motors_pub{ORB_ID(actuator_motors)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1}; //ms

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _failure_detector_status_sub{ORB_ID(failure_detector_status)};

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ custom subscriber | publisher ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	uORB::SubscriptionCallbackWorkItem _servo_angle_sub{this, ORB_ID(servo_angle)};
	uORB::Publication<thrust_command_s> _thrust_command_pub{ORB_ID(thrust_command)};
	uORB::Publication<wrench_command_s> _wrench_command_pub{ORB_ID(wrench_command)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};



	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	matrix::Matrix<float, 4,4> _custom_effectiveness;
	matrix::Vector3f _torque_sp;
	matrix::Vector3f _thrust_sp;
	matrix::Vector4f _servo_ang{0.0,0.0,0.0,0.0}; // custom
	bool _publish_controls{true};

	// Reflects motor failures that are currently handled, not motor failures that are reported.
	// For example, the system might report two motor failures, but only the first one is handled by CA
	uint16_t _handled_motor_failure_bitmask{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	bool _armed{false};
	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};
	hrt_abstime _last_status_pub{0};

	bool _has_slew_rate{false};
	float dumi = 0.f;

	matrix::Matrix<float, 4, 4> _mix;
	matrix::Vector<float, 4> _actuator_min; 	///< Minimum actuator values
	matrix::Vector<float, 4> _actuator_max; 	///< Maximum actuator values
	matrix::Vector<float, 4> _actuator_sp;  	///< Actuator setpoint
	matrix::Vector<float, 4> _control_sp;   	///< Control setpoint

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_AIRFRAME>) _param_ca_airframe,
		(ParamInt<px4::params::CA_METHOD>) _param_ca_method,
		(ParamInt<px4::params::CA_FAILURE_MODE>) _param_ca_failure_mode,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)

};
