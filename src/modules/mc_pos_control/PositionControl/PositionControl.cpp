/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

//lib/geo/geo.h float CONSTANTS_ONE_G = 9.80665f;

using namespace matrix;

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setAccelerationLimits(const float acc_horizontal, const float acc_vertical)
{

	_lim_acc_horizontal = acc_horizontal;
	_lim_acc_vertical = acc_vertical;

}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 1.0f);//math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}


void PositionControl::updateAttitude(matrix::Dcmf R){

	_w2b = R.transpose();
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	float yaw_deg = 90.f;
	float yaw_rad = yaw_deg*3.14f / 180.0f;
    float c = cosf(yaw_rad);
    float s = sinf(yaw_rad);

    matrix::Dcmf R_yaw;

    R_yaw(0, 0) = c;
    R_yaw(0, 1) = -s;
    R_yaw(0, 2) = 0.0f;

    R_yaw(1, 0) = s;
    R_yaw(1, 1) = c;
    R_yaw(1, 2) = 0.0f;

    R_yaw(2, 0) = 0.0f;
    R_yaw(2, 1) = 0.0f;
    R_yaw(2, 2) = 1.0f;

	_pos = states.position; // world
	_vel = states.velocity; // world
	_yaw = states.yaw; // RPY
	_vel_dot = states.acceleration; // body
}

void PositionControl::setInputSetpoint(const matrix::Vector4f &pose_setpoint)
{
	_pos_sp(0) = pose_setpoint(0);
	_pos_sp(1) = pose_setpoint(1);
	_pos_sp(2) = pose_setpoint(2);
		
	_yaw_sp = pose_setpoint(3);
	_yawspeed_sp = 0.f;
}

bool PositionControl::update(const float dt, bool arm_flag, bool pos_flag)
{
	bool valid = _inputValid();

	if (valid && arm_flag) {
		
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	}

	if(!arm_flag){

		// FOR SAFETY FLIGHT
		
		_vel_sp = {0.f, 0.f, 0.f};
		_acc_sp = {0.f, 0.f, 0.f};
		_thr_sp = {0.f, 0.f, 0.f};

	}

	if(!pos_flag){
		_vel_sp(0) = 0.f;
		_vel_sp(1) = 0.f;
		
		_acc_sp(0) = 0.f;
		_acc_sp(1) = 0.f;

		_thr_sp(0) = 0.f;
		_thr_sp(1) = 0.f;
	}



	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P-position controller
	// Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p) - _vel.emult(_gain_pos_d);

	Vector3f vel_sp_position;

	vel_sp_position(0) = (_pos_sp(0) - _pos(0))*static_cast<float>(1.0) - _vel(0)*static_cast<float>(0.0);  //SEUK 여기는 하드코딩 되어있음.
	vel_sp_position(1) = (_pos_sp(1) - _pos(1))*static_cast<float>(1.0) - _vel(1)*static_cast<float>(0.0);
	vel_sp_position(2) = (_pos_sp(2) - _pos(2))*static_cast<float>(0.5) - _vel(2)*static_cast<float>(0.1);
	// pz : 1,   dz : 0.05
	// pz : 0.5, dz : 0.025
	// pz : 0.5, dz : 0.25
	// pz : 0.5, dz : 0.1

	// 0,0,0 한번 체크해보기, 그리고 1, 0,0 체크해보기 --> 이전단계 제어기에서 빨간색이 뒤에 까만색과 일치하는지 (1,0,0 일때) : clear

	// 이 코드 그대로 두고 velovity z contraints를 더 빡세게 해서 다시 한번만 비행해보기!! --> 원래 게인으로!!

	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	//ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position); //custom
	// make sure there are no NAN elements for further reference while constraining
	//ControlMath::setZeroIfNanVector3f(vel_sp_position);
	
	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp(0) = math::constrain(vel_sp_position(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(vel_sp_position(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	//ControlMath::constrainXY_custom(vel_sp_position,_lim_vel_horizontal);
	//ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(vel_sp_position(2), static_cast<float>(-0.5), static_cast<float>(0.5));//_lim_vel_down -0.5

	// 여기까진 normalization 파트가 없음
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void PositionControl::_velocityControl(const float dt) // 사실상 여기가 문제임
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);


	// PID velocity control
	// Vector3f gravity = {0.0, 0.0, -9.81};
	Vector3f vel_error =  _w2b*(_vel_sp - _vel); //_w2b
	
	//_pos_sp = vel_error;
	
	// Vector3f zero3{0, 0, 0};
	// Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
	// Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) - _vel_dot.emult(_gain_vel_d);
	Vector3f acc_sp_velocity;
	
	acc_sp_velocity(0) = vel_error(0)*(static_cast<float>(1.0001)) - _vel_dot(0)*(static_cast<float>(0.0));
	acc_sp_velocity(1) = vel_error(1)*(static_cast<float>(1.0001)) - _vel_dot(1)*(static_cast<float>(0.0));
	acc_sp_velocity(2) = vel_error(2)*(static_cast<float>(0.1)) - _vel_dot(2)*(static_cast<float>(0.02));
	
	// acl xy 
	// Pxy = 2.0 --> fast performance

	// acl_z 
	// Pz = 0.2, Dz = 0.01 --> good performance
	// Pz = 0.4, Dz = 0.02 

	// No control input from setpoints or corresponding states which are NAN	
	// acc_sp_velocity가 NAN값이 아니면 _acc_sp에 값을 더해주는 함수
	
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_acc_sp(0) = acc_sp_velocity(0);
	_acc_sp(1) = acc_sp_velocity(1);
	//_acc_sp(2) = acc_sp_velocity(2) + _vel_int(2);

	_acc_sp(0) = math::constrain(_acc_sp(0), -_lim_acc_horizontal, _lim_acc_horizontal);
	_acc_sp(1) = math::constrain(_acc_sp(1), -_lim_acc_horizontal, _lim_acc_horizontal);
	_acc_sp(2) = math::constrain(_acc_sp(2), -_lim_acc_vertical, 0.f);

	float mass = 8.f;
	
	_thr_sp(0) =  mass*_acc_sp(0);
	_thr_sp(1) =  mass*_acc_sp(1);
	_thr_sp(2) =  _acc_sp(2);
	//if(_acc_sp(2) <= 0.f){ _thr_sp(2) = 1.0;} //custom

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// 여기서부터 thrust limitation 걸려있음 잘 보고 수정해야함
	//_lim_thr_min/max는 parameter 설정으로 update되는 애들 지금은 N단위로 정의해 놨음(1~15(N))
	// Integrator anti-windup in vertical direction

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	// _vel_int += vel_error.emult(_gain_vel_i) * dt;
	_vel_int(2) += vel_error(2)*_gain_vel_i(2) * dt;

	
	if ((_thr_sp(2) <= _lim_thr_min && vel_error(2) <= 0.f) ||
	    (_thr_sp(2) >= _lim_thr_max && vel_error(2) >= 0.f)) {
		vel_error(2) = 0.f;
		}
	
	//_thr_sp(2) = -_thr_sp(2); // custom : for below contraints logic

	// Prioritize vertical control while keeping a horizontal margin
	/**/
	/*
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm(); // XY방향 thrust 총합 크기 계산 normalization 아님
	const float thrust_max_squared = math::sq(_lim_thr_max); // 왜 제곱하지?

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	
	const float thrust_z_max_squared = -(thrust_max_squared - math::sq(allocated_horizontal_thrust)); //custom
	// 수평방향분에 할당된 thrust크기를 총 발생시킬수 있는 thrust 총합 값에서 뺴면 z 방향 thrust 여유 크기가 나옴

	// Saturate maximal vertical thrust
	//thrust_z_max_squared = 0.0;
	_thr_sp(2) = -math::max(-_thr_sp(2), sqrtf(thrust_z_max_squared)); // Z방향 여유크기 saturation보다 크면 -> 값제한
	
	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = -(thrust_max_squared - math::sq(_thr_sp(2))); // 수직방향분 여유분 계산한 다음에 병진방향 여유분 계산
	float thrust_max_xy = 0.f;
	//_thr_sp.xy() = thrust_max_xy_squared;
	
	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}s

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = (thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy); // 수평방향분 thrust 값 제한
	}*/

	//vel_error.xy() = -Vector2f(vel_error); //Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);
		
	
	

}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) { // 수직방향 추력과 병진방향 추력이 서로 영향을 준다면 --> true

		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2); // 병진방향으로 이동하면서 발생하는 수직방향의 추력손실을 매꿔주는 텀?
	}
	
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized(); // 좌표계를 비행좌표계로 바꾼거


	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt); 

	// NED = North-East-Down 기준 좌표계 (PX4 기본 좌표계)
	// ned_z = "아래 방향", 즉 중력 방향 (지면 방향)
	// 아무래도 가속도 커맨드가 곧 translational force command라고 하고 servo allocation 해야할듯
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust; //body_z
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint,bool &pos_flag, Vector4f &manual_input) const
{
	//ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	matrix::Vector4f blank_vec{0.f,0.f,0.f,0.f};
	if(pos_flag){
		ControlMath::setAttitude(_yaw_sp, attitude_setpoint, blank_vec);
	}
	else{
		ControlMath::setAttitude(_yaw_sp, attitude_setpoint, manual_input);
	}

	_thr_sp.copyTo(attitude_setpoint.thrust_body); // custom
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
