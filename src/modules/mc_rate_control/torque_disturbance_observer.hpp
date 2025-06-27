#pragma once


#include <matrix/matrix/math.hpp>
#include <uORB/topics/torque_dhat.h> //custom
using matrix::Vector;

void torque_DOB(float dt, matrix::Vector3f tau_rpy_desired, matrix::Vector3f imu_omega, matrix::Vector3f &tau_rpy_tilde, torque_dhat_s &torque_dhat, bool compensate_flag);
