#pragma once

#include <uORB/topics/center_of_mass.h>
#include <matrix/matrix/math.hpp>

using matrix::Vector;

//void dob_based_com_estimator(matrix::Vector3f torque_dhat, matrix::Vector3f body_force_desired, matrix::Vector3f &present_com_hat, matrix::Vector3f &past_com_hat);

void dob_based_com_estimator(float dt, matrix::Vector3f torque_dhat, matrix::Vector3f body_force_desired, center_of_mass_s &com_log, matrix::Vector3f &past_com_hat,matrix::Vector3f &com_hat_tilde);


