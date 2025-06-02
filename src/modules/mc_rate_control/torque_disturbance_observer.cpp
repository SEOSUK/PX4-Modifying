
#include "torque_disturbance_observer.hpp"
#include <matrix/matrix/math.hpp>

using matrix::Vector;


////////////// torque DOB /////////////////

 matrix::Matrix<float, 2,2>  MinvQ_T_A;
 matrix::Matrix<float, 2,1>  MinvQ_T_B;

 matrix::Matrix<float, 1,2>  MinvQ_T_C_x;
 matrix::Matrix<float, 1,2>  MinvQ_T_C_y;
 matrix::Matrix<float, 1,2>  MinvQ_T_C_z;


matrix::Matrix<float, 2,2>  Q_T_A;
matrix::Matrix<float, 2,1>  Q_T_B;
matrix::Matrix<float, 1,2>  Q_T_C;

 matrix::Matrix<float, 2,1>  MinvQ_T_X_x;
 matrix::Matrix<float, 2,1>  MinvQ_T_X_x_dot;
 matrix::Matrix<float, 1,1>  MinvQ_T_X_y;

 matrix::Matrix<float, 2,1>  Q_T_X_x;
 matrix::Matrix<float, 2,1>  Q_T_X_x_dot;
 matrix::Matrix<float, 1,1>  Q_T_X_y;

 matrix::Matrix<float, 2,1>  MinvQ_T_Y_x;
 matrix::Matrix<float, 2,1>  MinvQ_T_Y_x_dot;
 matrix::Matrix<float, 1,1>  MinvQ_T_Y_y;
 
 
 matrix::Matrix<float, 2,1>  Q_T_Y_x;
 matrix::Matrix<float, 2,1>  Q_T_Y_x_dot;
 matrix::Matrix<float, 1,1>  Q_T_Y_y;
 

 matrix::Matrix<float, 2,1>  MinvQ_T_Z_x;
 matrix::Matrix<float, 2,1>  MinvQ_T_Z_x_dot;
 matrix::Matrix<float, 1,1>  MinvQ_T_Z_y;
 
 
 matrix::Matrix<float, 2,1>  Q_T_Z_x;
 matrix::Matrix<float, 2,1>  Q_T_Z_x_dot;
 matrix::Matrix<float, 1,1>  Q_T_Z_y;
 

float torque_dob_fc = 10.f; //origin :: 20.f
float dhat_tau_r = 0.f;
float dhat_tau_p = 0.f;
float dhat_tau_y = 0.f;

float tautilde_r_d =0.f;
float tautilde_p_d =0.f;
float tautilde_y_d =0.f;
float root2 = sqrtf(2.0f); // root(2) = 1.414 :: damping factor = 0.707

///// Original MoI /////
float Jxx = 0.25f;
float Jyy = 0.25f;
float Jzz = 0.25f;



float constrain_with_sign(float value, float limit = 10.0f)
{
  if (!std::isfinite(value)) {
    return 0.f; // 또는 다른 대체값
  }


    if (fabsf(value) > limit) {
        return copysignf(limit, value); // limit의 크기를 value의 부호와 함께 반환
    } else {
        return value; // 그대로 통과
    }
}



// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void torque_DOB(float dt, matrix::Vector3f tau_rpy_desired, matrix::Vector3f imu_omega, matrix::Vector3f &tau_rpy_tilde, torque_dhat_s &torque_dhat)
{

  // Desired torque inputs

  float tau_r=tau_rpy_desired(0);
  float tau_p=tau_rpy_desired(1);
  float tau_y=tau_rpy_desired(2);

  float fc2 = pow(torque_dob_fc, 2);



  //========================[1] Q-Filter Definition (Same for all axes)========================//
  Q_T_A(0,0) = -root2*torque_dob_fc; Q_T_A(0,1) = -fc2;
  Q_T_A(1,0) =  1.0f;                Q_T_A(1,1) = 0.0f;

  Q_T_B(0,0) = 1.0f;
  Q_T_B(1,0) = 0.0f;

  Q_T_C(0,0) = 0.0f;       Q_T_C(0,1) = fc2;

  //========================[2] M^-1 Q Filter Definition (Different per axis)===================//
  MinvQ_T_A(0,0) = -root2*torque_dob_fc; MinvQ_T_A(0,1) = -fc2;
  MinvQ_T_A(1,0) = 1.0f;              MinvQ_T_A(1,1) = 0.f;

  MinvQ_T_B(0,0) = 1.0f;
  MinvQ_T_B(1,0) = 0.f;

  MinvQ_T_C_x(0,0) = Jxx*fc2; MinvQ_T_C_x(0,1) = 0.f;
  MinvQ_T_C_y(0,0) = Jyy*fc2; MinvQ_T_C_y(0,1) = 0.f;
  MinvQ_T_C_z(0,0) = Jzz*fc2; MinvQ_T_C_z(0,1) = 0.f;

  //========================[3] Roll Axis DoB===============================================//

  MinvQ_T_X_x_dot = MinvQ_T_A*MinvQ_T_X_x+MinvQ_T_B*imu_omega(0);
  MinvQ_T_X_x    += MinvQ_T_X_x_dot* dt;
  MinvQ_T_X_y     = MinvQ_T_C_x*MinvQ_T_X_x;

  Q_T_X_x_dot     = Q_T_A*Q_T_X_x+Q_T_B*tautilde_r_d;
  Q_T_X_x        += Q_T_X_x_dot*dt;
  Q_T_X_y         = Q_T_C*Q_T_X_x;

  dhat_tau_r      = constrain_with_sign(MinvQ_T_X_y(0,0) - Q_T_X_y(0,0));

  //========================[4] Pitch Axis DoB==============================================//

  MinvQ_T_Y_x_dot = MinvQ_T_A*MinvQ_T_Y_x+MinvQ_T_B*imu_omega(1);
  MinvQ_T_Y_x    += MinvQ_T_Y_x_dot*dt;
  MinvQ_T_Y_y     = MinvQ_T_C_y*MinvQ_T_Y_x;

  Q_T_Y_x_dot     = Q_T_A*Q_T_Y_x+Q_T_B*tautilde_p_d;
  Q_T_Y_x        += Q_T_Y_x_dot*dt;
  Q_T_Y_y         = Q_T_C*Q_T_Y_x;

  dhat_tau_p      = constrain_with_sign(MinvQ_T_Y_y(0,0) - Q_T_Y_y(0,0));

  //========================[5] Yaw Axis DoB===============================================//

  MinvQ_T_Z_x_dot = MinvQ_T_A*MinvQ_T_Z_x+MinvQ_T_B*imu_omega(2);
  MinvQ_T_Z_x    += MinvQ_T_Z_x_dot*dt;
  MinvQ_T_Z_y     = MinvQ_T_C_z*MinvQ_T_Z_x;

  Q_T_Z_x_dot     = Q_T_A*Q_T_Z_x+Q_T_B*tautilde_y_d;
  Q_T_Z_x        += Q_T_Z_x_dot*dt;
  Q_T_Z_y         = Q_T_C*Q_T_Z_x;

  dhat_tau_y      = constrain_with_sign(MinvQ_T_Z_y(0,0) - Q_T_Z_y(0,0));

  //========================[6] Estimate Error and Torque Compensation=======================//
  


  tau_rpy_tilde(0) = tau_r - dhat_tau_r;
  tau_rpy_tilde(1) = tau_p - dhat_tau_p;
  tau_rpy_tilde(2) = tau_y - dhat_tau_y;

  tautilde_r_d = tau_r - dhat_tau_r;
  tautilde_p_d = tau_p - dhat_tau_p;
  tautilde_y_d = tau_y - dhat_tau_y;

  // tau_rpy_desired(0) = tautilde_r_d;
  // tau_rpy_desired(1) = tautilde_p_d;
  // tau_rpy_desired(2) = tautilde_y_d;  // Optional: Yaw compensation not used
  torque_dhat.xyz[0] = dhat_tau_r;
  torque_dhat.xyz[1] = dhat_tau_p;
  torque_dhat.xyz[2] = dhat_tau_y;

}
