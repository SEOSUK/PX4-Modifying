#include "dob_based_com_estimator.hpp"
#include <matrix/matrix/math.hpp>
#include <mathlib/math/Functions.hpp> // math::constrain

using matrix::Vector;

// 상태 유지 변수
/*
static bool com_xy_fixed = false;
static bool allow_z_update = false;
static matrix::Vector2f com_xy_last = {};
static int stable_counter_xy = 0;
static constexpr int STABLE_COUNT_THRESHOLD = 200;  // 약 200 루프 (20ms * 200 = 4초)
static constexpr float XY_STABLE_EPSILON = 0.0003f;
static constexpr float TORQUE_THRESHOLD_XY = 0.05f;
/static int loop_count = 0;
constexpr int MIN_COUNT_BEFORE_STABILITY_CHECK = 100; // 100 루프 후 안정성 체크 시작
constexpr int LOOP_COUNT_MAX = 500; // 약 10초 */


// 최대 가능한 버퍼 크기 지정 (최악의 경우 대비)
static constexpr int MAX_WINDOW_SIZE = 3000; // 3초 @ 최소 1000Hz 예상
static matrix::Vector3f past_com_buffer[MAX_WINDOW_SIZE] = {};
static int com_sample_idx = 0;
static int com_avg_window_size = 0;
static bool com_buffer_filled = false;

// 천천히 수렴하는 필터 상태
static matrix::Vector3f com_target_fixed = {}; // 3초 평균으로 고정된 com 값
static matrix::Vector3f com_update_filtered = {}; // 최종적으로 천천히 따라가는 com
static matrix::Vector3f com_update = {}; // 제어기로 들어가는 최종 값

// 파라미터
static constexpr float com_update_to_target_step = 0.00005f; // com_update가 com_update_filtered를 따르는 속도


// LPF 계수 (2초 정도 시간 상수)
static constexpr float com_update_alpha = 0.01f; // 작은 값일수록 느리게 수렴
static constexpr float com_update_alpha2 = 0.001f;
// 활성 상태 flag 및 동적 타겟 값
//static bool com_active_flag = false;
static matrix::Vector3f com_active_target = {};
static constexpr float ramp_step = 0.0005f; // 매 루프당 약 0.0005씩 변경 (~20ms 기준 2초 걸림)

matrix::Vector3f compute_average_com()
{
    matrix::Vector3f sum{};
    int valid_size = com_buffer_filled ? com_avg_window_size : com_sample_idx;

    for (int i = 0; i < valid_size; ++i) {
        sum += past_com_buffer[i];
    }
    return (valid_size > 0) ? sum / valid_size : matrix::Vector3f{};
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

matrix::Matrix<float, 3,3> b_F_X_mat; // [F]x
matrix::Matrix<float, 3,3> J_hat_mat; // body frame J hat 

matrix::Matrix<float, 2,2>  Q_T_A_est;
matrix::Matrix<float, 2,1>  Q_T_B_est;
matrix::Matrix<float, 1,2>  Q_T_C_est;

matrix::Matrix<float, 2,1>  Q_T_X_x_est;
matrix::Matrix<float, 2,1>  Q_T_X_x_dot_est;
matrix::Matrix<float, 1,1>  Q_T_X_y_est;

matrix::Matrix<float, 2,1>  Q_T_Y_x_est;
matrix::Matrix<float, 2,1>  Q_T_Y_x_dot_est;
matrix::Matrix<float, 1,1>  Q_T_Y_y_est;

matrix::Matrix<float, 2,1>  Q_T_Z_x_est;
matrix::Matrix<float, 2,1>  Q_T_Z_x_dot_est;
matrix::Matrix<float, 1,1>  Q_T_Z_y_est;

matrix::Matrix<float,3,3> MoI;
matrix::Matrix<float,3,3> MoI_inv;
matrix::Matrix<float,3,3> A;
matrix::Vector3f b_F_LPF;

matrix::Vector3f present_com_hat{0.f,0.f,0.f};
//matrix::Vector3f past_com_hat{0.f,0.f,0.f};
//matrix::Vector3f com_hat_tilde{0.f,0.f,0.f};
matrix::Vector3f com_hat_dot;

float root2_est = sqrtf(2.0f);
float torque_dob_fc_est = 10.f; //origin :: 1
float est_gamma = 0.0003f; // estimator gain
float k = 1.f/0.8f; // voltage drop gain
float Jxx_est = 0.25f;
float Jyy_est = 0.25f;
float Jzz_est = 0.25f;



void constrain_vector3_components(matrix::Vector3f &v)
{
    v(0) = math::constrain(v(0), -0.1f, 0.1f);  // x축 제한
    v(1) = math::constrain(v(1), -0.1f, 0.1f);  // y축 제한
    v(2) = math::constrain(v(2), -0.1f, 0.0f);  // z축 제한
}

//void dob_based_com_estimator(matrix::Vector3f torque_dhat, matrix::Vector3f body_force_desired, matrix::Vector3f &present_com_hat, matrix::Vector3f &past_com_hat);
void dob_based_com_estimator(float dt, matrix::Vector3f torque_dhat, matrix::Vector3f body_force_desired, center_of_mass_s &com_log, matrix::Vector3f &past_com_hat,matrix::Vector3f &com_hat_tilde)
{   
    float fc2 = pow(torque_dob_fc_est, 2);

    
    //========================[1] Q-Filter Definition (Same for all axes)========================//
    Q_T_A_est(0,0) = -root2_est*torque_dob_fc_est; Q_T_A_est(0,1) = -fc2;
    Q_T_A_est(1,0) = 1.0f;                        Q_T_A_est(1,1) = 0.0f;

    Q_T_B_est(0,0) = 1.0f;
    Q_T_B_est(1,0) = 0.0f;

    Q_T_C_est(0,0) = 0.0f;       Q_T_C_est(0,1) = fc2;
    
    //========================[2] X Axis Force===============================================//

    Q_T_X_x_dot_est     = Q_T_A_est*Q_T_X_x_est+Q_T_B_est*body_force_desired(0);
    Q_T_X_x_est        += Q_T_X_x_dot_est*dt;
    Q_T_X_y_est         = Q_T_C_est*Q_T_X_x_est;

    b_F_LPF(0) = Q_T_X_y_est(0,0);

    //========================[3] Y Axis Force===============================================//

    Q_T_Y_x_dot_est     = Q_T_A_est*Q_T_Y_x_est+Q_T_B_est*body_force_desired(1);
    Q_T_Y_x_est        += Q_T_Y_x_dot_est*dt;
    Q_T_Y_y_est         = Q_T_C_est*Q_T_Y_x_est;

    b_F_LPF(1) = Q_T_Y_y_est(0,0);

    //========================[4] Z Axis Force===============================================//

    Q_T_Z_x_dot_est     = Q_T_A_est*Q_T_Z_x_est+Q_T_B_est*body_force_desired(2);
    Q_T_Z_x_est        += Q_T_Z_x_dot_est*dt;
    Q_T_Z_y_est         = Q_T_C_est*Q_T_Z_x_est;

    b_F_LPF(2) = Q_T_Z_y_est(0,0);


    //========================[5] Force LPF Cross Product Matrix ===============================================//
    b_F_X_mat(0,0) = 0.f;          b_F_X_mat(0,1) = -b_F_LPF(2);    b_F_X_mat(0,2) = b_F_LPF(1); 
    b_F_X_mat(1,0) = b_F_LPF(2);   b_F_X_mat(1,1) = 0.f;            b_F_X_mat(1,2) =-b_F_LPF(0);
    b_F_X_mat(2,0) =-b_F_LPF(1);   b_F_X_mat(2,1) = b_F_LPF(0);     b_F_X_mat(2,2) = 0.f;

    //========================[6] Definition A transpose ========================================//

    MoI(0,0) = Jxx_est; MoI(0,1) = 0.f;     MoI(0,2) = 0.f;
    MoI(1,0) = 0.f;     MoI(1,1) = Jyy_est; MoI(1,2) = 0.f;
    MoI(2,0) = 0.f;     MoI(2,1) = 0.f;     MoI(2,2) = Jzz_est;
    
    matrix::geninv(MoI, MoI_inv);

    // A = k * MoI_inv * b_F_X_mat;
    A = MoI_inv * b_F_X_mat;

    //========================[7] estimated com integration ========================================//
    
    com_hat_tilde = present_com_hat - past_com_hat;

    com_hat_dot = A.transpose() * torque_dhat;

    past_com_hat += est_gamma*com_hat_dot;

    constrain_vector3_components(past_com_hat);
    
    // dt 기반 평균 윈도우 크기 계산 (최초 한 번만)
    static constexpr float com_avg_duration_sec = 3.0f;
    if (com_avg_window_size == 0 && dt > 1e-5f) {
        com_avg_window_size = math::constrain(static_cast<int>(com_avg_duration_sec / dt), 1, MAX_WINDOW_SIZE);
    }

    // 평균 버퍼 갱신
    past_com_buffer[com_sample_idx] = past_com_hat;
    com_sample_idx = (com_sample_idx + 1) % com_avg_window_size;
    if (com_sample_idx == 0) {
        com_buffer_filled = true;
    }

    /*
    b_F_X_mat(0,0) = 0.f;                   b_F_X_mat(0,1) = -body_force_desired(2);        b_F_X_mat(0,2) = body_force_desired(1); 
    b_F_X_mat(1,0) = body_force_desired(2); b_F_X_mat(1,1) = 0.f;                           b_F_X_mat(1,2) =-body_force_desired(0);
    b_F_X_mat(2,0) =-body_force_desired(1); b_F_X_mat(2,1) = body_force_desired(0);         b_F_X_mat(2,2) = 0.f;
    */
    
    // 평균 및 필터링 수행
    com_target_fixed = compute_average_com();
    
    com_update_filtered = com_update_alpha * com_target_fixed + (1.f - com_update_alpha) * com_update_filtered;

    // === com_update가 com_update_filtered를 따라감 ===

    com_update = com_update_alpha2 * com_update_filtered + (1.f - com_update_alpha2) * com_update;
    

    // ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

    // === com_update가 com_update_filtered를 따라감 ===
    /*
    if (!com_xy_fixed) {
        // (1) XY 업데이트: 먼저 com_update(0), com_update(1) 수렴 감지
        com_update(0) = com_update_alpha2 * com_update_filtered(0) + (1.f - com_update_alpha2) * com_update(0);
        com_update(1) = com_update_alpha2 * com_update_filtered(1) + (1.f - com_update_alpha2) * com_update(1);

        float dx = fabsf(com_update(0) - com_xy_last(0));
        float dy = fabsf(com_update(1) - com_xy_last(1));
        if (loop_count > MIN_COUNT_BEFORE_STABILITY_CHECK) {
            
        if (dx < XY_STABLE_EPSILON && dy < XY_STABLE_EPSILON) {
            stable_counter_xy++;
            } else {
            stable_counter_xy = 0;
            }

        if (stable_counter_xy >= STABLE_COUNT_THRESHOLD) {
            com_xy_fixed = true;}
        }

        com_xy_last(0) = com_update(0);
        com_xy_last(1) = com_update(1);
        
        // z는 업데이트 하지 않음
        com_update(2) = 0.0f; // 또는 이전 값 유지
    } else {
        // (2) XY는 고정 상태. torque_dhat 안정 시 Z축 업데이트 시작
        if (!allow_z_update) {
            if (fabsf(torque_dhat(0)) < TORQUE_THRESHOLD_XY &&
                fabsf(torque_dhat(1)) < TORQUE_THRESHOLD_XY) {
                allow_z_update = true;
            }
        }

        // XY는 고정값 유지
        com_update(0) = com_xy_last(0);
        com_update(1) = com_xy_last(1);

        // Z축 업데이트 조건 달성 시에만 업데이트
        if (allow_z_update) {
            com_update(2) = com_update_alpha2 * com_update_filtered(2) + (1.f - com_update_alpha2) * com_update(2);
        }
    }
    loop_count++;
    if(loop_count>LOOP_COUNT_MAX) // 약 10초)
    {

        loop_count = LOOP_COUNT_MAX;
    }*/

    
    com_log.present_com_hat[0] = com_update_filtered(0); // after filtering
    com_log.present_com_hat[1] = com_update_filtered(1);
    com_log.present_com_hat[2] = com_update_filtered(2);

    com_log.past_com_hat[0] = past_com_hat(0); // before filtering
    com_log.past_com_hat[1] = past_com_hat(1);
    com_log.past_com_hat[2] = past_com_hat(2);

    com_log.com_tilde[0] = com_hat_tilde(0); // present_com_hat - com_update
    com_log.com_tilde[1] = com_hat_tilde(1);
    com_log.com_tilde[2] = com_hat_tilde(2);


    
    com_log.com_update[0] = com_update(0); // for update on control allocator matrix
    com_log.com_update[1] = com_update(1); 
    com_log.com_update[2] = com_update(2); 
    

}