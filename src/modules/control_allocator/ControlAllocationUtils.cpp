#include "ControlAllocationUtils.hpp"
#include <math.h> 
float force_to_pwm_scale(float force)
{
	float a = 2.0962e-05f;
	float b = 0.0085f;
	float c = -36.0347f;

    float pwm_min = 1100.0f;
	float pwm_max = 1900.0f;
    float pwm_scaled = 0.f;

	float discriminant = b * b - 4.0f * a * (c - force);

	if (discriminant < 0.0f) {
		// 안정성을 위해 예외 처리 (루트 안이 음수면 sqrt 불가)
        pwm_scaled = 0.0f;
	}
    else {
        float pwm = (-b + sqrtf(discriminant)) / (2.0f * a);
        //pwm_scaled(i) = pwm;  // 실제 PWM 수치
        pwm_scaled = (pwm - pwm_min) / (pwm_max - pwm_min);
    }

    return pwm_scaled;
    
}