#include "pid.h"

pid::pid(float _h, float _kp, float _b, float _ki, float _kd, float _n, float _kt)
    : h{_h}, kp{_kp}, b{_b}, ki{_ki}, kd{_kd}, N{_n}, kt{_kt}, 
      i{0.0}, d{0.0}, y_old{0.0}, last_v{0.0}
{}

float pid::compute_control(float r, float y) {
    // Proportional with se tpoin weighting (optional)
    float p_term_b = setpoint_weighting ? b : 1.0f;
    float p = kp * (p_term_b * r - y);

    // Derivative action with filter N
    if (derivative_enabled && kd > 0) {
        float ad = kd / (kd + N * h);
        float bd = kd * N / (kd + N * h);
        d = ad * d - bd * (y - y_old);
    } else {
        d = 0;
    }

    // Control signal (v)
    last_v = p + i + d; 

    float u = last_v;
    return u;
}

void pid::update_params(float new_kp, float new_ki, float new_kd, float new_b, float current_r) {
    // Bumpless Transfer: i_new = i_old + kp_old*(b_old*r - y) - kp_new*(b_new*r - y)
    i = i + kp * (b * current_r - y_old) - new_kp * (new_b * current_r - y_old);

    kp = new_kp;
    ki = new_ki;
    kd = new_kd;
    b = new_b;
}
