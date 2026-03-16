#ifndef PID_H
#define PID_H

class pid {
    // State variables and internal parametersS
    float i, d, kp, ki, kd, b, h, y_old, N, kt;
    float last_v; // Control signal before saturation

public:
    // Flags for anti-windup; set-point weighting and derivative filter
    bool anti_windup = true;        
    bool setpoint_weighting = true; 
    bool derivative_enabled = true; 

    explicit pid(float _h, float _kp=0.05, float _b=0.5, 
                 float _ki=0.06, float _kd=0.0, float _n=10.0, float _kt=1.0);	
    
    ~pid(){};

    // u(k) computation
    float compute_control(float r, float y);

    // Update states and applies anti-windup, if active
    void housekeep(float r, float y, float u);

    // Update gains with the Bumpless Transfer
    void update_params(float new_kp, float new_ki, float new_kd, float new_b, float current_r);
};


inline void pid::housekeep(float r, float y, float u) {
    float e = r - y;
    float bi = ki * h;
    float ao = kt * h; // Back-calculation gain

    if (anti_windup) {
        // i = i + bi*(r-y) + ao*(u-v)
        i = i + bi * e + ao * (u - last_v);
    } else {
        i = i + bi * e; // Normal integration (without protection)
    }
    
    y_old = y;
}

#endif
