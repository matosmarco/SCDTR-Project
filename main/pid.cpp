#include "pid.h"

pid::pid(float _h, float _kp, float _b, 
                 float _ki, float _kd, float _n)

	// member variable initialization list
	:h{_h}, kp {_kp}, b {_b}, ki {_ki}, kd {_kd}, N {_n}, i {0.0}, d {0.0}, y_old{0.0}

	{} // should check arguments validity
	

float pid::compute_control(float r, float y){
	
	float p = kp*(b*r-y); // proportional
	float ad = kd/(kd+N*h); 
	float bd = kd*N/(kd+N*h);
	d = ad*d - bd*(y - y_old);
	float u = p+i+d;
	
	if(u < 0) u = 0;
       	if(u > 4095) u= 4095;

	return u;

}
