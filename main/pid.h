#ifndef PID_H
#define PID_H

class pid{

	float i, d, kp, ki, kd, b, h, y_old, N;

public:
	bool anti_windup = false; // Flag para ligar/desligar o anti-windup
	float last_u = 0;        // Guardamos o último duty cycle para verificar saturação
	explicit pid(float _h, float _kp=1.0, float _b=1.0, 
          float _ki=1.0, float _kd=0.0, float _n=10.0);	
	
	~pid(){};
	float compute_control(float r, float y);

	void housekeep(float r, float y);


};

inline void pid::housekeep(float r, float y){
	float e = r-y;
	//i+= ki*h*e;
	if (anti_windup) {
        // Lógica de Clamping: Só integra se não estiver saturado
        if (!((last_u >= 1.0f && e > 0) || (last_u <= 0.0f && e < 0))) {
            i += ki*h*e;
        }
    } else {
        i += ki*h*e; // Sem anti-windup, integra sempre
    }
	y_old = y;
}

#endif //PID_H
