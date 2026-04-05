#ifndef LUMINAIRE_H
#define LUMINAIRE_H

enum class LuminaireState{
	OFF,
	LOW,
	HIGH
};

class Luminaire{
private:
	int node_id; 
public:
	Luminaire(); // Const
	LuminaireState state;

	// Reference limits
	float low_bound;
	float high_bound;
	
	char stream_var = '\0'; // Saves 'l' (lux), 'u' (duty cycle) ou 's' (setpoint)
  bool streaming = false;

	// Control variables
	float reference;
	float current_u;  // Current Duty cycle (0.0 to 1.0)
	bool feedback_on = false; // By default the PID is ON
	bool feedforward_on = true;
	void updateReference();
	// Getter and Setter for the ID
	int getId() const;
	void setId(int new_id);
};

	#endif
