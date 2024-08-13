#include "../include/PID.hpp"


PID::PID() {}

PID::PID(int direction, float sample_time_ms, float kp, float ki, float kd){

    Set_ControllerDirection(direction);
    Set_Sample_Time(sample_time_ms);
    Set_Tuning_Parameters(kp, ki, kd);
    Set_PIDMode(AUTO);
    Integrator = 0;
    Differentiator = 0;
    prev_time = (float) duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    output_data = 0;
    Set_Output_Limits(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
}


void PID::Set_ControllerDirection(int direction) {
	
	controller_direction = direction;
}

void PID::Set_Sample_Time(float new_dt_ms) {
	
	if (new_dt_ms > 0) {
		
		float ratio = new_dt_ms / dt_ms;
		
		integral_gain *= ratio; // (Ki * dt) * integral(e(t) <- IS EQUIVALENT TO -> Ki integral(e(t) * dt)
		derivative_gain /= ratio; // (Kd / dt) * de <- IS EQUIVALENT TO -> Kd * (de/dt)
		dt_ms = new_dt_ms;
	}
}

void PID::Set_Tuning_Parameters(float kp, float ki, float kd) {
	
	if (proportional_gain < 0.0 || integral_gain < 0.0 || derivative_gain < 0.0) return;
	
	float dt_secs = dt_ms / 1000;
	proportional_gain = kp;
	integral_gain = ki * dt_secs;
	derivative_gain = kd / dt_secs;
	
	if (controller_direction == REVERSE) {				
		
		proportional_gain = 0 - proportional_gain;
		integral_gain = 0 - integral_gain;
		derivative_gain = 0 - derivative_gain;
	}
}


void PID::Set_PIDMode(int mode) { 
	
	bool newAutoMode = (mode == AUTO);
	
    if (mode == AUTO && pid_mode == MANUAL) 
        previous_measurement = current_measurement;	// Backup. Keep derivative from spiking

    pid_mode = newAutoMode;
}




void PID::Set_Output_Limits(float min, float max) {
	
	if (min > max) return;
	min_output = min;		
	max_output = max;			
	
	if (output_data > max_output) 
        output_data = max_output;			
	else if (output_data < min_output) 
        output_data = min_output;	
	
	if (Integrator > max_output) 
        Integrator = max_output;	
	else if (Integrator < min_output) 
        Integrator = min_output;
}



float PID::PID_Update(float set_point, float measurement) {
	
	if (pid_mode == MANUAL) {
        std::cout << "ERROR: Leave Manual Mode to start PID" << std::endl;
        return -1.f;
    }
	
	float current_time = (float) duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
	float dt = (current_time - prev_time);
	
	if (dt >= dt_ms) {
		
		current_measurement = measurement;
		
        // Proportional Term
		float current_error = set_point - current_measurement;
		float proportional_term = proportional_gain * current_error;
		
        // Integral Term
		Integrator += integral_gain * current_error;
		if (Integrator > max_output) 
            Integrator = max_output;
		else if (Integrator < min_output) 
            Integrator = min_output;
		
        // Derivative Term
		Differentiator = current_measurement - previous_measurement;
		
        // PID formula
		output_data = proportional_term + Integrator + (derivative_gain * Differentiator);
		if (output_data > max_output) 
            output_data = max_output;
		else if (output_data < min_output) 
            output_data = min_output;
		
		// Update Measurement & Time
		previous_measurement = current_measurement;
		prev_time = current_time;
	}
	return output_data;
}


