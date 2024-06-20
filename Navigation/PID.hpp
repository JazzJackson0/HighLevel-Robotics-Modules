#pragma once
#include <iostream>
#include <chrono>
#include <limits>

using namespace std::chrono;

#define MANUAL 0
#define AUTO 1	

#define DIRECT 0 // Larger inputs lead to Larger outputs
#define REVERSE 1 // Larger inputs lead to Smaller outputs


class PID {

    private:
        float proportional_gain;
        float integral_gain;
        float derivative_gain;

        float Integrator;
        float Differentiator;

        // Integral Clamp Values
        float max_output;
	    float min_output;

        float previous_measurement;
        float current_measurement;
        
        float dt_ms;
        float prev_time;
        
        int controller_direction;
        bool pid_mode;
        
        float output_data;


    public:

        PID();


        PID(int direction, float sample_time_ms, float kp, float ki, float kd);

        /**
         * @brief Set or Update PID controller Direction
         * 			|||  Direct: Larger inputs lead to Larger outputs
         * 			|||  Reverse: Larger inputs lead to Smaller outputs
         * 
         * @param direction Direction of PID: (0) Direct or (1) Reverse
         * @return ** void 
         */
        void Set_ControllerDirection(int direction);


        /**
         * @brief Sets or Updates the PID Tuning parameters.
         * 
         * @param kp Proportional Gain Coefficient
         * @param ki Integral Gain Coefficient
         * @param kd Derivative Gain Coefficient
         * @return ** void 
         */
        void Set_Tuning_Parameters(float kp, float ki, float kd);


        /**
         * @brief Set or Update Vehicle to Manual Mode or AUtomatic Mode
         * 			|||  Manual Mode: Vehicle is remote controlled by a user and not the PID algorithm
         * 			|||  Automatic Mode: Vehicle is controlled by PID algorithm
         * 
         * @param mode Vehicle Mode: Manual or Automatic
         * @return ** void 
         */
        void Set_PIDMode(int mode);


        /**
         * @brief Sets or Updates the PID Sample Time.
         * 
         * @param new_dt_ms Time interval between calculations
         * @return ** void 
         */
        void Set_Sample_Time(float new_dt_ms);


        /**
         * @brief Sets output limits to prevent integral windup
         * 
         * @param min Min PID Output value
         * @param max Max PID Output value
         * @return ** void 
         */
        void Set_Output_Limits(float min, float max);


        /**
         * @brief Run PID on incoming data.
         * 
         * @param set_point Set point to compare incoming value to.
         * @param measurement current incoming value to input into the PID
         * @return ** float 
         */
        float PID_Update(float set_point, float measurement);

};