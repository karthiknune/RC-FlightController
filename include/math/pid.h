#pragma once

class PIDController {
    public:
        PIDController(float kp, float ki, float kd, float max_output, float max_integral); //constructor
        float compute(float setpoint, float measured_value, float dt); //compute pid output
        void PIDreset();

        float getkp(); 
        float getki();
        float getkd();
    
    private:
        //tuning params
        float kp;
        float ki;
        float kd;
        float previous_error;
        float integral;
        float max_output;
        float max_integral;
};