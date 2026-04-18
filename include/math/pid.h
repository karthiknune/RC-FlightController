#pragma once

class PIDController {
    public:
        PIDController(float kp, float ki, float kd, float max_output, float max_integral); //constructor
        float compute(float setpoint, float measured_value, float dt); //compute pid output
        void PIDreset();

        void setTunings(float kp, float ki, float kd);
        void setLimits(float max_output, float max_integral);

        float getkp(); 
        float getki();
        float getkd();
        float getMaxOutput();
        float getMaxIntegral();
    
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