class myPIDController 
{
    private: 
        // Controller Gains 
        double Kp; 
        double Ki;
        double Kd;

        int sampleRate = 10;

        // Derivative low-pass filter time constant
        double tau; 

        // Output limits 
        double limMin;
        double limMax;

        // Controller time (in seconds)
        double T = 0.02;

        // Controller "memory"
        double integrator = 0; 
        double differentiator; 
        double proportional;

        double error = 0;
        double prevError; 
        double prevMeasurement; 

        unsigned long prevTime;
        unsigned long timeDiff;

        //Anti-wind-up via integation clamping
        double limMinInt, limMaxInt; 

        // Controller Output 
        double out; 
    
    public: 
        myPIDController(double Kp_, double Ki_, double Kd_, double limMin_, double limMax_):
                        Kp(Kp_), Ki(Ki_), Kd(Kd), limMin(limMin_), limMax(limMax_) 
                        {
                            prevTime = millis();
                            timeDiff = 0;

                            limMaxInt = (limMax - limMin)/2;
                            limMinInt = -(limMax - limMin)/2;
                        };

        double getOut() 
        {
            return out;
        }
        
        double update(double setpoint, double measurement) 
        { 
            timeDiff = millis() - prevTime;

            if(timeDiff > sampleRate) {
                // Error signal
                error = setpoint - measurement;
                
                // Proportional
                proportional = error;

                // Integral
                //integrator = integrator + 0.5 * Ki * T * (error + prevError);

                integrator = Ki * (integrator + error * timeDiff);

                // Clamp Integrator
                if (integrator > limMaxInt) {
                    integrator = limMaxInt;
                } else if (integrator < limMinInt) {
                    integrator = limMinInt;
                }

                // Derivative (band-limited differentiator by using low-pass filter)
                // differentiator = -(2.0 * Kd * (measurement - prevMeasurement)) // Note: derivative-on-measurement! 
                //                     + (2.0 * tau - T) * differentiator
                //                     / (2.0 * tau + T);

                differentiator = (error - prevError)/timeDiff ;

                // Compute output and apply limits
                out = Kp * proportional + integrator + Kd * differentiator + 1500;
                
                // Limit controller output. The limMax and limMin terms are set by the user 
                if (out > limMax) {
                    out = limMax;
                } else if (out < limMin) {
                    out = limMin;
                } 

                // Store error
                prevError = error;
                prevMeasurement = measurement;
                prevTime = millis();
            }

            return out;
        }

        void setTunings(double Kp, double Ki, double Kd) {
            this -> Kp = Kp;
            this -> Ki = Ki;
            this -> Kd = Kd;
        }
};
