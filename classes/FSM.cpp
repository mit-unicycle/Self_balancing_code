#include "PIDControl.cpp"
#include "VESC.cpp"
#include "mqttClient.cpp"
#include "Gyro.cpp"
#include "mpuCalibration.cpp"
#include "Scheduler.cpp"
#include <math.h>

/**
    Class that implements the finite state machine for the balancing robot.
*/
class FSM {
    private:
        enum class State {
            Disabled,
            Balance,
            Tuning,
            Debug,
            ESCoff
        };
        State currentState;
        State prevState;

        boolean prevButton = true;
        boolean updateButton = true;

        //----------------------DEFINE PID PARAMETERS---------------------------------
        const int maxPower = 60; //max RPM of motors
        const int cutoffAngle = 45; //angle after which the motor gets disbaled
        int minLimit = -15000 , maxLimit = 15000, center = 0;
        const double minOut = minLimit - minLimit*(100-maxPower)/100, maxOut = maxLimit - maxLimit*(100-maxPower)/100;
        
        double rightSetPoint = -5;
        const double Rp = 2750; //Proportional path weight
        const double Ri = 6050; //Integration path weight
        const double Rd = 60; //Derivation path weight
        
        double leftSetPoint = 2.5;
        const double Lp = 2750; //Proportional path weight
        const double Li = 4100; //Integration path weight
        const double Ld = 30; //Derivation path weight

        double rollAngle = 0;
        double rightAngle = 0;
        double rightOutput = center;

        double leftOutput = center;
        double pitchAngle = 0;
        double leftAngle = 0;

        //-------------------------------DEFINE OBJECTS----------------------------------
        PIDControl PIDRight{Rp, Ri, Rd, minOut, maxOut, rightSetPoint};
        PIDControl PIDLeft{Lp, Li, Ld, minOut, maxOut, leftSetPoint};

        Gyro gyro;
        VESC vescLeft{D4, D5};
        VESC vescRight{D6, D7};

        Scheduler escLimit{300}; //limit the rate of the ESC (Hz)
        Scheduler buttonLimit{10}; //limit the rate for the button (Hz)
        Scheduler PIDLimit{300}; //limit the rate of the PID calculations (Hz)
        Scheduler printLimit{100};

        mpuCalibration calibrate;

        //------------------------OTHER FIELDS AND METHODS--------------------------------
        boolean tuneSetup = false;

        unsigned long lastTime;
        unsigned long currentTime;

        /**
         * Print the frequency of the loop to the serial monitor
        */
        void printFreq() {
            currentTime = micros();
            unsigned long loopTime = currentTime - lastTime;
            lastTime = currentTime;

            float frequency = 1000000.0 / loopTime; // Convert loop time to frequency in Hz

            Serial.print("Frequency: ");
            Serial.print(frequency);
            Serial.println(" Hz");
        }

        /**
         * Convert the enum to a string
        */
        String enumToString(State value) {
            switch (value) {
                case State::Disabled:
                    return "Disabled";
                case State::Debug:
                    return "Debug";
                case State::Tuning:
                    return "Tuning";
                case State::ESCoff:
                    return "ESC off";
                case State::Balance:
                    return "Balance";
                default:
                    return "Unknown";         
            }
        }

        /**
         * Print the values to the serial monitor
        */
        void printValues() {
            Serial.print(" Left Angle= ");
            Serial.print(leftAngle);
            Serial.print(" Left Output = ");
            Serial.print(leftOutput);
            Serial.print(" Right Angle= ");
            Serial.print(rightAngle);
            Serial.print(" Right Output = ");
            Serial.print(rightOutput);
            Serial.print(" State = ");
            Serial.println(enumToString(currentState));
        }

        /**
         * Project the angles to the right and left wheels
        */
        void projectAngles(){
            rightAngle = -sin(M_PI/4)*rollAngle - cos(M_PI/4)*pitchAngle;
            leftAngle = -cos(M_PI/4)*rollAngle + sin(M_PI/4)*pitchAngle;
        }

    public:
        FSM() : currentState(State::ESCoff) {}

        /**
         * Setup the FSM
        */
        void setup() {
            pinMode(0, INPUT_PULLUP);
            gyro.setup();
            vescRight.setup();
            vescLeft.setup();

            vescRight.disable();
        }

        /**
         * Loop the FSM
        */
        void loop() {
            // update angles
            gyro.loop();
            rollAngle = gyro.getRoll();
            pitchAngle = gyro.getPitch();

            //printFreq();
            projectAngles();

            if(buttonLimit.hasPassed()){updateButton = true;} //Set updateButton to true if time limit is reached

            //Safety features and On/Off
            if (abs(rightAngle) > cutoffAngle || abs(leftAngle) > cutoffAngle) {currentState = State::Disabled;}

            //Compute PID output
            if(PIDLimit.hasPassed()){
                rightOutput = PIDRight.compute(rightAngle);
                leftOutput = PIDLeft.compute(leftAngle);
            }


            //FSM states
            switch (currentState) {

                //State for disabling the robot (after a fall for example)
                case State::Disabled:
                    if(escLimit.hasPassed()){
                        vescRight.setDuty(0);
                        vescLeft.setDuty(0);

                        if(updateButton) {
                            if(digitalRead(0) && !prevButton ){currentState = State::ESCoff;}
                        }

                        printValues();
                    }
                    delay(10);
                    break;
                
                //State for turning off the ESC
                case State::ESCoff:
                    if(updateButton) {
                        if(digitalRead(0) && !prevButton ){currentState = State::Debug;}
                    }

                    if(escLimit.hasPassed()){
                        vescRight.setDuty(0);
                        vescLeft.setDuty(0);
                        printValues();
                    }
                    break;

                //State for balancing the robot (no prints to serial monitor)
                case State::Balance:
                    if(updateButton) {
                        if(digitalRead(0) && !prevButton ){currentState = State::ESCoff;}
                    }

                    if(escLimit.hasPassed()){
                        //vescRight.setRPM(rightOutput);
                        vescLeft.setRPM(leftOutput);
                    }
                    break;

                //State for debugging the robot
                case State::Debug:
                    if(updateButton) {
                        if(digitalRead(0) && !prevButton ){currentState = State::ESCoff;}
                    }

                    printValues();

                    if(escLimit.hasPassed()){
                        //vescRight.setRPM(rightOutput);
                        vescLeft.setRPM(leftOutput);
                    }
                    break;

                //State for tuning the mpu6050 and getting the offsets
                case State::Tuning:
                    if(!tuneSetup){
                        calibrate.setup();
                        tuneSetup = true;
                    } else {
                        if(calibrate.loop()) {
                            tuneSetup = false;
                            delay(5000);
                            currentState = State::Disabled;
                        }
                    }
                    break;
            }
            
            if(updateButton){prevButton = digitalRead(0);}
            updateButton = false;
        }

        /**
         * Set the tuning parameters for the right PID controller
        */
        void setTuningsRight(double Rp, double Ri, double Rd){
            PIDRight.setTunings(Rp, Ri, Rd);
        }

        /**
         * Set the tuning parameters for the left PID controller
        */
        void setTuningsLeft(double Lp, double Li, double Ld){
            PIDLeft.setTunings(Lp, Li, Ld);
        }

        /**
         * Set the setpoint for the right PID controller
        */
        void setSetpointRight(double setpoint){
            PIDRight.resetOutput();
            PIDRight.setSetpoint(setpoint);
        }

        /**
         * Set the setpoint for the left PID controller
        */
        void setSetpointLeft(double setpoint){
            PIDLeft.resetOutput();
            PIDLeft.setSetpoint(setpoint);
        }

        /**
         * Get the current parameters array as a vector
         * @return the current parameters array as a vector
        */
        std::vector<double> getParams(){
            std::vector<double> rollParams = PIDRight.getParams();
            std::vector<double> pitchParams = PIDLeft.getParams();
            
            std::vector<double> concatenatedVector;
            concatenatedVector.reserve(rollParams.size() + pitchParams.size()); // Reserve space for efficient concatenation

            // Append elements from vector1
            for (const auto& element : rollParams) {
                concatenatedVector.push_back(element);
            }

            // Append elements from vector2
            for (const auto& element : pitchParams) {
                concatenatedVector.push_back(element);
            }

            return concatenatedVector;
        }
};