#include <PID_v1.h>
#include <Arduino.h>
#include "AvgBuffer.cpp"

/**
 * Wrapper class for the PID library
 * 
 * Libraries used: 
 *  - PID by Brett Beauregard
 * 
 * Written by David Farah
*/
class PIDControl {
  private:
    double kp;    // Proportional gain
    double ki;    // Integral gain
    double kd;    // Derivative gain
    double minOut;    // Minimum output value
    double maxOut;    // Maximum output value

    double setpoint;      // Setpoint value
    double origSetpoint;  // Original setpoint value (set by the user)

    double input;       // Input value
    double output = 0;  // Output value

    double accumOut = 0;  //used for setpoint shifting
    double accumIn = 0;   //used for setpoint shifting

    AvgBuffer inputAvg{100};  //moving average on the input with a window of 100
    AvgBuffer outputAvg{50};  //moving average on the output with a window of 50

    double inputAvgVal;  //average of the input
    double outputAvgVal; //average of the output

    PID pid;    // PID object

    /**
     * Shift the setpoint automatically in by +-0.5 using an LQR algorithm to reduce overshoot and
     * make the system more stable
     * 
     * The algorithm is as follows:
     *  - The input and output are averaged over a window of 100 and 50 respectively
     *  - The output is then converted to a logarithmic scale and constrained
     *  - The input average is constrained to be within +-0.5 of the original setpoint
     *  - The setpoint is then calculated by subtracting a multiple of the output from the input and constraining it
     * 
     * This will cause the setpoint to want to drift towards the average tilt angle of the system (setpoint), however
     * the output will try to counteract this drift by pushing the setpoint in the opposite direction (penalizing a high output). 
     * Meaning the higher the output, the more the setpoint will be pushed in the opposite direction, and vice versa, causing a 
     * dynamic equilibrium towards the real setpoint.
    */
    void adjustSetpoint(){
      inputAvg.add(input);
      outputAvg.add(output);

      inputAvgVal = inputAvg.calculate();
      outputAvgVal = outputAvg.calculate();

      accumOut = constrain((abs(outputAvgVal) > 0) ? log10(abs(outputAvgVal) + 1) : 0, -0.5, 0.5);
      accumOut = (outputAvgVal >= 0) ? -accumOut : accumOut ;
      accumIn = constrain(inputAvgVal, origSetpoint - 0.5, origSetpoint + 0.5);

      setpoint = constrain(accumIn - 1.1*accumOut, origSetpoint - 0.5, origSetpoint + 0.5);  // change the setpoint dynamically
    }

    /**
     * Print the values to Serial for debugging
    */
    void printValues(){
      Serial.print(" Input = ");
      Serial.print(input);
      Serial.print(" Output = ");
      Serial.print(output);
      Serial.print(" Setpoint = ");
      Serial.print(setpoint);
      Serial.print(" Output Avg = ");
      Serial.print(outputAvgVal);
      Serial.print(" Input Avg = ");
      Serial.print(inputAvgVal);
      Serial.print(" Accum Out = ");
      Serial.print(accumOut);
      Serial.print(" Accum In = ");
      Serial.println(accumIn);
    }

  public:

    /**
     * Constructor
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param minOut Minimum output value
     * @param maxOut Maximum output value
     * @param setPoint Setpoint value
    */
    PIDControl(double kp, double ki, double kd, double minOut, double maxOut, double setPoint) : 
      kp(kp), ki(ki), kd(kd), minOut(minOut), maxOut(maxOut), setpoint(setPoint), pid(&input, &output, &setpoint, kp, ki, kd, P_ON_E, DIRECT) {
        pid.SetOutputLimits(minOut, maxOut);
        pid.SetSampleTime(5);
        pid.SetMode(AUTOMATIC);

        origSetpoint = setPoint;
      }

    /**
     * Set the setpoint value
    */
    void setSetpoint(double setpoint) {
      this->setpoint = setpoint;
      origSetpoint = setpoint;
    }

    /**
     * Set the PID gains
    */
    void setTunings(double Kp, double Ki, double Kd){
      pid.SetTunings(Kp, Ki, Kd);
    
    }

    /**
     * Set the PID sample time
    */
    void setSampleTime(int NewSampleTime){
      pid.SetSampleTime(NewSampleTime);
    }

    /**
     * Compute the output value
    */
    double compute(double input) {
      this->input = input;

      double prevOut = output;

      pid.Compute();
      
      adjustSetpoint();
      //printValues();
      
      return this->output;
    }

    /**
     * Get the current setpoint value
    */
    double getSetpoint(){
      return this->setpoint;
    }

    /**
     * Reset the PID controller to introduce new gains and setpoint
    */
    void resetOutput(){
      output = (minOut + maxOut) / 2;

      pid = PID(&input, &output, &setpoint, kp, ki, kd, P_ON_E, DIRECT);
      
      pid.SetOutputLimits(minOut, maxOut);
      pid.SetSampleTime(5);
      pid.SetMode(AUTOMATIC);
    }

    /**
     * Get the current PID gains
    */
    std::vector<double> getParams(){
      return {pid.GetKp(), pid.GetKi(), pid.GetKd(), origSetpoint};
    }
};