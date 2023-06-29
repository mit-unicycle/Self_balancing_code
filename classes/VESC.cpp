#include <Arduino.h>
#include <VescUart.h>
#include <SoftwareSerial.h>

/**
 * Class that controls the VESC using 2 Digital pins via the UART protocol (Serial)
 * 
 * Libraries used: 
 *  - VescUart by SolidGeek
 *  - EspSoftwareSerial by plerup
 */
class VESC {
  private:
    SoftwareSerial vescSerial; // RX, TX pins for ESP8266
    VescUart vesc;  // VescUart object
    boolean disabled;  //whether the vesc is disabled or not

  public:

    /**
     * Constructor for the VESC class
     * @param RX The RX pin for the ESP8266
     * @param TX The TX pin for the ESP8266
     */
    VESC(int RX , int TX): vescSerial(RX, TX) {}

    /**
     * Method that is called in the beginning of excecution for setup
     */
    void setup(){
        /** Setup SoftwareSerial port */
        vescSerial.begin(74880);

        /** Define which ports to use as UART */
        vesc.setSerialPort(&vescSerial);
    }

    /**
     * Method that takes in a duty cycle percentage and sets the VESC to that duty cycle 
     * @param duty The duty cycle percentage 
    */
    void setDuty(double duty){
      if(duty > 100) duty = 100;
      if(duty < -100) duty = -100;

      if(!disabled) vesc.setDuty(duty/100);
      else {
        vesc.setDuty(0);
      }
    }

    /**
     * Method that takes in an RPM value and sets the VESC to that RPM
     * @param rpm The RPM value
    */
    void setRPM(double rpm){
      if(!disabled) vesc.setRPM(rpm);
      else {
        vesc.setRPM(0);
      }
    }

    /**
     * Method that that gets data from the VESC and prints it to the serial monitor
    */
    void printData(){
        if ( vesc.getVescValues() ) {
            Serial.println(vesc.data.rpm);
            Serial.println(vesc.data.inpVoltage);
            Serial.println(vesc.data.ampHours);
            Serial.println(vesc.data.tachometerAbs);
        }
        else
        {
            Serial.println("Failed to get data!");
        }
    }

    /**
     * Method that enables the VESC
    */
    void enable(){
      disabled = false;
    }

    /**
     * Method that disables the VESC
    */
    void disable(){
      disabled = true;
    }
};