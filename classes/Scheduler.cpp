/**
 * Class that provides a simple scheduler based on a desired frequency.
*/
class Scheduler{
    private:
        unsigned long lastTime;
        unsigned long currentTime;
        double delay = 0;

    public:
        /**
         * Constructor for the Scheduler class.
         * Sets the desired frequency in Hz.
         * 
         * @param frequency The desired frequency in Hz.
         */
        Scheduler(double frequency) {
            delay = 1 / frequency * 1000000;
            lastTime = micros();
        }

        /**
         * Checks if the specified time delay has passed since the last call.
         * 
         * @return Returns true if the time delay has passed, false otherwise.
         */
        boolean hasPassed(){
            currentTime = micros();

            if (currentTime - lastTime > delay) {
                lastTime = currentTime;
                return true;
            }
            else {
                return false;
            }
        }
};
