#ifndef ROTATION_H
#define ROTATION_H

using namespace std;

class timer;

class Rotational_motion : public Sense_motion{

    public:
        /* Rotational motion detector -- For pedaling, elliptical, etc */
        void check_quadrant(int);
        vector<int> quadrants;
        int get_quadrant(){return current_quadrant;} // Current quadrant
        void updates_revs(); // Updates revolutions, depending on what type of revolution was detected (elliptical revolution, circular revolution etc)
        int get_revolutions(){ return revolutions; } // Revolutions
        double get_period(){return period;} // get f^-1, period.
        double get_RPM(){return RPM;}
        Rotational_motion(double,double); // Constructor
        ~Rotational_motion(); // Destructor
        double get_kph(){ return *kph;}
        double get_mph(){ return *mph;}

        timer * timer1 ; // timer1

    private:
        int current_quadrant; // This is equal to the "average" quadrant that is appearing, that is, the mode of the vector quadrants.
        int revolutions; // How many complete revolutions
        int quadrant_cycles[5]; // 4 cycles, used to look for the pattern
        int quadrant_change; // Detects changes in quadrants
        double period; // Period of one revolution, time. period = f^-1, where f is frequency in hertz (rev/s)
        double RPM; // Average RPM

        // Determined during run time
        double *wheel_diameter = new double; // Wheel diameter in inches
        double *gear_ratio = new double; // Gear ratio, common one is 2:1
        double *kph = new double; // Mts/sec
        double *mph = new double; // Mts/sec


};

#endif
