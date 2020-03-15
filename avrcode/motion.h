#ifndef MOTION_H
#define MOTION_H


using namespace std;

class Sense_motion{
    public:
        void read_serial();
        void separates_values(string); // Separates all the values and save them
        double a_xval(){return a_x;}
        double a_yval(){return a_y;}
        double a_zval(){return a_z;}
        double tiltY_val(){return tilt_y;}
        double tiltX_val(){return tilt_x;}
        Sense_motion();

        friend class Rotational_motion; // Prototype of friend class for rototational motion
        friend class Linear_motion; // Prototype of friend class for lienar motion


    private:
        double a_x,a_y,a_z,tilt_y,tilt_x; // x,y,z axis for accelerometer and gyroscope respectively
};


#endif
