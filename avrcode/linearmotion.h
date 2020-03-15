#ifndef LINEARMOTION_H
#define LINEARMOTION_H

class Linear_motion : public Sense_motion{

    public:
        // Continously stores latest samples
        void store_latest_samples(int);
        vector<double> samples_z;
        vector<double> samples_y;
        // Detect peaks
        void detect_peaks();
        // Detect steps by looking at the defined wave pattern of a step
        void detect_steps();
        // Calculate speed by integrating
        void estimate_speed();

        // Temp members
        //void tmp_read_samples(); // Temporary function that read samples from a text file, this is temporary because samples are read in real time via serial port.
        //vector<double> tmp_read_samples_vector_z; // Vector to hold samples as strings
        //vector<double> tmp_read_samples_vector_y; // Vector to hold samples as strings
        //int tmp_read_samples_varcount; // Public temporary variable to read each individual sample

        timer * timer1 ; // timer1

        double get_velocity(){return linear_velocity;}
        double get_kph(){return kph;}
        double get_mph(){return mph;}
        int get_totalsteps(){return total_steps;}

        Linear_motion();
    private:
        // Is curent samples[1] value is a peak? see comments of detect_peaks() function to see algorithm and why it's samples[1]...
        //Hint, if samples[1] is a peak there should be a tangent line to the right and to the left
        bool peak_z, peak_y;
        bool phase1,phase2,step; // The three phases needed to determine if it's a step or not
        int n_iterations_step; // Number of iterations since the last step detected (running/treading)
        double fz1,fz2; // Previous and current readings, for integration
        double integral_z, linear_velocity; // Integral and Velocity
        double kph, mph; // Kilometers per hour and miles per hour
        int total_steps; // Total steps

};

   // const double Linear_motion::peak_min = 0.2; // Minimum peak, ignore all peaks below this threshold

#endif
