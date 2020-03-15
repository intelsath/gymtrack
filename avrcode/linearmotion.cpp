// Linear motion functions
// Detects linear motion such as running

Linear_motion::Linear_motion(){
    n_iterations_step = 0;
    peak_z = false;
    peak_y = false;
    phase1 = false; // Z-peak+, Y-
    phase2 = false; // Y-peak+, Y>Z
    step = false;
    fz1 = 0.0;
    fz2 = 0.0;
    linear_velocity = 0.0;
    total_steps = 0;
    timer1 = new timer;
}
/*
// Temporary function
void Linear_motion::tmp_read_samples(){
    // If vector hasn't been initialized
    if( tmp_read_samples_vector_z.size() <= 0 ){
        double z,y;
        char ch;
        ifstream fileread("/home/luis/octave/gymtrack/treadmill5mph.txt");
        while( fileread ){
            fileread >> z >> ch >> y;
            tmp_read_samples_vector_z.push_back(z); // Pushes back to vector
            tmp_read_samples_vector_y.push_back(y); // Pushes back to vector
        }
        fileread.close();

    }

    // Give values
    a_z = tmp_read_samples_vector_z[tmp_read_samples_varcount];
    a_y = tmp_read_samples_vector_y[tmp_read_samples_varcount];

    tmp_read_samples_varcount++;




}*/

/* Store latest samples */
void Linear_motion::store_latest_samples(int samples_analyze){
    int n = samples_analyze; // Store latest n samples

    // Initialize vector if it hasn't been initialized, Z axis
    if( samples_z.size() < n ){
        for(int i=0; i<=n; i++){
            samples_z.push_back(-100); // Push back n items into vector, initializes at -100
        }
    } // Y axis
    if( samples_y.size() < n ){
        for(int i=0; i<=n; i++){
            samples_y.push_back(-100); // Push back n items into vector, initializes at -100
        }
    }

    // "Push back" all items before putting any new items
    for( int i=n-1; i>0; i-- ){
        samples_z[i] = samples_z[i-1];
        samples_y[i] = samples_y[i-1];
    }
    // Adds the last item to the first element of vector everytime
    samples_z[0] = a_z;
    samples_y[0] = a_y;

}

/* Function that detects the peaks of a discrete wave signal */
void Linear_motion::detect_peaks(){
    const int n = 3; // Samples to analyze, samples to store in vector
    const double peak_min = 0.3; // Minimum peak, ignore all peaks below this threshold
    store_latest_samples(n); // Keeps storing latest samples to analyze them

    // Derivative at an specific point of discrete samples, reads tangent lines from left and right
    double dz_dtleft,dz_dtright,dy_dtleft,dy_dtright; // Respective slopes at that particular point, from the left and from the right

    // Read positive samples only, we only care about positive peaks
    // Read latest 3 samples to know if it's a peak by reading the slope at that point
    if( samples_z[1] >= peak_min ){ // The peak must be high enough and positive
        // Is it a peak? Let's assume it's a peak... It should look like: /\
        //if it's a peak then there should be a positive tangent line (from the left) and a negative tangent line (from the right)
        dz_dtleft = (samples_z[1] - samples_z[2])/SAMPLE_PERIOD; // Slope of left tangent line, M = y2-y1/(x2-x1), here (x2-x1) is positive because x2>x1
        dz_dtright = (samples_z[1] - samples_z[0])/(-SAMPLE_PERIOD); // Slope of right tangent line, M = y1-y2/(x1-x2), here (x1-x2) is negative because x2>x1

        if( dz_dtleft > 0 && dz_dtright < 0  ) peak_z = true; // There is in fact a peak
        else peak_z = false;
    }else peak_z = false; // If no sample is analyzed, then there is no peak
    // Read positive samples only, we only care about positive peaks
    // Read latest 3 samples to know if it's a peak by reading the slope at that point
    if( samples_y[1] >= peak_min ){ // The peak must be high enough and positive
        // Is it a peak? Let's assume it's a peak... It should look like: /\
        //if it's a peak then there should be a positive tangent line (from the left) and a negative tangent line (from the right)
        dy_dtleft = (samples_y[1] - samples_y[2])/SAMPLE_PERIOD; // Slope of left tangent line, M = y2-y1/(x2-x1), here (x2-x1) is positive because x2>x1
        dy_dtright = (samples_y[1] - samples_y[0])/(-SAMPLE_PERIOD); // Slope of right tangent line, M = y1-y2/(x1-x2), here (x1-x2) is negative because x2>x1

        if( dy_dtleft > 0 && dy_dtright < 0  ) peak_y = true; // There is in fact a peak
        else peak_y = false;
    } else peak_y = false; // NO sample analyzed, no peak




}

// Detect steps by looking for a pattern
void Linear_motion::detect_steps(){
    // Detect pattern of a step in the wave
    // Pattern goes like this:
    // 1) Z-Peak+ and Y- => A positive peak in Z while Y is negative
    // 2) Y-peak+ => A positive peak in Y AND Y>Z
    // 3) Ignores further peaks that are close by, ignores the following n_i samples, n_i * 0.01 seconds, if 0.01 = sampling period.

    detect_peaks(); // Detect peaks
    int n_i = 20; // For third step, ignores the following n_1 samples after a step has been detected
    n_iterations_step++; // Number of iterations since last step detected
    step = false; // Start detecting steps

    // First step: Checks for a Z-peak while Y is negative
    // Z-peak+, Y-
    if( peak_z && samples_y[1] < 0 ){
        phase1 = true;
    }
    // Second step: Phase1 has to be followed by a positive Y-peak
    // Y-peak+, Y>Z
    if( phase1 && peak_y && samples_y[1] > samples_z[1] ){ // There should also be at least 1 sample in between since phase 1 happened
        phase2 = true;
    }

    // Third step, ignores n_1 samples after the last step found
    // Ignores nearby samples, it would be impossible to have another step that close
    if( phase1 && phase2 && n_iterations_step < n_i ){
        phase1 = false;
        phase2 = false;
    }

    // Step detected
    if( phase1 && phase2 ){
        //cout << "Step at: " << tmp_read_samples_varcount-2 << endl;
        n_iterations_step = 0; // Resets value
        phase1 = false;
        phase2 = false;
        total_steps++;
        step = true; // A step was given!
    }



}

void Linear_motion::estimate_speed(){

    if( !step ){
        // Initial values
        fz1 = a_z;
        /* Integration */
        if(fz1 > 0 && fz2 > 0){
            integral_z += ( (fz1 + fz2)/2) * (0.01) *(9.8);
        }
        else if( fz1 < 0  ){
            fz1 = 0.0;
        }
        else if( fz2 < 0 ){
            fz2 = 0.0;
        }
        fz2 = a_z; // Second value
    }

    if( ((timer1->get_thistime()/1000) > 3.0) && linear_velocity != 0 ) {
        linear_velocity = 0.0;
        kph = 0.0;
        mph = 0.0;
        phase1 = false;
        phase2 = false;
    }

    // After continously taking the integral, see if a step was given to make sure the speed is real
    if( step && integral_z > 0.0){ // Velocity has to be greater than 1 mph, at least

        timer1->timer_stop(); // Stops timer
        linear_velocity = integral_z; // m/s
        kph = (linear_velocity * 60 * 60) / 1000;
        mph = kph*0.62137; // Miles per hour
        integral_z = 0.0;

        timer1->timer_start(); // Starts timer;


    }


}
