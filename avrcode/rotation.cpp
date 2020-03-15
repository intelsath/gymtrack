/* Rotational motion functions */
// Rotational motion constructor
Rotational_motion::Rotational_motion(double diameter,double ratiogear){
        current_quadrant = 0; // This is equal to the "average" quadrant that is appearing, that is, the mode of the vector quadrants.
        revolutions = 0;
        quadrant_change = 0;
        quadrant_cycles[0] = 0;
        quadrant_cycles[1] = 0;
        quadrant_cycles[2] = 0;
        quadrant_cycles[3] = 0;
        RPM = 0;
        period = 0;
        *wheel_diameter = diameter; // Wheel's ratio
        *gear_ratio = ratiogear;    // Gear ratio
        *kph = 0.0;
        *mph = 0.0;
        timer1 = new timer;
}

// Release memory
Rotational_motion::~Rotational_motion(){
        delete wheel_diameter;
        delete gear_ratio;
        delete mph;
        delete kph;
        delete timer1;
}

// The number of quadrants saved in quadrants' vector should be added as a parameter
// This parameter should change as the speed changes
void Rotational_motion::check_quadrant(int num_quadrants){

    /* Checks average and gives it a quadrant depending on the sign of (z,y):
     -+ = I quadrant
     ++ = ++ II quadrant
     +- = III quadrant
     -- IV quadrant
     Note that clockwise and counterclockwise is possible, so --,+-,++,-+ or -+, ++, +-, -- are valid. The pattern is what matters.
    */
    // Note: Using "signs" to make logic more 'understandable'.

    bool sign_z,sign_y; // + = 1, - = 0
    int signs_quadrant; // Quadrant associated with the signs of the z, y axis
    int n = num_quadrants; // Number of quadrants saved in quadrants vector
    map<int,int> quadrant_counts; // Count the number of the times the quadrant appears on the vector list, uses this map to find mode
    int maxnum = 0; // max num, used to find mode (the max number of times the quadrant appeared)
    int stat_mode = 0; // Statistical mode of which one appears more.
    const int initiliaze_quadrant_vector = 32; // Number of items inside vectore to be analyzed, this should be > n, where n = num_quadrants, samples to be analyzed
    // quadrant_counts[QUADRANT] = NUM OF TIMES QUADRANT APPEARED ON LIST
    quadrant_counts[1] = 0;
    quadrant_counts[2] = 0;
    quadrant_counts[3] = 0;
    quadrant_counts[4] = 0;

    assert( initiliaze_quadrant_vector > n ); // Vector should be initialized with more elements than n (number of quadrants to be analyzed) to have enough "space"


    /* Updates vector */
    /* ------------------------------------------------------ */
    // Initialize vector if it hasn't been initialized
    if( quadrants.size() < initiliaze_quadrant_vector ){
        for(int i=0; i<=initiliaze_quadrant_vector; i++){
            quadrants.push_back(0); // Push back n items into vector, initializes at 0
        }
    }


    // Assign the signs, if z,y < 0 then 0,0 (--). if z,y>=0 1,1 (++) and so on
    a_z >= 0 ? sign_z = 1 : sign_z = 0;
    a_y >= 0 ? sign_y = 1 : sign_y = 0;

    // Quadrants associated with the signs combinations
    // sign_z = sinθ, sign_y = cosθ... because r = cos(θ)i + sin(θ)j, vector representing circuit in a circular path
    if( sign_y == 1 && sign_z == 1 ) signs_quadrant = 1; // First quadrant, ++
    if( sign_y == 0 && sign_z == 1 ) signs_quadrant = 2; // Second quadrant, -+
    if( sign_y == 0 && sign_z == 0 ) signs_quadrant = 3; // Third quadrant, --
    if( sign_y == 1 && sign_z == 0 ) signs_quadrant = 4; // Fourth quadrant, +-

    // "Push back" all items before putting any new items
    for( int i=n-1; i>0; i-- ){
        quadrants[i] = quadrants[i-1];
    }
    // Adds the last quadrant to the first item of the vector everytime
    quadrants[0] = signs_quadrant;

    /* Quadrant detector */
    /* Use statistical mode to know which is the current quadrant (works like a filter, filters noisy data)  */
    /* ------------------------------------------------------ */

    // Counts ocurrences, to see which one appeared most often
    for( int i=0; i<n; i++ ){
        quadrant_counts[quadrants[i]]++; // Updates number of times the specific quadrant appears on the vector list
    }

    // Checks map to see that there aren't repeated modes, or no mode at all.
    // If at least two quadrants appeared the same number of times, exit function
    // O(n^2) operation...
    for( int quadrant=1; quadrant<=4; quadrant++ ){
        if( quadrant_counts[quadrant] == 0 ) continue; // Do not compare zeroes!
        for( int quadrant_comp=1; quadrant_comp<=4; quadrant_comp++ ){
            if( quadrant == quadrant_comp ) continue; // Don't compare quadrant_counts[1] == quadran_counts[1] for instance, it will obviously be the same num.
            if( quadrant_counts[quadrant] == quadrant_counts[quadrant_comp] ){
                // If two numbers were found to be the same
                return; // exits fundtion
            }
        }

    }

    // Finds mode
    // Iterates through the map container to find the mode
    for( int quadrant=1; quadrant<=4; quadrant++ ){
        // Finds maximum in the quadrant counter map, and use maximum to find the mode (the one that appeared most often)
        if ( quadrant_counts[quadrant] > maxnum ){
            maxnum = quadrant_counts[quadrant];
            stat_mode = quadrant; // Quadrant that appeared the most
        }
    }

    current_quadrant = stat_mode; // Current quadrant = the mode of the vector list, the quadrant that appeared the most

}


// Was a revolution detected?
void Rotational_motion::updates_revs(){
    // Decide how many quadrants will be saved in the quadrants' vector to take average (mode)
    // Note that that "average" becomes irrelevant at high speeds, therefore n = 1 should be given to disable that average
    int n = 15; // Number of samples to analyze and take "mode" of
    const int pattern_check = 4; // How correct has to be the pattern

    int truevals_cc = 0; // True values for counter clockwise pattern, if truevals = 4, a counter-clockwise revolution was detect, that means that all I,II,III,IV are true.
    int truevals_cw = 0; // True values for clockwise pattern, if truevals_cw = 5, then a revolution was detected in the clockise direction
    bool rev_detected = false; // was a revolution detected?

    // Change number of samples to analyze depending on the speed
    // Makes speed detection more stable. By changing the number of samples being analyzed depending on the speed makes sure that
    // We are not "averaging" more samples that the ones being thrown by the accelerometer when the speed is high enough.
    if( RPM > 35 && RPM <= 50 ) n = 9;
    else if( RPM > 50 && RPM <= 60 ) n = 7;
    else if( RPM > 60 && RPM <= 70 ) n = 5;
    else if( RPM > 70 ) n = 3;
    else n = 15; // When it is really slow
    check_quadrant(n); // Checks which quadrant we are on right no

    // If too much time has passed since epoch (since last revolution was detected), resets everything
    if( ((timer1->get_thistime()/1000) > 4.0) && period != 0 ) {
        quadrant_cycles[0] = 0;
        quadrant_cycles[1] = 0;
        quadrant_cycles[2] = 0;
        quadrant_cycles[3] = 0;
        period = 0;
        RPM = 0;
        *kph = 0;
        *mph = 0;
    }

    if( current_quadrant != quadrant_change ){ // Just record changes
        quadrant_change = current_quadrant; // Gets current quadrant

        // "Push back" other elements to let the new one in
        for( int i=3;i>0; i-- ){
            quadrant_cycles[i] = quadrant_cycles[i-1];
        }
        quadrant_cycles[0] = quadrant_change; // Store the new quadrant in the first element of quadrant cycles array

        // Counter-clockwise
        if( quadrant_cycles[0] == 1 ) truevals_cc++;
        if( quadrant_cycles[1] == 2 ) truevals_cc++;
        if( quadrant_cycles[2] == 3 ) truevals_cc++;
        if( quadrant_cycles[3] == 4 ) truevals_cc++;
        // Clockwise
        if( quadrant_cycles[0] == 4 ) truevals_cw++;
        if( quadrant_cycles[1] == 3 ) truevals_cw++;
        if( quadrant_cycles[2] == 2 ) truevals_cw++;
        if( quadrant_cycles[3] == 1 ) truevals_cw++;


        // If most of the quadrants are in the right place, either for CC, or CW direction, a revolution was detected
        //RPM < rpm_change ? pattern_check = 3 : pattern_check = 4; // Check for the full complete pattern or just most of them, depending on the speed
        truevals_cc >= pattern_check || truevals_cw >= pattern_check ? rev_detected = true : rev_detected = false;
        //if( truevals_cc == pattern_check && truevals_cw == pattern_check ) timer1->timer_start();

        // Revolution detected
        if( rev_detected ){
            revolutions++; // Revolution detected, so add one
            quadrant_cycles[3] = 0; quadrant_cycles[2] = 0;  quadrant_cycles[1] = 0;  // Restarts all stored quadrants too, except last onw
            timer1->timer_stop(); // Stops timer

            // How much time passed since the last revolution was detected? Use this to calculate RPM
            period = timer1->get_deltatime()/1000.00; // period in seconds
            period != 0 ? RPM = (1/period)*60 : RPM = 0; // 1/period = rev/s, * 60 = rev/min (if period = 0, RPM = 0)
            *kph = ((*wheel_diameter*(*gear_ratio)) * PI * RPM)/39.370/1000.00*60; // (gear inch)*PI*RPM = inches traveled per minute, 1meter = in/39.370
            *mph = *kph*0.62137; // Miles per hour
            rev_detected = false;
            timer1->timer_start(); // Starts timer
         }
        // Reset values
        truevals_cc = 0;
        truevals_cw = 0;

    }

}

