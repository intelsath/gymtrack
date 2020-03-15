#include <iostream>
#include <cstdlib>
#include <fstream>
#include <unistd.h> // usleep delay
#include <cstring>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <map>
#include <ctime>
#include <chrono>
#include <cassert> // troubleshooting

#define PI 3.14159265359
#define SAMPLE_PERIOD 0.01 // Inverse of sampling rate, duration of each sample

#include "motion.h"
#include "timer.h"
#include "timer.cpp"
#include "rotation.h"
#include "rotation.cpp"
#include "linearmotion.h"
#include "linearmotion.cpp"



using namespace std;


/* NOTE TO SELF:
    Fix the reading values, you can do:
        file >> double >> char >> double;
*/

Sense_motion::Sense_motion(){
    a_x = 0.0;
    a_y = 0.0;
    a_z = 0.0;
    tilt_y = 0.0;
    tilt_x = 0.0;
}

void Sense_motion::separates_values(string data){

    // Change to array string
    int size = data.size(); // size of string
    const char *chars = data.c_str();
    vector<char> serial_data;
    vector<string> values;
    string tmp;
    int num_elem = 0; // Number of variables received, number of elements that should be in the vector.
    timer action; //


    // Puts everything on a char vector
    for(int i=0; i<=size;i++){
        serial_data.push_back(chars[i]);
    }

    // Separates values into a string vector
    for( int i=0; i<=size;i++ ){

        if( serial_data[i] != ',' ){
            tmp += serial_data[i];
        }

        if( serial_data[i] == ',' ){
            values.push_back(tmp);
            //continue;
            tmp = "";
            num_elem++;
        }
    }

    // Makes sure every element was recorded correctly, if not, just keep last data
    if( values.size() >= num_elem ){
        a_x = atof(values[0].c_str());
        a_y = atof(values[1].c_str());
        a_z = atof(values[2].c_str());
        tilt_y = atof(values[3].c_str());
        tilt_x = atof(values[4].c_str());
        //action.assigns_time(atof(values[5].c_str())); // delta time gets updated

    }

}

// Executes python script and reads seria data from text file
void Sense_motion::read_serial(){
    // Don't leave loop until new data is found
    string old_tmp;
    string string_data;


    // Reads what's on file first to know if the incoming data is new
    ifstream file_data("data.txt");
    file_data >> old_tmp;
    file_data.close();

    // Waits for new data! An optional parity sample bit CAN be added at the end of the string sent by the MCU to differentiate repeated data with old data
    do{
        //system("python motion.py"); // Executes python script that writes to file

        /* Reads file */
        ifstream file("data.txt");
        file >> string_data;
        file.close();

        if( string_data.length() <= 0 ) return; // Ignores empty data, useful when program tries to read file when it is being modified

        separates_values(string_data); // Separates values and stores them in variables as double

        // Removes gravity according to tilt : cos(theta)*gravity = z, sin(theta)*gravity = y, where gravity = g's
        // Since angle is -90 <= tilt <= 90, cosine is always positive, and sine can be positive or negative
        // Note: Z-axis is usually negative, gravity  = -1
        /* Eliminates components of gravity when tilting in on X and Y axes */
        a_z += sin(tilt_y*PI/180) /*- (1-cos(tilt_x*PI/180) )*/; // Assumes circuit is always facing up (gravity in z-axis is negative)
        a_y -= cos(tilt_y*PI/180); // converts from degrees to radians

        //a_x -= sin(tilt_x*PI/180); // Removes gravity component in the x axis too

        // Ignores really small values
        //if( a_x > -0.05 && a_x < 0.05 ) a_x = 0.00;
        //if( a_y > -0.05 && a_y < 0.05 ) a_y = 0.00;
        //if( a_z > -0.05 && a_z < 0.05 ) a_z = 0.00;

    }while(old_tmp == string_data); // Waits for new data

}

//double tmp=0;
//double tmp2=0;
//double sum,avg_speed;
int main(){

    // Pedaling, wheel's diameter in inches, and gear's ratio (common one: 26 inches, 2:1 ratio)
    Rotational_motion pedaling(26,2); // Pedaling object, to detect rotation
    Linear_motion running; // running object

    //ofstream fileout("log.txt"); // keeps log of z values of the acceleration to analyze them (like in matlab)

    // Menu pptions, select a game mode
    char select_menu;
    cout << "GymTrack 1.0 beta " << endl;
    cout << "a) Cycling" << endl << "b) Running" << endl;
    cout << "\n\n" << "Please select a game mode to enable your Gymtrack: ";
    cin >> select_menu;

    ofstream fileout("log.txt");

    while(1){

        switch(select_menu){
            case 'a':
                // Retrieves info sent to serial port by circuit
                pedaling.read_serial();

                // Reads rotation
                pedaling.updates_revs();

                cout << "Revs: " << pedaling.get_revolutions() << ", " << "RPM: " << pedaling.get_RPM() << ", ";
                cout << "KPH: " << pedaling.get_kph() << ", " << "MPH: " << pedaling.get_mph() << endl;

                //fileout << pedaling.a_zval() << "," << pedaling.a_yval() << endl; // Saves to file
                //fileout.flush();
            break;

            case 'b':


                // Retrieves info sent to serial port by circuit
                running.read_serial();
                //running.tmp_read_samples(); // Temporary function replacing read_serial()
                //if( running.tmp_read_samples_varcount > running.tmp_read_samples_vector_z.size() ) goto stop; // Stop program, tmp_

                // Detect peaks
                running.detect_steps();
                // Detect speed
                running.estimate_speed();

                //if( tmp != running.get_totalsteps() || tmp2 != running.get_mph() ){
                cout << "Steps: " << running.get_totalsteps() << ", " << "speed: " << running.get_mph() << endl;
                //sum+=running.get_mph();
                //}
                //tmp = running.get_totalsteps();
                //tmp2 = running.get_mph();





            break;

           /* case 'c':
                pedaling.read_serial();
                cout.precision(3);
                cout << fixed << pedaling.a_xval() << "," << pedaling.a_yval() << "," << pedaling.a_zval() << ", ";
                cout << fixed << pedaling.tiltY_val() << endl;

                //cout << "Revolutions: " << testrevs << endl;

                fileout << pedaling.a_zval()  << "," << pedaling.a_yval() << "\r\n"; // Writes z and y axis acceleration values
                fileout.flush();


            break;
            */

            default:
                // Do nothing
            break;


            //cout << running.a_zval() << ", " << running.a_yval() << endl;
            //fileout << running.a_zval()  << "," << running.a_yval() << "\r\n"; // Writes z and y axis acceleration values
        }


    }


    //stop: // Stops program, tmp_
    //avg_speed = sum/running.get_totalsteps();
    //cout << "Average speed: " << avg_speed << endl;
    return 0;

}
