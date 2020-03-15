#ifndef TIMER_H
#define TIMER_H

//class Sense_motion;

using namespace std;
/* Timer built using chrono library, C++11 required*/
class timer : public Sense_motion{
    public:
        void timer_start();
        void timer_stop();
        double get_thistime(); // Current time since epoch
        double get_deltatime(){ return deltatime; }
    private:
        double deltatime; // Time it took from timer_start to timer_stop
        std::chrono::high_resolution_clock::time_point epoch; // Chrono epoch time
};



#endif
