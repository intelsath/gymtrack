// Timer functions
// Getting time from MCU, its resolution depend on the value MCU gives

void timer::timer_start(){
    epoch = std::chrono::high_resolution_clock::now();

}

void timer::timer_stop(){
    deltatime = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - epoch).count(); // delta time
}

double timer::get_thistime(){
   return chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - epoch).count(); // time sicne epoch
}
