#ifndef SUBSCRIBE_PID_ERRORS_H
#define SUBSCRIBE_PID_ERRORS_H

#include <ros/ros.h>
#include "ark_msgs/PidErrors.h"
#include <math.h>

#include "../shared_memory.h"
#include "../gui/threadgui.h"
#include <sys/time.h>
#include <iostream>

class Subscribe_pid_errors
{
public:
    Subscribe_pid_errors(Shared_Memory* shared_memory, threadGUI* t_gui);
    void pidErrorsCb(const ark_msgs::PidErrorsConstPtr &msg);

    float kpx;
    float kix;
    float kdx;
    float kpy;
    float kiy;
    float kdy;
    float kppsi;
    float kipsi;
    float kdpsi;
    float accelmax_x;
    float accelmax_y;
    float deaccelmax_x;
    float deaccelmax_y;
    float vmax_x;
    float vmax_y;
    float inr;
    float x3;
    float y3;

private:
    Shared_Memory* shared_memory;
    threadGUI* t_gui;

    long prev_time;
    float sumerrorvx;
    float sumerrorvy;
    float sumerrorpsi;
    float preverrorvx;
    float preverrorvy;
    float preverrorpsi;
    float targetv_x;
    float targetv_y;
    long cout_prev_time;

    int channel12y_mid;
    int channel12x_mid;
    int channel34y_mid;
    int channel34x_mid;

    long getMilliSecs();
};
#endif // SUBSCRIBE_PID_ERRORS_H
