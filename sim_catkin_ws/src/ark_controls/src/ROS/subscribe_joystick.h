#ifndef SUBSCRIBE_JOYSTICK_H
#define SUBSCRIBE_JOYSTICK_H

#include <ros/ros.h>
#include "mavros_msgs/OverrideRCIn.h"

#include "../shared_memory.h"
#include "../gui/threadgui.h"
#include <iostream>

class Subscribe_joystick
{
public:
    Subscribe_joystick(Shared_Memory* shared_memory, threadGUI* t_gui);
    void joystickCb(const mavros_msgs::OverrideRCInConstPtr &msg);

private:
    Shared_Memory* shared_memory;
    threadGUI* t_gui;
    bool turned_off_sc;
};
#endif // SUBSCRIBE_JOYSTICK_H
