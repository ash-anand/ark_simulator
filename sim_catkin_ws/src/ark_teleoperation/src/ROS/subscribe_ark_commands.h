#ifndef SUBSCRIBE_ARK_COMMANDS_H
#define SUBSCRIBE_ARK_COMMANDS_H

#include <ros/ros.h>
#include "mavros_msgs/OverrideRCIn.h"

#include "../shared_memory.h"
#include "../gui/threadgui.h"

class Subscribe_ark_commands
{
public:
    Subscribe_ark_commands(Shared_Memory* shared_memory, threadGUI* t_gui);
    void rosSharedControlCb(const mavros_msgs::OverrideRCInConstPtr &msg);

private:
    Shared_Memory* shared_memory;
    threadGUI* t_gui;
};
#endif // SUBSCRIBE_ARK_COMMANDS_H
