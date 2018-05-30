#include "subscribe_ark_commands.h"

Subscribe_ark_commands::Subscribe_ark_commands(Shared_Memory* shared_memory, threadGUI* t_gui)
{
    this->shared_memory = shared_memory;
    this->t_gui = t_gui;    
}

void Subscribe_ark_commands::rosSharedControlCb(const mavros_msgs::OverrideRCInConstPtr &msg)
{
    if (this->shared_memory->getSharedControl() && this->shared_memory->getOverride())
    {
        this->t_gui->gui->channel12->setXValue(msg->channels[0]);
        this->t_gui->gui->channel12->setYValue(msg->channels[1]);
        this->t_gui->gui->channel34->setYValue(msg->channels[2]);
        this->t_gui->gui->channel12->setXValue(msg->channels[3]);        
    }
}
