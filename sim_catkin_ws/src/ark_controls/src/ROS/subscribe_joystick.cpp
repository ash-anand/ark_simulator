#include "subscribe_joystick.h"

Subscribe_joystick::Subscribe_joystick(Shared_Memory* shared_memory, threadGUI* t_gui)
{
    this->shared_memory = shared_memory;
    this->t_gui = t_gui;
    turned_off_sc = false;
}

void Subscribe_joystick::joystickCb(const mavros_msgs::OverrideRCInConstPtr &msg)
{
    /*
    If RC_Override is true:
        if (sc true and btn 1 pressed):
            sc = false
        elseif (sc true and btn 1 not pressed):
            sc = true
        if (sc false)
            set axis position from joy
    */
    if (this->shared_memory->getOverride())
    {
        if (this->shared_memory->getSharedControl() && msg->channels[5] == 2000)
        {
            this->t_gui->gui->set_check_shared_control(false);
            turned_off_sc = true;
        }
        else if (turned_off_sc && msg->channels[5] == 1000)
        {
            this->t_gui->gui->set_check_shared_control(true);
            turned_off_sc = false;
        }
        if (!this->shared_memory->getSharedControl()) 
        {
            //throttle
            this->t_gui->gui->channel34->setYValue(msg->channels[2]);
            //roll
            this->t_gui->gui->channel12->setXValue(msg->channels[0]);
            //pitch
            this->t_gui->gui->channel12->setYValue(msg->channels[1]);
            //yaw
            this->t_gui->gui->channel34->setXValue(msg->channels[3]);
        }

    }
}
