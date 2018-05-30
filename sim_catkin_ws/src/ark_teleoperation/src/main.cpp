//Qt
#include <QApplication>

#include "shared_memory.h"
#include "gui/threadgui.h"
#include "ROS/thread_ros.h"
#include "ROS/mavros_setstreamrate.h"
#include "ROS/subscribe_mavros_state.h"
#include "ROS/subscribe_ark_commands.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_erle_teleoperation");
    QApplication a(argc, argv);


    Shared_Memory* share_memory = new Shared_Memory();

    MAVROS_setStreamRate setStreamRate;
    Subscribe_mavros_state mavros_state(share_memory);

    ros::NodeHandle n;
    ros::Subscriber mavros_state_sub = n.subscribe("/mavros/state",
                                                   1,
                                                   &Subscribe_mavros_state::mavrosStateCb,
                                                   &mavros_state);
    Thread_ROS* t_ros = new Thread_ROS(share_memory);
    t_ros->start();
    threadGUI* t_gui = new threadGUI(share_memory, t_ros);
    t_gui->start();

    Subscribe_ark_commands ros_shared_control(share_memory, t_gui);
    ros::Subscriber ark_commands_sub = n.subscribe("/ark/commands",
                                                   1,
                                                   &Subscribe_ark_commands::rosSharedControlCb,
                                                   &ros_shared_control);

    a.connect(&a, SIGNAL(lastWindowClosed()), t_gui->gui, SLOT(on_closed_event()));

    a.exec();
}
