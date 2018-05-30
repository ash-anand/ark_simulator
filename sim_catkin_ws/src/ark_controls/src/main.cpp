//Qt
#include <QApplication>

#include "shared_memory.h"
#include "gui/threadgui.h"
#include "ROS/thread_ros.h"
#include "ROS/mavros_setstreamrate.h"
#include "ROS/subscribe_mavros_state.h"
#include "ROS/subscribe_pid_errors.h"
#include "ROS/subscribe_joystick.h"
#include <dynamic_reconfigure/server.h>
#include <ark_controls/VariablesConfig.h>

void dr_callback(ark_controls::VariablesConfig &config, uint32_t level, Subscribe_pid_errors *pid_errors) {
    pid_errors->kpx = config.kpx;
    pid_errors->kix = config.kix;
    pid_errors->kdx = config.kdx;
    pid_errors->kpy = config.kpy;
    pid_errors->kiy = config.kiy;
    pid_errors->kdy = config.kdy;
    pid_errors->kppsi = config.kppsi;
    pid_errors->kipsi = config.kipsi;
    pid_errors->kdpsi = config.kdpsi;
    pid_errors->accelmax_x = config.accelmax_x;
    pid_errors->accelmax_y = config.accelmax_y;
    pid_errors->deaccelmax_x = config.deaccelmax_x;
    pid_errors->deaccelmax_y = config.deaccelmax_y;
    pid_errors->vmax_x = config.vmax_x;
    pid_errors->vmax_y = config.vmax_y;
    pid_errors->inr = config.inr;
    pid_errors->x3 = (config.vmax_x * config.vmax_x)/(2*config.deaccelmax_x) + config.inr;
    pid_errors->y3 = (config.vmax_y * config.vmax_y)/(2*config.deaccelmax_y) + config.inr;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ark_controls");
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

    Subscribe_pid_errors pid_errors(share_memory, t_gui);
    ros::Subscriber pid_errors_sub = n.subscribe("/ark/pid_errors",
                                                   1,
                                                   &Subscribe_pid_errors::pidErrorsCb,
                                                   &pid_errors);

    Subscribe_joystick joystick(share_memory, t_gui);
    ros::Subscriber joystick_sub = n.subscribe("/mavros/rc/joystick",
                                                   1,
                                                   &Subscribe_joystick::joystickCb,
                                                   &joystick);

    dynamic_reconfigure::Server<ark_controls::VariablesConfig> server;
    dynamic_reconfigure::Server<ark_controls::VariablesConfig>::CallbackType f;

    f = boost::bind(&dr_callback, _1, _2, &pid_errors);
    server.setCallback(f);


    a.connect(&a, SIGNAL(lastWindowClosed()), t_gui->gui, SLOT(on_closed_event()));

    a.exec();
}
