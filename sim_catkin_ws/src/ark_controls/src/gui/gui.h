#ifndef GUI_H
#define GUI_H

//Qt
#include <QtGui>

//standard
#include <iostream>

//ROS
#include <ros/ros.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../shared_memory.h"
#include "../ROS/thread_ros.h"

#include "RC_Widget.h"

class GUI:public QWidget
{
    Q_OBJECT

public:
    GUI(Shared_Memory* share_memory, Thread_ROS* t_ros);
    ~GUI();
    void updateThreadGUI();
    bool getEnd_thread();
    void set_check_shared_control(bool state);
    RC_Widget* channel12;
    RC_Widget* channel34;
    std::vector<int> rc_maxlimits;
    std::vector<int> rc_minlimits;
    bool record_pid;

private:
    Shared_Memory* share_memory;
    void config_RC();
    void config_mode();

    QGridLayout* mainLayout;

    QLabel* label_mode;
    QComboBox* combo_mode;

    QLabel* label_pitch;
    QLabel* label_roll;
    QLabel* label_throttle;
    QLabel* label_yaw;

    QCheckBox* check_override;
    QCheckBox* check_shared_control;

    Thread_ROS* t_ros;
    bool end_thread;
    ///
signals:
    void signal_updateGUI();

public slots:
    void on_updateGUI_recieved();
    void on_combo_mode_changed();
    void on_check_override_changed();
    void on_check_shared_control_changed();
    void keyPressEvent(QKeyEvent *e);
    void on_closed_event();
};

#endif // GUI_H
