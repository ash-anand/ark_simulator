#include <ros/ros.h>
#include "mavros_msgs/OverrideRCIn.h"


class Overrider
{
    ros::NodeHandle nh_;
    ros::Publisher rc_pub;
    ros::Subscriber rc_sub;

public:
    Overrider()
    {
        rc_pub = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
        rc_sub = nh_.subscribe("/mavros/rc/override_relay", 10, &Overrider::overrideCb, this);     
    }

    ~Overrider()
    {
    }
  
private:

    void overrideCb(const mavros_msgs::OverrideRCIn::ConstPtr& msg)
    {
        mavros_msgs::OverrideRCIn input = *msg;
        rc_pub.publish(input);
    }   
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "overrider");
    ros::NodeHandle n;

    Overrider ov;
    ros::spin();
    return 0;
}