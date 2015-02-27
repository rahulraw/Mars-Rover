#include <ros.h>
#include <std_msgs/Bool.h>

class AutoShutDown
{
  public:
    void callback(const std_msgs::Bool& off);
    void AutoShutOff(ros::NodeHandle nh);
    void run();
    boolean turn_off;
  private:
    int _pin = 4;
    ros::Subscriber<std_msgs::Bool> sub;
};
