#include <ros.h>
#include <std_msgs/Bool.h>

class AutoShutDown
{
    static void callback(const std_msgs::Bool& off);
  public:
    void AutoShutOff(ros::NodeHandle nh);
    void run();
  private:
    int _pin = 4;
    boolean turn_off;
    ros::Subscriber<std_msgs::Bool> sub;
};
