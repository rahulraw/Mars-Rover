#include <ros.h>
#include <std_msgs/Bool.h>

class AutoShutDown
{
  public:
    void messageCb(const std_msgs::Bool& off);
    void AutoShutOff(ros::NodeHandle nh);
    void run();

  private:
    int _pin = 4;
    boolean turn_off;
};
