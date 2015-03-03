#include <ros.h>
#include <std_msgs/Bool.h>

class AutoShutDown
{
  public:
    AutoShutoff(ros::NodeHandle nh);
    void callback(const std_msgs::Bool& off);
    void run();
  private:
    int _pin = 4;
    boolean turn_off;
    ros::Subscriber<std_msgs::Bool> sub("shutoff", &this.callback);
};
