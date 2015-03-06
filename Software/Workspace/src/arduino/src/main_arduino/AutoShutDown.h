#include <std_msgs/Bool.h>

class AutoShutDown
{
  public:
    void messageCb(const std_msgs::Bool& off) {
      if (off.data) 
      {
        digitalWrite(this->pin, LOW);
      }
      else 
      {
        digitalWrite(this->pin, HIGH);
      }
    }

    AutoShutDown(int pin) {
      this->pin = pin;
    }
  private:
    int pin = 4;
};

