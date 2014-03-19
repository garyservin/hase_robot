#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT  0x44
#define KEYCODE_UP    0x41
#define KEYCODE_DOWN  0x42
#define KEYCODE_E     0x65
#define KEYCODE_D     0x64
#define KEYCODE_SPACE 0x20
#define KEYCODE_PLUS  0x2B
#define KEYCODE_MINUS 0x2D

class TeleopHase
{
public:
  TeleopHase();
  void keyLoop();
private:
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  double linear_max, angular_max;
  ros::Publisher twist_pub_;
};

TeleopHase::TeleopHase():
  linear_(0.3),
  angular_(0.3),
  linear_max(1.0),
  angular_max(1.0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("commands/velocity", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_hase");
  TeleopHase teleop_hase;

  signal(SIGINT,quit);

  teleop_hase.keyLoop();

  return(0);
}


void TeleopHase::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the hase.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    //linear_ = angular_ = 0;
    // Add a temporary variable to hold linear velocity

    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        if (angular_ < 0.0)
            angular_ = -angular_;
        linear_ = 0.0;
        angular_ = 0.6;//angular_;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        if (angular_ < 0.0)
            angular_ = -angular_;
        linear_ = 0.0;
        angular_ = -0.6;//-angular_;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("UP");
        if (linear_ < 0.0)
            linear_ = -linear_;
        linear_ = linear_;
        angular_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
        if (linear_ < 0.0)
            linear_ = -linear_;
        linear_ = -linear_;
        angular_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_E:
        ROS_DEBUG("ENABLE");
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DISABLE");
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_PLUS:
        ROS_DEBUG("PLUS");
        if(linear_ < linear_max)
        {
          linear_ += 0.1;
        }
        dirty = true;
        break;
      case KEYCODE_MINUS:
        ROS_DEBUG("MINUS");
        if(linear_ > -linear_max)
        {
          linear_ -= 0.1;
        }
        dirty = true;
        break;
    }

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);
      dirty=false;
    }
  }
  return;
}
