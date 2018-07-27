#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <Eigen/Core>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class VSTelop
{
public:
  VSTelop();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  Eigen::Vector2d direction;
  ros::Publisher twist_pub_;
  
};

VSTeleop::VSTeleop():
{
  direction = {0.0, 0.0};
  dir_pub = nh_.advertise<uncalibrated_visual_servoing::KBDirection>("/teleop", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();``
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "VS_teleop");
  VSTeleop VS_teleop;
  signal(SIGINT,quit);

  vs_teleop.keyLoop();
  
  return(0);
}


void VSTeleop::keyLoop()
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
  puts("Use arrow keys to move the arm.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        direction[1] = -1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        direction[1] = 1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        direction[0] = 1.0
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        direction[0] = -1.0;
        dirty = true;
        break;
    }
   

    uncalibrated_visual_servoing::KBDirection kb_direction;
    kb_direction.x = direction[0];
    kb_direction.y = direction[1];
    if(dirty ==true)
    {
      twist_pub_.publish(kb_direction);    
      dirty=false;
    }
  }


  return;
}
