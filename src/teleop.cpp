#include <ros/ros.h>
#include "uncalibrated_visual_servoing/Teleop.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <Eigen/Core>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_T 0x74
#define KEYCODE_O 0x6F
#define KEYCODE_C 0x63

class VSTeleop
{
public:
  VSTeleop();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  Eigen::Vector2d direction;
  ros::Publisher dir_pub;
  
};

VSTeleop::VSTeleop()
{
  direction = {0.0, 0.0};
  dir_pub = nh_.advertise<uncalibrated_visual_servoing::Teleop>("/teleop", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "VS_teleop");
  VSTeleop VS_Teleop;
  signal(SIGINT,quit);

  VS_Teleop.keyLoop();
  
  return(0);
}


void VSTeleop::keyLoop()
{
  char c;
  bool dirty=false;
  // char * mode;

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
  puts("Use arrow keys to move the arm. q to quit, t to toggle modes");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    // ROS_INFO("value: 0x%02X\n", c);
    // ROS_DEBUG("value: 0x%02X\n", c);
      
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        direction[0] = -1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        direction[0] = 1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        direction[1] = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        direction[1] = -1.0;
        dirty = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("QUIT");
        direction[0] = -9.0;
        dirty = true;
        break;
      case KEYCODE_O:
        ROS_DEBUG("OPEN");
        direction[0] = -5.0;
        dirty = true;
        break;
      case KEYCODE_C:
        ROS_DEBUG("CLOSE");
        direction[0] = 5.0;
        dirty = true;
        break;
      case KEYCODE_T:
        ROS_DEBUG("TOGGLE");
        direction[1] = -5.0;
        dirty = true;
        break;
          
    }
    uncalibrated_visual_servoing::Teleop kb_direction;
    kb_direction.dir_2D.push_back(direction[0]);
    kb_direction.dir_2D.push_back(direction[1]);
    if(dirty ==true)
    {
      dir_pub.publish(kb_direction);
      direction = {0.0, 0.0};    
      dirty=false;
    }
  }


  return;
}
