

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define LINEAR_VELOCITY_SCALE 1
#define ANGULAR_VELOCITY_SCALE 1

#define LEFT_JOY_HORIZONTAL_INDEX	0
#define LEFT_JOY_VERTICAL_INDEX		1
class TeleopPlatform
{
public:
  TeleopPlatform();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int status_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};

TeleopPlatform::TeleopPlatform()
{
  
  nh_.param("status", status_, status_);
  
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 3);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopPlatform::joyCallback, this);

}

void TeleopPlatform::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z =   LINEAR_VELOCITY_SCALE*joy->axes[LEFT_JOY_HORIZONTAL_INDEX];
  twist.linear.x  = LEFT_JOY_VERTICAL_INDEX*joy->axes[LEFT_JOY_VERTICAL_INDEX];
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_hunter");
  TeleopPlatform teleop_hunter;

  ros::spin();
}
