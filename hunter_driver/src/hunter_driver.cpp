#include <ros/ros.h>



#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include <iostream>

#include <stdlib.h>

#include <geometry_msgs/Twist.h>

/* TEST */
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>


#include <nanotec_n5_driver/NanotecN5Driver.h>
#include <nanotec_n5_driver/NanotecN5Port.h>
#include <cereal_port/CerealPort.h>

#define NODE_ID_RIGHT_MOTOR 0x02
#define NODE_ID_LEFT_MOTOR  0x01


class  HunterDriver
{
public:
    HunterDriver(nanotec::NanotecN5Port * port);
    void stop();
    void platformStateMachine(void);
private:
  int status_;
  void velCallback(const geometry_msgs::Twist twist);
   
  ros::NodeHandle nh_;
   
  ros::Publisher sate_pub_;
  ros::Subscriber state_sub_,vel_sub_;
  
  nanotec::NanotecN5Driver Left_Wheel,Right_Wheel;
   
};

HunterDriver::HunterDriver(nanotec::NanotecN5Port * port)
{
  
   Left_Wheel.initDriver(port,NODE_ID_LEFT_MOTOR);
   Right_Wheel.initDriver(port,NODE_ID_RIGHT_MOTOR);
   
   
   Left_Wheel.WriteObject(CANOPEN_CONTROL_WORD,0x00,MESSAGE_SIZE_2,0x00 ); 
   Right_Wheel.WriteObject(CANOPEN_CONTROL_WORD,0x00,MESSAGE_SIZE_2,0x00);
   
   Left_Wheel.loadDefaultSettings();
   Right_Wheel.loadDefaultSettings();
   
   
   
   vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1,&HunterDriver::velCallback,this);
   nh_.param("status", status_, status_);
}

void HunterDriver::stop(void)
{
  Left_Wheel.stateMachine(FOWARDS,QUICK_STOP,0);
  Right_Wheel.stateMachine(FOWARDS,QUICK_STOP, 0);
}

void HunterDriver::platformStateMachine(void)
{
  Left_Wheel.stateMachine(FOWARDS,0,0);
  Right_Wheel.stateMachine(FOWARDS,0,0);
}

void HunterDriver::velCallback(const geometry_msgs::Twist twist)
{
  // kinematic
  // send velocity to drivers
  double v,w,left_speed,right_speed;
  v = twist.linear.x;
  w = twist.angular.z;
  
  left_speed  = ((v - (DISTANCE_BETWEEN_AXES/2)*w)/TIRE_PERIMETER) ;
  right_speed = -((v + (DISTANCE_BETWEEN_AXES/2)*w)/TIRE_PERIMETER) ; 
  
  Left_Wheel.setTargetVelocity((int32_t)(left_speed *80));
  Right_Wheel.setTargetVelocity((int32_t)(right_speed*80));
  printf("\n[DEBUG] velocity: %lf %lf\n", left_speed,right_speed);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Nanotec_CAN_controller");
    
    
    std::string port_name = "/dev/serial/by-id/usb-LAWICEL_CANUSB_LWRVLSH1-if00-port0";
    nanotec::NanotecN5Port can_port;
    can_port.openPort(port_name, 115200);

    can_port.SendMessage((char*)"S8\r");
    // Open CAN channel
    can_port.SendMessage((char*)"O\r");
    
    HunterDriver driver(&can_port);
    
    
    //nanotec::NanotecN5Driver Left_Wheel(&can_port,NODE_ID_LEFT_MOTOR);
    //nanotec::NanotecN5Driver Right_Wheel(&can_port,NODE_ID_RIGHT_MOTOR);
    
    //Left_Wheel.WriteObject(CANOPEN_CONTROL_WORD,0x00,MESSAGE_SIZE_2,0x00 ); 
    //Right_Wheel.WriteObject(CANOPEN_CONTROL_WORD,0x00,MESSAGE_SIZE_2,0x00);
    
    // STATE MACHINE
    int  state = 0;
    bool flag_bootup_finished_left_wheel = false;
    bool flag_bootup_finished_right_wheel = false;

 
    
    int flow       = FOWARDS;
    int stop 	   = 0;
    int clearfault = 0;
    int start_order= 1;
    int cntr	  =  0;
    
    ros::Rate r(100);
    //Left_Wheel.setTargetVelocity(10);
    
    
    while(ros::ok())
    {
      
      driver.platformStateMachine();
      
//        state=Left_Wheel.stateMachine(flow,stop, clearfault);
//        if(state == CANOPEN_SWITCH_ON_DISABLED && start_order == HOLD)
// 	 flow =HOLD;
      
       //if(state == CANOPEN_OPERATION_ENABLED)
       //{
       //	 start_order = 0;
       //  stop = QUICK_STOP;
	 
       //}
       
        
	
        ros::spinOnce();
	
	
	//printf("\nActual Velocity : %d\n",Left_Wheel.getActualVelocity());
    }
    driver.stop();
    //Left_Wheel.stop();
    //Right_Wheel.stop();
    // Close CAN channel
    
    
     std::cout <<"\nClose CAN channel" << std::endl;
 

}

