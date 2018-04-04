#include "ros/ros.h"
#include "rdr_ugv_control/ugv_msg.h"
#include "sensor_msgs/Joy.h"
#include "act_drive/Act_drive.h"
#include "std_msgs/Int16.h"

#include <cstdio>
#include <ctime>

class toCommand
{
protected: 
    int joy_mode;
    ros::NodeHandle n;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Publisher pub;
    ros::Rate r;
    act_drive::Act_drive drive_msg;

public:
    toCommand():r(10.0){
            pub = n.advertise<act_drive::Act_drive>("act_base/act_drive", 1);
            sub1 = n.subscribe("act_cmd/drive", 1, &toCommand::updatecmd_pub, this);
            sub2 = n.subscribe("act_cmd/lift", 1, &toCommand::updatelift_pub, this);
            sub3 = n.subscribe("act_base/joy_mode", 1, &toCommand::updatestate, this);
            drive_msg.linear = 0;
            drive_msg.angular = 0;
            drive_msg.lift = 0;
            joy_mode = 0;
            while(n.ok()){
                ros::spinOnce();
                r.sleep();
            }
    }

    void updatecmd_pub(const rdr_ugv_control::ugv_msg::ConstPtr& msg){
        drive_msg.linear = msg->LinearVelocity;   //linear range : -1.5 to 1.5
        drive_msg.angular = msg->SteeringAngle;  //steering angle range: -1.5 to 1.5


        if(!joy_mode){
        
            pub.publish(drive_msg);
        }
        else{
            drive_msg.angular = 0;
            drive_msg.linear = 0;
          }
    }

    void updatelift_pub(const std_msgs::Int16::ConstPtr& msg){
        drive_msg.lift = msg->data;
        pub.publish(drive_msg);
    }

    void updatestate(const std_msgs::Int16::ConstPtr& msg){
	joy_mode = msg->data;
    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_translator");
    toCommand cmd;

    return 0;
}
