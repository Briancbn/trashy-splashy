#include "ros/ros.h"
#include "act_drive/Act_drive.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <cstdio>
#include <ctime>

class toCommand
{
protected: 
    ros::NodeHandle n;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Publisher pub;
    ros::Rate r;
    act_drive::Act_drive drive_msg;
    std::clock_t startcmd;
    std::clock_t lastcmd;
    std::clock_t startlift;
    std::clock_t lastlift;


public:
    toCommand():r(10.0){
            pub = n.advertise<act_drive::Act_drive>("act_base/act_drive", 1);
            sub1 = n.subscribe("act_base/cmd_vel", 1, &toCommand::updatecmd_pub, this);
            sub2 = n.subscribe("act_base/lift_vel", 1, &toCommand::updatelift_pub, this);
            drive_msg.linear = 0;
            drive_msg.angular = 0;
            drive_msg.lift = 0;
            startcmd = std::clock();
            startlift = std::clock();
            while(n.ok()){
                lastcmd = std::clock();
                lastlift = std::clock();
                ros::spinOnce();
                double durationcmd = lastcmd - startcmd;
                double durationlift = lastlift - startlift;
                if (durationcmd > 350 && durationcmd < 1000){
                    publishcmd0();
                }

                if (durationlift > 350 && durationlift < 1000){
                    publishlift0();
                }

                r.sleep();
            }
    }

    void updatecmd_pub(const geometry_msgs::Twist::ConstPtr& msg){
        drive_msg.linear = msg->linear.y;
        drive_msg.angular = msg->angular.z;
        pub.publish(drive_msg);
        startcmd = std::clock();
    }

    void updatelift_pub(const std_msgs::Int16::ConstPtr& msg){
        drive_msg.lift = msg->data;
        pub.publish(drive_msg);
        startlift = std::clock();
    }

    void publishcmd0(){
        drive_msg.linear = 0;
        drive_msg.angular = 0;
        pub.publish(drive_msg);
    }

    void publishlift0(){
        drive_msg.lift = 0;
        pub.publish(drive_msg);
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_translator");
    toCommand cmd;

    return 0;
}
