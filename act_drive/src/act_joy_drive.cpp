#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "act_drive/Act_drive.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

class Ps3Joy
{
protected:
    int lock;
    std_msgs::Int16 right_light_status, left_light_status, front_light_status, back_light_status, joy_mode;
    int left_toggle, right_toggle, start_toggle;
    double drive_scale, turn_scale;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub_left_light, pub_right_light, pub_back_light, pub_front_light, pub_joy_mode;
    ros::Rate r;
    act_drive::Act_drive drive_msg;
    
public:
    Ps3Joy() : lock(1), r(10.0){
        pub = n.advertise<act_drive::Act_drive>("act_base/act_drive", 1);

        pub_left_light = n.advertise<std_msgs::Int16>("act_base/left_light_control", 1);
        pub_right_light = n.advertise<std_msgs::Int16>("act_base/right_light_control", 1);
        pub_front_light = n.advertise<std_msgs::Int16>("act_base/front_light_control", 1);
        pub_back_light = n.advertise<std_msgs::Int16>("act_base/back_light_control", 1);
        pub_joy_mode = n.advertise<std_msgs::Int16>("act_base/joy_mode", 1);

        sub = n.subscribe("act_base/joy", 1, &Ps3Joy::update, this);
        ros::param::param<double>("~drive_scale", drive_scale, 1.5);
        ros::param::param<double>("~turn_scale", turn_scale, 1.5);


        right_light_status.data = 0;
        left_light_status.data = 0;
        back_light_status.data = 0;
        front_light_status.data = 0;

        drive_msg.linear = 0.0;
        drive_msg.angular = 0.0;
        drive_msg.lift = 0.0;
        while(n.ok()){
            ros::spinOnce();
	          joy_mode.data = !lock;
	          pub_joy_mode.publish(joy_mode);
            r.sleep();
        }
    }

    void update(const sensor_msgs::Joy::ConstPtr& joymsg){
        if(joymsg->buttons[3] > 0){
            if(start_toggle == 0){
                start_toggle = joymsg->buttons[3];
                lock = !lock;
            }
        }
        else{
            start_toggle = 0;
        }

 
        if(!lock){
            drive_msg.linear = drive_scale * joymsg->axes[1];
            drive_msg.angular = turn_scale * joymsg->axes[2];
            drive_msg.lift = joymsg->buttons[4] - joymsg->buttons[6];
            pub.publish(drive_msg);
            if(joymsg->buttons[10] > 0){
                if(left_toggle == 0){
                    left_toggle = joymsg->buttons[10];
                    left_light_status.data = 2 * !left_light_status.data;
                    pub_left_light.publish(left_light_status);
                }
            }
            else{
                left_toggle = 0;
            }
            if(joymsg->buttons[11] > 0){
                if(right_toggle == 0){
                    right_toggle = joymsg->buttons[11];
                    right_light_status.data = 2 * !right_light_status.data;
                    pub_right_light.publish(right_light_status);
                }
            }
            else{
                right_toggle = 0;
            }
            if(joymsg->axes[1] < 0){
                front_light_status.data = 1;
                back_light_status.data = 1;
                pub_front_light.publish(front_light_status);
                pub_back_light.publish(back_light_status);
            }
            else{
                front_light_status.data = 0;
                back_light_status.data = 0;
                pub_front_light.publish(front_light_status);
                pub_back_light.publish(back_light_status);
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "act_joy_drive");
    Ps3Joy joy;
    return 0;
}
