#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "base_drive/Base_drive.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

class SteamJoy
{
protected:
    bool lock, lock_toggle, e_stop_toggle;
    std_msgs::Bool e_stop_status;
    double drive_scale, turn_scale;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub_e_stop;
    ros::Rate r;
    base_drive::Base_drive drive_msg;
    
public:
    SteamJoy() : lock(1), r(10.0){
        pub = n.advertise<base_drive::Base_drive>("base_drive", 1);

        pub_e_stop = n.advertise<std_msgs::Bool>("e_stop", 1);

        sub = n.subscribe("joy", 1, &SteamJoy::update, this);
        ros::param::param<double>("~drive_scale", drive_scale, 200);
        ros::param::param<double>("~turn_scale", turn_scale, 1.0);


        e_stop_status.data = true;
        lock = true;
        lock_toggle = false;
        e_stop_toggle = false;
        drive_msg.linear = 0.0;
        drive_msg.angular = 0.0;
        if(n.ok()){
            pub_e_stop.publish(e_stop_status);
            ros::spinOnce();
            r.sleep();
        }
        while(n.ok()){
            ros::spinOnce();
            r.sleep();
        }
    }

    void update(const sensor_msgs::Joy::ConstPtr& joymsg){
        if(joymsg->buttons[7] > 0){
            if(!e_stop_toggle){
                e_stop_toggle = true;
                e_stop_status.data = !e_stop_status.data;
	            pub_e_stop.publish(e_stop_status);
                lock = true;
            }
        }
        else{
            e_stop_toggle = false;
        }

        if(joymsg->buttons[6] > 0){
            if(!lock_toggle){
                lock_toggle = true;
                lock = !lock;
            }
        }
        else{
            lock_toggle = false;
        }


        if(!lock){
            drive_msg.linear = drive_scale * joymsg->axes[1];
            drive_msg.angular = turn_scale * joymsg->axes[3];
            pub.publish(drive_msg);
        }
        else{
            drive_msg.linear = 0;
            drive_msg.angular = 0;
            pub.publish(drive_msg);
        }

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_drive");
    SteamJoy joy;
    return 0;
}
