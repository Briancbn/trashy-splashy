#define STEER_RATIO     -0.080
#define ENCODER_RATIO   -0.000165

#define FRONT_BACK_L    1.05


class Base_Odom
{
protected:
    ros::NodeHandle n;
    ros::Subscriber steer_sub, encoder_sub, drive_sub;
    ros::Publisher odom_pub;
    ros::Rate r;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    act_drive::Act_drive cmd;


    double vx, vy, vth, m, rad, newm;
    double x, y, z, th, dt, t;

    double counts;
    double new_counts;
    double steer;
    double encoder_ratio;
    double steer_ratio;
    double front_back_l;

    const double deg_to_rad(const double deg){
        return M_PI / 180 * deg;
    }

    const double rad_to_deg(const double rad){
        return rad * 180 / M_PI;
    }

    const double steer_to_rad(const int steer){
        double rad = deg_to_rad((double)steer * steer_ratio);
        return rad;
    }

    const double counts_to_m(const int counts){
        double m = (double)counts * encoder_ratio;
    }


public:
    Base_Odom(double _steer_ratio, double _encoder_ratio, double _front_back_l);
    void update_counts(const std_msgs::Int32::ConstPtr& msg);
    void update_steer(const std_msgs::Float32::ConstPtr& msg);
    void update_cmd(const act_drive::Act_drive::ConstPtr& msg);
    void update_position();
    void pub_tf();
    void pub_odom();

};

