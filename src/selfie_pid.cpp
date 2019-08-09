#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <selfie_drone/MsgState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pid.h>

tf2::Vector3 current;

void errCB(const selfie_drone::MsgState::ConstPtr &msg){
    current.setX(msg->box_size);
    current.setY(msg->x_mid);
    current.setZ(msg->y_mid);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "selfie_pid");
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber err_sub = nh.subscribe("/centroid", 10, &errCB);
    ros::Rate rate(10);

    PIDController<tf2::Vector3> pid;
    pid.setDt(0.1);
    pid.setKp(1.0);

    tf2::Vector3 target, control_value;
    target.setX(0.3);
    target.setY(0.5);
    target.setZ(0.618);

    geometry_msgs::Twist twist;

    while( ros::ok() ){
        control_value = pid.calculate(target, current);
        twist.linear.x = control_value.getX();
        twist.linear.y = control_value.getY();
        twist.linear.z = control_value.getZ();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}