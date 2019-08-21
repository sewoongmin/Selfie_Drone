#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <pid.h>
#include <selfie_drone/MsgState.h>

geometry_msgs::PoseStamped pose;
selfie_drone::MsgState centroid;

void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pose = *msg;
}

void errCB(const selfie_drone::MsgState::ConstPtr &msg){
    centroid = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "selfie_pid");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/camila1/mavros/local_position/pose", 10, &poseCB);
    ros::Subscriber err_sub = nh.subscribe("/data_state", 10, &errCB);
    ros::ServiceClient goto_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetpointLocal>("/multi_setpoint_local");
    ros::Rate rate(10);

    PIDController<tf2::Vector3> pid;
    pid.setDt(0.1);

    int cnt =0;
    double control_x, control_y, control_z;

    while( ros::ok() ){
        double kp, ki, kd;
        tf2::Vector3 control;
        nh.getParam("/selfie_pid/pid/kp", kp);
        nh.getParam("/selfie_pid/pid/ki", ki);
        nh.getParam("/selfie_pid/pid/kd", kd);
        pid.setKp(kp);

        control = pid.calculate(tf2::Vector3(0.07, 0.5, 0.618), 
            tf2::Vector3(centroid.box_size, centroid.x_mid, centroid.y_mid));

        swarm_ctrl_pkg::srvMultiSetpointLocal msg;
        msg.request.formation = "POINT";
        msg.request.x = pose.pose.position.x + control.getX();
        msg.request.y = pose.pose.position.y + control.getY();
        msg.request.z = pose.pose.position.z + control.getZ();
        
        if(cnt%10 == 0){
            ROS_INFO_STREAM("control_x : " << control.getX());
            ROS_INFO_STREAM("control_y : " << control.getY());
            ROS_INFO_STREAM("control_z : " << control.getZ());
        }
        cnt++;
        
        if (goto_client.call(msg) && msg.response.success)
            ;
        else
           ROS_ERROR("Failed to call multi_setpoint_local service.");
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}