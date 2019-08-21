#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <pid.h>
#include <tensorflow_object_detector/MsgState.h>

geometry_msgs::PoseStamped pose;
tf2::Vector3 current;

void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pose = *msg;
}

void errCB(const tensorflow_object_detector::MsgState::ConstPtr &msg){
    current.setX(msg->box_size);
    current.setY(msg->x_mid);
    current.setZ(msg->y_mid);
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
    pid.setKp(2.0);

    tf2::Vector3 target, control_value;
    target.setX(0.07);
    target.setY(0.5);
    target.setZ(0.618);

    while( ros::ok() ){
        control_value = pid.calculate(target, current);
        swarm_ctrl_pkg::srvMultiSetpointLocal msg;
        msg.request.formation = "POINT";
        msg.request.x = pose.pose.position.x + control_value.getX();
        msg.request.y = pose.pose.position.y + control_value.getY();
        msg.request.z = pose.pose.position.z + control_value.getZ();

        if (goto_client.call(msg) && msg.response.success)
            ;
        else
            ROS_ERROR("Failed to call multi_setpoint_local service.");
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}