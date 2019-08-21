#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <pid.h>
#include <selfie_drone/MsgState.h>

geometry_msgs::PoseStamped pose;
double current_x, current_y, current_z;

void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    pose = *msg;
}

void errCB(const selfie_drone::MsgState::ConstPtr &msg){
    current_x = msg->box_size;
    current_y = msg->x_mid;
    current_z = msg->y_mid;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "selfie_pid");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/camila1/mavros/local_position/pose", 10, &poseCB);
    ros::Subscriber err_sub = nh.subscribe("/data_state", 10, &errCB);
    ros::ServiceClient goto_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetpointLocal>("/multi_setpoint_local");
    ros::Rate rate(10);

    PIDController<double> pid_x, pid_y, pid_z;
    pid_x.setDt(0.1);

    int cnt =0;
    double target_x, target_y, target_z;
    double control_x, control_y, control_z;
    target_x = 0.07;
    target_y = 0.5;
    target_z = 0.618;

    while( ros::ok() ){
        double kp, ki, kd;
        nh.getParam("pid/kp", kp);
        nh.getParam("pid/ki", ki);
        nh.getParam("pid/kd", kd);
        pid_x.setKp(kp);    
        pid_y.setKi(ki);    
        pid_z.setKd(kd);    

        control_x = pid_x.calculate(target_x, current_x);
        control_y = pid_y.calculate(target_y, current_y);
        control_z = pid_z.calculate(target_z, current_z);

        swarm_ctrl_pkg::srvMultiSetpointLocal msg;
        msg.request.formation = "POINT";
        msg.request.x = pose.pose.position.x + control_x;
        msg.request.y = pose.pose.position.y + control_y;
        msg.request.z = pose.pose.position.z + control_z;
        
        if(cnt%10 == 0){
            ROS_INFO_STREAM("control_x : " << control_x);
            ROS_INFO_STREAM("control_y : " << control_y);
            ROS_INFO_STREAM("control_z : " << control_z);
        }

        if (goto_client.call(msg) && msg.response.success)
            ;
        else
            ROS_ERROR("Failed to call multi_setpoint_local service.");
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}