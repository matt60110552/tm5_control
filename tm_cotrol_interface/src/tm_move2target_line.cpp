// picking/src/tm_DL_binpicking/src/pose_estimation.cpp
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>
#include <time.h>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

//TM5 Driver
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"

#include  <tm_msgs/FeedbackState.h>

#define D2R 0.01745329252
#define R2D 57.29577951
#define NUMBER_OF_DOFS    6
using namespace std;

double tm_end_effector_p[6];// x,y,z(m),rx,ry,rz(rad)
double tm_end_effector_j[6];
Eigen::Matrix<double,4,4> T_tm_end_effector;

double tm_target_p[6];
double tm_target_j[6];
int start_cmd=0;

void ToolPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& ToolPose)
{
    tm_end_effector_p[0] = ToolPose->pose.position.x;
    tm_end_effector_p[1] = ToolPose->pose.position.y;
    tm_end_effector_p[2] = ToolPose->pose.position.z;

    tf::Quaternion q(
        ToolPose->pose.orientation.x,
        ToolPose->pose.orientation.y,
        ToolPose->pose.orientation.z,
        ToolPose->pose.orientation.w);
    tf::Matrix3x3 m(q);

    T_tm_end_effector <<   0., 0., 0., tm_end_effector_p[0],
            0., 0., 0., tm_end_effector_p[1],
            0., 0., 0., tm_end_effector_p[2],
            0., 0., 0., 1.;
    Eigen::Matrix<double,3,3> rot;
    m.getRPY(tm_end_effector_p[3], tm_end_effector_p[4], tm_end_effector_p[5]);

}

void ToolPose_Callback2(const tm_msgs::FeedbackState::ConstPtr& ToolPose)
{
    tm_end_effector_p[0] = ToolPose->tool_pose[0];
    tm_end_effector_p[1] = ToolPose->tool_pose[1];
    tm_end_effector_p[2] = ToolPose->tool_pose[2];
    tm_end_effector_p[3] = ToolPose->tool_pose[3];
    tm_end_effector_p[4] = ToolPose->tool_pose[4];
    tm_end_effector_p[5] = ToolPose->tool_pose[5];
}

void Target_Callback(const geometry_msgs::Transform::ConstPtr& TargetPose)
{
    tm_target_p[0] = TargetPose->translation.x;
    tm_target_p[1] = TargetPose->translation.y;
    tm_target_p[2] = TargetPose->translation.z;

    tf::Quaternion q(
        TargetPose->rotation.x,
        TargetPose->rotation.y,
        TargetPose->rotation.z,
        TargetPose->rotation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);


    start_cmd=1;
}

void Startmove_Callback(const std_msgs::Int32::ConstPtr& startmove)
{
    start_cmd=startmove->data;
    ROS_ERROR("start_cmd = %d / %d",start_cmd,startmove->data);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle node_handle;

    ROS_INFO("Subscribe /tool_position");
    ros::CallbackQueue tool_pose;
    // ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/tool_pose", 10, ToolPose_Callback, ros::VoidPtr(), &tool_pose);
    ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<tm_msgs::FeedbackState>("/feedback_states", 10, ToolPose_Callback2, ros::VoidPtr(), &tool_pose);
    ros::Subscriber sub_toolpose = node_handle.subscribe(ops_toolpose);
    ros::AsyncSpinner async_spinner_toolpose(1, &tool_pose);
    async_spinner_toolpose.start();

    ROS_INFO("Subscribe /robot_target");
    ros::CallbackQueue robot_target;
    ros::SubscribeOptions ops_robot_target = ros::SubscribeOptions::create<geometry_msgs::Transform>("/robot_target", 10, Target_Callback, ros::VoidPtr(), &robot_target);
    ros::Subscriber sub_robot_target = node_handle.subscribe(ops_robot_target);
    ros::AsyncSpinner async_spinner_robot_target(1, &robot_target);
    async_spinner_robot_target.start();

    ROS_INFO("Subscribe /start_move");
    ros::CallbackQueue start_move;
    ros::SubscribeOptions ops_start_move = ros::SubscribeOptions::create<std_msgs::Int32>("/start_move", 1, Startmove_Callback, ros::VoidPtr(), &start_move);
    ros::Subscriber sub_start_move = node_handle.subscribe(ops_start_move);
    ros::AsyncSpinner async_spinner_start_move(1, &start_move);
    async_spinner_start_move.start();

    ros::ServiceClient client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    tm_msgs::SetPositions target_pose;
    target_pose.request.motion_type = tm_msgs::SetPositions::Request::LINE_T;
    target_pose.request.velocity = 0.5;//rad/s
    target_pose.request.acc_time = 0.1;
    target_pose.request.blend_percentage = 0;
    target_pose.request.fine_goal  = false;

    std::vector<double> home={0.5134511108398437, 0.16386395263671874, 0.40, -M_PI, 0.0, M_PI/2};

    // ros::Publisher pub_estimation;
    ros::Rate r(10);
    int valid=0;
    while(ros::ok()){
        if(start_cmd!=0){
            cout << start_cmd<<">>>> T06" << endl;
            target_pose.request.positions.clear();
            for(int i=0;i<6;i++){
            // target[i]=tm_target_j[i];
                target_pose.request.positions.push_back(tm_target_p[i]);
            }

            if (client.call(target_pose))                             
            {
                start_cmd=0;
                if (target_pose.response.ok) ROS_INFO_STREAM("SetPositions to robot");
                else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
            }
            else
            {
                start_cmd=0;
                ROS_ERROR_STREAM("Error SetPositions to robot");
                return 1;
            }
        }
        r.sleep();
    }
    return 0;
}
