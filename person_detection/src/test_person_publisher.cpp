#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_person_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("/person_list", 10);
    
    // 只发布一次
    ros::Duration(1.0).sleep();  // 等待订阅者连接
    
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_init";
    
    // 人物0
    geometry_msgs::Pose pose0;
    pose0.position.x = 10.0;
    pose0.position.y = 15.0;
    pose0.position.z = 0.0;
    pose0.orientation.x = 0;
    pose0.orientation.w = 1.0;
    msg.poses.push_back(pose0);
    
    // 人物1
    geometry_msgs::Pose pose1;
    pose1.position.x = 25.0;
    pose1.position.y = 30.0;
    pose1.position.z = 0.0;
    pose1.orientation.x = 1;
    pose1.orientation.w = 1.0;
    msg.poses.push_back(pose1);
    
    pub.publish(msg);
    ROS_INFO("Published 2 persons (one time only)");
    
    ros::spin();  // 保持节点运行
    return 0;
}