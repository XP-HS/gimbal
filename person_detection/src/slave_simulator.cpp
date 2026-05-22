#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

class SlaveSimulator {
private:
    int slave_id_;
    ros::Subscriber task_sub_;
    ros::Publisher status_pub_;
    ros::Timer timer_;
    ros::Timer status_timer_;
    
    enum State { IDLE, BUSY, RETURNING };
    State state_;
    int current_person_id_;
    double task_start_time_;
    double task_duration_;
    double return_duration_;
    
    ros::NodeHandle nh_;
    
public:
    SlaveSimulator(int id) : slave_id_(id), state_(IDLE), current_person_id_(-1), nh_() {
        nh_.param<double>("task_duration", task_duration_, 8.0);
        nh_.param<double>("return_duration", return_duration_, 5.0);
        
        task_sub_ = nh_.subscribe("/task_assignment", 10, &SlaveSimulator::taskCallback, this);
        status_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/slave_status", 10);
        
        ROS_INFO("Slave %d started, IDLE", slave_id_);
        
        timer_ = nh_.createTimer(ros::Duration(0.5), &SlaveSimulator::stateMachineCallback, this);
        status_timer_ = nh_.createTimer(ros::Duration(1.0), &SlaveSimulator::statusPublishCallback, this);
    }
    
    void taskCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (state_ != IDLE) return;
        
        current_person_id_ = (int)msg->point.x;
        task_start_time_ = ros::Time::now().toSec();
        state_ = BUSY;
        
        ROS_INFO("Slave %d: Person %d task started", slave_id_, current_person_id_);
    }
    
    void stateMachineCallback(const ros::TimerEvent&) {
        double elapsed = ros::Time::now().toSec() - task_start_time_;
        
        if (state_ == BUSY && elapsed >= task_duration_) {
            state_ = RETURNING;
            ROS_INFO("Slave %d: Task done, returning", slave_id_);
        }
        else if (state_ == RETURNING && elapsed >= task_duration_ + return_duration_) {
            state_ = IDLE;
            // 不要立即重置！保留 person_id 让 task_manager 知道谁完成了
            ROS_INFO("Slave %d: Returned, IDLE (person %d completed)", 
                     slave_id_, current_person_id_);
            
            // 延迟0.5秒后重置，确保 task_manager 已经处理了完成状态
            ros::Timer reset_timer = nh_.createTimer(ros::Duration(0.5), 
                [this](const ros::TimerEvent&) {
                    current_person_id_ = -1;
                    ROS_DEBUG("Slave %d: Reset person_id to -1", slave_id_);
                }, true);
        }
    }
    
    void statusPublishCallback(const ros::TimerEvent&) {
        geometry_msgs::PointStamped status_msg;
        status_msg.header.stamp = ros::Time::now();
        status_msg.header.frame_id = "camera_init";
        status_msg.point.x = slave_id_;
        status_msg.point.y = (double)state_;
        status_msg.point.z = current_person_id_;
        status_pub_.publish(status_msg);
        
        ROS_DEBUG("Slave %d publishing status: state=%d, person=%d", 
                  slave_id_, state_, current_person_id_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "slave_simulator");
    
    int slave_id = 1;
    ros::NodeHandle nh("~");
    nh.param<int>("slave_id", slave_id, 1);
    
    SlaveSimulator slave(slave_id);
    ros::spin();
    return 0;
}