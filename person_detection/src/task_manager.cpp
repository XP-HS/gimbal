#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <map>
#include <queue>
#include <cmath>

class TaskManager {
private:
    ros::Subscriber person_list_sub_;
    ros::Subscriber slave_status_sub_;
    ros::Publisher task_pub_;
    ros::Publisher marker_pub_;
    ros::Timer viz_timer_;         // 可视化定时器
    ros::Timer status_print_timer_; // 状态打印定时器（修复打印用）
    
    // ============ 人员信息 ============
    struct Person {
        int id;
        double x, y, z;
        enum Status { PENDING, ASSIGNED, COMPLETED };
        Status status;
        int assigned_slave;
        double assign_time;
        double complete_time;
        int detection_count;
    };
    std::map<int, Person> persons_;
    std::queue<int> pending_queue_;      // 待分配队列
    
    // ============ 从机队列（只需要状态） ============
    struct Slave {
        int id;
        bool is_busy;           // 是否忙碌
        int current_person_id;  // 当前任务人物ID（-1表示无任务）
        int state;              // 0=空闲, 1=忙碌, 2=返航中
        double last_heartbeat;  // 最后心跳时间
    };
    std::map<int, Slave> slaves_;
    std::vector<int> idle_slaves_;      // 空闲从机ID列表
    
    // ============ 参数 ============
    double task_timeout_;
    double completed_keep_time_;
    double heartbeat_timeout_;   // 心跳超时（秒）
    std::string target_frame_;
    
    // 统计
    int total_assigned_;
    int total_completed_;
    
public:
    TaskManager() : total_assigned_(0), total_completed_(0) {
        ros::NodeHandle nh("~");
        
        nh.param<double>("task_timeout", task_timeout_, 60.0);
        nh.param<double>("completed_keep_time", completed_keep_time_, 120.0);
        nh.param<double>("heartbeat_timeout", heartbeat_timeout_, 30.0);
        nh.param<std::string>("target_frame", target_frame_, "camera_init");
        
        person_list_sub_ = nh.subscribe("/person_list", 10, 
                                        &TaskManager::personListCallback, this);
        slave_status_sub_ = nh.subscribe("/slave_status", 10,
                                        &TaskManager::slaveStatusCallback, this);
        task_pub_ = nh.advertise<geometry_msgs::PointStamped>("/task_assignment", 10);
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/task_visualization", 10);
        
        // ===================== 修复：使用成员变量定时器 =====================
        // 可视化定时器
        viz_timer_ = nh.createTimer(ros::Duration(1.0), 
            [this](const ros::TimerEvent&) {
                publishVisualization(ros::Time::now().toSec());
            });
        
        ROS_INFO("========================================");
        ROS_INFO("Task Manager Started");
        ROS_INFO("  - Task timeout: %.1f s", task_timeout_);
        ROS_INFO("  - Heartbeat timeout: %.1f s", heartbeat_timeout_);
        ROS_INFO("========================================");
        
        // 状态打印定时器（你要的打印就在这里）
        status_print_timer_ = nh.createTimer(ros::Duration(10.0), 
                                            &TaskManager::statusPrintCallback, this);
    }

    // ============ 人员回调 ============
    void personListCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        double current_time = ros::Time::now().toSec();
        
        for (const auto& pose : msg->poses) {
            // 查找是否已有此人
            int person_id = findPersonByPosition(pose.position.x, pose.position.y);
            
            if (person_id == -1) {
                // 新人
                Person new_person;
                new_person.id = getNextPersonId();
                new_person.x = pose.position.x;
                new_person.y = pose.position.y;
                new_person.z = pose.position.z;
                new_person.status = Person::PENDING;
                new_person.assigned_slave = -1;
                new_person.detection_count = 1;
                
                persons_[new_person.id] = new_person;
                pending_queue_.push(new_person.id);
                
                ROS_INFO("[NEW] Person %d at (%.1f, %.1f) - added to queue", 
                         new_person.id, new_person.x, new_person.y);
            } else {
                // 更新位置
                auto& person = persons_[person_id];
                person.x = pose.position.x;
                person.y = pose.position.y;
                person.z = pose.position.z;
                person.detection_count++;
            }
        }
        
        // 清理
        cleanupCompletedPersons(current_time);
        
        // 检查超时
        checkTimeoutTasks(current_time);
        
        // 分配任务
        assignTasks();
        
        // 可视化
        publishVisualization(current_time);
    }
    
    // ============ 从机状态回调（核心：维护空闲队列） ============
    void slaveStatusCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        double current_time = ros::Time::now().toSec();
        
        int slave_id = (int)msg->point.x;
        int state = (int)msg->point.y;      // 0=空闲, 1=忙碌, 2=返航中
        int person_id = (int)msg->point.z;
        bool is_busy = (state != 0);
        
        auto it = slaves_.find(slave_id);
        
        if (it == slaves_.end()) {
            // 新从机注册
            Slave new_slave;
            new_slave.id = slave_id;
            new_slave.is_busy = is_busy;
            new_slave.current_person_id = person_id;
            new_slave.state = state;
            new_slave.last_heartbeat = current_time;
            slaves_[slave_id] = new_slave;
            
            if (!is_busy) {
                idle_slaves_.push_back(slave_id);
            }
            
            ROS_INFO("[SLAVE] %d registered (state=%d)", slave_id, state);
        } else {
            // 状态变化检测
            bool was_busy = it->second.is_busy;
            
            it->second.is_busy = is_busy;
            it->second.current_person_id = person_id;
            it->second.state = state;
            it->second.last_heartbeat = current_time;
            
            // 任务完成：从忙碌变为空闲
            if (was_busy && !is_busy) {
                // 使用回调参数中的 person_id
                int completed_person_id = person_id;
                
                if (completed_person_id != -1) {
                    auto person_it = persons_.find(completed_person_id);
                    if (person_it != persons_.end() && 
                        person_it->second.status == Person::ASSIGNED) {
                        person_it->second.status = Person::COMPLETED;
                        person_it->second.complete_time = current_time;
                        total_completed_++;
                        
                        ROS_INFO("[COMPLETE] Person %d rescued by Slave %d", 
                                completed_person_id, slave_id);
                    }
                }
                
                // 将空闲从机加入队列
                idle_slaves_.push_back(slave_id);
                
                ROS_INFO("[SLAVE] %d is now IDLE, added to queue", slave_id);
                
                // 立即尝试分配新任务
                assignTasks();
            }
            
            // 从忙碌变为返航或其他状态（不加入空闲队列）
            if (was_busy && is_busy && state == 2) {
                ROS_INFO("[SLAVE] %d is RETURNING (person %d)", 
                        slave_id, person_id);
            }
        }
        
        // 检查心跳超时的从机（失联）
        checkHeartbeatTimeout(current_time);
        
        // 更新空闲队列（移除失联的）
        updateIdleQueue();
    }
    
    // ============ 任务分配（使用空闲队列） ============
    void assignTasks() {
        if (pending_queue_.empty()) {
            return;
        }
        
        if (idle_slaves_.empty()) {
            ROS_DEBUG("No idle slaves, waiting...");
            return;
        }
        
        double current_time = ros::Time::now().toSec();
        
        // 依次分配任务给空闲从机
        while (!pending_queue_.empty() && !idle_slaves_.empty()) {
            int person_id = pending_queue_.front();
            pending_queue_.pop();
            
            auto person_it = persons_.find(person_id);
            if (person_it == persons_.end() || 
                person_it->second.status != Person::PENDING) {
                continue;
            }
            
            // 取出第一个空闲从机
            int slave_id = idle_slaves_.front();
            idle_slaves_.erase(idle_slaves_.begin());
            
            // 更新人员状态
            person_it->second.status = Person::ASSIGNED;
            person_it->second.assigned_slave = slave_id;
            person_it->second.assign_time = current_time;
            
            // 更新从机状态
            auto slave_it = slaves_.find(slave_id);
            if (slave_it != slaves_.end()) {
                slave_it->second.is_busy = true;
                slave_it->second.current_person_id = person_id;
                slave_it->second.state = 1;  // 忙碌
            }
            
            // 发布任务
            geometry_msgs::PointStamped task_msg;
            task_msg.header.stamp = ros::Time::now();
            task_msg.header.frame_id = target_frame_;
            task_msg.point.x = person_id;           // 人物ID
            task_msg.point.y = person_it->second.x; // 目标X
            task_msg.point.z = person_it->second.y; // 目标Y
            task_pub_.publish(task_msg);
            
            total_assigned_++;
            
            ROS_INFO("[ASSIGN] Person %d → Slave %d at (%.1f, %.1f)", 
                     person_id, slave_id, 
                     person_it->second.x, person_it->second.y);
        }
    }
    
    // ============ 辅助函数 ============
    int findPersonByPosition(double x, double y) {
        const double MATCH_RADIUS = 1.5;
        
        for (const auto& pair : persons_) {
            const auto& person = pair.second;
            if (person.status == Person::COMPLETED) continue;
            
            double dist = std::hypot(x - person.x, y - person.y);
            if (dist < MATCH_RADIUS) {
                return pair.first;
            }
        }
        return -1;
    }
    
    void checkTimeoutTasks(double current_time) {
        for (auto& pair : persons_) {
            auto& person = pair.second;
            
            if (person.status == Person::ASSIGNED) {
                if (current_time - person.assign_time > task_timeout_) {
                    // 超时，重新分配
                    person.status = Person::PENDING;
                    int old_slave = person.assigned_slave;
                    person.assigned_slave = -1;
                    
                    // 释放从机
                    auto slave_it = slaves_.find(old_slave);
                    if (slave_it != slaves_.end()) {
                        slave_it->second.is_busy = false;
                        slave_it->second.current_person_id = -1;
                        slave_it->second.state = 0;
                        idle_slaves_.push_back(old_slave);
                    }
                    
                    pending_queue_.push(person.id);
                    
                    ROS_WARN("[TIMEOUT] Person %d task timeout, reassigning", person.id);
                }
            }
        }
    }
    
    void checkHeartbeatTimeout(double current_time) {
        for (auto& pair : slaves_) {
            if (current_time - pair.second.last_heartbeat > heartbeat_timeout_) {
                if (pair.second.is_busy) {
                    // 忙碌中的从机失联，将其任务重新分配
                    ROS_WARN("[HEARTBEAT] Slave %d lost! Reassigning task", pair.first);
                    
                    // 找到该从机的任务
                    for (auto& person_pair : persons_) {
                        if (person_pair.second.assigned_slave == pair.first &&
                            person_pair.second.status == Person::ASSIGNED) {
                            person_pair.second.status = Person::PENDING;
                            pending_queue_.push(person_pair.first);
                            break;
                        }
                    }
                }
                
                // 标记为失联（不加入空闲队列）
                pair.second.is_busy = false;
                pair.second.state = 0;
            }
        }
    }
    
    void updateIdleQueue() {
        // 清理空闲队列中已失联的从机
        auto it = idle_slaves_.begin();
        while (it != idle_slaves_.end()) {
            auto slave_it = slaves_.find(*it);
            if (slave_it == slaves_.end() || slave_it->second.is_busy) {
                it = idle_slaves_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void cleanupCompletedPersons(double current_time) {
        auto it = persons_.begin();
        while (it != persons_.end()) {
            if (it->second.status == Person::COMPLETED) {
                if (current_time - it->second.complete_time > completed_keep_time_) {
                    it = persons_.erase(it);
                    continue;
                }
            }
            ++it;
        }
    }
    
    // ============ 可视化 ============
    void publishVisualization(double current_time) {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;
        
        for (const auto& pair : persons_) {
            const auto& person = pair.second;
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "task_status";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = person.x;
            marker.pose.position.y = person.y;
            marker.pose.position.z = person.z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35;
            
            switch (person.status) {
                case Person::PENDING:
                    marker.color.r = 1.0; marker.color.g = 0.2; marker.color.b = 0.2;
                    break;
                case Person::ASSIGNED:
                    marker.color.r = 1.0; marker.color.g = 0.8; marker.color.b = 0.0;
                    break;
                case Person::COMPLETED:
                    marker.color.r = 0.2; marker.color.g = 0.8; marker.color.b = 0.2;
                    break;
            }
            marker.color.a = 0.8;
            marker.lifetime = ros::Duration(0);  // 0 表示永不过期
            marker_array.markers.push_back(marker);
            
            // 文字标签
            visualization_msgs::Marker text;
            text = marker;
            text.ns = "task_labels";
            text.id = marker_id++;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.scale.z = 0.12;
            
            char buf[32];
            if (person.status == Person::ASSIGNED) {
                snprintf(buf, sizeof(buf), "P%d→S%d", person.id, person.assigned_slave);
            } else if (person.status == Person::PENDING) {
                snprintf(buf, sizeof(buf), "P%d", person.id);
            } else {
                snprintf(buf, sizeof(buf), "✓P%d", person.id);
            }
            text.text = buf;
            text.color.r = text.color.g = text.color.b = 1.0;
            text.pose.position.z += 0.4;
            marker_array.markers.push_back(text);
        }
        
        marker_pub_.publish(marker_array);
    }
    
    void statusPrintCallback(const ros::TimerEvent&) {
        int pending = 0, assigned = 0, completed = 0;
        for (const auto& pair : persons_) {
            switch (pair.second.status) {
                case Person::PENDING: pending++; break;
                case Person::ASSIGNED: assigned++; break;
                case Person::COMPLETED: completed++; break;
            }
        }
        
        ROS_INFO("=== Status ===");
        ROS_INFO("  Persons: PEND=%d, ASSIGN=%d, DONE=%d", pending, assigned, completed);
        ROS_INFO("  Slaves: %zu total, %zu idle", slaves_.size(), idle_slaves_.size());
        ROS_INFO("  Tasks: assigned=%d, completed=%d", total_assigned_, total_completed_);
        ROS_INFO("==============");
    }
    
    int getNextPersonId() {
        static int next_id = 0;
        return next_id++;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_manager");
    TaskManager tm;
    ros::spin();
    return 0;
}