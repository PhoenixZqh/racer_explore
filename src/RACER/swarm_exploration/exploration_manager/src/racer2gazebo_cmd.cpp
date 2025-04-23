#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class CommandConverter {
public:
    CommandConverter() : nh_("~") {
        // 订阅 PositionCommand
        cmd_sub_ = nh_.subscribe("/planning/pos_cmd_1", 1, &CommandConverter::posCmdCallback, this);
        // 订阅小车位置 (odom)
        odom_sub_ = nh_.subscribe("/odom", 1, &CommandConverter::odomCallback, this);
        // 发布 Twist 到 /cmd_vel
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // 初始化变量
        rcv_cmd_ = false;
        rcv_odom_ = false;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
        // 获取小车当前位置和朝向
        current_pose_ = odom->pose.pose;
        // 从四元数中提取 yaw 角度
        current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
        rcv_odom_ = true;
    }

    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd) {
        // 保存目标位置和 yaw
        target_position_ = cmd->position;
        target_yaw_ = cmd->yaw;
        rcv_cmd_ = true;
    }

    void publishTwist() {
        geometry_msgs::Twist twist;

        if (rcv_cmd_ && rcv_odom_) {
            // 计算位置误差
            double dx = target_position_.x - current_pose_.position.x;
            double dy = target_position_.y - current_pose_.position.y;
            double distance_error = sqrt(dx * dx + dy * dy);

            // 计算目标角度（朝向目标点）
            double target_angle = atan2(dy, dx);
            // 计算角度误差（目标点角度与当前 yaw 的差）
            double angle_error = target_angle - current_yaw_;
            // 归一化角度到 [-pi, pi]
            angle_error = atan2(sin(angle_error), cos(angle_error));

            // 比例控制生成线速度和角速度
            const double kp_linear = 0.8;   // 线速度比例增益
            const double kp_angular = 1.0;  // 角速度比例增益
            const double max_linear = 1.0;  // 最大线速度 (m/s)
            const double max_angular = 10.0; // 最大角速度 (rad/s)

            // 生成线速度（基于距离误差）
            twist.linear.x = std::min(max_linear, kp_linear * distance_error);
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;

            // 生成角速度（基于角度误差）
            twist.angular.z = std::min(max_angular, std::max(-max_angular, kp_angular * angle_error));
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;

            // 如果距离误差很小，停止运动
            if (distance_error < 0.05) { // 5cm 阈值
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
            }

            // 调试信息
            ROS_INFO_STREAM("Distance error: " << distance_error << " m, Angle error: " << angle_error << " rad");
        } else {
            // 未接收到命令或 odom，发布零速度
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
        }

        twist_pub_.publish(twist);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_, odom_sub_;
    ros::Publisher twist_pub_;
    geometry_msgs::Point target_position_; // 目标位置
    geometry_msgs::Pose current_pose_;     // 当前位置
    double target_yaw_, current_yaw_;      // 目标和当前 yaw
    bool rcv_cmd_, rcv_odom_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_converter");
    CommandConverter converter;

    ros::Rate rate(100); // 100 Hz
    while (ros::ok()) {
        converter.publishTwist();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}