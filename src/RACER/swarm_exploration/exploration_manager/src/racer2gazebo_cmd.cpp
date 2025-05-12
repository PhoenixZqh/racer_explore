// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <quadrotor_msgs/PositionCommand.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_datatypes.h>

// class CommandConverter
// {
// public:
//     CommandConverter()
//         : nh_("~")
//     {
//         // 从参数服务器获取话题前缀和后缀
//         nh_.param<std::string>("car_id", car_id_, "car1");
//         nh_.param<std::string>("cmd_suffix", cmd_suffix_, "_1");

//         // 构造话题名称
//         std::string cmd_topic = "/planning/pos_cmd" + cmd_suffix_;
//         std::string odom_topic = "/" + car_id_ + "/odom";
//         std::string twist_topic = "/" + car_id_ + "/cmd_vel";

//         // 订阅 PositionCommand
//         cmd_sub_ = nh_.subscribe(cmd_topic, 1, &CommandConverter::posCmdCallback, this);
//         // 订阅小车位置 (odom)
//         odom_sub_ = nh_.subscribe(odom_topic, 1, &CommandConverter::odomCallback, this);
//         // 发布 Twist 到 /cmd_vel
//         twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic, 1);

//         // 初始化变量
//         rcv_cmd_ = false;
//         rcv_odom_ = false;

//         ROS_INFO("CommandConverter initialized for %s with cmd suffix %s", car_id_.c_str(), cmd_suffix_.c_str());
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
//     {
//         // 获取小车当前位置和朝向
//         current_pose_ = odom->pose.pose;
//         // 从四元数中提取 yaw 角度
//         current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
//         rcv_odom_ = true;
//     }

//     void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
//     {
//         // 保存目标位置和 yaw
//         target_position_ = cmd->position;
//         target_yaw_ = cmd->yaw;
//         rcv_cmd_ = true;
//     }

//     void publishTwist()
//     {
//         geometry_msgs::Twist twist;

//         if (rcv_cmd_ && rcv_odom_)
//         {
//             // 计算位置误差
//             double dx = target_position_.x - current_pose_.position.x;
//             double dy = target_position_.y - current_pose_.position.y;
//             double distance_error = sqrt(dx * dx + dy * dy);

//             // 计算目标角度（朝向目标点）
//             double target_angle = atan2(dy, dx);
//             // 计算角度误差（目标点角度与当前 yaw 的差）
//             double angle_error = target_angle - current_yaw_;

//             // 归一化角度到 [-pi, pi]
//             angle_error = atan2(sin(angle_error), cos(angle_error));

//             // 如果角度误差小于一定阈值，则认为朝向目标方向
//             const double angle_threshold = 0.025;  // 角度误差小于 0.05 时认为已到目标方向
//             const double distance_threshold = 0.1; // 距离误差小于 0.1 时认为已到达目标位置

//             // 比例控制生成线速度和角速度
//             const double kp_linear = 0.4;   // 线速度比例增益
//             const double kp_angular = 1.0;  // 角速度比例增益
//             const double max_linear = 1.2;  // 最大线速度 (m/s)
//             const double max_angular = 3.5; // 最大角速度 (rad/s)

//             // 生成线速度（基于距离误差）
//             twist.linear.x = std::min(max_linear, kp_linear * distance_error);
//             twist.linear.y = 0.0;
//             twist.linear.z = 0.0;

//             // 生成角速度（基于角度误差）
//             // 如果角度误差较小，则减小角速度，避免过度转向
//             if (fabs(angle_error) > angle_threshold)
//             {
//                 twist.angular.z = std::min(max_angular, std::max(-max_angular, kp_angular * angle_error));
//             }
//             else
//             {
//                 twist.angular.z = 0.0; // 角度误差足够小，停止旋转
//             }
//             twist.angular.x = 0.0;
//             twist.angular.y = 0.0;

//             // 如果距离误差很小，停止运动
//             if (distance_error < distance_threshold)
//             {
//                 twist.linear.x = 0.0;
//                 twist.angular.z = 0.0;
//             }

//             // 调试信息
//             ROS_INFO_STREAM("[" << car_id_ << "] Distance error: " << distance_error << " m, Angle error: " << angle_error << " rad");
//         }
//         else
//         {
//             // 未接收到命令或 odom，发布零速度
//             twist.linear.x = 0.0;
//             twist.linear.y = 0.0;
//             twist.linear.z = 0.0;
//             twist.angular.x = 0.0;
//             twist.angular.y = 0.0;
//             twist.angular.z = 0.0;
//         }

//         twist_pub_.publish(twist);
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber cmd_sub_, odom_sub_;
//     ros::Publisher twist_pub_;
//     geometry_msgs::Point target_position_; // 目标位置
//     geometry_msgs::Pose current_pose_;     // 当前位置
//     double target_yaw_, current_yaw_;      // 目标和当前 yaw
//     bool rcv_cmd_, rcv_odom_;
//     std::string car_id_;     // 车辆ID，例如 "car1", "car2"
//     std::string cmd_suffix_; // 命令话题后缀，例如 "_1", "_2"
// };

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "command_converter");
//     CommandConverter converter;

//     ros::Rate rate(100); // 100 Hz
//     while (ros::ok())
//     {
//         converter.publishTwist();
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class CommandConverter
{
public:
    CommandConverter()
        : nh_("~")
    {
        nh_.param<std::string>("car_id", car_id_, "car1");
        nh_.param<std::string>("cmd_suffix", cmd_suffix_, "_1");

        std::string cmd_topic = "/planning/pos_cmd" + cmd_suffix_;
        std::string odom_topic = "/" + car_id_ + "/odom";
        std::string twist_topic = "/" + car_id_ + "/cmd_vel";

        cmd_sub_ = nh_.subscribe(cmd_topic, 1, &CommandConverter::posCmdCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic, 1, &CommandConverter::odomCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic, 1);

        rcv_cmd_ = false;
        rcv_odom_ = false;

        ROS_INFO("CommandConverter initialized for %s with cmd suffix %s", car_id_.c_str(), cmd_suffix_.c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        current_pose_ = odom->pose.pose;
        current_yaw_ = tf::getYaw(odom->pose.pose.orientation);
        rcv_odom_ = true;
    }

    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
    {
        target_position_ = cmd->position;
        target_yaw_ = cmd->yaw;
        rcv_cmd_ = true;
    }

    void publishTwist()
    {
        geometry_msgs::Twist twist;

        if (rcv_cmd_ && rcv_odom_)
        {
            double dx = target_position_.x - current_pose_.position.x;
            double dy = target_position_.y - current_pose_.position.y;
            double distance_error = sqrt(dx * dx + dy * dy);

            double target_angle = atan2(dy, dx);
            double angle_to_target = target_angle - current_yaw_;
            angle_to_target = atan2(sin(angle_to_target), cos(angle_to_target));

            double yaw_error = target_yaw_ - current_yaw_;
            yaw_error = atan2(sin(yaw_error), cos(yaw_error));

            const double angle_threshold = 0.02;
            const double distance_threshold = 0.1;
            const double final_yaw_control_distance = 0.4;

            const double kp_linear = 0.8;
            const double kp_angular = 1.5;
            const double max_linear = 1.0;
            const double max_angular = 3.0;

            if (distance_error < distance_threshold)
            {
                twist.linear.x = 0.0;
                if (fabs(yaw_error) > angle_threshold)
                {
                    twist.angular.z = std::min(max_angular, std::max(-max_angular, kp_angular * yaw_error));
                }
                else
                {
                    twist.angular.z = 0.0;
                }
            }
            else
            {
                twist.linear.x = std::min(max_linear, kp_linear * distance_error);
                if (distance_error < final_yaw_control_distance)
                {
                    twist.angular.z = std::min(max_angular, std::max(-max_angular, kp_angular * yaw_error));
                }
                else
                {
                    twist.angular.z = std::min(max_angular, std::max(-max_angular, kp_angular * angle_to_target));
                }
            }

            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;

            ROS_INFO_STREAM("[" << car_id_ << "] Dist err: " << distance_error << " | Angle to target: " << angle_to_target << " | Yaw error: " << yaw_error);
        }
        else
        {
            twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
            twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
        }

        twist_pub_.publish(twist);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_, odom_sub_;
    ros::Publisher twist_pub_;
    geometry_msgs::Point target_position_;
    geometry_msgs::Pose current_pose_;
    double target_yaw_, current_yaw_;
    bool rcv_cmd_, rcv_odom_;
    std::string car_id_, cmd_suffix_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_converter");
    CommandConverter converter;

    ros::Rate rate(100); // 100 Hz
    while (ros::ok())
    {
        converter.publishTwist();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
