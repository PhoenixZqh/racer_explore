#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/PositionCommand.h>

class CommandConverter
{
public:
    CommandConverter()
        : nh_("~")
    {
        // 订阅 PositionCommand
        cmd_sub_ = nh_.subscribe("/planning/pos_cmd_1", 1, &CommandConverter::posCmdCallback, this);
        // 发布 Twist 到 /cmd_vel
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // 初始化变量
        last_yaw_ = 0.0;
        last_time_ = ros::Time::now();
        rcv_cmd_ = false;

        // 调试信息
        // ROS_INFO("CommandConverter initialized: sub=command, pub=/cmd_vel");
    }

    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
    {
        rcv_cmd_ = true;
        cmd_ = *cmd;
    }

    void publishTwist()
    {
        geometry_msgs::Twist twist;

        if (rcv_cmd_)
        {
            // 计算线速度：将 velocity.x, velocity.y 投影到小车前进方向
            // 假设小车朝向与 yaw 一致，线速度为速度矢量的大小
            double vx = cmd_.velocity.x;
            double vy = cmd_.velocity.y;
            twist.linear.x = sqrt(vx * vx + vy * vy); // 速度模长
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;

            // 计算角速度：基于 yaw 的变化率
            double current_yaw = cmd_.yaw;
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time_).toSec();

            if (dt > 0.0)
            {
                double yaw_rate = (current_yaw - last_yaw_) / dt;
                // 限制角速度，防止突变
                const double max_yaw_rate = 1.0; // 最大角速度（rad/s）
                twist.angular.z = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw_rate));
            }
            else
            {
                twist.angular.z = 0.0;
            }

            // 更新上次的值
            last_yaw_ = current_yaw;
            last_time_ = current_time;

            // 调试信息
            // ROS_INFO_STREAM("Converted: linear.x=" << twist.linear.x << ", angular.z=" << twist.angular.z);
        }
        else
        {
            // 未接收到命令时，发布零速度
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
    ros::Subscriber cmd_sub_;
    ros::Publisher twist_pub_;
    quadrotor_msgs::PositionCommand cmd_;
    bool rcv_cmd_;
    double last_yaw_;
    ros::Time last_time_;
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