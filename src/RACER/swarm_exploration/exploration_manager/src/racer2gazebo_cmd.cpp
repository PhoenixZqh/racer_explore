#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <msg_set/DroneState.h>

class CommandConverter
{
public:
    CommandConverter()
        : nh_("~"), car_id_num_(-1), mission_done_(false), mission_done_time_(0.0)
    {
        nh_.param<std::string>("car_id", car_id_, "car1");
        nh_.param<std::string>("cmd_suffix", cmd_suffix_, "_1");

        std::string cmd_topic = "/planning/pos_cmd" + cmd_suffix_;
        std::string odom_topic = "/" + car_id_ + "/odom";
        std::string twist_topic = "/" + car_id_ + "/cmd_vel";
        std::string drone_state_topic = "/swarm_expl/drone_state";

        // 提取 car_id 中的数字部分作为 car_id_num_
        car_id_num_ = -1; // 默认
        for (size_t i = 0; i < car_id_.size(); ++i)
        {
            if (isdigit(car_id_[i]))
            {
                car_id_num_ = std::stoi(car_id_.substr(i));
                break;
            }
        }
        if (car_id_num_ == -1)
        {
            ROS_WARN("Failed to extract car_id_num from car_id: %s", car_id_.c_str());
        }

        cmd_sub_ = nh_.subscribe(cmd_topic, 1, &CommandConverter::posCmdCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic, 1, &CommandConverter::odomCallback, this);
        drone_state_sub_ = nh_.subscribe(drone_state_topic, 10, &CommandConverter::droneStateCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>(twist_topic, 1);

        rcv_cmd_ = false;
        rcv_odom_ = false;

        ROS_INFO("CommandConverter initialized for %s with cmd suffix %s", car_id_.c_str(), cmd_suffix_.c_str());
    }

    void droneStateCallback(const msg_set::DroneStateConstPtr &msg)
    {
        if (msg->drone_id == car_id_num_)
        {
            if (msg->cur_state == 5)
            {
                if (!mission_done_)
                {
                    mission_done_ = true;
                    mission_done_time_ = ros::Time::now(); // Record the time when mission is done
                    ROS_INFO("[%s] Mission completed. Will stop in 5 seconds...", car_id_.c_str());
                }
            }
            else
            {
                mission_done_ = false;
            }
        }
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

        if (mission_done_ && (ros::Time::now() - mission_done_time_).toSec() >= 5.0)
        {
            // 任务完成且过了5秒，停止一切运动
            twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
            twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
            twist_pub_.publish(twist);
            ROS_INFO_STREAM("[" << car_id_ << "] Stopping after 5 seconds delay.");
            return;
        }

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

            const double kp_linear = 0.6;
            const double kp_angular = 1.5;
            const double max_linear = 1.2;
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
    ros::Subscriber cmd_sub_, odom_sub_, drone_state_sub_;
    ros::Publisher twist_pub_;
    geometry_msgs::Point target_position_;
    geometry_msgs::Pose current_pose_;
    double target_yaw_, current_yaw_;
    bool rcv_cmd_, rcv_odom_;
    bool mission_done_;
    ros::Time mission_done_time_; // New variable to store the time when mission is done
    std::string car_id_, cmd_suffix_;
    int car_id_num_;
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