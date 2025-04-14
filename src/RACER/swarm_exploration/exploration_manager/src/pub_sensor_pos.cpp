#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>
#include <string>

class DepthCameraPosePublisher
{
public:
    DepthCameraPosePublisher(const std::string &robot_name, const std::string &source_frame, const std::string &target_frame, const std::string &topic_name)
        : robot_name_(robot_name), source_frame_(source_frame), target_frame_(target_frame)
    {
        // 初始化TF2缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化发布器
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name, 10);

        // 设置发布频率
        rate_ = std::make_shared<ros::Rate>(10.0); // 10Hz
    }

    void publishPose()
    {
        while (ros::ok())
        {
            try
            {
                // 查询TF变换
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                    target_frame_, source_frame_, ros::Time(0), ros::Duration(1.0));

                // 创建PoseStamped消息
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = target_frame_;
                pose_msg.pose.position.x = transform.transform.translation.x;
                pose_msg.pose.position.y = transform.transform.translation.y;
                pose_msg.pose.position.z = transform.transform.translation.z;
                pose_msg.pose.orientation = transform.transform.rotation;

                // 发布位姿
                pose_pub_.publish(pose_msg);

                // 调试信息：将四元数转换为欧拉角
                tf2::Quaternion quat(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                ROS_INFO_STREAM(robot_name_ << " Depth Camera Pose: "
                                            << "x=" << pose_msg.pose.position.x << ", y=" << pose_msg.pose.position.y << ", z=" << pose_msg.pose.position.z << ", roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_STREAM("Failed to get transform for " << robot_name_ << ": " << ex.what());
            }

            rate_->sleep();
        }
    }

private:
    std::string robot_name_;
    std::string source_frame_;
    std::string target_frame_;
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Publisher pose_pub_;
    std::shared_ptr<ros::Rate> rate_;
};

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "depth_camera_pose_publisher");

    // 获取参数
    ros::NodeHandle nh("~");
    std::string robot1_name, robot2_name;
    nh.param<std::string>("robot1_name", robot1_name, "car1");
    nh.param<std::string>("robot2_name", robot2_name, "car2");

    // 创建两个发布器实例
    DepthCameraPosePublisher robot1_publisher(
        robot1_name,
        robot1_name + "_support_depth",
        "world",
        "/" + robot1_name + "/depth_camera_pose");
    DepthCameraPosePublisher robot2_publisher(
        robot2_name,
        robot2_name + "_support_depth",
        "world",
        "/" + robot2_name + "/depth_camera_pose");

    // 使用多线程发布两辆车的位姿
    std::thread t1(&DepthCameraPosePublisher::publishPose, &robot1_publisher);
    std::thread t2(&DepthCameraPosePublisher::publishPose, &robot2_publisher);

    // 保持主线程运行
    ros::spin();

    // 等待线程结束
    t1.join();
    t2.join();

    return 0;
}