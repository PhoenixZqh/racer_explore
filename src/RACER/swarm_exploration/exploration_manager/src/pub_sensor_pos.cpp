#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class DepthCameraPosePublisher
{
public:
    DepthCameraPosePublisher()
    {
        // 初始化TF2缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 初始化发布器
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/depth_camera_pose", 10);

        // 设置发布频率
        rate_ = std::make_shared<ros::Rate>(10.0); // 10Hz

        // 调试信息
        // ROS_INFO("Initialized DepthCameraPosePublisher: source_frame=support_depth, target_frame=world, topic=/depth_camera_pose");
    }

    void publishPose()
    {
        while (ros::ok())
        {
            try
            {
                // 查询TF变换
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                    "world", "support_depth", ros::Time(0), ros::Duration(1.0));

                // 创建PoseStamped消息
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "world";
                pose_msg.pose.position.x = transform.transform.translation.x;
                pose_msg.pose.position.y = transform.transform.translation.y;
                pose_msg.pose.position.z = transform.transform.translation.z;

                // 将原始四元数转换为 tf2::Quaternion
                tf2::Quaternion orig_quat(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);

                // 应用第一个旋转：绕 Z 轴 -90 度
                tf2::Quaternion z_correction_quat;
                z_correction_quat.setRPY(0, 0, -M_PI / 2); // -90 度绕 Z 轴
                tf2::Quaternion intermediate_quat = orig_quat * z_correction_quat;

                // 应用第二个旋转：绕 Y 轴 -90 度
                tf2::Quaternion y_correction_quat;
                y_correction_quat.setRPY(0, -M_PI / 2, 0); // -90 度绕 Y 轴
                tf2::Quaternion adjusted_quat = intermediate_quat * y_correction_quat;

                // 将调整后的四元数赋值给 pose_msg
                pose_msg.pose.orientation = tf2::toMsg(adjusted_quat);

                // 发布位姿
                pose_pub_.publish(pose_msg);

                // 调试信息：将四元数转换为欧拉角
                double roll, pitch, yaw;
                tf2::Matrix3x3(adjusted_quat).getRPY(roll, pitch, yaw);

                // ROS_INFO_STREAM("Depth Camera Pose: "
                //                 << "x=" << pose_msg.pose.position.x
                //                 << ", y=" << pose_msg.pose.position.y
                //                 << ", z=" << pose_msg.pose.position.z
                //                 << ", roll=" << roll
                //                 << ", pitch=" << pitch
                //                 << ", yaw=" << yaw);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_STREAM("Failed to get transform from support_depth to world: " << ex.what());
            }

            rate_->sleep();
        }
    }

private:
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

    // 创建发布器实例
    DepthCameraPosePublisher publisher;

    // 运行发布循环
    publisher.publishPose();

    return 0;
}