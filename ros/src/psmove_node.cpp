#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include "sawPSMove/mtsPSMove.h"

class PSMoveROSNode : public rclcpp::Node {
public:
    PSMoveROSNode()
    : Node("psmove_ros_node")
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("psmove/pose", 10);
        imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("psmove/imu", 10);
        joy_pub_  = this->create_publisher<sensor_msgs::msg::Joy>("psmove/joy", 10);

        // create cisst component
        move_comp_ = new mtsPSMove("PSMove");
        mtsComponentManager::GetInstance()->AddComponent(move_comp_);
        mtsComponentManager::GetInstance()->CreateAll();
        mtsComponentManager::GetInstance()->StartAll();

        // start a timer to poll & publish
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&PSMoveROSNode::publishData, this));
    }

private:
    void publishData() {
        prmPositionCartesianGet pose;
        vctFixedSizeVector<double,4> quat;
        vct3 pos;
        uint32_t buttons;
        double trigger;

        move_comp_->GetPose(pose);
        move_comp_->GetQuaternion(quat);
        move_comp_->GetPosition(pos);
        move_comp_->GetButtons(buttons);
        move_comp_->GetTrigger(trigger);

        // PoseStamped
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = pos[0];
        pose_msg.pose.position.y = pos[1];
        pose_msg.pose.position.z = pos[2];
        pose_msg.pose.orientation.w = quat[0];
        pose_msg.pose.orientation.x = quat[1];
        pose_msg.pose.orientation.y = quat[2];
        pose_msg.pose.orientation.z = quat[3];
        pose_pub_->publish(pose_msg);

        // IMU (orientation only)
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.orientation = pose_msg.pose.orientation;
        imu_pub_->publish(imu_msg);

        // Joy
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.axes.resize(1);
        joy_msg.axes[0] = trigger;
        joy_msg.buttons.resize(16);
        for (int i=0; i<16; ++i) {
            joy_msg.buttons[i] = (buttons & (1<<i)) ? 1 : 0;
        }
        joy_pub_->publish(joy_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    mtsPSMove* move_comp_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PSMoveROSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
