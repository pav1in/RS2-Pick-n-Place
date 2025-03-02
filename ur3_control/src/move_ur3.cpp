#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class UR3Mover : public rclcpp::Node {
public:
    UR3Mover() : Node("ur3_mover") {
        // Create a publisher to send joint trajectory commands
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/scaled_joint_trajectory_controller/joint_trajectory", 10);

        // Wait for a bit to ensure connection
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Move the UR3 to a new position
        send_joint_command();
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

    void send_joint_command() {
        // Create a trajectory message
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};  // Target joint angles (radians)
        point.time_from_start.sec = 3;  // Move in 3 seconds

        // Add the point to the trajectory message
        traj_msg.points.push_back(point);

        // Publish the message
        RCLCPP_INFO(this->get_logger(), "Sending joint command to UR3...");
        publisher_->publish(traj_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR3Mover>());
    rclcpp::shutdown();
    return 0;
}
