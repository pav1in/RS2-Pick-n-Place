#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class UR3RealCircleMotion : public rclcpp::Node
{
public:
    UR3RealCircleMotion() : Node("ur3_circle_motion_real")
    {
        move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");

        geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
        double radius = 0.05;  
        int num_points = 20;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (int i = 0; i < num_points; i++)
        {
            double theta = (2 * M_PI * i) / num_points;
            geometry_msgs::msg::Pose pose = start_pose;
            pose.position.x += radius * cos(theta);
            pose.position.y += radius * sin(theta);
            waypoints.push_back(pose);
        }

        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction > 0.9)
        {
            RCLCPP_INFO(this->get_logger(), "Executing circular motion on the real UR3...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group->execute(plan);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory planning failed!");
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR3RealCircleMotion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
