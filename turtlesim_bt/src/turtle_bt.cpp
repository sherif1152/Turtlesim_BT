#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <string>
#include <chrono>
#include <thread>
#include <cmath>

using namespace BT;
using namespace std::chrono_literals;

int battery_level = 60;
// CheckBattery
BT::NodeStatus CheckBattery()
{
    if (battery_level >= 50)
    {
        return NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("BT"), "Battery low!");
        return NodeStatus::FAILURE;
    }
}
// ChargeBattery
BT::NodeStatus ChargeBattery()
{

    battery_level += 30;
    if (battery_level > 100)
        battery_level = 100;

    RCLCPP_WARN(rclcpp::get_logger("ChargeBattery"), "Wait for the Battery to charge....");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    RCLCPP_WARN(rclcpp::get_logger("ChargeBattery"), "Charge Battery... Now to %d%%", battery_level);
    return NodeStatus::SUCCESS;
}

// DrainBattery
BT::NodeStatus DrainBattery()
{
    battery_level -= 20;
    if (battery_level < 0)
        battery_level = 0;

    RCLCPP_WARN(rclcpp::get_logger("DrainBattery"), "Draining battery... Now at %d%%", battery_level);
    return NodeStatus::SUCCESS;
}

//  SaySomething
BT::NodeStatus SaySomething(const BT::TreeNode &self)
{
    auto msg = self.getInput<std::string>("message");
    if (!msg)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BT"), "Message input missing");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("BT"), "[Robot Says]: %s", msg.value().c_str());
    return BT::NodeStatus::SUCCESS;
}

class MoveTo : public BT::CoroActionNode
{
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    turtlesim::msg::Pose current_pose_;
    bool pose_received_ = false;
    const double threshold = 0.1;

public:
    MoveTo(const std::string &name, const NodeConfiguration &config)
        : BT::CoroActionNode(name, config)
    {
        auto time_str = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        node_ = rclcpp::Node::make_shared("move_turtle_node_" + time_str);
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        pose_subscriber_ = node_->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MoveTo::poseCallback, this, std::placeholders::_1));
    }

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("idgoal"),
            BT::OutputPort<std::string>("status"),
            BT::InputPort<double>("goal_x", "Target X"),
            BT::InputPort<double>("goal_y", "Target Y"),
            BT::InputPort<double>("linear_speed", 1.0, "Linear speed"),
            BT::InputPort<double>("angular_speed", 0.0, "Angular speed")};
    }

    BT::NodeStatus tick() override
    {
        auto idgoal = getInput<std::string>("idgoal");
        double x = this->getInput<double>("goal_x").value();
        double y = this->getInput<double>("goal_y").value();
        double linear_speed = this->getInput<double>("linear_speed").value();
        double angular_speed = this->getInput<double>("angular_speed").value();

        RCLCPP_INFO(node_->get_logger(), "Moving to %s at (%.2f, %.2f)", idgoal.value().c_str(), x, y);

        while (rclcpp::ok())
        {
            if (!pose_received_)
            {
                rclcpp::spin_some(node_);
                rclcpp::Rate loop_rate(10);
                continue;
            }

            double dx = x - current_pose_.x;
            double dy = y - current_pose_.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < threshold)
            {
                RCLCPP_INFO(node_->get_logger(), "Turtlesim Reached to goal!");
                break;
            }

            double target_theta = std::atan2(dy, dx);
            double angle_error = target_theta - current_pose_.theta;

            // Normalize angle
            while (angle_error > M_PI)
                angle_error -= 2 * M_PI;
            while (angle_error < -M_PI)
                angle_error += 2 * M_PI;

            geometry_msgs::msg::Twist msg;

            if (std::fabs(angle_error) > 0.1)
            {
                msg.angular.z = (angle_error > 0 ? 1 : -1) * angular_speed;
                msg.linear.x = 0.0;
            }
            else
            {
                msg.linear.x = linear_speed;
                msg.angular.z = 0.0;
            }

            publisher_->publish(msg);
            rclcpp::spin_some(node_);
            rclcpp::Rate loop_rate(10);
            setStatusRunningAndYield();
        }

        geometry_msgs::msg::Twist stop_msg;
        publisher_->publish(stop_msg);
        setOutput("status", "Arrived at " + idgoal.value());
        pose_received_ = false; // Reset for the next tick

        return BT::NodeStatus::SUCCESS;
    }
};

class SetColorLineNode : public BT::SyncActionNode
{
public:
    SetColorLineNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("idgoal")};
    }
    BT::NodeStatus tick() override
    {
        auto node = rclcpp::Node::make_shared("color_changer");

        // Create client for the SetPen service
        auto client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        // Wait for the service to be available
        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(node->get_logger(), "Service /set_pen not available");
            return BT::NodeStatus::FAILURE;
        }

        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();

        // Get the idgoal input parameter
        auto idgoal = getInput<std::string>("idgoal");

        if (!idgoal)
        {
            RCLCPP_ERROR(node->get_logger(), "idgoal input missing");
            return BT::NodeStatus::FAILURE;
        }

        // Set color based on battery level
        if (idgoal.value() == "first_goal")
        {
            request->r = 0;
            request->g = 200;
            request->b = 0;
        }
        else if (idgoal.value() == "second_goal")
        {
            request->r = 200;
            request->g = 0;
            request->b = 200;
        }
        else if (idgoal.value() == "charging")
        {
            request->r = 255;
            request->g = 215;
            request->b = 0;
        }
        else
        {
            request->r = 0;
            request->g = 0;
            request->b = 0; // Default: black
        }

        RCLCPP_INFO(node->get_logger(), idgoal.value().c_str());
        // Set pen width and other properties
        request->width = 3;
        request->off = 0;

        // Send the request to the SetPen service
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service /set_pen");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }
};
class SetBackgroundColorNode : public BT::SyncActionNode
{
public:
    SetBackgroundColorNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("idgoal")};
    }

    BT::NodeStatus tick() override
    {
        auto node = rclcpp::Node::make_shared("background_color_changer");
        auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/turtlesim");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(node->get_logger(), "Waiting for /turtlesim parameter service...");
        }

        auto mode = getInput<std::string>("idgoal");
        if (!mode)
        {
            RCLCPP_ERROR(node->get_logger(), "Missing 'mode' input");
            return BT::NodeStatus::FAILURE;
        }

        int r = 0, g = 0, b = 0;
        if (mode.value() == "first_goal")
        {
            r = 173;
            g = 0;
            b = 100;
        }
        else if (mode.value() == "second_goal")
        {
            r = 173;
            g = 216;
            b = 230;
        }
        else if (mode.value() == "charging")
        {
            r = 47;
            g = 79;
            b = 79;
        }
        else
        {
            r = 255;
            g = 255;
            b = 255; // Default: white
        }

        try
        {
            client->set_parameters({
                rclcpp::Parameter("background_r", r),
                rclcpp::Parameter("background_g", g),
                rclcpp::Parameter("background_b", b),
            });
            RCLCPP_INFO(node->get_logger(), "Background set to [%d, %d, %d]", r, g, b);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set background color: %s", ex.what());
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_runner");
    BehaviorTreeFactory factory;

    // Register nodes
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    factory.registerSimpleAction("SaySomething", SaySomething, {InputPort<std::string>("message")});
    factory.registerSimpleAction("ChargeBattery", std::bind(ChargeBattery));
    factory.registerSimpleAction("DrainBattery", std::bind(DrainBattery));
    factory.registerNodeType<MoveTo>("MoveTo");
    factory.registerNodeType<SetColorLineNode>("SetColorLine");
    factory.registerNodeType<SetBackgroundColorNode>("SetBackgroundColor");

    std::string package_share_dir = ament_index_cpp::get_package_share_directory("turtlesim_bt");
    std::string tree_file_path = package_share_dir + "/config/turtle_tree.xml";
    auto tree = factory.createTreeFromFile(tree_file_path);

    std::string log_file_path = package_share_dir + "/config/bt_log.fbl";
    BT::FileLogger file_logger(tree, log_file_path.c_str());

    BT::PublisherZMQ publisher(tree);

    // BT::StdCoutLogger logger(tree);
    rclcpp::Rate rate(2);
    while (rclcpp::ok())
    {
        tree.tickRoot();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
