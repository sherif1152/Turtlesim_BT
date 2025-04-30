#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <string>
#include <chrono>
#include <thread>

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

public:
    MoveTo(const std::string &name, const NodeConfiguration &config)
        : BT::CoroActionNode(name, config)
    {
        auto time_str = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        node_ = rclcpp::Node::make_shared("move_turtle_node_" + time_str);
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("idgoal"),
            BT::OutputPort<std::string>("status"),
            BT::InputPort<double>("goal_x", "Target X"),
            BT::InputPort<double>("goal_y", "Target Y"),
            BT::InputPort<double>("linear_speed", 1.0, "Linear speed"),
            BT::InputPort<double>("angular_speed", 0.0, "Angular speed"),
            BT::InputPort<double>("duration", 10.0, "Duration in seconds")};
    }

    BT::NodeStatus tick() override
    {
        auto idgoal = getInput<std::string>("idgoal");
        double x = this->getInput<double>("goal_x").value();
        double y = this->getInput<double>("goal_y").value();
        double linear_speed = this->getInput<double>("linear_speed").value();
        double angular_speed = this->getInput<double>("angular_speed").value();
        double duration = this->getInput<double>("duration").value();

        RCLCPP_INFO(node_->get_logger(), "Moving to %s at (%.2f, %.2f)", idgoal.value().c_str(), x, y);

        auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok())
        {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= duration)
                break;

            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_speed;
            msg.angular.z = angular_speed;
            publisher_->publish(msg);
            rclcpp::spin_some(node_);

            setStatusRunningAndYield(); // Needed for CoroActionNode
        }

        geometry_msgs::msg::Twist stop_msg;
        publisher_->publish(stop_msg);
        setOutput("status", "Arrived at " + idgoal.value());

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

    auto tree = factory.createTreeFromFile("/home/sherif/bt_demo/bt_ros/src/turtlesim_bt/config/turtle_tree.xml");

    // BT::StdCoutLogger logger(tree);
    BT::FileLogger file_logger(tree, "/home/sherif/bt_demo/bt_ros/src/turtlesim_bt/config/bt_log.fbl"); 
    BT::PublisherZMQ publisher(tree);

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
