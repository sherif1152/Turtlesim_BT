#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
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

// printBatteryLevel
BT::NodeStatus PrintBatteryLevel()
{
    RCLCPP_INFO(rclcpp::get_logger("PrintBatteryLevel"), "Battery level: %d%%", battery_level);
    return NodeStatus::SUCCESS;
}

//  SaySomething
BT::NodeStatus SaySomething(const BT::TreeNode &self)
{
    auto msg = self.getInput<std::string>("message").value();
    RCLCPP_INFO(rclcpp::get_logger("BT"), "[Robot Says]: %s", msg.c_str());
    return NodeStatus::SUCCESS;
}

class MoveTo : public BT::CoroActionNode
{
public:
    MoveTo(const std::string &name, const NodeConfiguration &config)
        : BT::CoroActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("move_turtle_node_" + std::to_string(rand()));
        publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("goal_x", "Target X"),
            BT::InputPort<double>("goal_y", "Target Y"),
            BT::InputPort<double>("linear_speed", 1.0, "Linear speed"),
            BT::InputPort<double>("angular_speed", 0.0, "Angular speed"),
            BT::InputPort<double>("duration", 10.0, "Duration in seconds")};
    }

    BT::NodeStatus tick() override
    {
        double x = this->getInput<double>("goal_x").value();
        double y = this->getInput<double>("goal_y").value();
        double linear_speed = this->getInput<double>("linear_speed").value();
        double angular_speed = this->getInput<double>("angular_speed").value();
        double duration = this->getInput<double>("duration").value();

        RCLCPP_INFO(node_->get_logger(), "Moving to %.1f, %.1f", x, y);

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
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

BT::NodeStatus SetColorBasedOnBattery()
{
    auto node = rclcpp::Node::make_shared("color_changer");

    auto client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(node->get_logger(), "Service /set_pen not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();

    if (battery_level < 50)
    {
        request->r = 255;
        request->g = 0;
        request->b = 0;
    }
    else
    {
        request->r = 0;
        request->g = 255;
        request->b = 0;
    }

    request->width = 3;
    request->off = 0;

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service /set_pen");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node->get_logger(), "Changed color based on battery level");

    return BT::NodeStatus::SUCCESS;
}



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
    factory.registerSimpleAction("PrintBatteryLevel", std::bind(PrintBatteryLevel));
    factory.registerNodeType<MoveTo>("MoveTo");
    factory.registerSimpleAction("SetColorBasedOnBattery", std::bind(SetColorBasedOnBattery));

    auto tree = factory.createTreeFromFile("/home/sherif/bt_ros/src/turtlesim_bt/config/turtle_tree.xml");

    //BT::StdCoutLogger logger(tree);

    rclcpp::Rate rate(2);
    while (rclcpp::ok())
    {
        tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();
    return 0;
}
