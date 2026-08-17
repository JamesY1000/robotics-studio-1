#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ExampleNodeCPP : public rclcpp::Node
{
public:
    ExampleNodeCPP() : Node("example_node_cpp"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic_cpp", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ExampleNodeCPP::timerCallback, this));
    }

private:
    void timerCallback()
    {
       auto msg = std_msgs::msg::String();
       msg.data = "Publishing the " + std::to_string(count_);
       publisher_->publish(msg);
       RCLCPP_INFO(this->get_logger(), "Publishing the %s message", msg.data.c_str());
       count_ ++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExampleNodeCPP>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}