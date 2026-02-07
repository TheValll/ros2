#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;

class SubscriberNode : public rclcpp::Node
{
    public:
        SubscriberNode() : Node("subscriber")
        {   
            subscriber_ = this->create_subscription<example_interfaces::msg::String>("simple_topic", 10, std::bind(&SubscriberNode::callback_topic, this, _1));
            RCLCPP_INFO(this->get_logger(), "Subscriber has been started ...");
        }
        
    private:
        void callback_topic(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        }

        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}