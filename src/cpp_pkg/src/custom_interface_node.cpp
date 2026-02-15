#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/minimal_interface.hpp"

using namespace std::chrono_literals;

class CustomInterfaceNode : public rclcpp::Node
{
    public:
        CustomInterfaceNode() : Node("custom_interface_node")
        {
            publisher_ = this->create_publisher<custom_interfaces::msg::MinimalInterface>("custom_interface", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&CustomInterfaceNode::publish_example, this));
            RCLCPP_INFO(this->get_logger(), "Custom interface node is running ...");
        }
        
    private:
        void publish_example()
        {
            auto msg = custom_interfaces::msg::MinimalInterface();
            msg.a = 12.56;
            msg.b = true;
            msg.c = "Basic text";
            publisher_->publish(msg);
        } 
    
        rclcpp::Publisher<custom_interfaces::msg::MinimalInterface>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}