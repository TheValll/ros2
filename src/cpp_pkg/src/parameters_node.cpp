#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class ParametersNode : public rclcpp::Node
{
    public:
        ParametersNode() : Node("parameters_node")
        {   
            this->declare_parameter("message", "Simple publisher");
            publisher_ = this->create_publisher<example_interfaces::msg::String>("simple_topic", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&ParametersNode::publish_example, this));
            RCLCPP_INFO(this->get_logger(), "Publisher with parameters has been started ...");
        }
        
    private:
        void publish_example()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = this->get_parameter("message").as_string();
            publisher_->publish(msg);
        } 
    
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParametersNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}