#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
    public:
        PublisherNode() : Node("publisher")
        {
            publisher_ = this->create_publisher<example_interfaces::msg::String>("simple_topic", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::publish_example, this));
            RCLCPP_INFO(this->get_logger(), "Publisher has been started ...");
        }
        
    private:
        void publish_example()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = "Simple publisher";
            publisher_->publish(msg);
        } 
    
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}