#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node
{
    public:
        MinimalNode() : Node("minimal_node")
        {
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MinimalNode::timer_callback, this));
        }
        
    private:
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Minimal node is running ...");
        }

        rclcpp::TimerBase::SharedPtr timer_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}