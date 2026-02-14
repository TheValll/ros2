// Based on the AddTwoInts interface
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class ServerNode : public rclcpp::Node
{
    public:
        ServerNode() : Node("server_node")
        {
            server_ = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", std::bind(&ServerNode::callback_server, this, _1, _2));

            RCLCPP_INFO(this->get_logger(), "Server node is running ...");
        }
        
    private:
        void callback_server(const example_interfaces::srv::AddTwoInts::Request::SharedPtr req, const example_interfaces::srv::AddTwoInts::Response::SharedPtr res)
        {
            res->sum = req->a + req->b;
        }

        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}