// Based on the AddTwoInts interface
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class ClientNode : public rclcpp::Node
{
    public:
        ClientNode() : Node("client_node")
        {   
            client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

            RCLCPP_INFO(this->get_logger(), "Client node is running ...");
        }

        void call_service(int a, int b)
        {
            while (!client_->wait_for_service(1s)){
                RCLCPP_WARN(this->get_logger(), "Waiting for the server ...");
            }

            auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            req->a = a;
            req->b = b;

            client_->async_send_request(req,std::bind(&ClientNode::callback_response, this, _1));
        }
        
    private:
        void callback_response(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
        {
            auto res = future.get();
            RCLCPP_INFO(get_logger(), "Sum: %d", (int)res->sum);
        }
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
}; 

int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClientNode>();
    node->call_service(3, 7);
    node->call_service(10, 3);
    node->call_service(5, 4);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}