#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_asio_executor/asio_single_threaded_executor.hpp"
#include <boost/asio.hpp>

using namespace std::chrono_literals;

class AsioExampleNode : public rclcpp::Node {
public:
  AsioExampleNode(boost::asio::io_context& io_context)
    : Node("asio_example_node"), io_context_(io_context), counter_(0) {
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("status", 10);
    async_task_service_ = this->create_service<std_srvs::srv::Trigger>(
      "async_task",
      [this](const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
             const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<std_srvs::srv::Trigger::Request>) {

        RCLCPP_INFO(this->get_logger(), "Received async_task service call");

        boost::asio::co_spawn(
          io_context_,
          [this, service_handle, request_header]() -> boost::asio::awaitable<void> {
            auto response = std::make_shared<std_srvs::srv::Trigger::Response>();

            auto status_msg = std_msgs::msg::String();
            status_msg.data = "Starting async task...";
            status_publisher_->publish(status_msg);

            boost::asio::steady_timer timer(io_context_, 2s);
            co_await timer.async_wait(boost::asio::use_awaitable);
            counter_++;
            response->success = true;
            response->message = "Async task completed successfully! Counter: " + std::to_string(counter_);

            RCLCPP_INFO(this->get_logger(), "Async task completed: %s", response->message.c_str());

            status_msg.data = "Async task completed!";
            status_publisher_->publish(status_msg);
            service_handle->send_response(*request_header, *response);
          },
          boost::asio::detached);
      });
    }

private:
  boost::asio::io_context& io_context_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr async_task_service_;
  int counter_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::AsioSingleThreadedExecutor>();
  auto node = std::make_shared<AsioExampleNode>(executor->get_io_context());
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}

