#pragma once

#include <chrono>
#include <memory>
#include <thread>

#include <boost/asio.hpp>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rclcpp {
namespace executors {

class AsioSingleThreadedExecutor : public rclcpp::Executor {
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AsioSingleThreadedExecutor)

  RCLCPP_PUBLIC
  explicit AsioSingleThreadedExecutor(const rclcpp::ExecutorOptions& options = rclcpp::ExecutorOptions())
    : rclcpp::Executor(options), io_context_(), work_guard_(boost::asio::make_work_guard(io_context_)) {}

  RCLCPP_PUBLIC
  ~AsioSingleThreadedExecutor() { work_guard_.reset(); }

  RCLCPP_PUBLIC
  boost::asio::io_context& get_io_context() { return io_context_; }

  RCLCPP_PUBLIC
  void spin() {
    if (spinning.exchange(true)) {
      throw std::runtime_error("spin() called while already spinning");
    }
    RCPPUTILS_SCOPE_EXIT(wait_result_.reset(); this->spinning.store(false); work_guard_.reset(););

    wait_result_.reset();
    entities_need_rebuild_ = true;

    while (rclcpp::ok(context_) && spinning.load()) {
      rclcpp::AnyExecutable any_executable;
      if (get_next_executable(any_executable, std::chrono::milliseconds(0))) {
        execute_any_executable(any_executable);
      }

      io_context_.poll();

      if (!io_context_.stopped()) {
        std::this_thread::yield();
      }
    }

    io_context_.stop();
  }

  RCLCPP_PUBLIC
  void spin_some(std::chrono::nanoseconds max_duration = std::chrono::nanoseconds(0)) {
    auto start = std::chrono::steady_clock::now();
    auto max_duration_not_elapsed = [max_duration, start]() {
      if (std::chrono::nanoseconds(0) == max_duration) {
        return true;
      }
      return (std::chrono::steady_clock::now() - start) < max_duration;
    };

    while (rclcpp::ok(this->context_) && max_duration_not_elapsed()) {
      rclcpp::AnyExecutable any_executable;
      if (get_next_executable(any_executable, std::chrono::milliseconds(0))) {
        execute_any_executable(any_executable);
      }

      io_context_.poll_one();

      if (!max_duration_not_elapsed()) {
        break;
      }
    }
  }

private:
  RCLCPP_DISABLE_COPY(AsioSingleThreadedExecutor)

  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
};

}  // namespace executors
}  // namespace rclcpp
