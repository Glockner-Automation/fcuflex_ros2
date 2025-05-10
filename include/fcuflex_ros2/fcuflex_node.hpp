#ifndef FCUFLEX_NODE_HPP_
#define FCUFLEX_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "fcuflex_ros2/msg/afd_status.hpp"
#include "fcuflex_ros2/srv/get_value.hpp"
#include "fcuflex_ros2/srv/set_value.hpp"
#include "fcuflex_ros2/fcuflex_client.hpp"

namespace fcuflex_ros2
{

class FCUFLEXNode : public rclcpp::Node
{
public:
  FCUFLEXNode();
  ~FCUFLEXNode();

private:
  // FCUFLEX client
  std::shared_ptr<FCUFLEXClient> client_;

  // Publisher for status messages
  rclcpp::Publisher<msg::AFDStatus>::SharedPtr status_pub_;

  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;

  // Configuration parameters for connection
  std::string fcuflex_host_;
  int fcuflex_port_;

  // Trigger services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr weigh_payload_srv_;

  // Services for setting commands
  rclcpp::Service<srv::SetValue>::SharedPtr set_force_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_position_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_control_mode_srv_;

  // Services for generic parameter get/set
  rclcpp::Service<srv::GetValue>::SharedPtr get_param_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_param_srv_;

  // Callback methods
  void updateCallback();
  bool updateAFDStatus(msg::AFDStatus & status);

  void connectCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void disconnectCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void weighPayloadCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void setForceCallback(
    const std::shared_ptr<srv::SetValue::Request> request,
    std::shared_ptr<srv::SetValue::Response> response);

  void setPositionCallback(
    const std::shared_ptr<srv::SetValue::Request> request,
    std::shared_ptr<srv::SetValue::Response> response);

  void setControlModeCallback(
    const std::shared_ptr<srv::SetValue::Request> request,
    std::shared_ptr<srv::SetValue::Response> response);

  void getParamCallback(
    const std::shared_ptr<srv::GetValue::Request> request,
    std::shared_ptr<srv::GetValue::Response> response);

  void setParamCallback(
    const std::shared_ptr<srv::SetValue::Request> request,
    std::shared_ptr<srv::SetValue::Response> response);

  // Node configuration parameters
  std::string topic_name_;
  size_t max_points_;
  double update_rate_;
  double force_min_;
  double force_max_;
  double position_min_;
  double position_max_;
};

} // namespace fcuflex_ros2

#endif // FCUFLEX_NODE_HPP_

