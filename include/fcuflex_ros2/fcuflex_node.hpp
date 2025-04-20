#ifndef FCUFLEX_NODE_HPP
#define FCUFLEX_NODE_HPP

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
  // ROS parameters
  std::string fcuflex_host_;
  int fcuflex_port_;
  double update_rate_;
  
  // FCUFLEX client
  std::unique_ptr<FCUFLEXClient> client_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;
  
  // Publishers
  rclcpp::Publisher<msg::AFDStatus>::SharedPtr status_pub_;
  
  // Service servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr weigh_payload_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_force_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_position_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_control_mode_srv_;
  rclcpp::Service<srv::GetValue>::SharedPtr get_param_srv_;
  rclcpp::Service<srv::SetValue>::SharedPtr set_param_srv_;
  
  // Callbacks
  void updateCallback();
  void connectCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void disconnectCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void weighPayloadCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void setForceCallback(const std::shared_ptr<srv::SetValue::Request> req,
                       std::shared_ptr<srv::SetValue::Response> res);
  void setPositionCallback(const std::shared_ptr<srv::SetValue::Request> req,
                          std::shared_ptr<srv::SetValue::Response> res);
  void setControlModeCallback(const std::shared_ptr<srv::SetValue::Request> req,
                             std::shared_ptr<srv::SetValue::Response> res);
  void getParamCallback(const std::shared_ptr<srv::GetValue::Request> req,
                       std::shared_ptr<srv::GetValue::Response> res);
  void setParamCallback(const std::shared_ptr<srv::SetValue::Request> req,
                       std::shared_ptr<srv::SetValue::Response> res);
  
  // Helpers
  bool updateAFDStatus(msg::AFDStatus& status);
};

} // namespace fcuflex_ros2

#endif // FCUFLEX_NODE_HPP