#include "fcuflex_ros2/fcuflex_node.hpp"

#include <functional>
#include <string>
#include <memory>
#include <stdexcept>

namespace fcuflex_ros2
{

FCUFLEXNode::FCUFLEXNode()
: Node("fcuflex_node")
{
  // Declare and get parameters
  this->declare_parameter("fcuflex_host", "192.168.0.12");
  this->declare_parameter("fcuflex_port", 80);
  this->declare_parameter("update_rate", 10.0);
  
  fcuflex_host_ = this->get_parameter("fcuflex_host").as_string();
  fcuflex_port_ = this->get_parameter("fcuflex_port").as_int();
  update_rate_ = this->get_parameter("update_rate").as_double();
  
  RCLCPP_INFO(this->get_logger(), "FCUFLEX host: %s, port: %d, update rate: %.1f Hz",
    fcuflex_host_.c_str(), fcuflex_port_, update_rate_);
  
  // Initialize client
  client_ = std::make_unique<FCUFLEXClient>();
  
  // Create publishers
  status_pub_ = this->create_publisher<fcuflex_ros2::msg::AFDStatus>(
    "~/status", 10);
  
  // Create service servers
  connect_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/connect",
    std::bind(&FCUFLEXNode::connectCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  disconnect_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/disconnect",
    std::bind(&FCUFLEXNode::disconnectCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  weigh_payload_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/weigh_payload",
    std::bind(&FCUFLEXNode::weighPayloadCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  set_force_srv_ = this->create_service<fcuflex_ros2::srv::SetValue>(
    "~/set_force",
    std::bind(&FCUFLEXNode::setForceCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  set_position_srv_ = this->create_service<fcuflex_ros2::srv::SetValue>(
    "~/set_position",
    std::bind(&FCUFLEXNode::setPositionCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  set_control_mode_srv_ = this->create_service<fcuflex_ros2::srv::SetValue>(
    "~/set_control_mode",
    std::bind(&FCUFLEXNode::setControlModeCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  get_param_srv_ = this->create_service<fcuflex_ros2::srv::GetValue>(
    "~/get_param",
    std::bind(&FCUFLEXNode::getParamCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  set_param_srv_ = this->create_service<fcuflex_ros2::srv::SetValue>(
    "~/set_param",
    std::bind(&FCUFLEXNode::setParamCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  
  // Create timer for periodic updates
  update_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / update_rate_),
    std::bind(&FCUFLEXNode::updateCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "FCUFLEX node initialized");
  
  // Try to connect automatically
  if (!client_->connect(fcuflex_host_, fcuflex_port_)) {
    RCLCPP_WARN(this->get_logger(), 
      "Could not connect to FCUFLEX at %s:%d. Use the 'connect' service to retry.",
      fcuflex_host_.c_str(), fcuflex_port_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Connected to FCUFLEX at %s:%d",
      fcuflex_host_.c_str(), fcuflex_port_);
  }
}

FCUFLEXNode::~FCUFLEXNode()
{
  if (client_ && client_->isConnected()) {
    client_->disconnect();
    RCLCPP_INFO(this->get_logger(), "Disconnected from FCUFLEX");
  }
}

void FCUFLEXNode::updateCallback()
{
  if (!client_->isConnected()) {
    return;
  }
  
  try {
    auto status = std::make_unique<fcuflex_ros2::msg::AFDStatus>();
    if (updateAFDStatus(*status)) {
      status_pub_->publish(std::move(status));
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error updating status: %s", e.what());
  }
}

void FCUFLEXNode::connectCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Unused
  
  if (client_->isConnected()) {
    response->success = true;
    response->message = "Already connected to FCUFLEX";
    return;
  }
  
  bool success = client_->connect(fcuflex_host_, fcuflex_port_);
  response->success = success;
  
  if (success) {
    response->message = "Successfully connected to FCUFLEX at " + 
                         fcuflex_host_ + ":" + std::to_string(fcuflex_port_);
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  } else {
    response->message = "Failed to connect to FCUFLEX at " + 
                         fcuflex_host_ + ":" + std::to_string(fcuflex_port_);
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void FCUFLEXNode::disconnectCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Unused
  
  if (!client_->isConnected()) {
    response->success = true;
    response->message = "Already disconnected from FCUFLEX";
    return;
  }
  
  bool success = client_->disconnect();
  response->success = success;
  
  if (success) {
    response->message = "Successfully disconnected from FCUFLEX";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  } else {
    response->message = "Failed to disconnect from FCUFLEX";
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
  }
}

void FCUFLEXNode::weighPayloadCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request; // Unused
  
  if (!client_->isConnected()) {
    response->success = false;
    response->message = "Not connected to FCUFLEX";
    return;
  }
  
  try {
    // Check if the device is in position to weigh
    auto weighAtPosResult = client_->getParameter("/afd/weighAtPosition");
    if (!weighAtPosResult.contains("data") || 
        !weighAtPosResult["data"].contains("/afd/weighAtPosition") ||
        weighAtPosResult["data"]["/afd/weighAtPosition"].get<int>() != 1) {
      response->success = false;
      response->message = "AFD is not in position to weigh. Check orientation.";
      return;
    }
    
    bool success = client_->executeCommand("/afd/weighPayload");
    response->success = success;
    
    if (success) {
      response->message = "Successfully started payload weighing procedure";
    } else {
      response->message = "Failed to start payload weighing procedure";
    }
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error weighing payload: ") + e.what();
  }
}

void FCUFLEXNode::setForceCallback(
  const std::shared_ptr<fcuflex_ros2::srv::SetValue::Request> request,
  std::shared_ptr<fcuflex_ros2::srv::SetValue::Response> response)
{
  if (!client_->isConnected()) {
    response->success = false;
    response->message = "Not connected to FCUFLEX";
    return;
  }
  
  try {
    double force_value = std::stod(request->value);
    bool success = client_->setParameter("/afd/commandForce", force_value);
    response->success = success;
    
    if (success) {
      response->message = "Successfully set command force to " + request->value;
    } else {
      response->message = "Failed to set command force";
    }
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error setting force: ") + e.what();
  }
}

void FCUFLEXNode::setPositionCallback(
  const std::shared_ptr<fcuflex_ros2::srv::SetValue::Request> request,
  std::shared_ptr<fcuflex_ros2::srv::SetValue::Response> response)
{
  if (!client_->isConnected()) {
    response->success = false;
    response->message = "Not connected to FCUFLEX";
    return;
  }
  
  try {
    double position_value = std::stod(request->value);
    bool success = client_->setParameter("/afd/commandPosition", position_value);
    response->success = success;
    
    if (success) {
      response->message = "Successfully set command position to " + request->value;
    } else {
      response->message = "Failed to set command position";
    }
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error setting position: ") + e.what();
  }
}

void FCUFLEXNode::setControlModeCallback(
  const std::shared_ptr<fcuflex_ros2::srv::SetValue::Request> request,
  std::shared_ptr<fcuflex_ros2::srv::SetValue::Response> response)
{
  if (!client_->isConnected()) {
    response->success = false;
    response->message = "Not connected to FCUFLEX";
    return;
  }
  
  try {
    int mode_value = std::stoi(request->value);
    // Validate mode
    if (mode_value != 0 && mode_value != 1 && mode_value != 3) {
      response->success = false;
      response->message = "Invalid control mode. Valid values are: 0 (Position), 1 (Force), 3 (Soft Touch)";
      return;
    }
    
    bool success = client_->setParameter("/afd/controlMode", mode_value);
    response->success = success;
    
    if (success) {
      std::string mode_name;
      switch (mode_value) {
        case 0: mode_name = "Position Mode"; break;
        case 1: mode_name = "Force Mode"; break;
        case 3: mode_name = "Soft Touch Mode"; break;
      }
      response->message = "Successfully set control mode to " + mode_name;
    } else {
      response->message = "Failed to set control mode";
    }
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error setting control mode: ") + e.what();
  }
}

void FCUFLEXNode::getParamCallback(
    const std::shared_ptr<fcuflex_ros2::srv::GetValue::Request> request,
    std::shared_ptr<fcuflex_ros2::srv::GetValue::Response> response)
  {
    if (!client_->isConnected()) {
      response->success = false;
      response->message = "Not connected to FCUFLEX";
      return;
    }
    
    try {
      auto result = client_->getParameter(request->param);
      
      // Changed from INFO to DEBUG to reduce console output
      RCLCPP_DEBUG(this->get_logger(), "Parameter response: %s", result.dump().c_str());
      
      if (result.contains("status") && result["status"] == "success" && 
          result.contains("data")) {
        response->success = true;
        
        // Extract the value, which could be of different types
        auto& data = result["data"];
        auto key = request->param;
        if (data.contains(key)) {
          auto& value = data[key];
          if (value.is_string()) {
            response->value = value.get<std::string>();
          } else if (value.is_number_integer()) {
            response->value = std::to_string(value.get<int>());
          } else if (value.is_number_float()) {
            response->value = std::to_string(value.get<double>());
          } else if (value.is_boolean()) {
            response->value = value.get<bool>() ? "true" : "false";
          } else {
            response->value = value.dump();
          }
          response->message = "Parameter retrieved successfully";
        } else if (data.is_string()) {
          // For simple string responses (Telnet interface might return)
          response->value = data.get<std::string>();
          response->message = "Parameter retrieved successfully";
        } else {
          response->success = false;
          response->message = "Parameter not found in response";
        }
      } else {
        response->success = false;
        response->message = "Error retrieving parameter";
        if (result.contains("data")) {
          if (result["data"].is_string()) {
            response->message += ": " + result["data"].get<std::string>();
          } else {
            response->message += ": " + result["data"].dump();
          }
        }
      }
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Error getting parameter: ") + e.what();
    }
  }

void FCUFLEXNode::setParamCallback(
    const std::shared_ptr<fcuflex_ros2::srv::SetValue::Request> request,
    std::shared_ptr<fcuflex_ros2::srv::SetValue::Response> response)
{
    if (!client_->isConnected()) {
        response->success = false;
        response->message = "Not connected to FCUFLEX";
        return;
    }

    try {
        // Try to determine the type of the value
        bool success = false;
        
        // Try as integer
        try {
        int int_value = std::stoi(request->value);
        success = client_->setParameter(request->param, int_value);
        } catch (const std::invalid_argument&) {
        // Not an integer, try as double
        try {
            double double_value = std::stod(request->value);
            success = client_->setParameter(request->param, double_value);
        } catch (const std::invalid_argument&) {
            // Not a number, treat as string
            success = client_->setParameter(request->param, request->value);
        }
        }
        
        response->success = success;
        if (success) {
        response->message = "Successfully set parameter " + request->param;
        } else {
        response->message = "Failed to set parameter " + request->param;
        }
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Error setting parameter: ") + e.what();
    }
}

bool FCUFLEXNode::updateAFDStatus(fcuflex_ros2::msg::AFDStatus& status)
{
  try {
    // Get parameters individually since stateObject might not work with Telnet
    auto actualForce = client_->getParameter("/afd/actualForce");
    if (actualForce.contains("status") && actualForce["status"] == "success" && 
        actualForce.contains("data") && actualForce["data"].contains("/afd/actualForce")) {
      
      RCLCPP_DEBUG(this->get_logger(), "Actual force: %s", 
                  actualForce["data"]["/afd/actualForce"].dump().c_str());
      
      status.actual_force = actualForce["data"]["/afd/actualForce"].get<double>();
    }
    
    auto actualPosition = client_->getParameter("/afd/actualPosition");
    if (actualPosition.contains("status") && actualPosition["status"] == "success" && 
        actualPosition.contains("data") && actualPosition["data"].contains("/afd/actualPosition")) {
      status.actual_position = actualPosition["data"]["/afd/actualPosition"].get<double>();
    }
    
    auto commandForce = client_->getParameter("/afd/commandForce");
    if (commandForce.contains("status") && commandForce["status"] == "success" && 
        commandForce.contains("data") && commandForce["data"].contains("/afd/commandForce")) {
      status.command_force = commandForce["data"]["/afd/commandForce"].get<double>();
    }
    
    auto commandPosition = client_->getParameter("/afd/commandPosition");
    if (commandPosition.contains("status") && commandPosition["status"] == "success" && 
        commandPosition.contains("data") && commandPosition["data"].contains("/afd/commandPosition")) {
      status.command_position = commandPosition["data"]["/afd/commandPosition"].get<double>();
    }
    
    auto payloadWeight = client_->getParameter("/afd/payloadWeight");
    if (payloadWeight.contains("status") && payloadWeight["status"] == "success" && 
        payloadWeight.contains("data") && payloadWeight["data"].contains("/afd/payloadWeight")) {
      status.payload_weight = payloadWeight["data"]["/afd/payloadWeight"].get<double>();
    }
    
    auto accelGravity = client_->getParameter("/afd/accelGravity");
    if (accelGravity.contains("status") && accelGravity["status"] == "success" && 
        accelGravity.contains("data") && accelGravity["data"].contains("/afd/accelGravity")) {
      status.accel_gravity = accelGravity["data"]["/afd/accelGravity"].get<double>();
    }
    
    auto controlMode = client_->getParameter("/afd/controlMode");
    if (controlMode.contains("status") && controlMode["status"] == "success" && 
        controlMode.contains("data") && controlMode["data"].contains("/afd/controlMode")) {
      status.control_mode = controlMode["data"]["/afd/controlMode"].get<int>();
    }
    
    auto toolCom = client_->getParameter("/fcu/toolCom");
    if (toolCom.contains("status") && toolCom["status"] == "success" && 
        toolCom.contains("data") && toolCom["data"].contains("/fcu/toolCom")) {
      status.tool_connected = toolCom["data"]["/fcu/toolCom"].get<int>() == 1;
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error updating AFD status: %s", e.what());
    return false;
  }
}

} // namespace fcuflex_ros2

// Entry point
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fcuflex_ros2::FCUFLEXNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}