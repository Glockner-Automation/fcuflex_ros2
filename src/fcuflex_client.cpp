#include "fcuflex_ros2/fcuflex_client.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

namespace fcuflex_ros2
{

// Constructor
FCUFLEXClient::FCUFLEXClient()
  : io_service_(std::make_unique<boost::asio::io_service>()),
    socket_(nullptr),
    connected_(false),
    host_(""),
    port_(0)
{
}

// Destructor
FCUFLEXClient::~FCUFLEXClient()
{
  disconnect();
}

// Connect to FCUFLEX
bool FCUFLEXClient::connect(const std::string& host, int port)
{
  if (connected_) {
    return true;  // Already connected
  }

  try {
    // Create socket
    socket_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_service_);
    
    // Resolve hostname to endpoint
    boost::asio::ip::tcp::resolver resolver(*io_service_);
    boost::asio::ip::tcp::resolver::query query(host, std::to_string(port));
    
    boost::system::error_code resolve_ec;
    auto endpoint_iterator = resolver.resolve(query, resolve_ec);
    if (resolve_ec) {
      // std::cerr << "Resolve error: " << resolve_ec.message() << std::endl;
      return false;
    }
    
    // Connect to the server with timeout
    boost::system::error_code connect_ec;
    boost::asio::connect(*socket_, endpoint_iterator, connect_ec);
    if (connect_ec) {
      // std::cerr << "Connect error: " << connect_ec.message() << std::endl;
      return false;
    }
    
    // Save connection details
    host_ = host;
    port_ = port;
    connected_ = true;
    
    // For HTTP port, we need to format requests differently
    if (port == 80) {
      auto response = getParameter("/fcu/modelName");
      if (response.contains("status") && response["status"] == "success") {
        return true;
      }
    } else if (port == 23) {
      // For Telnet, we need to handle initial banner
      boost::asio::streambuf response;
      boost::system::error_code error;
      boost::asio::read_until(*socket_, response, ">>", error);
      
      if (!error) {
        // Now send a test command
        auto response = getParameter("/fcu/modelName");
        if (response.contains("status") && response["status"] == "success") {
          return true;
        }
      }
    }
    
    // If we got here, the connection test failed
    disconnect();
    return false;
  } catch (const std::exception& e) {
    std::cerr << "Connection error: " << e.what() << std::endl;
    disconnect();
    return false;
  }
}

// Disconnect from FCUFLEX
bool FCUFLEXClient::disconnect()
{
  if (!connected_) {
    return true;  // Already disconnected
  }
  
  try {
    if (socket_ && socket_->is_open()) {
      // Gracefully close the TCP connection
      boost::system::error_code ec;
      socket_->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
      socket_->close();
    }
    socket_.reset();
    connected_ = false;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Disconnection error: " << e.what() << std::endl;
    return false;
  }
}

// Check if connected
bool FCUFLEXClient::isConnected() const
{
  return connected_ && socket_ && socket_->is_open();
}

// Get parameter from FCUFLEX
nlohmann::json FCUFLEXClient::getParameter(const std::string& param)
{
  if (!connected_) {
    nlohmann::json result;
    result["status"] = "fail";
    result["data"] = "Not connected to FCUFLEX";
    return result;
  }
  
  if (port_ == 80) {
    // HTTP format for port 80
    return sendHTTPRequest("GET", param);
  } else {
    // Telnet format for port 23
    return sendTelnetRequest(param);
  }
}

// Set parameter (string value)
bool FCUFLEXClient::setParameter(const std::string& param, const std::string& value)
{
  if (!connected_) {
    return false;
  }
  
  std::string request = param + "=" + value;
  nlohmann::json response;
  
  if (port_ == 80) {
    response = sendHTTPRequest("GET", request);
  } else {
    response = sendTelnetRequest(request);
  }
  
  return (response.contains("status") && response["status"] == "success");
}

// Set parameter (double value)
bool FCUFLEXClient::setParameter(const std::string& param, double value)
{
  if (!connected_) {
    return false;
  }
  
  std::string request = param + "=" + std::to_string(value);
  nlohmann::json response;
  
  if (port_ == 80) {
    response = sendHTTPRequest("GET", request);
  } else {
    response = sendTelnetRequest(request);
  }
  
  return (response.contains("status") && response["status"] == "success");
}

// Set parameter (int value)
bool FCUFLEXClient::setParameter(const std::string& param, int value)
{
  if (!connected_) {
    return false;
  }
  
  std::string request = param + "=" + std::to_string(value);
  nlohmann::json response;
  
  if (port_ == 80) {
    response = sendHTTPRequest("GET", request);
  } else {
    response = sendTelnetRequest(request);
  }
  
  return (response.contains("status") && response["status"] == "success");
}

// Execute command (no parameters)
bool FCUFLEXClient::executeCommand(const std::string& command)
{
  if (!connected_) {
    return false;
  }
  
  nlohmann::json response;
  
  if (port_ == 80) {
    response = sendHTTPRequest("GET", command);
  } else {
    response = sendTelnetRequest(command);
  }
  
  return (response.contains("status") && response["status"] == "success");
}

// Send HTTP request for web interface (port 80)
nlohmann::json FCUFLEXClient::sendHTTPRequest(const std::string& method, const std::string& request_path)
{
  nlohmann::json result;
  
  if (!connected_ || !socket_ || !socket_->is_open()) {
    result["status"] = "fail";
    result["data"] = "Not connected to FCUFLEX";
    return result;
  }
  
  try {
    // Format HTTP request
    std::stringstream request_stream;
    request_stream << method << " /" << request_path << " HTTP/1.1\r\n";
    request_stream << "Host: " << host_ << "\r\n";
    request_stream << "Accept: application/json\r\n";
    request_stream << "Connection: keep-alive\r\n";
    request_stream << "\r\n";
    
    std::string request = request_stream.str();
    
    // Send the request
    boost::asio::write(*socket_, boost::asio::buffer(request));
    
    // Receive the response
    boost::asio::streambuf response_buffer;
    boost::system::error_code error;
    
    // Read HTTP headers
    boost::asio::read_until(*socket_, response_buffer, "\r\n\r\n", error);
    if (error) {
      throw boost::system::system_error(error);
    }
    
    // Process the headers to get content length
    std::istream response_stream(&response_buffer);
    std::string http_version;
    response_stream >> http_version;
    
    unsigned int status_code;
    response_stream >> status_code;
    
    std::string status_message;
    std::getline(response_stream, status_message);
    
    // Check status code
    if (status_code != 200) {
      result["status"] = "fail";
      result["data"] = "HTTP error: " + std::to_string(status_code) + " " + status_message;
      return result;
    }
    
    // Read headers
    std::string header;
    int content_length = -1;
    while (std::getline(response_stream, header) && header != "\r") {
      if (header.substr(0, 16) == "Content-Length: ") {
        content_length = std::stoi(header.substr(16));
      }
    }
    
    // Read content
    if (content_length > 0) {
      // If Content-Length header present, read exact amount
      if (response_buffer.size() < static_cast<std::size_t>(content_length)) {
        boost::asio::read(*socket_, response_buffer, 
                        boost::asio::transfer_exactly(content_length - response_buffer.size()), 
                        error);
      }
    } else {
      // Otherwise, read until EOF
      while (boost::asio::read(*socket_, response_buffer, 
                              boost::asio::transfer_at_least(1), error)) {
        if (error)
          break;
      }
    }
    
    if (error && error != boost::asio::error::eof) {
      throw boost::system::system_error(error);
    }
    
    // Extract the content
    std::string response_body(
      boost::asio::buffer_cast<const char*>(response_buffer.data()), 
      response_buffer.size());
    
    // Parse the JSON response
    try {
      if (!response_body.empty()) {
        result = nlohmann::json::parse(response_body);
      } else {
        result["status"] = "fail";
        result["data"] = "Empty response";
      }
    } catch (const nlohmann::json::parse_error& e) {
      result["status"] = "fail";
      result["data"] = std::string("JSON parse error: ") + e.what() + 
                      ", Response: " + response_body;
    }
  } catch (const boost::system::system_error& e) {
    result["status"] = "fail";
    result["data"] = std::string("Communication error: ") + e.what();
    
    // Check if we need to reconnect
    if (e.code() == boost::asio::error::connection_reset ||
        e.code() == boost::asio::error::broken_pipe ||
        e.code() == boost::asio::error::connection_aborted) {
      connected_ = false;
    }
  } catch (const std::exception& e) {
    result["status"] = "fail";
    result["data"] = std::string("Error: ") + e.what();
  }
  
  return result;
}

// Send Telnet request for console interface (port 23)
nlohmann::json FCUFLEXClient::sendTelnetRequest(const std::string& request)
{
  nlohmann::json result;
  
  if (!connected_ || !socket_ || !socket_->is_open()) {
    result["status"] = "fail";
    result["data"] = "Not connected to FCUFLEX";
    return result;
  }
  
  try {
    // Clear any pending data in the buffer
    boost::asio::streambuf clear_buffer;
    boost::system::error_code clear_error;
    if (socket_->available() > 0) {
      boost::asio::read(*socket_, clear_buffer, 
                       boost::asio::transfer_at_least(socket_->available()),
                       clear_error);
    }
    
    // Ensure the request is properly terminated
    std::string full_request = request;
    if (full_request.back() != '\n') {
      full_request += '\n';
    }
    
    // Send the request
    boost::asio::write(*socket_, boost::asio::buffer(full_request));
    
    // Receive the response
    boost::asio::streambuf response_buffer;
    boost::system::error_code error;
    
    // Read until prompt (>>) or timeout
    size_t bytes_read = boost::asio::read_until(*socket_, response_buffer, ">>", error);
    
    if (error && error != boost::asio::error::eof) {
      throw boost::system::system_error(error);
    }
    
    // Extract the response text
    std::string response_str(
      boost::asio::buffer_cast<const char*>(response_buffer.data()), 
      bytes_read);
    
    // *** REMOVE ANY std::cerr OR std::cout STATEMENTS HERE ***
    // Do NOT print the raw response or any debug info to the console
    
    // Find where the actual response begins (after our command and newline)
    size_t cmd_pos = response_str.find(request);
    if (cmd_pos != std::string::npos) {
      // Look for the end of the echoed command line
      size_t start = response_str.find("\r\n", cmd_pos);
      if (start != std::string::npos) {
        start += 2; // Skip the \r\n
        
        // Find the end of the response (before the prompt)
        size_t end = response_str.find(">>", start);
        if (end != std::string::npos) {
          // Extract just the response portion
          std::string clean_response = response_str.substr(start, end - start);
          
          // Trim whitespace and control characters
          clean_response.erase(0, clean_response.find_first_not_of(" \r\n\t"));
          size_t last_non_ws = clean_response.find_last_not_of(" \r\n\t");
          if (last_non_ws != std::string::npos) {
            clean_response.erase(last_non_ws + 1);
          }
          
          // Check if it's an error response
          if (clean_response.find("Error:") == 0) {
            result["status"] = "fail";
            result["data"] = clean_response;
          }
          // Check if it's just "OK"
          else if (clean_response == "OK") {
            result["status"] = "success";
            result["data"] = "OK";
          }
          // It must be a value response
          else {
            result["status"] = "success";
            
            // Handle parameter values
            // Special case for parameter requests
            if (request.find("=") == std::string::npos) {
              // This is a get parameter request
              // Create a JSON object with the parameter name and value
              result["data"] = nlohmann::json::object();
              result["data"][request] = clean_response;
              
              // Try to convert numeric values
              try {
                double num_value = std::stod(clean_response);
                result["data"][request] = num_value;
              } catch (const std::exception&) {
                // Not a number, keep as string
              }
            } else {
              // This is a set parameter request
              result["data"] = "OK";
            }
          }
        }
      }
    }
    
    // If we couldn't parse the response properly, log and return error
    if (!result.contains("status")) {
      result["status"] = "fail";
      result["data"] = "Unable to parse response";
    }
    
  } catch (const boost::system::system_error& e) {
    // Use a ROS logger if available instead of std::cerr
    result["status"] = "fail";
    result["data"] = std::string("Communication error: ") + e.what();
    
    // Check if we need to reconnect
    if (e.code() == boost::asio::error::connection_reset ||
        e.code() == boost::asio::error::broken_pipe ||
        e.code() == boost::asio::error::connection_aborted) {
      connected_ = false;
    }
  } catch (const std::exception& e) {
    result["status"] = "fail";
    result["data"] = std::string("Error: ") + e.what();
  }
  
  return result;
}

} // namespace fcuflex_ros2