#ifndef FCUFLEX_CLIENT_HPP
#define FCUFLEX_CLIENT_HPP

#include <string>
#include <memory>
#include <nlohmann/json.hpp>
#include <boost/asio.hpp>

namespace fcuflex_ros2
{

class FCUFLEXClient
{
public:
  FCUFLEXClient();
  ~FCUFLEXClient();

  bool connect(const std::string& host, int port = 80);
  bool disconnect();
  bool isConnected() const;

  // For commands that return data
  nlohmann::json getParameter(const std::string& param);
  
  // For commands that set parameters
  bool setParameter(const std::string& param, const std::string& value);
  bool setParameter(const std::string& param, double value);
  bool setParameter(const std::string& param, int value);

  // Execute command with no parameters
  bool executeCommand(const std::string& command);

private:
  nlohmann::json sendHTTPRequest(const std::string& method, const std::string& request_path);
  nlohmann::json sendTelnetRequest(const std::string& request);
  
  std::unique_ptr<boost::asio::io_service> io_service_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;
  bool connected_;
  std::string host_;
  int port_;
};

} // namespace fcuflex_ros2

#endif // FCUFLEX_CLIENT_HPP