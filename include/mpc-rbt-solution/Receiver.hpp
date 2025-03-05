#ifndef COMMUNICATION_EXAMPLE_RECEIVER_H
#define COMMUNICATION_EXAMPLE_RECEIVER_H

#include <mpc-rbt-solution/util/Socket.hpp>
#include <mpc-rbt-solution/util/Utils.hpp>

namespace Receiver
{
class Node : public Socket::UDP
{
public:
  explicit Node(const Utils::Config::Receiver & receiverConfig)
  : Socket::UDP(receiverConfig.localPort), config(receiverConfig)
  {
    UDP udp(128);
    udp.create();
    udp.configure();
    udp.bind();
    callback = std::bind(&Receiver::Node::onDataReceived(const Socket::IPFrame & frame), this, std::placeholders::_1); 
    //UNIMPLEMENTED(__PRETTY_FUNCTION__);
  }

  void run();

private:
  void onDataReceived(const Socket::IPFrame & frame);

  Utils::Message data{};

  Utils::Config::Receiver config;
  std::function<void(const Socket::IPFrame &)> callback;
  rclcpp::Logger logger{rclcpp::get_logger("receiver")};

  friend class TestMock;
};
}  // namespace Receiver

namespace Receiver
{
class TestMock : public Node
{
public:
  TestMock(const Utils::Config::Receiver & receiverConfig) : Receiver::Node(receiverConfig) {}

  [[nodiscard]] const Utils::Message & getData() const { return data; }
};
}  // namespace Receiver

#endif  // COMMUNICATION_EXAMPLE_RECEIVER_H
