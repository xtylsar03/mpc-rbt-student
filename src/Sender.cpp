#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  data.x += 1.5;
  data.y += 2.0;
  data.z -= 0.5;

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };

  // Používáme hotovou funkci od autorů
  Utils::Message::serialize(frame, data);

  if (!this->send(frame)) {
      RCLCPP_WARN(logger, "Failed to send data!");
  }

  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);
  RCLCPP_INFO(logger, "\n\tstamp: %ld\n\tx: %f\n\ty: %f\n\tz: %f", data.timestamp, data.x, data.y, data.z);
}
