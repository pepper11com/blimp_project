#ifndef BLIMP_NAVIGATION_CPP__BLIMP_CONTROLLER_HPP_
#define BLIMP_NAVIGATION_CPP__BLIMP_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <asio.hpp>

namespace blimp_navigation_cpp
{

class BlimpController
{
public:
  BlimpController(const std::string & device, int baud_rate);
  ~BlimpController();

  void set_servo_us(size_t idx, int us);
  void set_motor_pct(size_t motor_idx, double pct);

private:
  void sender_loop();
  void send_servos();
  void send_motors();

  asio::io_context io_ctx_;
  asio::serial_port serial_port_;
  
  std::mutex mtx_;
  std::vector<uint16_t> servo_values_;
  std::vector<uint16_t> motor_values_;
  
  std::atomic<bool> run_thread_;
  std::thread sender_thread_;
};

}  // namespace blimp_navigation_cpp

#endif  // BLIMP_NAVIGATION_CPP__BLIMP_CONTROLLER_HPP_