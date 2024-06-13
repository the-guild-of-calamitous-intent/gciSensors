#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gciSensors.hpp>

using namespace LSM6DSOX;
using namespace gci::sensors;


namespace myengine {

class HelloWorld : public rclcpp::Node {
public:
  HelloWorld(): Node("helloworld") {
    RCLCPP_INFO(this->get_logger(), "Hello World! CTRL+C to exit!");
  }
  ~HelloWorld() {
    RCLCPP_INFO(this->get_logger(), "Bye World!");
  }
};

} // namespace myengine


int main(int argc, char* argv[]) {
  gciLSM6DSOX IMU(LSM6DSOX_ADDRESS,1);
  IMU.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<myengine::HelloWorld>());
  rclcpp::shutdown();
  return 0;
}