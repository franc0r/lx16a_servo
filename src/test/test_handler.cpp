#include "../SerialServoHandler.h"



void servo_status_cb(const uint8_t id, const francor::servo::Status_Lx16a& status)
{
  std::cout << "id: " << (int)id  << std::endl;
  std::cout << status << std::endl;
}



int main(int argc, char const *argv[])
{
  if(argc != 2)
  {
    std::cout << "Wronge Usage... use: " << argv[0] << "<path_to_xml_config>" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  francor::servo::SerialServoHandler handler("/dev/ttyUSB0", argv[1]);

  if(!handler.init())
  {
    std::cout << "Error and init handler will exit" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  handler.attach_servo_status_update_callback(std::bind(servo_status_cb, std::placeholders::_1, std::placeholders::_2));
  handler.set_rate_pos_request(20);
  handler.set_rate_speed_request(20);

  handler.spin();

  return 0;
}
