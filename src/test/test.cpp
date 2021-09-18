#include "../SerialCommand_Lx16a.h"

#include "../francor/Serial.h"

#include "../Servo_Lx16a.h"

#include "../ConfigParser_Lx16a.h"

#include <thread>

francor::SerialCom com("/dev/ttyUSB0", francor::B_115200);

francor::servo::SerialParser_lx16a parser;

void read_thrd()
{
  // usleep(1000000);
  std::cout << "Enter Thrd" << std::endl;
  while(1)
  {
    uint8_t data = 0;
    try{
      com.receive(data);
    }
    catch(std::exception& e)
    {
      std::cout << "what?" << e.what() << std::endl;
      usleep(100000);
    }
    std::cout << "got byte" << std::endl;
    auto msg = parser.push_msg_byte(data);
    if(msg)
    {
      std::cout << "----> got msg" << std::endl;
      std::cout << "msg.value().id: " << (int)msg.value().id << std::endl;
      std::cout << "msg.value().cmd_id: " << (int)msg.value().cmd_id << std::endl;
      std::cout << "msg.value().param_1: " << (int)msg.value().param_1 << std::endl;
      std::cout << " msg.value().param_2: " <<  (int)msg.value().param_2 << std::endl;
    }

    // std::cout << (int)data << std::endl;
  }
}

int main(int argc, char const *argv[])
{
  //test xml
  // francor::XMLParser parser;
  // parser.load("hans");

  if(argc != 2)
  {
    std::cout << "Error at usage... use: " << argv[0] << "<config>" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  auto params = francor::servo::parseXML(argv[1]);
  for(auto& e : params)
  {
    std::cout << e << std::endl;
  }

  if(com.connect() != francor::DeviceSucces)
  {
    std::cout << "Error will exit" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  std::thread thrd(read_thrd);

  // int dings = 1;

  // int pos = 0;
  while(1)
  {
    auto data = francor::servo::SerialCommand_Lx16a::create_set_pos_cmd(2, 200, 1);
    std::cout << "size: " << data.size() << std::endl;
    // for(auto& e : data)
    // {
    //   std::cout << (int)e << std::endl;
    // }
    // std::cout << "" << std::endl;
    com.transmit(data);
    com.transmit(francor::servo::SerialCommand_Lx16a::create_get_angle_limit_cmd(2));
    // com.transmit(francor::servo::SerialCommand_Lx16a::create_get_pos_cmd(2));
    std::cout << "press enter" << std::endl;
    getchar();
    com.transmit(francor::servo::SerialCommand_Lx16a::create_set_pos_cmd(2, 800, 5000));
    com.transmit(francor::servo::SerialCommand_Lx16a::create_get_pos_cmd(2));
    std::cout << "press enter" << std::endl;
    getchar();
    // std::cout << "press enter" << std::endl;
    // getchar();


    
    // com.transmit(francor::SerialCommand_Lx16a::create_set_pos_cmd(2, pos));
    // usleep(5000);
    // pos += dings;
    // if(pos >= 1000)
    //   dings =  -2;
    // else if(pos <= 0)
    //   dings = 2;
  }


  return 0;
}
