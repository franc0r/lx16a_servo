#include <iostream>
#include <thread>

#include "../SerialServoHandler.h"



int main(int argc, char const *argv[])
{

  if(argc != 4)
  {
    std::cout << "Wrong Usage... try: " << argv[0] << " <serial_device> <id_old> <id_new>" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  francor::servo::SerialServoHandler handler(argv[1]);

  if(!handler.init())
  {
    std::cout << "Error at init Handler will exit" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  uint8_t id_old = std::stoi(argv[2]);
  uint8_t id_new = std::stoi(argv[3]);


  std::cout << "set new id(" << (int)id_new << ") to servo with current id: " << (int)id_old << std::endl;

  if(handler.set_id(id_old, id_new))
  {
    std::cout << "setting id was succesful" << std::endl;
  }
  else
  {
    std::cout << "ERROR!!! setting id was not succesful" << std::endl;
  }
  

  // auto ids = handler.find_servos();
  // std::cout << "---------------------------------------------" << std::endl;
  // std::cout << "found " << ids.size() << " Servos:" << std::endl;
  // for(auto& e : ids)
  // {
  //   std::cout << "ID: " << (int)e << std::endl;
  // }
  // std::cout << "---------------------------------------------" << std::endl;
  handler.shutdown();

  francor::Duration d(2.0);
  d.sleep();


  return 0;
}
