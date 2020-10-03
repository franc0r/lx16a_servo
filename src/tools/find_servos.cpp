#include <iostream>
#include <thread>

#include "../SerialServoHandler.h"



int main(int argc, char const *argv[])
{

  if(argc != 2)
  {
    std::cout << "Wrong Usage... try: " << argv[0] << " <serial_device>" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  francor::servo::SerialServoHandler handler(argv[1]);

  if(!handler.init())
  {
    std::cout << "Error at init Handler will exit" << std::endl;
    ::exit(EXIT_FAILURE);
  }

  auto ids = handler.find_servos();
  std::cout << "---------------------------------------------" << std::endl;
  std::cout << "found " << ids.size() << " Servos:" << std::endl;
  for(auto& e : ids)
  {
    std::cout << "ID: " << (int)e << std::endl;
  }
  std::cout << "---------------------------------------------" << std::endl;
  handler.shutdown();

  francor::Duration d(2.0);
  d.sleep();


  return 0;
}
