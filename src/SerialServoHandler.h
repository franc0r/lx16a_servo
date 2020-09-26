/**
 * @file   SerialServoHandler.h
 * @author Michael Schmidpeter
 * @date   2019-10-24
 * @brief  handles the serial port, reads input handles it to servo classes
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */

#ifndef SERIALSERVOHANDLER_H_
#define SERIALSERVOHANDLER_H_

namespace francor::servo{


/**
 * @brief 
 * @todo some kind of notify via callbacks like when updates speed or pos or ... callback says id of servo
 * @todo sinleton 
 * @todo function for sending cmds e.g. from servo class
 * 
 */
class SerialServoHandler{
public:
  SerialServoHandler();
  ~SerialServoHandler();
  
private:
};

} //namespace francor::servo

#endif  //SERIALSERVOHANDLER_H_