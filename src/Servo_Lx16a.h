/**
 * @file   Servo_Lx16a.h
 * @author Michael Schmidpeter
 * @date   2019-10-24
 * @brief  todo
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */
#ifndef SERVO_LX16A_H_
#define SERVO_LX16A_H_

#include <iostream>
#include <cassert>
#include <functional>

#include <francor_base/angle.h>

#include "SerialCommand_Lx16a.h"

namespace francor::servo{


const double SERVO_RAD_MIN = -2.0944;
const double SERVO_RAD_MAX = 2.0944;
const double SERVO_RAD_MIN_EXTENDED = -2.61799;
const double SERVO_RAD_MAX_EXTENDED = 2.61799;
const uint16_t SERVO_POS_MIN = 0;
const uint16_t SERVO_POS_MAX = 1000;

const uint16_t SERVO_MAX_TIME = 10000; //for speed calc (lowest speed)


struct Params_Lx16a{
  const uint8_t     id;     //servo id
  const std::string name;   //name of servo (only for this lib)
  const double  max_v_in;   //max input voltage
  const double  min_v_in;   //min input voltage
  const int16_t max_temp;   //max possible temperature
  double  max_angle_rad;    //angle range 300° (extended) -> -+150° -> +2.62 rad
  double  min_angle_rad;    // -2.62 rad
  bool    mode_servo;       //true servo false motor

  // Params_Lx16a() = delete;
  Params_Lx16a(const uint8_t      id_, 
               const std::string& name_, 
               const double       max_v_in_,
               const double       min_v_in_,
               const int16_t      max_temp_,
               const double       max_angle_rad_,
               const double       min_angle_rad_,
               const bool         mode_servo_
               ) :
    id(id_),
    name(name_),
    max_v_in(max_v_in_),
    min_v_in(min_v_in_),
    max_temp(max_temp_),
    max_angle_rad(max_angle_rad_),
    min_angle_rad(min_angle_rad_),
    mode_servo(mode_servo_)
  { }
};

/**
 * @brief 
 * 
 * @todo add good default values
 * 
 */
struct Status_Lx16a{
  francor::base::NormalizedAngle pos;
  double  speed = 0;
  uint8_t  temp = 0;
  double v_in = 0;
  uint8_t  error_state = 0;
  Status_Lx16a() = default;
  Status_Lx16a(const francor::base::NormalizedAngle& pos_,
               const double  speed_,
               const uint8_t  temp_,
               const double v_in_,
               const uint8_t  error_state_
               ) :
    pos(pos_),
    speed(speed_),
    temp(temp_),
    v_in(v_in_),
    error_state(error_state_)
  { }
};

/**
 * @brief 
 * 
 * @todo extended mode via offset  //at first only use extended mode
 */
class Servo_Lx16a{
public:
  Servo_Lx16a(const Params_Lx16a& param) :
    _param(param)
  {

  }

  ~Servo_Lx16a()
  {}
  
  void init_servo()
  {
    this->set_min_max_v_in(_param.min_v_in, _param.max_v_in);
    this->set_min_max_angle(_param.min_angle_rad, _param.max_angle_rad);
  }

  void set_pos(const francor::base::NormalizedAngle& angle)
  {
    auto pos = this->rad_to_servoangle(angle.radian());
    auto data = SerialCommand_Lx16a::create_set_pos_cmd(_param.id, pos);
  }

  /**
   * @brief 
   * 
   * in servo mode rotates until max/min angle , in motor mode just speed //todo also check if possible in servomode (with stop at end ...)
   * todo use motor mode internaly for speed cmd in servo mode (needed for sensor head e.g.)
   * also todo check if send_pos with time is also an option for send speed in servo mode (for smooth moving)
   * 
   * @param speed -1.0 .. 1.0  -> max speed pos rot .. max speed neg rot
   * 
   */
  void set_speed(const double speed)
  {
    auto tmp_speed = constrain(speed, -1.0, 1.0);
    //stop if speed near 0
    if(std::abs(tmp_speed) < 0.00001)
    {
      //stop
      _send_serial_cb(SerialCommand_Lx16a::create_stop_cmd(_param.id));
    }

    //set min/max angle with given speed (converted to write time)
    //only for testing ... not wroking for end approach  todo compare with current angle
    uint16_t time = (1.0 - (1+std::abs(tmp_speed)) * 0.5) * SERVO_MAX_TIME;
    
    _send_serial_cb(SerialCommand_Lx16a::create_set_pos_cmd(_param.id, (speed > 0 ? SERVO_POS_MAX : SERVO_POS_MIN), time));
  } 

  Status_Lx16a get_status() const
  {
    return _status;
  }


  //notify cb
  void attach_status_chagned_callback(const std::function<void(const Status_Lx16a&)>& f)
  {
    _status_cb = f;
  }

  void attach_send_serial_callback(const std::function<void(std::vector<uint8_t>)>& f)
  {
    _send_serial_cb = f;
  }

//update call

  /**
   * @brief 
   * 
   * @todo may do this fnc friend or somthing (not callable from user)
   * 
   * @param status 
   */
  void update_status(const Status_Lx16a& status)
  {
    _status = status;
    if(_status_cb)
    {
      _status_cb(_status);
    }
  }

private:

  void set_min_max_v_in(const double min_v_in, const double max_v_in)
  {
    const uint16_t min_v = static_cast<uint16_t>(std::round(min_v_in * 1000));
    const uint16_t max_v = static_cast<uint16_t>(std::round(max_v_in * 1000));

    auto data = SerialCommand_Lx16a::create_set_v_limit_cmd(_param.id, min_v, max_v);

    assert(_send_serial_cb);
    _send_serial_cb(data);
    
  }

  void set_min_max_angle(const francor::base::NormalizedAngle& min_angle, const francor::base::NormalizedAngle& max_angle)
  {
    const uint16_t min_ang = this->rad_to_servoangle(min_angle.radian());
    const uint16_t max_ang = this->rad_to_servoangle(max_angle.radian());

    auto data = SerialCommand_Lx16a::create_set_angle_limit_cmd(_param.id, min_ang, max_ang);

    assert(_send_serial_cb);
    _send_serial_cb(data);
  }


  /**
   * @brief 
   * 500 angle = 0 deg 0 rad, 0 angle = -120 deg -2.0944 rad, 1000 angle = 120 deg 2.0944 rad
   * @param rad 
   * @return int16_t 
   */
  uint16_t rad_to_servoangle(const francor::base::NormalizedAngle& angle)
  {
    double rad = constrain(angle.radian(), SERVO_RAD_MIN, SERVO_RAD_MAX);
    
    return rescale(rad, SERVO_RAD_MIN, SERVO_RAD_MAX, SERVO_POS_MIN, SERVO_POS_MAX);
  }

  francor::base::NormalizedAngle servoangle_to_rad(const uint16_t servoangle)
  {
    uint16_t pos = constrain(servoangle, SERVO_POS_MIN, SERVO_POS_MAX);

    double rad = rescale(pos, SERVO_POS_MIN, SERVO_POS_MAX, SERVO_RAD_MIN, SERVO_RAD_MAX);
    return rad;
  }

  //a la arduino
  static inline double rescale(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static inline double constrain(const double vel, const double low, const double high)
  {
    return vel < low ? low : (vel > high ? high : vel);
  } 



private: //data
  std::function<void(const Status_Lx16a&)> _status_cb;
  std::function<void(const std::vector<uint8_t>&)> _send_serial_cb;

  Params_Lx16a _param;

  Status_Lx16a _status;

  //is used for extended mode
  int8_t  _offset;
  //desired value
  uint16_t _desired_pos;
  int16_t  _desired_speed;
};

} //ns 

inline std::ostream& operator<<(std::ostream& os, const francor::servo::Params_Lx16a& p)
{
  os << "lxa-16 Servo param:" << std::endl;
  os << "  id:            " << (int)p.id << std::endl;
  os << "  name:          " << p.name << std::endl;
  os << "  max_v_in:      " << p.max_v_in << std::endl;
  os << "  min_v_in:      " << p.min_v_in << std::endl;
  os << "  max_temp:      " << p.max_temp << std::endl;
  os << "  max_angle_rad: " << p.max_angle_rad << std::endl;
  os << "  min_angle_rad: " << p.min_angle_rad << std::endl;
  os << "  mode_servo:    " << p.mode_servo << std::endl;
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const francor::servo::Status_Lx16a& s)
{
  os << "lxy-16 Servo status:" << std::endl;
  os << "  pos: " << s.pos.radian() << std::endl;
  os << "  speed: " << s.speed << std::endl;
  os << "  temp: " << s.temp << std::endl;
  os << "  v_in: " << s.v_in << std::endl;
  os << "  error_state: " << s.error_state << std::endl;

  return os;
}

#endif  //SERVO_LX16A_H_