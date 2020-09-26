#ifndef LX16A_CMD_H_
#define LX16A_CMD_H_

#include <iostream>
#include <vector>
#include <cstdint>
#include <optional>

namespace francor::servo{

constexpr uint8_t                     LOBOT_SERVO_FRAME_HEADER         = 0x55;
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_MOVE_TIME_WRITE      = std::make_pair(1,  7);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_MOVE_TIME_READ       = std::make_pair(2,  3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = std::make_pair(7,  7);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_MOVE_TIME_WAIT_READ  = std::make_pair(8,  3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_MOVE_START           = std::make_pair(11, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_MOVE_STOP            = std::make_pair(12, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ID_WRITE             = std::make_pair(13, 4);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ID_READ              = std::make_pair(14, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = std::make_pair(17, 4);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ANGLE_OFFSET_WRITE   = std::make_pair(18, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ANGLE_OFFSET_READ    = std::make_pair(19, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ANGLE_LIMIT_WRITE    = std::make_pair(20, 7);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_ANGLE_LIMIT_READ     = std::make_pair(21, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_VIN_LIMIT_WRITE      = std::make_pair(22, 7);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_VIN_LIMIT_READ       = std::make_pair(23, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = std::make_pair(24, 4);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = std::make_pair(25, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_TEMP_READ            = std::make_pair(26, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_VIN_READ             = std::make_pair(27, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_POS_READ             = std::make_pair(28, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = std::make_pair(29, 7);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_OR_MOTOR_MODE_READ   = std::make_pair(30, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = std::make_pair(31, 4);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = std::make_pair(32, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_LED_CTRL_WRITE       = std::make_pair(33, 4);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_LED_CTRL_READ        = std::make_pair(34, 3);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_LED_ERROR_WRITE      = std::make_pair(35, 4);
constexpr std::pair<uint8_t, uint8_t> LOBOT_SERVO_LED_ERROR_READ       = std::make_pair(36, 3);

class SerialCommand_Lx16a{
public:
  

  inline static uint8_t compute_checksum(const std::vector<uint8_t>& data)
  {
    if(data.size() < 6)
    {
      std::cout << "ERROR.... could not attach checksum (too less data)" << std::endl;
      return 0;
    }
    
    unsigned int temp = 0;
    for(unsigned int i = 2; i < data.size() -1; i++)
    {
      temp += data[i];
    }
    temp = ~temp;
    return static_cast<uint8_t>(temp);
  }


  inline static std::vector<uint8_t> create_set_pos_cmd(const uint8_t id, const uint16_t pos, const uint16_t time = 1)
  {
    uint16_t tmp_pos = constrain<uint16_t>(pos, 0, 1000);

    auto data = init_cmd(LOBOT_SERVO_MOVE_TIME_WRITE, id);
    data[5] = get_low_byte(tmp_pos);
    data[6] = get_high_byte(tmp_pos);
    data[7] = get_low_byte(time);
    data[8] = get_high_byte(time);
    data[9] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_stop_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_MOVE_STOP, id);
    data[5] = compute_checksum(data);
    return data;  
  }

  inline static std::vector<uint8_t> create_set_id_cmd(const uint8_t id_old, const uint8_t id_new)
  {
    //for now not use (untested)
    std::vector<uint8_t> ret;
    return ret;


    auto data = init_cmd(LOBOT_SERVO_ID_WRITE, id_old);
    data[5] = id_new;
    data[6] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_id_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_ID_READ, id);
    data[5]   = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_set_angle_offset_cmd(const uint8_t id, const int8_t offset)
  {
    auto tmp_offset = constrain<uint8_t>(offset, -125, 125);

    auto data = init_cmd(LOBOT_SERVO_ANGLE_OFFSET_ADJUST, id);
    data[5] = tmp_offset;
    data[6] = compute_checksum(data);
    
    return data;
  } 

  inline static std::vector<uint8_t> create_set_flash_angle_offset_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_ANGLE_OFFSET_WRITE, id);
    data[5] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_angle_offset_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_ANGLE_OFFSET_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }


  inline static std::vector<uint8_t> create_set_angle_limit_cmd(const uint8_t id, const uint16_t min_angle, const uint16_t max_angle)
  {
    auto tmp_min = constrain<uint16_t>(min_angle, 0, 1000);
    auto tmp_max = constrain<uint16_t>(max_angle, 0, 1000);

    //switch min max if needed
    if(tmp_min > tmp_max) 
    {
      auto tmp = tmp_min;
      tmp_min = tmp_max;
      tmp_max = tmp;
    }


    auto data = init_cmd(LOBOT_SERVO_ANGLE_LIMIT_WRITE, id);
    data[5] = get_low_byte(min_angle);
    data[6] = get_high_byte(min_angle);
    data[7] = get_low_byte(max_angle);
    data[8] = get_high_byte(max_angle);
    data[9] = compute_checksum(data);

    return data;
  }

  inline static std::vector<uint8_t> create_get_angle_limit_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_ANGLE_LIMIT_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_set_v_limit_cmd(const uint8_t id, const uint16_t min_v, const uint16_t max_v)
  {
    auto tmp_min = constrain<uint16_t>(min_v, 4500, 12000);
    auto tmp_max = constrain<uint16_t>(max_v, 4500, 12000);

    //switch min max if needed
    if(tmp_min > tmp_max) 
    {
      auto tmp = tmp_min;
      tmp_min = tmp_max;
      tmp_max = tmp;
    }

    auto data = init_cmd(LOBOT_SERVO_VIN_LIMIT_WRITE, id);
    data[5] = get_low_byte(min_v);
    data[6] = get_high_byte(min_v);
    data[7] = get_low_byte(max_v);
    data[8] = get_high_byte(max_v);
    data[9] = compute_checksum(data);

    return data;
  }
  
  inline static std::vector<uint8_t> create_get_v_limit_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_VIN_LIMIT_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_set_temp_limit_cmd(const uint8_t id, const uint8_t max_temp)
  {
    auto tmp_temp = constrain<uint8_t>(max_temp, 0, 85);
    auto data = init_cmd(LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, id);
    data[5] = tmp_temp;
    data[6] = compute_checksum(data);

    return data;
  }

  inline static std::vector<uint8_t> create_get_temp_limit_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_TEMP_MAX_LIMIT_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_temp_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_TEMP_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }
 
  inline static std::vector<uint8_t> create_get_v_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_VIN_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_pos_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_POS_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }
  
  /**
   * @brief Create a set mode cmd object
   * 
   * @param id 
   * @param mode false = servo, true = motor
   * @param speed 
   * @return std::vector<uint8_t> 
   */
  inline static std::vector<uint8_t> create_set_mode_cmd(const uint8_t id, const bool mode, const int16_t speed)
  {
    auto tmp_speed = constrain<int16_t>(speed, -1000, 1000);

    auto data = init_cmd(LOBOT_SERVO_OR_MOTOR_MODE_WRITE, id);
    data[5] = (mode ? 1 : 0);
    data[6] = 0;
    data[7] = get_low_byte(tmp_speed);
    data[8] = get_high_byte(tmp_speed);
    data[9] = compute_checksum(data);

    return data;
  }

  inline static std::vector<uint8_t> create_get_mode_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_OR_MOTOR_MODE_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }
  
  inline static std::vector<uint8_t> create_set_load_cmd(const uint8_t id, const bool enable)
  {
    auto data = init_cmd(LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, id);
    data[5] = (enable ? 1 : 0);
    data[6] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_load_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_LOAD_OR_UNLOAD_READ, id);
    data[5] = compute_checksum(data);
    return data;

  }
  
  inline static std::vector<uint8_t> create_set_led_cmd(const uint8_t id, const bool enable)
  {
    auto data = init_cmd(LOBOT_SERVO_LED_CTRL_WRITE, id);
    data[5] = (enable ? 1 : 0);
    data[6] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_led_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_LED_CTRL_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }

  inline static std::vector<uint8_t> create_get_error_cmd(const uint8_t id)
  {
    auto data = init_cmd(LOBOT_SERVO_LED_ERROR_READ, id);
    data[5] = compute_checksum(data);
    return data;
  }

private:

  inline static uint8_t get_low_byte(const uint16_t data) 
  {
    return static_cast<uint8_t>(data);
  }
  
  inline static uint8_t get_high_byte(const uint16_t data)
  {
    return static_cast<uint8_t>(data >> 8);
  }



  /**
   * @brief inits and creates the command vector filled with init data (header, size, id, command_id ) fills elements until [4]
   * 
   * structure of serial pkg
   * Header (2Byte) | ID | number Data Length | Command | Parameter      | Checksum
   * 0x55 0x55      | ID | Length             | Cmd     | Prm 1... Prm N | Checksum
   * 
   * @param cmd cmd pair (first cmd id, second size of cmd msg) 
   * @param id of servo
   * @return std::vector<uint8_t> serial cmd message (init not rdy..)
   */
  inline static std::vector<uint8_t> init_cmd(const std::pair<uint8_t, uint8_t> cmd, const uint8_t id)
  {
    int tmp_size = cmd.second + 3; //plus 3 because of header and checksum
    if(tmp_size < 6)
    {
      tmp_size = 6;
      std::cout << "warning size for init cmd is too small minimum is 6... will return size 6" << std::endl;
    }
    std::vector<uint8_t> data(tmp_size, 0);
    data[0] = data[1] = LOBOT_SERVO_FRAME_HEADER;
    data[2] = id;
    data[3] = cmd.second;
    data[4] = cmd.first;

    return data;
  }

  // Checksum:The calculation method is as follows:
  // Checksum=~(ID+ Length+Cmd+ Prm1+...PrmN)If the numbers in the
  // brackets are calculated and exceeded 255,Then take the lowest one byte, "~"
  // means Negation.



  //a la arduino
  // static inline double rescale(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
  // {
  //   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // }
  template<typename T>
  inline static T constrain(const T vel, const T low, const T high)
  {
    return vel < low ? low : (vel > high ? high : vel);
  } 

};


struct SerialReturn_Lx16a{
  uint8_t id;
  uint8_t cmd_id;
  uint16_t param_1;
  uint16_t param_2;

  SerialReturn_Lx16a(const uint8_t id_, const uint8_t cmd_id_, const uint16_t param_1_, const uint16_t param_2_) :
    id(id_), cmd_id(cmd_id_), param_1(param_1_), param_2(param_2_)
  { }
  SerialReturn_Lx16a() = delete;
};


class SerialParser_lx16a{
public:
  SerialParser_lx16a()
  {
    this->reset();
  }
  ~SerialParser_lx16a()
  { }

  /**
   * @brief parses the input byte by byte and returns the Serial Return if msg id rdy (std::optional)
   * 
   * @param byte byte got by Serial
   * @return std::optional<SerialCommand_Lx16a> empty if msg not rdy or Serial Return
   */
  std::optional<SerialReturn_Lx16a> push_msg_byte(const uint8_t byte)
  { 
    std::optional<SerialReturn_Lx16a> ret;

    if(msg.size() <= 1)     //read header
    {
      if(byte == LOBOT_SERVO_FRAME_HEADER)
      {
        msg.push_back(byte);
      }
      else
      {
        std::cout << "expected start byte, but wasnt..." << std::endl;
        this->reset();
        return ret;
      }
    }
    else if(msg.size() <= 4) //read id, length, cmd
    {
      msg.push_back(byte);
      
      if(msg.size() == 4) //check length
      {
        if(msg[3] < 4 || msg[3] > 7)
        {
          std::cout << "received length out of bounds" << std::endl;
          this->reset();
          return ret;
        }
      }
    }
    else //read rest + finish msg
    {
      msg.push_back(byte);

      if(msg.size() == msg[3] + 3u) // msg rdy -> check checksum
      {
        if(msg.back() != SerialCommand_Lx16a::compute_checksum(msg))
        {
          std::cout << "checksum error" << std::endl;
          this->reset();
          return ret;
        }
        
        uint16_t param_1 = 0;
        uint16_t param_2 = 0;
        switch(msg[3]) {
          case 4:
            param_1 = msg[5];
            break;
          case 5:
            param_1 = from_low_high_byte(msg[5], msg[6]);
            break;
          case 7:
            param_1 = from_low_high_byte(msg[5], msg[6]);
            param_2 = from_low_high_byte(msg[7], msg[8]);
            break;
          default:
            std::cout << "internal error while processing params" << std::endl;
            break;
        }

        SerialReturn_Lx16a serial_ret(msg[2], msg[4], param_1, param_2);
        this->reset(); //make clear for new msg
        ret = serial_ret;
      }
    }
    return ret;
  }
  
private:
  std::vector<uint8_t> msg;

  void reset()
  {
    msg.clear();
    msg.reserve(10);
  }

  inline static uint16_t from_low_high_byte(const uint8_t low_byte, const uint8_t high_byte)
  {
    return static_cast<uint16_t>(low_byte) + (static_cast<uint16_t>(high_byte) << 8);
  }

};



} //namespace francor

#endif  //LX16A_CMD_H_