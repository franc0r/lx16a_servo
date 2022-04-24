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

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

#include "Servo_Lx16a.h"
#include "ConfigParser_Lx16a.h"

#include "francor/Serial.h"
#include "francor/Timer.h"

namespace francor::servo{


/**
 * @brief 

 * 
 */
class SerialServoHandler{
public:
  SerialServoHandler(const std::string& serial_port, 
                     const std::string& path_to_cfg  = "") :
    _serial_port(serial_port, B_115200),
    _path_to_xml_cfg(path_to_cfg)
  { 
    if(path_to_cfg.empty())
    {
      _cfg_mode = false;
    }
  }
  ~SerialServoHandler() { }

  bool init()
  {
    if(_cfg_mode)
    {
      //read XML config:
      std::vector<Params_Lx16a> servo_params;
      try{
        servo_params = parseXML(_path_to_xml_cfg);
      }
      catch(const std::invalid_argument& e){
        std::cout << "Error while reading Config: " << e.what() << std::endl;
        return false;
      }


      //create Servos...
      for(auto& e : servo_params)
      {
        _servos.insert(std::make_pair(e.id, Servo_Lx16a(e)));
        _servos.at(e.id).attach_send_serial_callback(std::bind(&SerialServoHandler::send_serial, this, std::placeholders::_1));
      }
    }  

    if(_serial_port.connect() != DeviceSucces)
    {
      return false;
    }

    if(_cfg_mode)
    {
      for(auto& e : _servos)
      {
        e.second.init_servo();
      }
    }

    //start read thread
    _read_thrd = std::thread(std::bind(&SerialServoHandler::read_thrd, this));
    _read_thrd.detach();
    _send_thrd = std::thread(std::bind(&SerialServoHandler::send_thrd, this));
    _send_thrd.detach();

    _run = true;
    
    return true;
  }

  /**
   * @brief blocks unil rdy
   * 
   * @return std::vector<uint8_t> with ids of servos... 
   */
  std::vector<uint8_t> find_servos()
  {
    int id = 1;
    Time t_extra;
    Duration d(0.01);
    std::vector<uint8_t> ids;
    bool rdy = false;
    while(!rdy)
    {
      //request id
      if(id <= 253)
      {
        std::cout << "checking for id: " << id << std::endl;
        this->send_serial(SerialCommand_Lx16a::create_get_id_cmd(id));
        d.sleep();
      }
      else
      {
        d.sleep();
        if(id > 300)
        {
          rdy = true;
        }
      }

      ++id;

      std::vector<SerialReturn_Lx16a> in;
      _mtx_serial_in.lock();
      in.reserve(_serial_in.size());
      while(_serial_in.size())
      {
        in.push_back(_serial_in.front());
        _serial_in.pop();
      }
      _mtx_serial_in.unlock();

      //check for new servos
      for(auto& e : in)
      {
        std::cout << "-----> Found Servo with ID: " << (int)e.id << std::endl;
        ids.push_back(e.id);
      }

    }

    return ids;
  }

  /**
   * @brief Sets id to given servo. Please ensure only one Servo is connected to bus...
   * 
   * @param id_old 
   * @param id_new 
   * @return true 
   * @return false 
   */
  bool set_id(const uint8_t id_old, const uint8_t id_new)
  {
    Duration d(1);
    //just send new id
    this->send_serial(SerialCommand_Lx16a::create_set_id_cmd(id_old, id_new));
    d.sleep();
    this->send_serial(SerialCommand_Lx16a::create_get_id_cmd(id_new));
    //if after 1s no new msg is arrived then aboard 
    d.sleep();
    std::vector<SerialReturn_Lx16a> in;
    _mtx_serial_in.lock();
    in.reserve(_serial_in.size());
    while(_serial_in.size())
    {
      in.push_back(_serial_in.front());
      _serial_in.pop();
    }
    _mtx_serial_in.unlock();

    for(auto& e : in)
    {
      if(e.id == id_new)
      {
        return true;
      }
    }

    return false;

  }

  void spin_once()
  {
    if(!_cfg_mode)
    {
      //do nothing here...
      return;
    }
    //tick servo request
    this->tick_servo_request();

    //check if new data in serial in queue... if so then call callback with data
    std::vector<SerialReturn_Lx16a> in;
    _mtx_serial_in.lock();
    in.reserve(_serial_in.size());
    while(_serial_in.size())
    {
      in.push_back(_serial_in.front());
      _serial_in.pop();
    }
    _mtx_serial_in.unlock();

    //update servos:
    for(auto& e : in)
    {
      try
      {
        _servos.at(e.id).update_status(e);
      }
      catch(const std::exception& e)
      {
        std::cout << "Got Message from a servo with unknown ID (not defined in XML config)" << std::endl;
      }
    }
  }

  void spin()
  {
    while(_run.load())
    {
      this->spin_once();
      ::usleep(5000); //5ms
    }
  } 

  /**
   * @brief 
   * 
   * @todo check if ok so... or too slow???
   */
  void tick_servo_request() 
  {
    //debug
    // TimerAuto_ms t("time handle servo request: ");
    std::vector<std::vector<uint8_t>> data;
    if(Time::now() - _t_pos_req > _d_pos_request)
    {
      for(auto& e : _servos)
      {
        // std::cout << "reqest pos..." << std::endl;
        e.second.request_pos();

      }
      _t_pos_req = Time::now();
    }
    if(Time::now() - _t_speed_req > _d_speed_request)
    {
      for(auto& e : _servos)
      {
        e.second.request_speed();
      }
      _t_speed_req = Time::now();
    }
    if(Time::now() - _t_error_req > _d_error_request)
    {
      for(auto& e : _servos)
      {
        e.second.request_error();
      }
      _t_error_req = Time::now();
    }
    if(Time::now() - _t_temp_req > _d_temp_request)
    {
      for(auto& e : _servos)
      {
        e.second.request_temp();
      }
      _t_temp_req = Time::now();
    }
    if(Time::now() - _t_vin_req > _d_vin_request)
    {
      for(auto& e : _servos)
      {
        e.second.request_vin();
      }
      _t_vin_req = Time::now();
    }

  }

  void shutdown()
  {
    _run = false;
    
    _serial_port.disconnect();
  }
  
  // void attach_servo_update_callback(const std::function<void(const Params_Lx16a&, const Status_Lx16a&)>& f)
  // {
  //   _update_callback = f;
  // }

  const Servo_Lx16a& get_servo(const uint8_t id) const
  {
    return _servos.at(id);
  }

   std::map<uint8_t, Servo_Lx16a>& get_servos() 
  {
    return _servos;
  }

  void attach_servo_status_update_callback(const std::function<void(const uint8_t, const Status_Lx16a&)>& f)
  {
    for(auto& e : _servos)
    {
      e.second.attach_status_chagned_callback(f);
    }
  }

  void set_rate_pos_request(const double rate)
  {
      _d_pos_request.fromRate(rate);
  }
  void set_rate_speed_request(const double rate)
  {
      _d_speed_request.fromRate(rate);
  }
  void set_rate_error_request(const double rate)
  {
      _d_error_request.fromRate(rate);
  }
  void set_rate_temp_request(const double rate)
  {
      _d_temp_request.fromRate(rate);
  }
  void set_rate_vin_request(const double rate)
  {
      _d_vin_request.fromRate(rate);
  }

private:

  void read_thrd()
  {
    std::cout << "Enter serial read thread..." << std::endl;
    while(_run.load())
    {
      uint8_t data = 0;
      try{
        _serial_port.receive(data);
        // std::cout << "data: " << (int)data << std::endl;
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << std::endl;
        usleep(1000000);
        continue;
      }
      auto msg = _parser.push_msg_byte(data);
      if(msg.first)
      {
        //got valid msg and push in queue
        const std::lock_guard<std::mutex> lock(_mtx_serial_in);
        _serial_in.push(msg.second);
      }
    }
    std::cout << "Exit serial read thread..." << std::endl;
  }

  void send_thrd()
  {
    std::cout << "Enter serial sending thread" << std::endl;
    Duration d(0.002);
    while(_run.load())
    {
      _mtx_serial_out.lock();
      if(_serial_out.size() > 20)
      {
        std::cout << "warning... serial out buffer >20 .... size: " << _serial_out.size() << std::endl;
      }
      if(_serial_out.size())
      {
        _serial_port.transmit(_serial_out.front());
        _serial_out.pop();
      }
      _mtx_serial_out.unlock();
      d.sleep();
    }
    std::cout << "Exit serial sending thread" << std::endl;
  }

  void send_serial(const std::vector<uint8_t>& data)
  {
    // std::cout << "send serial..." << std::endl;
    // _serial_port.transmit(data);
    std::lock_guard<std::mutex> guard(_mtx_serial_out);
    _serial_out.push(data);
  }

private: //dataelements
  SerialCom _serial_port;
  std::thread _read_thrd;
  std::thread _send_thrd;

  SerialParser_lx16a _parser;

  std::map<uint8_t, Servo_Lx16a> _servos;

  std::mutex _mtx_serial_in;
  std::mutex _mtx_serial_out;
  std::queue<SerialReturn_Lx16a> _serial_in;
  std::queue<std::vector<uint8_t>> _serial_out;
  std::string _path_to_xml_cfg;

  // std::function<void(const Params_Lx16a&, const Status_Lx16a&)> _update_callback;

  Duration _d_pos_request   = Duration(1/20.0);//= 20.0;
  Duration _d_speed_request  = Duration(1/20.0);//= 20.0;
  Duration _d_error_request  = Duration(1/0.5);//= 0.5;
  Duration _d_temp_request   = Duration(1/0.5);//= 0.5;
  Duration _d_vin_request    = Duration(1/1.0);//= 1.0;

  Time _t_pos_req;
  Time _t_speed_req;
  Time _t_error_req;
  Time _t_temp_req;
  Time _t_vin_req;  

  bool _cfg_mode = true;
  std::atomic<bool> _run = true;
};

} //namespace francor::servo

#endif  //SERIALSERVOHANDLER_H_