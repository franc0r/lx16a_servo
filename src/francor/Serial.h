/**
 * @file   Serial.h
 * @author Michael Schmidpeter
 * @date   2019-10-22
 * @brief  todo
 * 
 * @todo  This is very old code -> must be refreshed!!!!
 * 
 * PROJECT: Franc0r
 * @see https://github.com/franc0r/Wiki/wiki
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <iostream>
#include <string>
#include <sstream>

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>

using namespace boost::asio;

namespace francor
{

  enum EnumBoudRate
  {
    B_9600 = 0,
    B_19200,
    B_38400,
    B_57600,
    B_115200
    //todo may implement more Boudrates
  };

  enum EnumComFormat
  {
    F_8N1 = 0,
    F_8N2
    //todo may implement more modes
  };

  enum EnumErrorDevice
  {
    DeviceSucces = 0,
    DeviceConnectFail,
    DeviceReceiveFail,
    DeviceTransmitFail
  };

  class SerialCom
  {
  private:
    std::string _deviceName; ///< name of serial device f.e.: "/dev/ttyUSB0"

    EnumBoudRate _boudRate; ///< choosen boudrate

    EnumComFormat _comFormat; ///< choosen format(8N1, 8N2 ....)

    // -- for boost io service --

    boost::asio::io_service *_ioService;
    boost::asio::serial_port *_port;

    // -- async_read + timeout --
    boost::asio::deadline_timer *_timer;

  public:
    /**
	 * @fn SerialCom(const std::string deviceName, EnumBoudRate boudRate = B_9600,EnumComFormat comFormat = F_8N1)
	 *
	 * @brief Constructor: take and save options
	 *
	 *
	 * @param[in]	EnumBoudRate : Use the in this header decleard enum EnumBoudrate to define
	 * 						 	   the Boudrate :  B_9600, ..., B_11520
	 * @param[in]	EnumComMode  : Use the in this header decleard enum EnumMode to define
	 * 		  				  	   communication mode :  M_Wait, M_NoDelay
	 * @param[in]	EnumFormat   : Use the in this header decleard enum EnumFormat to defne
	 * 		  				 	   communication format : F_8N1, F_8N2 ... more must be implemented
	 *
	 * @param[in] deviceName : Name of device (f.e. "/dev/ttyUSB0") as std::string
	 */
    SerialCom(const std::string deviceName, EnumBoudRate boudRate = B_9600, EnumComFormat comFormat = F_8N1)
    {
      _deviceName = deviceName;
      _boudRate = boudRate;
      _comFormat = comFormat;

      // -- init boost --

      _ioService = new boost::asio::io_service;
      _port = new boost::asio::serial_port(*_ioService);
      _timer = new boost::asio::deadline_timer(*_ioService);
    }

    /**
	 * @fn virtual ~SerialCom()
	 *
	 * @brief Destructor: disconcect device
	 */
    virtual ~SerialCom()
    {
      delete _ioService;
      delete _port;
    }

    /**
	 * @fn EnumErrorDevice connect()
	 *
	 * @brief connect and set settings to device
	 *
	 *
	 * @param[in,out]	void
	 *
	 *
	 * @return EnumErrorDevice : retruns DeviceSucces if connection was succesful or
	 * 							 DeviceConnectFail if connection failed :
	 * 							 DeviceSucces, DeviceConnectFail
	 */
    EnumErrorDevice connect()
    {

      try
      {
        //std::cout << "debug: try to open port\n";
        _port->open(_deviceName);
        //std::cout << "debug: port opend" << std::endl;

        // till yet no  other option is provided
        serial_port_base::flow_control flowCrtl(serial_port_base::flow_control::none);
        //std::cout << "debug: try to set FlowControll" << std::endl;
        _port->set_option(flowCrtl);
        //std::cout << "debug: FlowControll set" << std::endl;

        unsigned int baudrate = 0;

        //select BoudRate settings 9600...11520 .... may more

        switch (_boudRate)
        {
        case B_9600:
          baudrate = 9600;
          break;
        case B_19200:
          baudrate = 19200;
          break;
        case B_38400:
          baudrate = 38400;
          break;
        case B_57600:
          baudrate = 57600;
          break;
        case B_115200:
          baudrate = 115200;
          break;
        default:
          std::cout << "SystemError!!!... exit\n";
          exit(1);
          break;
        }

        //set Baudrate-settings to port....
        //std::cout << "debug: try to set baudrate" << std::endl;
        _port->set_option(serial_port_base::baud_rate(baudrate));
        //std::cout << "debug: baudrate set" << std::endl;

        unsigned int charsize = 0;

        //std::cout << "debug: try to set 8Nx" << std::endl;
        // select/set commFormat 8N1 or 8N2 ... may more
        serial_port_base::stop_bits stopBits_1(serial_port_base::stop_bits::one);
        serial_port_base::stop_bits stopBits_2(serial_port_base::stop_bits::two);
        serial_port_base::parity parityBits_0(serial_port_base::parity::none);
        switch (_comFormat)
        {
        case F_8N1:
          charsize = 8;
          //std::cout << "debug: -> try to set 8N1" << std::endl;
          _port->set_option(stopBits_1);
          _port->set_option(parityBits_0);
          //std::cout << "debug: -> 8N1 set" << std::endl;
          break;
        case F_8N2:
          charsize = 8;
          //std::cout << "debug: -> try to set 8N1" << std::endl;
          _port->set_option(stopBits_2);
          _port->set_option(parityBits_0);
          //std::cout << "debug: -> 8N1 set" << std::endl;
          break;
        default:
          std::cout << "SystemError!!!... exit\n";
          exit(1);
          break;
        }

        // set Charsize option to port ....
        _port->set_option(serial_port_base::character_size(charsize));

        //std::cout << "debug: 8Nx set" << std::endl;
      }
      catch (boost::system::system_error &error)
      {
        //debug
        std::cout << "Debug: Boost exeption: " << error.what() << std::endl;

        return DeviceConnectFail;
      }

      return DeviceSucces;
    }

    EnumErrorDevice disconnect()
    {
      _port->close();
      return DeviceSucces;
    }

    /**
	 * @fn receive
	 *
	 * @brief receives data from device
	 *
	 *
	 * @param[out] receiveData : reference to variable in whitch data shold
	 * 									   be written (8Bit)
	 *
	 * @return EnumErrorDevice : returns DeviceSucces if receive was succesful or
	 * 							 DeviceReceiveFail receive data failed
	 */
    EnumErrorDevice receive(unsigned char &receivedData)
    {
      _port->read_some(buffer(&receivedData, 1));

      return DeviceSucces;
    }

    /**
	 * @fn receive
	 *
	 * @brief receives data from device with timout at given delay
	 *
	 *
	 * @param[out] receiveData : reference to variable in whitch data shold
	 * 									   be written (8Bit)
	 * @param[in]  delay_us	   : time to timeout in [us]
	 *
	 * @return EnumErrorDevice : returns DeviceSucces if receive was succesful or
	 * 							 DeviceReceiveFail receive data failed
	 */
    EnumErrorDevice receive(unsigned char &receivedData, unsigned int delay_us)
    {
      //deadline_timer timer(*_ioService);
      unsigned char myBuffer[1];
      bool data_available = false;

      _port->async_read_some(boost::asio::buffer(myBuffer),
                             boost::bind(&SerialCom::read_callback,
                                         this,
                                         boost::ref(data_available),
                                         boost::ref(*_timer),
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));

      boost::posix_time::microsec delay_boost(delay_us);

      _timer->expires_from_now(delay_boost);

      _timer->async_wait(boost::bind(&SerialCom::wait_callback,
                                     this,
                                     boost::ref(*_port),
                                     boost::asio::placeholders::error));

      _ioService->run(); // will block until async callbacks are finished

      _ioService->reset();

      if (!data_available)
      {
        //no data;
        //std::cout << "debug: receive timed out or failed" << std::endl;
        return DeviceReceiveFail;
      }

      receivedData = myBuffer[0];

      return DeviceSucces;
    }

    /**
	 * @fn EnumErrorDevice transmit(unsigned char data)
	 *
	 * @brief transmits data to device
	 *
	 *
	 * @param[in]	data : data to transmit as unsigned char
	 *
	 *
	 * @return EnumErrorDevice : returns DeviceSucces if transmit was succesful or
	 * 							 DeviceTransmitFail transmit data failed
	 */
    EnumErrorDevice transmit(unsigned char data)
    {

      _port->write_some(buffer(&data, 1));
      return DeviceSucces;
    }

    EnumErrorDevice transmit(const std::vector<uint8_t> &data)
    {
      _port->write_some(buffer(&data.front(), data.size()));
      return DeviceSucces;
    }

    /**
	 * @fn const std::string getDeviceName() const
	 *
	 * @brief gets name of device
	 *
	 *
	 * @param[in,out] void
	 *
	 *
	 * @return returns name of device (f.e. "/dev/ttyUSB0") in format const std::string
	 */
    const std::string getDeviceName() const
    {
      return _deviceName;
    }

  private:
    /**
	 * @fn read_callback( bool& data_available,
	 * 				      boost::asio::deadline_timer& timeout,
	 *				      const boost::system::error_code& error,
	 *				      std::size_t bytes_transferred );
	 *
	 * @brief callback function for reading asynchron serial data with timeout
	 *
	 *
	 * @param[in,out]
	 *
	 *
	 * @return void
	 */
    void read_callback(bool &data_available,
                       boost::asio::deadline_timer &timeout,
                       const boost::system::error_code &error,
                       std::size_t bytes_transferred)
    {
      //std::cout << "debug: called -> SerialCom::read_callback(...)" << std::endl;
      if (error || !bytes_transferred)
      {
        // No data was read!
        data_available = false;
        return;
      }

      timeout.cancel(); // will cause wait_callback to fire with an error
      data_available = true;
    }

    /**
	 * @fn void wait_callback( boost::asio::serial_port& ser_port,
	 *		            	   const boost::system::error_code& error );
	 *
	 * @brief callback function for reading asynchron serial data with timeout
	 *
	 *
	 * @param[in,out]
	 *
	 *
	 * @return void
	 */
    void wait_callback(boost::asio::serial_port &ser_port,
                       const boost::system::error_code &error)
    {
      //std::cout << "debug: called -> SerialCom::wait_callback(...)" << std::endl;
      if (error)
      {
        // Data was read and this timeout was canceled
        return;
      }

      ser_port.cancel(); // will cause read_callback to fire with an error
    }
  };

} // namespace francor

#endif //SERIAL_H_