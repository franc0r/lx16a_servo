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

namespace francor
{

enum EnumBoudRate { B_9600 = 0,
                    B_19200,
                    B_38400,
                    B_57600,
                    B_115200
                    //todo may implement more Boudrates
                  };

enum EnumComFormat { F_8N1 = 0,
                     F_8N2
                     //todo may implement more modes
                   };


enum EnumErrorDevice { DeviceSucces = 0,
                       DeviceConnectFail,
                       DeviceReceiveFail,
                       DeviceTransmitFail
                     };

class SerialCom
{
private:

	std::string _deviceName;		///< name of serial device f.e.: "/dev/ttyUSB0"

	EnumBoudRate _boudRate;			///< choosen boudrate

	EnumComFormat _comFormat;		///< choosen format(8N1, 8N2 ....)


	// -- for boost io service --

	boost::asio::io_service* _ioService;
	boost::asio::serial_port* _port;

	// -- async_read + timeout --
	boost::asio::deadline_timer* _timer;
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
	SerialCom(const std::string deviceName, EnumBoudRate boudRate = B_9600,EnumComFormat comFormat = F_8N1);

	/**
	 * @fn virtual ~SerialCom()
	 *
	 * @brief Destructor: disconcect device
	 */
	virtual ~SerialCom();

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
	EnumErrorDevice connect();

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
	EnumErrorDevice receive(unsigned char& receivedData);

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
	EnumErrorDevice receive(unsigned char& receivedData,unsigned int delay_us);

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
	EnumErrorDevice transmit(unsigned char data);

  EnumErrorDevice transmit(const std::vector<uint8_t>& data);

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
	const std::string getDeviceName() const;


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
	void read_callback( bool& data_available,
					    boost::asio::deadline_timer& timeout,
					    const boost::system::error_code& error,
					    std::size_t bytes_transferred );


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
	void wait_callback( boost::asio::serial_port& ser_port,
			            const boost::system::error_code& error );
};

} /* namespace apps */

#endif  //SERIAL_H_