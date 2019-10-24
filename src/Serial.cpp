#include "Serial.h"

using namespace boost::asio;

namespace francor
{

SerialCom::SerialCom( const std::string deviceName,
					  EnumBoudRate boudRate,
					  EnumComFormat comFormat )
{
	_deviceName = deviceName;
	_boudRate = boudRate;
	_comFormat = comFormat;

	// -- init boost --

	_ioService = new boost::asio::io_service;
	_port = new boost::asio::serial_port(*_ioService);
	_timer = new boost::asio::deadline_timer(*_ioService);
}

SerialCom::~SerialCom()
{
	delete _ioService;
	delete _port;
}

EnumErrorDevice SerialCom::connect()
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

	} catch (boost::system::system_error& error)
	{
		//debug
		std::cout << "Debug: Boost exeption: " << error.what() << std::endl;

		return DeviceConnectFail;
	}

	return DeviceSucces;
}

EnumErrorDevice SerialCom::receive(unsigned char& receiveData)
{
	_port->read_some(buffer(&receiveData,1));

	return DeviceSucces;
}

EnumErrorDevice SerialCom::transmit(unsigned char data)
{

	_port->write_some(buffer(&data,1));
	return DeviceSucces;
}

EnumErrorDevice SerialCom::transmit(const std::vector<uint8_t>& data)
{
  _port->write_some(buffer(&data.front(),data.size()));
  return DeviceSucces;
}

EnumErrorDevice SerialCom::receive( unsigned char& receivedData,
								    unsigned int delay_us )
{
	//deadline_timer timer(*_ioService);
	unsigned char myBuffer[1];
	bool data_available = false;


	_port->async_read_some( boost::asio::buffer(myBuffer),
							boost::bind(&SerialCom::read_callback,
										this,
										boost::ref(data_available),
										boost::ref(*_timer),
										boost::asio::placeholders::error,
										boost::asio::placeholders::bytes_transferred)
							);

	boost::posix_time::microsec delay_boost(delay_us);

	_timer->expires_from_now(delay_boost);

	_timer->async_wait(boost::bind(&SerialCom::wait_callback,
								 this,
								 boost::ref(*_port),
								 boost::asio::placeholders::error));

	_ioService->run();  // will block until async callbacks are finished

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

const std::string SerialCom::getDeviceName() const
{
	return _deviceName;
}

void SerialCom::read_callback( bool& data_available,
							   boost::asio::deadline_timer& timeout,
							   const boost::system::error_code& error,
							   std::size_t bytes_transferred )
{
	//std::cout << "debug: called -> SerialCom::read_callback(...)" << std::endl;
    if (error || !bytes_transferred)
	{
	    // No data was read!
	    data_available = false;
	    return;
	}

	timeout.cancel();  // will cause wait_callback to fire with an error
	data_available = true;
}

void SerialCom::wait_callback( boost::asio::serial_port& ser_port,
							   const boost::system::error_code& error )
{
	//std::cout << "debug: called -> SerialCom::wait_callback(...)" << std::endl;
	if(error)
	{
		// Data was read and this timeout was canceled
		return;
	}

	ser_port.cancel();  // will cause read_callback to fire with an error
}

}/* namespace apps */