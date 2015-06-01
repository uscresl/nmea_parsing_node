/*
 * serial_tcp_interface.h
 *
 *  Created on: Jun 24, 2011
 *      Author: hordur
 */

#ifndef SERIAL_TCP_INTERFACE_H_
#define SERIAL_TCP_INTERFACE_H_

#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"

using namespace std;

class NPNIface
{
protected:

public:
	virtual void writeString(string s) = 0;
	virtual char readChar() = 0;
	virtual string readLine() {
		char c;
		string res;
		for(;;) {
			c = readChar();
			switch(c)
			{
			case '\r':
				break;
			case '\n':
				return res;
			default:
				res+=c;
			}
		}
	}
	virtual void close() = 0;
};

class NPNSerial : public NPNIface
{
private:
	boost::asio::io_service io;
	boost::asio::serial_port serial;
	boost::asio::streambuf buf;
public:
	NPNSerial(string port, int baud_rate) : io(), serial(io,port)
	{

		serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	}
	void writeString(string s)
	{
		boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
	}

	char readChar()
	{
		char c;
		boost::asio::read(serial,boost::asio::buffer(&c,1));
		return c;
	}
/*
	string readLine()
	{
		//boost::asio::streambuf buf;
		//size_t n = boost::asio::read_until(serial,buf,"\n");
		//buf.commit(n);
		//istream is(&buf);
		//string s;
		//is >> s;
		//cout << string_to_hex(s) << endl;
		//buf.consume(n);
		//return trim(s);

		//return s;
		char c;
		string res;
		for(;;) {
			c = readChar();
			switch(c)
			{
			case '\r':
				break;
			case '\n':
				return res;
			default:
				res+=c;
			}
		}
	}
*/
	void close()
	{
		serial.close();
	}

};



class NPNTcp : public NPNIface
{
private:
	boost::asio::io_service io;
	//boost::asio::ip::tcp::socket socket;
	boost::scoped_ptr<boost::asio::ip::tcp::socket> socket;
	boost::asio::ip::tcp::endpoint endpoint;
public:
	NPNTcp(string host, int port) : io() //, socket(io)
	{
		boost::asio::ip::tcp::resolver resolver(io);
		boost::asio::ip::tcp::resolver::query query(host,boost::lexical_cast<string>(port));
		endpoint = *resolver.resolve(query);
		connect();
	}
	bool connect()
	{
		boost::system::error_code err;
		socket.reset(new boost::asio::ip::tcp::socket(io));
		socket->connect(endpoint,err);
		if(err) {
			ROS_ERROR("Connection failed: %s",err.message().c_str());
	        	socket->close();
			return false;
		} else {
			return true;
		}
	}
	void writeString(string s)
	{
		boost::asio::write(*socket,boost::asio::buffer(s.c_str(),s.size()));
	}
	char readChar()
	{
		char c;
		boost::asio::read(*socket,boost::asio::buffer(&c,1));
		return c;
	}
	/*
	string readLine()
	{
		//boost::asio::read
		//socket.read_some()

//		boost::asio::streambuf buf;
//		size_t n = boost::asio::read_until(socket,buf,"\r");
//		buf.commit(n);
//		istream is(&buf);
//		string s;
//		is >> s;
//		cout << string_to_hex(s) << endl;
//		buf.consume(n);
//		return trim(s);


		char c;
		string res;
		for(;;) {
			c = readChar();
			switch(c)
			{
			case '\r':
				break;
			case '\n':
				return res;
			default:
				res+=c;
			}
		}

	}
	*/
	void close()
	{
		socket->close();
	}

};

#endif /* SERIAL_TCP_INTERFACE_H_ */
