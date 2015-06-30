/*
 * nmea_parsing_node.h
 *
 *  Created on: Jun 17, 2011
 *      Author: hordur
 */

#ifndef NMEA_PARSING_NODE_H_
#define NMEA_PARSING_NODE_H_

#include "ros/ros.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <queue>

#include <boost/foreach.hpp>
#define foreach         BOOST_FOREACH

#include "nmea_parsing_node/serial_tcp_interface.h"

using namespace std;

namespace nmea_parsing_node {


std::string string_to_hex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}


bool verifyFormat(string seq)
{
  if(seq.length() > 4 && seq.find('$')==0 && seq.find('*') > 0) {
    return true;
  }
  return false;
}

string calcChkSum(string seq)
{
  char chksum = 0;

  foreach(char i,seq) {
    if(i=='$') {
      continue;
    } else if(i=='*') {
      break;
    }
    chksum ^= i;
  }

  char cChksum[3];
  sprintf(cChksum,"%X",chksum);

  return string(cChksum);
}

bool verifyChkSum(string seq)
{
  char chksum = 0;

  foreach(char i,seq) {
    if(i=='$' || i=='\0') {
      continue;
    } else if(i=='*') {
      break;
    }
    chksum ^= i;
  }

  string oChksum = seq.substr(seq.length()-2,2);

  char cChksum[3];
  sprintf(cChksum,"%X",chksum);

  return oChksum.compare(cChksum)==0;

}

bool parseSeq(string seq, string& id, vector<string>& values)
{

  if(!verifyFormat(seq)) {
    return false;
  }

  if(!verifyChkSum(seq)) {
    return false;
  }

  string str = seq.substr(1,seq.length()-3);
  str = str.substr(0,str.find_last_not_of('*')+1);
  boost::split(values, str, boost::is_any_of(string(",")));
  id = values[0];
  values.erase(values.begin());

  return true;
}

struct NmeaMessage
{
  string command;
  vector<string> data;
  ros::Time time;
  string toStr() {
    string res = "$"+command+","+boost::join(data,",")+"*";
    return res+calcChkSum(res);
  }
};

bool parseSeq(string seq,NmeaMessage& msg) {
  return parseSeq(seq,msg.command,msg.data);
}

std::string trim(const std::string &str)
{
    size_t s = str.find_first_not_of(" \n\r\t",str.find_first_not_of('\0'));
    size_t e = str.find_last_not_of (" \n\r\t",str.find_last_not_of('\0'));

    if(( string::npos == s) || ( string::npos == e))
        return "";
    else
        return str.substr(s, e-s+1);
}



class NmeaParsingNode
{
protected:
  ros::NodeHandle node;
  ros::NodeHandle privnode;

  string serial_port;
  int baud_rate;
  string tcp_hostname;
  int tcp_port;

  boost::mutex q_mutex;
  boost::condition qAvail;

  queue<NmeaMessage> q;

  string mode;
  bool tcpMode;

  //NPNIface* iface;
  boost::scoped_ptr<NPNIface> iface;

  bool kill;

public:
  NmeaParsingNode() : privnode("~"), serial_port(""), baud_rate(0), tcp_hostname(""), tcp_port(0), mode("")
  {
    privnode.getParam("mode",mode);
    privnode.getParam("serial_port",serial_port);
    privnode.getParam("hostname",tcp_hostname);
    privnode.getParam("baud_rate",baud_rate);
    privnode.getParam("port",tcp_port);

    ROS_INFO("Mode: %s | Serial port: %s | Hostname: %s | Baud rate: %d | TCP port: %d", mode.c_str(), serial_port.c_str(), tcp_hostname.c_str(), baud_rate, tcp_port);

    kill = false;

    if(mode == "tcp" && tcp_hostname.size() > 0 && tcp_port > 0) {
      tcpMode = true;
    } else if(mode == "serial" && serial_port.size() > 0 && baud_rate > 0) {
      tcpMode = false;
    } else {
      // bad stuff
      // TODO exception
      ROS_ERROR("Error, No mode selected");
      kill = true;
      return;
    }

    bool res = false;
    if (tcpMode) {
      ROS_INFO("Running in TCP mode");
      res = openTcp();
    } else {
      ROS_INFO("Running in SERIAL mode");
      res = openSerial();
    }

  }

  bool run()
  {
    cout << "Running" << endl;
    if (kill) {
      return false;
    }
    boost::thread readThread(&NmeaParsingNode::readThreadFunc,this);
    boost::thread processThread(&NmeaParsingNode::processQueue,this);
    spin();
    kill = true;
    qAvail.notify_one();
    processThread.join();
    readThread.interrupt();
    readThread.join();
    close();
    //readThread.interrupt();
    //processThread.interrupt();
    return true;
  }

  bool writeData(string data) {
    iface->writeString(data);
    return true;
  }

  bool writeData(NmeaMessage msg) {
    return writeData(msg.toStr());
  }

  void close() {
    if(iface != NULL) {
      iface->close();
    }
  }

  virtual void spin ()
  {
    ros::spin();
  }

  virtual void readCallback(NmeaMessage msg) = 0;
  virtual void parseError(string line) = 0;

private:

  bool openSerial() {
    iface.reset(new NPNSerial(serial_port,baud_rate));
    return true;
  }

  bool openTcp() {
    iface.reset(new NPNTcp(tcp_hostname,tcp_port));
    return true;
  }

  bool processLine(string line) {
    NmeaMessage msg;
    msg.time = ros::Time::now();
    bool res = parseSeq(line,msg);
    //cout << "processLine(" << line << ")" << endl;
    if(res) {
      boost::mutex::scoped_lock lk(q_mutex);
      q.push(msg);
      qAvail.notify_one();
      return true;
    } else {
      parseError(line);
      return false;
    }
  }

  void processQueue()
  {
    boost::mutex::scoped_lock lk(q_mutex);
    while(ros::ok() && !kill) {
      while(q.empty() && !kill) {
        qAvail.wait(lk);
      }
      if(kill) {
        return;
      }
      NmeaMessage msg;
      msg = q.front();
      q.pop();
      readCallback(msg);
    }
  }

  void readThreadFunc() {
    while(ros::ok() && !kill)
    {
      string line = iface->readLine();
      if(line.length() > 0) {
        processLine(line);
      }
    }
  }


};

class NmeaTestingNode : public NmeaParsingNode
{
  NmeaTestingNode() : NmeaParsingNode()
  {
    // Load additional parameters
    // Open publishings / subscriptions
  }
};

}
#endif /* NMEA_PARSING_NODE_H_ */
