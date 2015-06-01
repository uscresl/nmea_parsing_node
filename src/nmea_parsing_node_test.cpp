/*
 * nmea_parsing_node.cpp
 *
 *  Created on: Jun 17, 2011
 *      Author: hordur
 */

#include <nmea_parsing_node/nmea_parsing_node.h>

#include "ros/ros.h"

#include <string>

using namespace std;
using namespace nmea_parsing_node;

class NmeaParsingNodeTest : public NmeaParsingNode
{
public:
	NmeaParsingNodeTest() : NmeaParsingNode()
	{

	}

	void readCallback(NmeaMessage msg)
	{
		cout << "Received: '" << msg.toStr() << "'" << endl;
	}
	void parseError(string line)
	{
		cout << "Error parsing: '" << line << "'(" << line.length() << "," << line.size() << ")" << endl;
		cout << string_to_hex(line) << endl;
	}

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "nmea_parsing_node");
	NmeaParsingNodeTest npn;
	npn.run();
	return(0);
}
