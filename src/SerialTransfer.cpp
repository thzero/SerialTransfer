#include "SerialTransfer.h"


/*
 void SerialTransfer::begin(Stream &_port, configST configs)
 Description:
 ------------
  * Advanced initializer for the SerialTransfer Class
 Inputs:
 -------
  * const Stream &_port - Serial port to communicate over
  * const configST configs - Struct that holds config
  values for all possible initialization parameters
 Return:
 -------
  * void
*/
void SerialTransfer::begin(Stream& _port, const configST configs)
{
	port = &_port;
	packet.begin(configs);
}


/*
 void SerialTransfer::begin(Stream &_port, const bool _debug, Stream &_debugPort)
 Description:
 ------------
  * Simple initializer for the SerialTransfer Class
 Inputs:
 -------
  * const Stream &_port - Serial port to communicate over
  * const bool _debug - Whether or not to print error messages; 0 = none, 1 = limited, 2 = verbose
  * const Stream &_debugPort - Serial port to print error messages
 Return:
 -------
  * void
*/
void SerialTransfer::begin(Stream& _port, const uint8_t _debug, Stream& _debugPort, uint32_t _timeout)
{
	debug   = _debug;
	debugPort = &_debugPort;
	port    = &_port;
	timeout = _timeout;
	packet.begin(_debug, _debugPort, _timeout);
}


/*
 uint8_t SerialTransfer::sendData(const uint16_t &messageLen, const uint8_t packetID)
 Description:
 ------------
  * Send a specified number of bytes in packetized form
 Inputs:
 -------
  * const uint16_t &messageLen - Number of values in txBuff
  to send as the payload in the next packet
  * const uint8_t packetID - The packet 8-bit identifier
 Return:
 -------
  * uint8_t numBytesIncl - Number of payload bytes included in packet
*/
uint16_t SerialTransfer::sendData(const uint16_t& messageLen, const uint16_t command, const uint8_t packetID)
{
	uint16_t numBytesIncl;

	if (debug == 2) {
		debugPort->printf("sendData.messageLen: %d, command: %d, packetID: %d\n", messageLen, command, packetID);
	}

	numBytesIncl = packet.constructPacket(messageLen, command, packetID);

	if (debug == 2) {
		debugPort->printf("sendData.numBytesIncl: %d\n", numBytesIncl);
		debugPort->print("sendData.premable: ");
		for (size_t i = 0; i < PREAMBLE_SIZE; i++)
			debugPort->printf("%d ", packet.preamble[i]);
		Serial.println();
		debugPort->print("sendData.message: ");
		for (size_t i = 0; i < numBytesIncl; i++)
			debugPort->printf("%d ", packet.txBuff[i]);
		Serial.println();
		debugPort->print("sendData.postamble: ");
		for (size_t i = 0; i < POSTAMBLE_SIZE; i++)
			debugPort->printf("%d ", packet.postamble[i]);
		Serial.println();
	}

	port->write(packet.preamble, sizeof(packet.preamble));
	port->write(packet.txBuff, numBytesIncl);
	port->write(packet.postamble, sizeof(packet.postamble));

	return numBytesIncl;
}


/*
 uint8_t SerialTransfer::available()
 Description:
 ------------
  * Parses incoming serial data, analyzes packet contents,
  and reports errors/successful packet reception
 Inputs:
 -------
  * void
 Return:
 -------
  * uint16_t bytesRead - Num bytes in RX buffer
*/
uint16_t SerialTransfer::available()
{
	bool    valid   = false;
	uint8_t recChar = 0xFF;

	if (port->available())
	{
		valid = true;

		if (debug == 2)
			debugPort->println("parsing...");
		while (port->available())
		{
			recChar = port->read();

			bytesRead = packet.parse(recChar, valid);
			status    = packet.status;

			if (status != CONTINUE)
			{
				if (status <= 0)
					reset();

				break;
			}
		}
		if (debug == 2)
			debugPort->println("...parsing");
	}
	else
	{
		bytesRead = packet.parse(recChar, valid);
		status    = packet.status;

		if (status <= 0)
			reset();
	}

	return bytesRead;
}


/*
 bool SerialTransfer::tick()
 Description:
 ------------
  * Checks to see if any packets have been fully parsed. This
  is basically a wrapper around the method "available()" and
  is used primarily in conjunction with callbacks
 Inputs:
 -------
  * void
 Return:
 -------
  * bool - Whether or not a full packet has been parsed
*/
bool SerialTransfer::tick()
{
	if (available())
		return true;

	return false;
}

/*
 uint8_t SerialTransfer::currentCommand()
 Description:
 ------------
  * Returns the command of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint16_t - command of the last parsed packet
*/
uint16_t SerialTransfer::currentCommand()
{
	return packet.currentCommand();
}

/*
 uint8_t SerialTransfer::currentPacketID()
 Description:
 ------------
  * Returns the ID of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t - ID of the last parsed packet
*/
uint8_t SerialTransfer::currentPacketID()
{
	return packet.currentPacketID();
}

/*
 uint8_t SerialTransfer::currentReceived()
 Description:
 ------------
  * Returns the received bytes of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint16_t - received bytes of the last parsed packet
*/
uint16_t SerialTransfer::currentReceived()
{
	return packet.currentReceived();
}


/*
 void SerialTransfer::reset()
 Description:
 ------------
  * Clears out the tx, and rx buffers, plus resets
  the "bytes read" variable, finite state machine, etc
 Inputs:
 -------
  * void
 Return:
 -------
  * void
*/
void SerialTransfer::reset()
{
	while (port->available())
		port->read();

	packet.reset();
	status = packet.status;
}
