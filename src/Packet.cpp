#include "Packet.h"


PacketCRC crc;


/*
 void Packet::begin(const configST& configs)
 Description:
 ------------
  * Advanced initializer for the Packet Class
 Inputs:
 -------
  * const configST& configs - Struct that holds config
  values for all possible initialization parameters
 Return:
 -------
  * void
*/
void Packet::begin(const configST& configs)
{
	debugPort    = configs.debugPort;
	debug        = configs.debug;
	packed        = configs.packed;
	callbacks    = configs.callbacks;
	callbacksLen = configs.callbacksLen;
	timeout 	 = configs.timeout;
}


/*
 void Packet::begin(const bool& _debug, Stream& _debugPort, const uint32_t& _timeout)
 Description:
 ------------
  * Simple initializer for the Packet Class
 Inputs:
 -------
  * const bool& _debug - Whether or not to print error messages
  * Stream &_debugPort - Serial port to print error messages
  * const uint32_t& _timeout - Number of ms to wait before
  declaring packet parsing timeout
 Return:
 -------
  * void
*/
void Packet::begin(const uint8_t& _debug, Stream& _debugPort, const uint32_t& _timeout)
{
	debugPort = &_debugPort;
	debug     = _debug;
	timeout   = _timeout;
}


/*
 uint8_t Packet::constructPacket(const uint16_t& messageLen, const uint8_t& command, const uint8_t& packetID)
 Description:
 ------------
  * Calculate, format, and insert the packet protocol metadata into the packet transmit
  buffer
 Inputs:
 -------
  * const uint16_t& messageLen - Number of values in txBuff
  to send as the payload in the next packet
  * const uint8_t& command - The packet 16-bit command
  * const uint8_t& packetID - The packet 8-bit identifier
 Return:
 -------
  * uint16_t - Number of payload bytes included in packet
*/
uint16_t Packet::constructPacket(const uint16_t& messageLen, const uint16_t& command, const uint8_t& packetID)
{
	uint16_t size = messageLen;
	if (messageLen > MAX_PACKET_SIZE)
		size = MAX_PACKET_SIZE;

	if (debug == 2)
		debugPort->printf("preamble.packed: %d\n", packed);
	if (packed) {
		calcOverhead(txBuff, (uint8_t)messageLen);
		stuffPacket(txBuff, (uint8_t)messageLen);
	}
	uint16_t crcVal = crc.calculate(txBuff, size);

	if (debug == 2)
	{
		debugPort->printf("preamble.packetID: %d\n", packetID);
		debugPort->printf("preamble.command: %d\n", command);
		debugPort->printf("preamble.overheadByte: %d\n", overheadByte);
		debugPort->printf("preamble.messageLen: %d %d %d\n", messageLen, MAX_PACKET_SIZE, size);
	}

	preamble[0] = START_BYTE;
	preamble[1] = packetID;
	preamble[2] = (command >> 8) & 0xFF; // Extract high byte
	preamble[3] = command & 0xFF;        // Extract low byte
	preamble[4] = overheadByte;
	preamble[5] = (size >> 8) & 0xFF; 	 // Extract high byte
	preamble[6] = size & 0xFF;        	 // Extract low byte

	if (debug == 2) 
	{
		debugPort->printf("preamble.command.high: %d\n", preamble[2]);
		debugPort->printf("preamble.command.low: %d\n", preamble[3]);
		debugPort->printf("preamble.messageLen.high: %d\n", preamble[5]);
		debugPort->printf("preamble.messageLen.low: %d\n", preamble[6]);
	}

	postamble[0] = (crcVal >> 8) & 0xFF; // Extract high byte
	postamble[1] = crcVal & 0xFF;        // Extract low byte
	postamble[POSTAMBLE_SIZE - 1] = STOP_BYTE;

	if (debug == 2) 
	{
		debugPort->printf("postamble.crcVal.high: %d\n", postamble[0]);
		debugPort->printf("postamble.crcVal.low: %d\n", postamble[1]);
		debugPort->printf("postamble.stop: %d\n", postamble[POSTAMBLE_SIZE - 1]);
	}

	return messageLen;
}


/*
 uint8_t Packet::parse(const uint8_t& recChar, const bool& valid)
 Description:
 ------------
  * Parses incoming serial data, analyzes packet contents,
  and reports errors/successful packet reception. Executes
  callback functions for parsed packets whos ID has a
  corresponding callback function set via
  "void Packet::begin(const configST configs)"
 Inputs:
 -------
  * const uint8_t& recChar - Next char to parse in the stream
  * const bool& valid - Set if stream is "available()" and clear if not
 Return:
 -------
  * uint16_t - Num bytes in RX buffer
*/

uint16_t Packet::parse(const uint8_t& recChar, const bool& valid)
{
	if (packetType == 0)
		return parseTypeMaxMessage(recChar, valid);
		
	if (packetType == 1)
		return parseTypeMinDelta(recChar, valid);

	return 0;
}

parseResults Packet::parseFreshCheck(const uint8_t& recChar, const bool& valid, bool packet_fresh, uint32_t current)
{
	parseResults results;

	if (!packet_fresh) //packet is stale, start over.
	{
		if (debug) {
			debugPort->println("ERROR: STALE PACKET");
			debugPort->printf("parse.packetStart: %u\n", packetStart);
			debugPort->printf("parse.current: %u\n", current);
			debugPort->printf("parse.timeout: %u\n", timeout);
			debugPort->printf("parse.(current - packetStart): %u\n", (current - packetStart));
			debugPort->printf("parse.((current - packetStart) < timeout): %u\n", ((current - packetStart) < timeout));
		}

		bytesRead   = 0;
		state       = find_start_byte;
		status      = STALE_PACKET_ERROR;
		packetStart = 0;
		packetLast  = 0;

		results.bytesRead = 0;
		results.success = false;
		return results;
	}

	results.bytesRead = 0;
	results.success = true;
	return results;
}

parseResults Packet::parseFindCommand(const uint8_t& recChar)
{
	parseResults results;
	
	// get the high value of the 16 byte length
	if (debug == 3) 
	{
		debugPort->println("parse.state: find_command");
		debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
		debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
		debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
	}
	if ((recChar >= 0) && (recChar <= UINT16_MAX))
	{
		recCharPrevious = recChar;
		state      = find_command2;
	}
	else
	{
		bytesRead = 0;
		state     = find_start_byte;
		status    = PAYLOAD_ERROR;

		if (debug)
			debugPort->println("ERROR: PAYLOAD_ERROR - COMMAND INVALID - LOW BYTE");

		reset();

		results.success = false;
		return results;
	}

	results.success = true;
	return results;
}

parseResults Packet::parseFindCommand2(const uint8_t& recChar)
{
	parseResults results;

	// get the low value of the 16 byte length
	if (debug == 3) 
	{
		debugPort->println("parse.state: find_command2");
		debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
		debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
		debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
	}
	if ((recChar >= 0) && (recChar <= UINT16_MAX))
	{
		command = ((uint16_t)recCharPrevious << 8) | recChar;  // high | low
		payIndex   = 0;
		state      = find_overhead_byte;

		if (debug == 3) 
		{
			debugPort->printf("parse.command: %d\n", command);
			debugPort->printf("parse.MAX_PACKET_SIZE: %d\n", MAX_PACKET_SIZE);
			debugPort->printf("parse.(command >= 0): %d\n", (command > 0));
			debugPort->printf("parse.(command <= MAX_PACKET_SIZE): %d\n", (command <= MAX_PACKET_SIZE));
			debugPort->printf("parse.(command >= 0) && (command <= MAX_PACKET_SIZE): %d\n", (command > 0) && (command <= MAX_PACKET_SIZE));
		}
		if (!((command >= 0) && (command <= MAX_PACKET_SIZE)))
		{
			command = 0;
			state     = find_start_byte;
			status    = PAYLOAD_ERROR;

			if (debug)
				debugPort->println("ERROR: PAYLOAD_ERROR - COMMAND INVALID");

			reset();

			results.bytesRead = bytesRead;
			results.success = false;
			return results;
		}
	}
	else
	{
		bytesRead = 0;
		state     = find_start_byte;
		status    = PAYLOAD_ERROR;

		if (debug)
			debugPort->println("ERROR: PAYLOAD_ERROR - COMMAND INVALID - HIGH BYTE");

		reset();

		results.bytesRead = bytesRead;
		results.success = false;
		return results;
	}

	results.bytesRead = bytesRead;
	results.success = true;
	return results;
}

parseResults Packet::parseFindIdByte(const uint8_t& recChar)
{
	parseResults results;

	if (debug == 3)
		debugPort->println("parse.state: find_id_byte");
	idByte = recChar;
	state  = find_command;
	
	results.success = true;
	return results;
}

parseResults Packet::parseFindOverheadByte(const uint8_t& recChar)
{
	parseResults results;

	if (debug == 3)
		debugPort->println("parse.state: find_overhead_byte");
	recOverheadByte = recChar;
	state           = find_payload_len;
	
	results.success = true;
	return results;
}

parseResults Packet::parseFindStartByte(const uint8_t& recChar)
{
	parseResults results;

	if (debug == 3)
		debugPort->println("parse.state: find_start_byte");
	if (recChar == START_BYTE)
	{
		state       = find_id_byte;
		packetStart = millis();	//start the timer
	}

	results.success = true;
	return results;
}

parseResults Packet::parseFindPayloadLength(const uint8_t& recChar)
{
	parseResults results;

	// get the high value of the 16 byte length
	if (debug == 2) 
	{
		debugPort->println("parse.state: find_payload_len");
		debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
		debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
		debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
	}
	if ((recChar >= 0) && (recChar <= UINT16_MAX))
	{
		recCharPrevious = recChar;
		state      = find_payload_len2;
	}
	else
	{
		bytesRead = 0;
		state     = find_start_byte;
		status    = PAYLOAD_ERROR;

		if (debug)
			debugPort->println("ERROR: PAYLOAD_ERROR - PAYLOAD LENGTH INVALID - LOW BYTE");

		reset();

		results.success = false;
		return results;
	}

	results.success = true;
	return results;
}

parseResults Packet::parseFindPayloadLength2(const uint8_t& recChar)
{
	parseResults results;

	// get the low value of the 16 byte length
	if (debug == 3) 
	{
		debugPort->println("parse.state: find_payload_len2");
		debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
		debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
		debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
	}
	if ((recChar >= 0) && (recChar <= UINT16_MAX))
	{
		bytesToRec = ((uint16_t)recCharPrevious << 8) | recChar;  // high | low
		payIndex   = 0;
		state      = find_payload;

		if (debug == 3) 
		{
			debugPort->printf("parse.bytesToRec: %d\n", bytesToRec);
			debugPort->printf("parse.MAX_PACKET_SIZE: %d\n", MAX_PACKET_SIZE);
			debugPort->printf("parse.(bytesToRec > 0): %d\n", (bytesToRec > 0));
			debugPort->printf("parse.(bytesToRec <= MAX_PACKET_SIZE): %d\n", (bytesToRec <= MAX_PACKET_SIZE));
			debugPort->printf("parse.(bytesToRec > 0) && (bytesToRec <= MAX_PACKET_SIZE): %d\n", (bytesToRec > 0) && (bytesToRec <= MAX_PACKET_SIZE));
		}
		if (!((bytesToRec > 0) && (bytesToRec <= MAX_PACKET_SIZE)))
		{
			bytesRead = 0;
			state     = find_start_byte;
			status    = PAYLOAD_ERROR;

			if (debug)
				debugPort->println("ERROR: PAYLOAD_ERROR - PAYLOAD LENGTH INVALID");

			reset();

			results.success = false;
			return results;
		}
	}
	else
	{
		bytesRead = 0;
		state     = find_start_byte;
		status    = PAYLOAD_ERROR;

		if (debug)
			debugPort->println("ERROR: PAYLOAD_ERROR - PAYLOAD LENGTH INVALID - HIGH BYTE");

		reset();

		results.success = false;
		return results;
	}

	results.success = true;
	return results;
}

parseResults Packet::parseFindPayload(const uint8_t& recChar)
{
	parseResults results;

	if (debug == 3) 
	{
		debugPort->println("parse.state: find_payload");
		debugPort->printf("parse.payIndex: %d\n", payIndex);
		debugPort->printf("parse.bytesToRec: %d\n", bytesToRec);
		debugPort->printf("parse.recChar: %d\n", recChar);
	}
	
	if (payIndex < bytesToRec)
	{
		rxBuff[payIndex] = recChar;
		payIndex++;

		if (payIndex == bytesToRec)
			state    = find_crc;
	}

	results.success = true;
	return results;
}

parseResults Packet::parseFindCrc(const uint8_t& recChar)
{
	parseResults results;

	// get the high value of the 16 byte crc
	if (debug == 3)
		debugPort->println("parse.state: find_crc");
	recCharPrevious = recChar;
	if (debug == 3)
		debugPort->printf("parse.recChar.high: %d\n", recChar);
	state = find_crc2;

	results.success = true;
	return results;
}

parseResults Packet::parseFindCrc2(const uint8_t& recChar)
{
	parseResults results;

	// get the low value of the 16 byte crc
	if (debug == 3)
		debugPort->println("parse.state: find_crc2");
	uint16_t calcCrc = crc.calculate(rxBuff, bytesToRec);
	if (debug == 3) 
	{
		debugPort->printf("parse.calcCrc: %d\n", calcCrc);
		debugPort->printf("parse.recChar.low: %d\n", recChar);
	}
	recvCrc = ((uint16_t)recCharPrevious << 8) | recChar;  // high | low
	if (debug == 3) {
		debugPort->printf("parse.recvCrc: %d\n", recvCrc);
		debugPort->printf("parse.calcCrc==recvCrc: %d\n", (calcCrc == recvCrc));
	}

	if (calcCrc == recvCrc) {
		state = find_end_byte;
	}
	else
	{
		bytesRead = 0;
		state     = find_start_byte;
		status    = CRC_ERROR;

		if (debug)
			debugPort->println("ERROR: CRC_ERROR");

		reset();
		results.success = false;
		return results;
	}

	results.success = true;
	return results;
}

parseResults Packet::parseFindEndByte(const uint8_t& recChar)
{
	parseResults results;
	
	if (debug == 3)
		debugPort->println("parse.state: find_end_byte");
	state = find_start_byte;

	if (recChar == STOP_BYTE)
	{
		if (packed)
			unpackPacket(rxBuff);

		bytesRead = bytesToRec;
		status    = NEW_DATA;

		if (callbacks)
		{
			if (idByte < callbacksLen)
				callbacks[idByte]();
			else if (debug)
			{
				debugPort->print(F("ERROR: No callback available for packet ID "));
				debugPort->println(idByte);
			}
		}
		packetStart = 0;	// reset the timer

		if (debug == 3)
		{
			debugPort->printf("parse.state2/status: %d %d\n", state, status);
			debugPort->println();
		}

		results.bytesRead = bytesToRec;
		results.success = true;
		return results;
	}

	bytesRead = 0;
	status    = STOP_BYTE_ERROR;

	if (debug)
		debugPort->println("ERROR: STOP_BYTE_ERROR");

	reset();
	results.success = false;
	return results;
}

parseResults Packet::parseUndefinedState(const uint8_t& recChar)
{
	parseResults results;
	
	if (debug)
	{
		debugPort->print("ERROR: Undefined state ");
		debugPort->println(state);
	}

	reset();
	bytesRead = 0;
	state     = find_start_byte;
	results.success = false;
	return results;
}

uint16_t Packet::parseTypeMaxMessage(const uint8_t& recChar, const bool& valid)
{
	uint32_t current = millis();
	bool packet_fresh = (packetStart == 0) || ((current - packetStart) < timeout);

	// if (!packet_fresh) //packet is stale, start over.
	// {
	// 	if (debug) {
	// 		debugPort->println("ERROR: STALE PACKET");
	// 		debugPort->printf("parse.packetStart: %u\n", packetStart);
	// 		debugPort->printf("parse.current: %u\n", current);
	// 		debugPort->printf("parse.timeout: %u\n", timeout);
	// 		debugPort->printf("parse.(current - packetStart): %u\n", (current - packetStart));
	// 		debugPort->printf("parse.((current - packetStart) < timeout): %u\n", ((current - packetStart) < timeout));
	// 	}

	// 	bytesRead   = 0;
	// 	state       = find_start_byte;
	// 	status      = STALE_PACKET_ERROR;
	// 	packetStart = 0;

	// 	return bytesRead;
	// }
	parseResults results = parseFreshCheck(recChar, valid, packet_fresh, current);
	if (!results.success)
		return results.bytesRead;

	if (valid)
	{
		if (debug == 3) 
		{
			debugPort->printf("parse.state: %d\n", state);
			debugPort->printf("parse.recChar: %d\n", recChar);
		}
		switch (state)
		{
			case find_start_byte: /////////////////////////////////////////
			{
				// if (debug == 3)
				// 	debugPort->println("parse.state: find_start_byte");
				// if (recChar == START_BYTE)
				// {
				// 	state       = find_id_byte;
				// 	packetStart = millis();	//start the timer
				// }

				parseFindStartByte(recChar);
				break;
			}

			case find_id_byte: ////////////////////////////////////////////
			{
				// if (debug == 3)
				// 	debugPort->println("parse.state: find_id_byte");
				// idByte = recChar;
				// state  = find_command;

				parseFindIdByte(recChar);
				break;
			}

			case find_command: ////////////////////////////////////////
			{
				// // get the high value of the 16 byte length
				// if (debug == 3) 
				// {
				// 	debugPort->println("parse.state: find_command");
				// 	debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
				// 	debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
				// 	debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
				// }
				// if ((recChar >= 0) && (recChar <= UINT16_MAX))
				// {
				// 	recCharPrevious = recChar;
				// 	state      = find_command2;
				// }
				// else
				// {
				// 	bytesRead = 0;
				// 	state     = find_start_byte;
				// 	status    = PAYLOAD_ERROR;

				// 	if (debug)
				// 		debugPort->println("ERROR: PAYLOAD_ERROR - COMMAND INVALID - LOW BYTE");

				// 	reset();
				// 	return bytesRead;
				// }

				parseResults results = parseFindCommand(recChar);
				if (!results.success)
					return results.bytesRead;

				break;
			}

			case find_command2: ////////////////////////////////////////
			{
				// // get the low value of the 16 byte length
				// if (debug == 3) 
				// {
				// 	debugPort->println("parse.state: find_command2");
				// 	debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
				// 	debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
				// 	debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
				// }
				// if ((recChar >= 0) && (recChar <= UINT16_MAX))
				// {
				// 	command = ((uint16_t)recCharPrevious << 8) | recChar;  // high | low
				// 	payIndex   = 0;
				// 	state      = find_overhead_byte;

				// 	if (debug == 3) 
				// 	{
				// 		debugPort->printf("parse.command: %d\n", command);
				// 		debugPort->printf("parse.MAX_PACKET_SIZE: %d\n", MAX_PACKET_SIZE);
				// 		debugPort->printf("parse.(command >= 0): %d\n", (command > 0));
				// 		debugPort->printf("parse.(command <= MAX_PACKET_SIZE): %d\n", (command <= MAX_PACKET_SIZE));
				// 		debugPort->printf("parse.(command >= 0) && (command <= MAX_PACKET_SIZE): %d\n", (command > 0) && (command <= MAX_PACKET_SIZE));
				// 	}
				// 	if (!((command >= 0) && (command <= MAX_PACKET_SIZE)))
				// 	{
				// 		command = 0;
				// 		state     = find_start_byte;
				// 		status    = PAYLOAD_ERROR;

				// 		if (debug)
				// 			debugPort->println("ERROR: PAYLOAD_ERROR - COMMAND INVALID");

				// 		reset();
				// 		return bytesRead;
				// 	}
				// }
				// else
				// {
				// 	bytesRead = 0;
				// 	state     = find_start_byte;
				// 	status    = PAYLOAD_ERROR;

				// 	if (debug)
				// 		debugPort->println("ERROR: PAYLOAD_ERROR - COMMAND INVALID - HIGH BYTE");

				// 	reset();
				// 	return bytesRead;
				// }

				parseResults results = parseFindCommand2(recChar);
				if (!results.success)
					return results.bytesRead;

				break;
			}

			case find_overhead_byte: //////////////////////////////////////
			{
				// if (debug == 3)
				// 	debugPort->println("parse.state: find_overhead_byte");
				// recOverheadByte = recChar;
				// state           = find_payload_len;

				parseResults results = parseFindOverheadByte(recChar);
				if (!results.success)
					return results.bytesRead;

				break;
			}

			case find_payload_len: ////////////////////////////////////////
			{
				// // get the high value of the 16 byte length
				// if (debug == 2) 
				// {
				// 	debugPort->println("parse.state: find_payload_len");
				// 	debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
				// 	debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
				// 	debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
				// }
				// if ((recChar >= 0) && (recChar <= UINT16_MAX))
				// {
				// 	recCharPrevious = recChar;
				// 	state      = find_payload_len2;
				// }
				// else
				// {
				// 	bytesRead = 0;
				// 	state     = find_start_byte;
				// 	status    = PAYLOAD_ERROR;

				// 	if (debug)
				// 		debugPort->println("ERROR: PAYLOAD_ERROR - PAYLOAD LENGTH INVALID - LOW BYTE");

				// 	reset();
				// 	return bytesRead;
				// }

				parseResults results = parseFindPayloadLength(recChar);
				if (!results.success)
					return results.bytesRead;

				break;
			}

			case find_payload_len2: ////////////////////////////////////////
			{
				// // get the low value of the 16 byte length
				// if (debug == 3) 
				// {
				// 	debugPort->println("parse.state: find_payload_len2");
				// 	debugPort->printf("parse.(recChar >= 0): %d\n", (recChar >= 0));
				// 	debugPort->printf("parse.(recChar <= UINT16_MAX): %d\n", (recChar <= UINT16_MAX));
				// 	debugPort->printf("parse.(recChar >= 0) && (recChar <= UINT16_MAX): %d\n", (recChar >= 0) && (recChar <= UINT16_MAX));
				// }
				// if ((recChar >= 0) && (recChar <= UINT16_MAX))
				// {
				// 	bytesToRec = ((uint16_t)recCharPrevious << 8) | recChar;  // high | low
				// 	payIndex   = 0;
				// 	state      = find_payload;

				// 	if (debug == 3) 
				// 	{
				// 		debugPort->printf("parse.bytesToRec: %d\n", bytesToRec);
				// 		debugPort->printf("parse.MAX_PACKET_SIZE: %d\n", MAX_PACKET_SIZE);
				// 		debugPort->printf("parse.(bytesToRec > 0): %d\n", (bytesToRec > 0));
				// 		debugPort->printf("parse.(bytesToRec <= MAX_PACKET_SIZE): %d\n", (bytesToRec <= MAX_PACKET_SIZE));
				// 		debugPort->printf("parse.(bytesToRec > 0) && (bytesToRec <= MAX_PACKET_SIZE): %d\n", (bytesToRec > 0) && (bytesToRec <= MAX_PACKET_SIZE));
				// 	}
				// 	if (!((bytesToRec > 0) && (bytesToRec <= MAX_PACKET_SIZE)))
				// 	{
				// 		bytesRead = 0;
				// 		state     = find_start_byte;
				// 		status    = PAYLOAD_ERROR;

				// 		if (debug)
				// 			debugPort->println("ERROR: PAYLOAD_ERROR - PAYLOAD LENGTH INVALID");

				// 		reset();
				// 		return bytesRead;
				// 	}
				// }
				// else
				// {
				// 	bytesRead = 0;
				// 	state     = find_start_byte;
				// 	status    = PAYLOAD_ERROR;

				// 	if (debug)
				// 		debugPort->println("ERROR: PAYLOAD_ERROR - PAYLOAD LENGTH INVALID - HIGH BYTE");

				// 	reset();
				// 	return bytesRead;
				// }

				parseResults results = parseFindPayloadLength2(recChar);
				if (!results.success)
					return results.bytesRead;

				break;
			}

			case find_payload: ////////////////////////////////////////////
			{
				// if (debug == 3) 
				// {
				// 	debugPort->println("parse.state: find_payload");
				// 	debugPort->printf("parse.payIndex: %d\n", payIndex);
				// 	debugPort->printf("parse.bytesToRec: %d\n", bytesToRec);
				// 	debugPort->printf("parse.recChar: %d\n", recChar);
				// }
				
				// if (payIndex < bytesToRec)
				// {
				// 	rxBuff[payIndex] = recChar;
				// 	payIndex++;

				// 	if (payIndex == bytesToRec)
				// 		state    = find_crc;
				// }

				parseFindPayload(recChar);

				break;
			}

			// case find_crc: ///////////////////////////////////////////
			// {
			// 	uint8_t calcCrc = crc.calculate(rxBuff, bytesToRec);

			// 	if (calcCrc == recChar)
			// 		state = find_end_byte;
			// 	else
			// 	{
			// 		bytesRead = 0;
			// 		state     = find_start_byte;
			// 		status    = CRC_ERROR;

			// 		if (debug)
			// 			debugPort->println("ERROR: CRC_ERROR");

			// 		reset();
			// 		return bytesRead;
			// 	}

			// 	break;
			// }

			case find_crc: ///////////////////////////////////////////
			{
				// // get the high value of the 16 byte crc
				// if (debug == 3)
				// 	debugPort->println("parse.state: find_crc");
				// recCharPrevious = recChar;
				// if (debug == 3)
				// 	debugPort->printf("parse.recChar.high: %d\n", recChar);
				// state = find_crc2;
				
				parseFindCrc(recChar);

				break;
			}

			case find_crc2: ////////////////////////////////////////
			{
				// // get the low value of the 16 byte crc
				// if (debug == 3)
				// 	debugPort->println("parse.state: find_crc2");
				// uint16_t calcCrc = crc.calculate(rxBuff, bytesToRec);
				// if (debug == 3) 
				// {
				// 	debugPort->printf("parse.calcCrc: %d\n", calcCrc);
				// 	debugPort->printf("parse.recChar.low: %d\n", recChar);
				// }
				// recvCrc = ((uint16_t)recCharPrevious << 8) | recChar;  // high | low
				// if (debug == 3) {
				// 	debugPort->printf("parse.recvCrc: %d\n", recvCrc);
				// 	debugPort->printf("parse.calcCrc==recvCrc: %d\n", (calcCrc == recvCrc));
				// }

				// if (calcCrc == recvCrc) {
				// 	state = find_end_byte;
				// }
				// else
				// {
				// 	bytesRead = 0;
				// 	state     = find_start_byte;
				// 	status    = CRC_ERROR;

				// 	if (debug)
				// 		debugPort->println("ERROR: CRC_ERROR");

				// 	reset();
				// 	return bytesRead;
				// }
				
				parseResults results = parseFindCrc2(recChar);
				if (!results.success)
					return results.bytesRead;

				break;
			}

			case find_end_byte: ///////////////////////////////////////////
			{
				// if (debug == 3)
				// 	debugPort->println("parse.state: find_end_byte");
				// state = find_start_byte;

				// if (recChar == STOP_BYTE)
				// {
				// 	if (packed)
				// 		unpackPacket(rxBuff);

				// 	bytesRead = bytesToRec;
				// 	status    = NEW_DATA;

				// 	if (callbacks)
				// 	{
				// 		if (idByte < callbacksLen)
				// 			callbacks[idByte]();
				// 		else if (debug)
				// 		{
				// 			debugPort->print(F("ERROR: No callback available for packet ID "));
				// 			debugPort->println(idByte);
				// 		}
				// 	}
				// 	packetStart = 0;	// reset the timer

				// 	if (debug == 3)
				// 	{
				// 		debugPort->printf("parse.state2/status: %d %d\n", state, status);
				// 		debugPort->println();
				// 	}
				// 	return bytesToRec;
				// }

				// bytesRead = 0;
				// status    = STOP_BYTE_ERROR;

				// if (debug)
				// 	debugPort->println("ERROR: STOP_BYTE_ERROR");

				// reset();
				// return bytesRead;

				parseResults results = parseFindEndByte(recChar);
				return results.bytesRead;
				break;
			}

			default:
			{
				// if (debug)
				// {
				// 	debugPort->print("ERROR: Undefined state ");
				// 	debugPort->println(state);
				// }

				// reset();
				// bytesRead = 0;
				// state     = find_start_byte;

				parseUndefinedState(recChar);
				break;
			}
		}
	}
	else
	{
		bytesRead = 0;
		status    = NO_DATA;
		return bytesRead;
	}

	if (debug == 3)
	{
		debugPort->printf("parse.state2: %d\n", state);
		debugPort->printf("parse.status: %d\n", status);
		debugPort->println();
	}

	bytesRead = 0;
	status    = CONTINUE;
	return bytesRead;
}

uint16_t Packet::parseTypeMinDelta(const uint8_t& recChar, const bool& valid)
{
	if (!valid) {
		bytesRead = 0;
		status    = NO_DATA;
		return bytesRead;
	}

	uint32_t current = millis();
	// bool packet_fresh = (packetStart == 0) || ((current - packetStart) < timeout);
	// bool packet_fresh = (packetStart == 0) || ((current - packetStart) < timeout);
	bool packet_fresh = (packetStart == 0) || ((current - packetLast) < timeout);
	packetLast = current;
	
	parseResults results = parseFreshCheck(recChar, valid, packet_fresh, current);
	if (!results.success)
		return results.bytesRead;

	if (debug == 3) 
	{
		debugPort->printf("parse.state/recChar: %d %d\n", state, recChar);
	}
	switch (state)
	{
		case find_start_byte: /////////////////////////////////////////
		{
			parseFindStartByte(recChar);
			break;
		}

		case find_id_byte: ////////////////////////////////////////////
		{
			parseFindIdByte(recChar);
			break;
		}

		case find_command: ////////////////////////////////////////
		{
			parseResults results = parseFindCommand(recChar);
			if (!results.success)
				return results.bytesRead;

			break;
		}

		case find_command2: ////////////////////////////////////////
		{
			parseResults results = parseFindCommand2(recChar);
			if (!results.success)
				return results.bytesRead;

			break;
		}

		case find_overhead_byte: //////////////////////////////////////
		{
			parseResults results = parseFindOverheadByte(recChar);
			if (!results.success)
				return results.bytesRead;

			break;
		}

		case find_payload_len: ////////////////////////////////////////
		{
			parseResults results = parseFindPayloadLength(recChar);
			if (!results.success)
				return results.bytesRead;

			break;
		}

		case find_payload_len2: ////////////////////////////////////////
		{
			parseResults results = parseFindPayloadLength2(recChar);
			if (!results.success)
				return results.bytesRead;

			break;
		}

		case find_payload: ////////////////////////////////////////////
		{
			parseFindPayload(recChar);
			break;
		}

		case find_crc: ///////////////////////////////////////////
		{
			parseFindCrc(recChar);
			break;
		}

		case find_crc2: ////////////////////////////////////////
		{
			parseResults results = parseFindCrc2(recChar);
			if (!results.success)
				return results.bytesRead;

			break;
		}

		case find_end_byte: ///////////////////////////////////////////
		{
			parseResults results = parseFindEndByte(recChar);
			return results.bytesRead;
			break;
		}

		default:
		{
			parseUndefinedState(recChar);
			break;
		}
	}

	if (debug == 3)
	{
		debugPort->printf("parse.state2/status: %d %d\n", state, status);
		debugPort->println();
	}

	bytesRead = 0;
	status    = CONTINUE;
	return bytesRead;
}
 
/*
 uint16_t Packet::currentCommand()
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
uint16_t Packet::currentCommand()
{
	return command;
}


/*
 uint8_t Packet::currentPacketID()
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
uint8_t Packet::currentPacketID()
{
	return idByte;
}


/*
 uint16_t Packet::currentReceived()
 Description:
 ------------
  * Returns the bytes received of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t - bytes received of the last parsed packet
*/
uint16_t Packet::currentReceived()
{
	return bytesToRec;
}


/*
 void Packet::calcOverhead(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Calculates the COBS (Consistent Overhead Stuffing) Overhead
  byte and stores it in the class's overheadByte variable. This
  variable holds the byte position (within the payload) of the
  first payload byte equal to that of START_BYTE
 Inputs:
 -------
  * uint8_t arr[] - Array of values the overhead is to be calculated
  over
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void Packet::calcOverhead(uint8_t arr[], const uint16_t& len)
{
	overheadByte = 0xFF;

	for (uint8_t i = 0; i < len; i++)
	{
		if (arr[i] == START_BYTE)
		{
			overheadByte = i;
			break;
		}
	}
}


/*
 int16_t Packet::findLast(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Finds last instance of the value START_BYTE within the given
  packet array
 Inputs:
 -------
  * uint8_t arr[] - Packet array
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * int16_t - Index of last instance of the value START_BYTE within the given
  packet array
*/
int16_t Packet::findLast(uint8_t arr[], const uint16_t& len)
{
	for (uint8_t i = (len - 1); i != 0xFF; i--)
		if (arr[i] == START_BYTE)
			return i;

	return -1;
}


/*
 void Packet::stuffPacket(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Enforces the COBS (Consistent Overhead Stuffing) ruleset across
  all bytes in the packet against the value of START_BYTE
 Inputs:
 -------
  * uint8_t arr[] - Array of values to stuff
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void Packet::stuffPacket(uint8_t arr[], const uint16_t& len)
{
	int16_t refByte = findLast(arr, len);

	if (refByte != -1)
	{
		for (uint8_t i = (len - 1); i != 0xFF; i--)
		{
			if (arr[i] == START_BYTE)
			{
				arr[i]  = refByte - i;
				refByte = i;
			}
		}
	}
}


/*
 void Packet::unpackPacket(uint8_t arr[], const uint8_t &len)
 Description:
 ------------
  * Unpacks all COBS-stuffed bytes within the array
 Inputs:
 -------
  * uint8_t arr[] - Array of values to unpack
  * const uint8_t &len - Number of elements in arr[]
 Return:
 -------
  * void
*/
void Packet::unpackPacket(uint8_t arr[])
{
	uint8_t testIndex = recOverheadByte;
	uint8_t delta     = 0;

	if (testIndex <= MAX_PACKET_SIZE)
	{
		while (arr[testIndex])
		{
			delta          = arr[testIndex];
			arr[testIndex] = START_BYTE;
			testIndex += delta;
		}
		arr[testIndex] = START_BYTE;
	}
}


/*
 void Packet::reset()
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
void Packet::reset()
{
	memset(txBuff, 0, sizeof(txBuff));
	memset(rxBuff, 0, sizeof(rxBuff));

	bytesRead   = 0;
	recvCrc   	= 0;
	packetStart = 0;
	packetLast  = 0;
}
