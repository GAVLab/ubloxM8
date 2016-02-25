/*
* Copyright (C) 2016 Swift Navigation Inc.
* Contact: Pasi Miettinen <pasi.miettinen@exafore.com>
*
* This source is subject to the license found in the file 'LICENSE'
* which must be be distributed together with this source. All other
* rights reserved.
*
* THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
* PURPOSE.
*/

#include "ubloxM8/ublox_m8.h"
#include <iostream>

using namespace std;
using namespace ublox_m8;

#define PI 3.14159265
#define LOG_PAYLOAD_LEN(log) ((((uint16_t) *(log+5)) << 8) +\
                               ((uint16_t) *(log+4)))

inline void printHex(char *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned) (unsigned char) data[i]);
    }
    printf("\n");
}

/*!
 * Default callback method for time stamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
inline double DefaultGetTime() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

// inline void DefaultAcknowledgementHandler() {
//     //std::cout << "Acknowledgment received." << std::endl;
// }

inline void DefaultDebugMsgCallback(const std::string &msg) {
    std::cout << "Ublox Debug: " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string &msg) {
    std::cout << "Ublox Info: " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string &msg) {
    std::cout << "Ublox Warning: " << msg << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Ublox Error: " << msg << std::endl;
}

inline void DefaultCfgPrtCallback(ublox_m8::CfgPrt port_settings,
        double time_stamp) {
    std::cout << "CFG-PRT:" << std::endl;
}

inline void DefaultCfgNav5Callback(ublox_m8::CfgNav5 cfg_nav5,
        double time_stamp) {
    std::cout << "CFG-NAV5:" << std::endl;
}

inline void DefaultNavSolCallback(ublox_m8::NavSol nav_sol, double time_stamp) {
    std::cout << "NAV-SOL: " << endl;
}

inline void DefaultNavStatusCallback(ublox_m8::NavStatus nav_status, double time_stamp) {
    std::cout << "GPS Fix Type: ";
    if (nav_status.fixtype == 0x00) {
        std::cout << "No Fix" << std::endl;
        std::cout << "TTFF: " << " none ms" << std::endl;
        std::cout << "Milliseconds since Startup/Reset: " << nav_status.msss
                << std::endl;
    } else if (nav_status.fixtype == 0x01) {
        std::cout << "Dead Reckoning Only" << std::endl;
    } else if (nav_status.fixtype == 0x02) {
        std::cout << "2D Fix" << std::endl;
    } else if (nav_status.fixtype == 0x03) {
        std::cout << "3D Fix" << std::endl;
    } else if (nav_status.fixtype == 0x04) {
        std::cout << "GPS + Dead Reckoning" << std::endl;
    } else if (nav_status.fixtype == 0x05) {
        std::cout << "Time Only" << std::endl;
    } else {
        std::cout << std::endl;
    }

    if (nav_status.fixtype != 0x00) {
        std::cout << "TTFF: " << (nav_status.ttff / 1000.) << " sec"
                << std::endl;
        std::cout << "Milliseconds since Startup/Reset: "
                << (nav_status.msss / 1000.) << " sec" << std::endl;
    }
}

inline void DefaultNavVelNEDCallback(ublox_m8::NavVelNED nav_vel_ned, double time_stamp) {
    std::cout << "NAV-VELNED: " << endl;
}

inline void DefaultNavSVInfoCallback(ublox_m8::NavSVInfo nav_sv_info, double time_stamp) {
    std::cout << "NAV-SVINFO: " << endl;
}

inline void DefaultNavTimeGPSCallback(ublox_m8::NavTimeGPS nav_gps_time,
        double time_stamp) {
    std::cout << "NAV-TIMEGPS: " << endl;
}

inline void DefaultNavTimeGLOCallback(ublox_m8::NavTimeGLO nav_glo_time,
        double time_stamp) {
    std::cout << "NAV-TIMEGLO: " << endl;
}

inline void DefaultNavTimeBDSCallback(ublox_m8::NavTimeBDS nav_bds_time,
        double time_stamp) {
    std::cout << "NAV-TIMEBDS: " << endl;
}

inline void DefaultNavTimeUTCCallback(ublox_m8::NavTimeUTC nav_utc_time,
        double time_stamp) {
    std::cout << "NAV-TIMEUTC: " << endl;
}

inline void DefaultNavDOPCallback(ublox_m8::NavDOP nav_dop, double time_stamp) {
    std::cout << "NAV-DOP: " << endl;
}

inline void DefaultNavDGPSCallback(ublox_m8::NavDGPS nav_dgps, double time_stamp) {
    std::cout << "NAV-DGPS: " << endl;
}

inline void DefaultNavClockCallback(ublox_m8::NavClock nav_clock, double time_stamp) {
    std::cout << "NAV-CLK: " << endl;
}

inline void DefaultNavPosLlhCallback(ublox_m8::NavPosLLH nav_position, double time_stamp){
    /*std:: cout << "NAV-POSLLH: " << endl <<
                  "  GPS milliseconds: " << nav_position.iTOW << std::endl <<
                  "  Latitude: " << nav_position.latitude_scaled << std::endl <<
                  "  Longitude: " << nav_position.longitude_scaled << std::endl <<
                  "  Height: " << nav_position.height << std::endl << std::endl;*/
}

inline void DefaultRxmSvsiCallback(ublox_m8::RxmSvsi sv_stat, double time_stamp) {
    std::cout << "RXM-SVSI: " << std::endl;
}

UbloxM8::UbloxM8() {
    serial_port_ = NULL;
    reading_status_ = false;
    parse_log_callback_ = NULL;
    time_handler_ = DefaultGetTime;
    //handle_acknowledgement_ = DefaultAcknowledgementHandler;
    cfg_prt_callback_ = DefaultCfgPrtCallback;
    cfg_nav5_callback_ = DefaultCfgNav5Callback;
    nav_pos_llh_callback_ = DefaultNavPosLlhCallback;
    nav_sol_callback_ = DefaultNavSolCallback;
    nav_status_callback_ = DefaultNavStatusCallback;
    nav_vel_ned_callback_ = DefaultNavVelNEDCallback;
    nav_sv_info_callback_ = DefaultNavSVInfoCallback;
    nav_time_gps_callback_ = DefaultNavTimeGPSCallback;
    nav_time_glo_callback_ = DefaultNavTimeGLOCallback;
    nav_time_bds_callback_ = DefaultNavTimeBDSCallback;
    nav_time_utc_callback_ = DefaultNavTimeUTCCallback;
    nav_dop_callback_ = DefaultNavDOPCallback;
    nav_dgps_callback_ = DefaultNavDGPSCallback;
    nav_clock_callback_ = DefaultNavClockCallback;
    rxm_svsi_callback_ = DefaultRxmSvsiCallback;
    log_debug_ = DefaultDebugMsgCallback;
    log_info_ = DefaultInfoMsgCallback;
    log_warning_ = DefaultWarningMsgCallback;
    log_error_ = DefaultErrorMsgCallback;
    //reading_acknowledgement_ = false;
    bytes_remaining_ = false;
    // header_length_ = 0;
    // msgID = 0;
    // data_read_ = NULL;
    buffer_index_ = 0;
    read_timestamp_ = 0;
    //parse_timestamp_ = 0;
    is_connected_ = false;
}

UbloxM8::~UbloxM8() {
    Disconnect();

    if (raw_log.is_open())
      raw_log.close();
}

bool UbloxM8::Connect(std::string port, int baudrate) {
  //serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
  serial::Timeout my_timeout(50, 200, 0, 200, 0);//(100, 1000, 0, 1000, 0);
  try {
    serial_port_ = new serial::Serial(port, baudrate, my_timeout);

    if (!serial_port_->isOpen()) {
      std::stringstream output;
      output << "Serial port: " << port << " failed to open.";
      log_error_(output.str());
      delete serial_port_;
      serial_port_ = NULL;
      is_connected_ = false;
      return false;
    } else {
      std::stringstream output;
      output << "Serial port: " << port << " opened successfully.";
      log_info_(output.str());
    }

        //! stop any incoming data and flush buffers
    //std::cout << "Flushing port" << std::endl;
    serial_port_->flush();
    
    //! stop any incoming nmea data
    SetCfgPrtUsb(true,true,false,false);

    //! wait for data to stop cominig in
    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    //    // clear serial port buffers
    //    serial_port_->flush();

    // look for GPS by sending ping and waiting for response
    if (!Ping()) {
      std::stringstream output;
      output << "Ublox GPS not found on port: " << port << std::endl;
      log_error_(output.str());
      delete serial_port_;
      serial_port_ = NULL;
      is_connected_ = false;
      return false;
    }
  } catch (std::exception e) {
         std::stringstream output;
         output << "Failed to open port " << port << "  Err: " << e.what();
         log_error_(output.str());
         serial_port_ = NULL;
         is_connected_ = false;
         return false;
     }

  // start reading
  StartReading();
  is_connected_ = true;
  return true;
}

void UbloxM8::ReadFile(std::string name)
{
  uint8_t buf[MAX_NOUT_SIZE];
  std::ifstream ifs(name, std::ios::binary);

  while (ifs.good()) {
    ifs.read((char *)buf, sizeof(buf));
    while (ifs.gcount() >\
      std::streamsize(DATA_BUF_SIZE - buffer_index_)) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    ReadFromFile(buf, ifs.gcount());
  }
  ifs.close();
}

void UbloxM8::ReadFromFile(unsigned char* buffer, unsigned int length)
{
  BufferIncomingData(buffer, length);
}

bool UbloxM8::Ping(int num_attempts) {
  try {
    while ((num_attempts--) > 0) {
      log_info_("Searching for Ublox receiver...");
      // request version information

      // ask for version
      PollMessage(MSG_CLASS_MON, MSG_ID_MON_VER);

      boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

      unsigned char result[5000];
      size_t bytes_read;
      bytes_read = serial_port_->read(result, 5000);

      //std::cout << "bytes read: " << (int)bytes_read << std::endl;
      //std::cout << dec << result << std::endl;

      if (bytes_read < 8) {
        stringstream output;
        output << "Only read " << bytes_read
                << " bytes in response to ping.";
        log_debug_(output.str());
        continue;
      }

      uint16_t length;
      // search through result for version message
      for (uint32_t ii = 0; ii < (bytes_read - 8); ii++) {
        //std::cout << hex << (unsigned int)result[ii] << std::endl;
        if (result[ii] == UBX_SYNC_BYTE_1) {
          if (result[ii + 1] != UBX_SYNC_BYTE_2)
            continue;
          if (result[ii + 2] != MSG_CLASS_MON)
            continue;
          if (result[ii + 3] != MSG_ID_MON_VER)
            continue;
          //std::cout << "length1:" << hex << (unsigned int)result[ii+4] << std::endl;
          //std::cout << "length2:" << hex << (unsigned int)result[ii+5] << std::endl;
          length = (result[ii + 4]) + (result[ii + 5] << 8);
          if (length < 40) {
            log_debug_("Incomplete version message received");
            //    //return false;
            continue;
          }

          string sw_version;
          string hw_version;
          string rom_version;
          sw_version.append((char*) (result + 6));
          hw_version.append((char*) (result + 36));
          //rom_version.append((char*)(result+46));
          log_info_("Ublox receiver found.");
          log_info_("Software Version: " + sw_version);
          log_info_("Hardware Version: " + hw_version);
          //log_info_("ROM Version: " + rom_version);
          return true;
        }
      }
      stringstream output;
      output << "Read " << bytes_read
             << " bytes, but version message not found.";
      log_debug_(output.str());
    }
  } catch (exception &e) {
      std::stringstream output;
      output << "Error pinging receiver: " << e.what();
      log_error_(output.str());
      return false;
  }

  return false;
}

void UbloxM8::Disconnect() {
	try {
		if (reading_status_) {
			StopReading();
			// TODO: wait here for reading to stop
		}
		if (serial_port_ != NULL) {
			if (serial_port_->isOpen())
				serial_port_->close();
			delete serial_port_;
			serial_port_ = NULL;
		}
	} catch (std::exception &e) {
		std::stringstream output;
				output << "Error disconnecting from ublox: " << e.what();
				log_error_(output.str());
	}
}

void UbloxM8::StartReading() {
	try {
		// create thread to read from sensor
		reading_status_ = true;
		read_thread_ptr_ = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&UbloxM8::ReadSerialPort, this)));
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error starting ublox read thread: " << e.what();
		log_error_(output.str());
	}
}

void UbloxM8::StopReading() {
    reading_status_ = false;
}

void UbloxM8::ReadSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status_) {
        // read data
        try {
            len = serial_port_->read(buffer, MAX_NOUT_SIZE);

            if (raw_log.is_open())
              raw_log.write((char *)buffer, len);

            // timestamp the read
            if (time_handler_) 
              read_timestamp_ = time_handler_();
            else 
              read_timestamp_ = 0;
            // add data to the buffer to be parsed
            BufferIncomingData(buffer, len);
        } catch (exception &e) {
            //stringstream output;
            //output << "Error reading serial port: " << e.what();
            //log_info_(output.str());
            Disconnect();
            return;
        }
    }
}

bool UbloxM8::CreateRawLog(std::string &name) {
  try {
    if (raw_log.is_open())
      raw_log.close();
  } catch (exception &e) {
    return false;
  }

  raw_log.open(name, std::ios::binary);
  
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// POLLING METHODS
/////////////////////////////////////////////////////////////////////////////

// Poll Message used to request for all SV
bool UbloxM8::PollMessage(uint8_t class_id, uint8_t msg_id) {
	try {
		uint8_t message[8];

		message[0]=UBX_SYNC_BYTE_1;        // sync 1
		message[1]=UBX_SYNC_BYTE_2;        // sync 2
		message[2]=class_id;
		message[3]=msg_id;
		message[4]=0;           // length 1
		message[5]=0;           // length 2
		message[6]=0;           // checksum 1
		message[7]=0;           // checksum 2

		uint8_t* msg_ptr = (uint8_t*) &message;

		calculateCheckSum(msg_ptr + 2, 4, msg_ptr + 6);

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  size_t bytes_written = serial_port_->write(message, 8);
          return bytes_written == 8;
        } else {
            log_error_("Unable to send poll message. Serial port not open.");
            return false;
        }

	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending ublox poll message: " << e.what();
		log_error_(output.str());
		return 0;
	}
}

// Poll Message used to request for one SV
bool UbloxM8::PollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid) {
    try {
		uint8_t message[9];

		message[0] = UBX_SYNC_BYTE_1;        // sync 1
		message[1] = UBX_SYNC_BYTE_2;        // sync 2
		message[2] = class_id;
		message[3] = msg_id;
		message[4] = 1;           // length 1
		message[5] = 0;           // length 2
		message[6] = svid;        // Payload
		message[7] = 0;           // checksum 1
		message[8] = 0;           // checksum 2

		uint8_t* msg_ptr = (uint8_t*) &message;
		calculateCheckSum(msg_ptr + 2, 5, msg_ptr + 7);

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		    size_t bytes_written = serial_port_->write(msg_ptr, 9);
            return bytes_written == 9;
        } else {
            log_error_("Unable to send poll ind. sv. message. Serial port not open.");
            return false;
        }

    } catch (std::exception &e) {
		std::stringstream output;
		output << "Error polling individual svs: " << e.what();
		log_error_(output.str());
		return 0;
	}
}

// (CFG-NAV5) Polls current navigation algorithms parameters
bool UbloxM8::PollCfgNav5() {
    log_info_("Polling for CFG-NAV5..");
    return PollMessage(MSG_CLASS_CFG, MSG_ID_CFG_NAV5);
}

// Poll Port Configuration
void UbloxM8::PollCfgPrt(uint8_t port_identifier)
{ // Port identifier = 3 for USB (default value if left blank)
  //                 = 1 or 2 for UART
    try {
        uint8_t message[9];
        message[0]=UBX_SYNC_BYTE_1;
        message[1]=UBX_SYNC_BYTE_2;
        message[2]=MSG_CLASS_CFG;
        message[3]=MSG_ID_CFG_PRT;
        message[4]=1;
        message[5]=0;
        message[6]=port_identifier;         //Port identifier for USB Port (3)
        message[7]=0;                       // Checksum A
        message[8]=0;                       // Checksum B

        unsigned char* msg_ptr = (unsigned char*)&message;
        calculateCheckSum(msg_ptr+2,5,msg_ptr+7);

        serial_port_->write(msg_ptr, sizeof(message));
        log_info_("Polling for Port Protocol Configuration.");
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error polling ublox port configuration: " << e.what();
        log_error_(output.str());
    }
}
void UbloxM8::PollCfgPrtCurrent()
{
        //Polls the configuration of the port currently being used.
        try {
        uint8_t message[8];
        message[0]=UBX_SYNC_BYTE_1;
        message[1]=UBX_SYNC_BYTE_2;
        message[2]=MSG_CLASS_CFG;
        message[3]=MSG_ID_CFG_PRT;
        message[4]=0; // payload length
        message[5]=0; // payload length
        message[6]=0;                       // Checksum A
        message[7]=0;                       // Checksum B

        unsigned char* msg_ptr = (unsigned char*)&message;
        calculateCheckSum(msg_ptr+2,4,msg_ptr+6);

        serial_port_->write(msg_ptr, sizeof(message));
        log_info_("Polling for Current Port Configuration.");
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in UbloxM8::PollCfgPrtCurrent(): " << e.what();
        log_error_(output.str());
    }
}

void UbloxM8::PollCfgPrtUsb()
{
    //Polls the configuration of the USB port, no matter what port is currently being used.
    PollCfgPrt(3);
}

// (NAV-SVINFO) Polls for Space Vehicle Information
bool UbloxM8::PollNavSvInfo() {
    log_debug_("Polling for NAV-SVINFO..");
    return PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_SVINFO);
}

// (NAV-STATUS) Polls for Receiver Navigation Status
bool UbloxM8::PollNavStatus() {
    log_debug_("Polling for Receiver NAV-STATUS..");
    return PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_STATUS);
}

// (RXM-SVSI) Polls for Satellite Status Info
bool UbloxM8::PollRxmSvsi() {
    log_debug_("Polling for RXM-SVSI..");
    return PollMessage(MSG_CLASS_RXM, MSG_ID_RXM_SVSI);
}

////////////////////////////////////////////////////////
// COMMANDS
////////////////////////////////////////////////////////
// Receiver Reset
bool UbloxM8::Reset(uint16_t nav_bbr_mask, uint8_t reset_mode) {
	try {
        ublox_m8::CfgRst message;

		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_RST;
		message.header.payload_length = 4;

		message.nav_bbr_mask = nav_bbr_mask;    //X2-Bitfield?
		// Startup Modes
		// Hotstart 0x000
		// Warmstart 0x0001
		// Coldstart 0xFFFF
		message.reset_mode = reset_mode;
		// Reset Modes:
		// Hardware Reset 0x00
		// Controlled Software Reset 0x01
		// Controlled Software Reset - Only GPS 0x02
		// Hardware Reset After Shutdown 0x04
		// Controlled GPS Stop 0x08
		// Controlled GPS Start 0x09

		//message.reserved = 0;

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 8, message.checksum);
        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
        } else {
            log_error_("Unable to send reset command. Serial port not open.");
            return false;
        }
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error resetting ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Receiver Reset Messages - Force Cold Start
bool UbloxM8::ResetToColdStart() {
    log_info_("Receiver reset to cold start state.");
    return Reset(0xFFFF, 0x02);
}

// Receiver Reset Messages - Force Warm Start
bool UbloxM8::ResetToWarmStart() {
    log_info_("Receiver reset to warm start state.");
    return Reset(0x0001, 0x02);
}

// Receiver Reset Messages - Force Hot Start
bool UbloxM8::ResetToHotStart() {
    log_info_("Receiver reset to hot start state.");
    return Reset(0x0000, 0x02);
}

////////////////////////////////////////////////////////
// INPUT METHODS
////////////////////////////////////////////////////////

// Send Message
bool UbloxM8::SendMessage(uint8_t* msg_ptr, size_t length)
{
    try {
        stringstream output1;
        //std::cout << length << std::endl;
        //std::cout << "Message Pointer" << endl;
        //printHex((char*) msg_ptr, length);
        size_t bytes_written;

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
          bytes_written=serial_port_->write(msg_ptr, length);
        } else {
            log_error_("Unable to send message. Serial port not open.");
            return false;
        }
        // check that full message was sent to serial port
        if (bytes_written == length) {
            return true;
        }
        else {
            log_error_("Full message was not sent over serial port.");
            output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
            log_error_(output1.str());
            return false;
        }
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error sending ublox message: " << e.what();
        log_error_(output.str());
        return false;
    }
}

// (CFG-NAV5) Cofigure Navigation Algorithm Parameters
bool UbloxM8::SetCfgNav5(uint8_t dynamic_model, uint8_t fix_mode){
	try {
        ublox_m8::CfgNav5 message;

		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_NAV5;
		message.header.payload_length = 36;

		message.mask = 0b00000101;
		message.dynamic_model = dynamic_model;
		message.fix_mode = fix_mode;
		message.fixed_altitude = 0;
		message.fixed_altitude_variance = 0;
		message.min_elevation = 0;
		message.dead_reckoning_limit = 0;
		message.pdop = 0;
		message.tdop = 0;
		message.pos_accuracy_mask = 0;
		message.time_accuracy_mask = 0;
		message.static_hold_threshold = 0;
		message.dgps_timeout = 0;
		message.reserved2 = 0;
		message.reserved3 = 0;
		message.reserved4 = 0;

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 36+4, message.checksum);

		return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring ublox navigation parameters: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// (CFG-MSG) Set message output rate for specified message
bool UbloxM8::SetCfgMsgRate(uint8_t class_id, uint8_t msg_id, uint8_t rate) {
  try {
    ublox_m8::CfgMsg message;
    message.header.sync1 = UBX_SYNC_BYTE_1;
    message.header.sync2 = UBX_SYNC_BYTE_2;
    message.header.message_class = MSG_CLASS_CFG;
    message.header.message_id = MSG_ID_CFG_MSG;
    message.header.payload_length = 3;

    message.message_class = class_id;
    message.message_id = msg_id;
    message.rate = rate;

    unsigned char* msg_ptr = (unsigned char*) &message;
    calculateCheckSum(msg_ptr + 2, 7, message.checksum);

    /* make sure that cfg messages are not sent too fast */
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
  } catch (std::exception &e) {
    std::stringstream output;
    output << "Error configuring ublox message rate: " << e.what();
    log_error_(output.str());
    return false;
  }
}

// Set Port Configuration
void UbloxM8::SetCfgPrtUsb(bool ubx_input, bool ubx_output,
        bool nmea_input, bool nmea_output) {
	try {
        ublox_m8::CfgPrt message;
		//std::cout << sizeof(message) << std::endl;
		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_PRT;
		message.header.payload_length = 20;

		message.port_id = 3;          //Port identifier for USB Port (3)
		message.reserved = 0;
		message.tx_ready = 0;
		message.reserved2 = 0;
		message.reserved3 = 0;
		message.input_mask = 0;       // Specifies input protocols
		message.output_mask = 0;      // Specifies output protocols
		message.reserved4 = 0;
		message.reserved5 = 0;

		if (ubx_input)
			message.input_mask = message.input_mask | 0x0001;   // set first bit
		else
			message.input_mask = message.input_mask & 0xFFFE;   // clear first bit

		if (nmea_input)
			message.input_mask = message.input_mask | 0x0002;   // set second bit
		else
			message.input_mask = message.input_mask & 0xFFFD;   // clear second bit

		if (ubx_output)
			message.output_mask = message.output_mask | 0x0001;   // set first bit
		else
			message.output_mask = message.output_mask & 0xFFFE;   // clear first bit

		if (nmea_output)
			message.output_mask = message.output_mask | 0x0002;   // set second bit
		else
			message.output_mask = message.output_mask & 0xFFFD;  // clear second bit

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 27, message.checksum);

		log_info_("Set Port Settings Message Sent");

		//printHex((char*) &message, sizeof(message));

		serial_port_->write(msg_ptr, sizeof(message));
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring ublox port: " << e.what();
		log_error_(output.str());
	}
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
void UbloxM8::BufferIncomingData(uint8_t *msg, size_t length) {
  //MOOSTrace("Inside BufferIncomingData\n");
  //cout << length << endl;
  //cout << 0 << ": " << dec << (int)msg[0] << endl;
  // add incoming data to buffer

  //printHex(reinterpret_cast<char*>(msg),length);
  try {
        unsigned short msgID = 0;
        
    for (unsigned int i = 0; i < length; i++) {
      //cout << i << ": " << hex << (int)msg[i] << dec << endl;
      // make sure buffer_index_ is not larger than buffer
      if (buffer_index_ >= DATA_BUF_SIZE) {
        buffer_index_ = 0;
        log_warning_(
            "Overflowed receiver buffer. See ublox.cpp BufferIncomingData");

      }
      //cout << "buffer_index_ = " << buffer_index_ << endl;

      if (buffer_index_ == 0) {	// looking for beginning of message
        if (msg[i] == UBX_SYNC_BYTE_1) {	// beginning of msg found - add to buffer
                    //cout << "got first bit" << endl;
          data_buffer_[buffer_index_++] = msg[i];
          bytes_remaining_ = 0;
        }	// end if (msg[i]
      } // end if (buffer_index_==0)
      else if (buffer_index_ == 1) {	// verify 2nd character of header
        if (msg[i] == UBX_SYNC_BYTE_2) {	// 2nd byte ok - add to buffer
                    //cout << " got second synch bit" << endl;
          data_buffer_[buffer_index_++] = msg[i];
        } else {
          // start looking for new message again
          buffer_index_ = 0;
          bytes_remaining_ = 0;
          //readingACK=false;
        } // end if (msg[i]==UBX_SYNC_BYTE_2)
      }	// end else if (buffer_index_==1)
      else if (buffer_index_ == 2) {	//look for ack

        if (msg[i] == MSG_CLASS_ACK) {   // ACK or NAK message class
          // Add function which takes class_id and msg_id and returns name of corresponding message

          if (msg[i + 1] == MSG_ID_ACK_ACK) {  // ACK Message
            std::cout << "Receiver Acknowledged Message " << std::endl;
            printf("0x%.2X ", msg[i + 4]);
            std::cout << " ";
            printf("0x%.2X ", msg[i + 5]);
            std::cout << endl;

          } else if (msg[i + 1] == MSG_ID_ACK_NAK) {   // NAK Message
            std::cout << "Receiver Did Not Acknowledged Message " << std::endl;
            printf("0x%.2X ", msg[i + 4]);
            std::cout << " ";
            printf("0x%.2X ", msg[i + 5]);
            std::cout << endl;
          }

          buffer_index_ = 0;
          bytes_remaining_ = 0;
          //readingACK = false;			//? Why is readingACK = false in if & else statement? - CC
        } else {
          data_buffer_[buffer_index_++] = msg[i];
          //readingACK = false;
        }
      } else if (buffer_index_ == 3) {
        // msg[i] and msg[i-1] define message ID
        data_buffer_[buffer_index_++] = msg[i];
        // length of header is in byte 4

        //printHex(reinterpret_cast < char * > (data_buffer_),4);

        msgID = ((data_buffer_[buffer_index_ - 2]) << 8)
            + data_buffer_[buffer_index_ - 1];
        //cout << "msgID = " << msgID << endl;
      } else if (buffer_index_ == 5) {
        // add byte to buffer
        data_buffer_[buffer_index_++] = msg[i];
        // length of message (payload + 2 byte check sum )
        bytes_remaining_ = ((data_buffer_[buffer_index_ - 1]) << 8)
            + data_buffer_[buffer_index_ - 2] + 2;

        //cout << "bytes_remaining_ = " << bytes_remaining_ << endl;

        ///cout << msgID << endl;
      } else if (buffer_index_ == 6) {	// set number of bytes
        data_buffer_[buffer_index_++] = msg[i];
        bytes_remaining_--;
      } else if (bytes_remaining_ == 1) {	// add last byte and parse
        data_buffer_[buffer_index_++] = msg[i];
        //std::cout << hex << (int)msg[i] << dec << std::endl;
        //cout << " msgID = " << msgID << std::endl;
        ParseLog(data_buffer_, msgID);
        // reset counters
        buffer_index_ = 0;
        bytes_remaining_ = 0;
        //cout << "Message Done." << std::endl;
      }  // end else if (bytes_remaining_==1)
      else {	// add data to buffer
        data_buffer_[buffer_index_++] = msg[i];
        bytes_remaining_--;
      }
    }	// end for
  } catch (std::exception &e) {
    std::stringstream output;
    output << "Error buffering incoming ublox data: " << e.what();
    log_error_(output.str());
  }
}

void UbloxM8::ParseLog(uint8_t *log, size_t logID) {
  try {
    uint16_t payload_length;
    uint8_t num_of_svs;
    uint8_t num_of_channels;
        

    switch (logID) {

    case CFG_PRT:
            ublox_m8::CfgPrt cur_port_settings;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_port_settings, log, payload_length+HDR_CHKSM_LENGTH);
      //printHex((char*) &cur_port_settings, sizeof(cur_port_settings));
      if (cfg_prt_callback_)
        cfg_prt_callback_(cur_port_settings, read_timestamp_);
      break;

    case CFG_NAV5:
            ublox_m8::CfgNav5 cur_nav5_settings;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav5_settings, log, payload_length+HDR_CHKSM_LENGTH);
      //printHex((char*) &cur_port_settings, sizeof(cur_port_settings));
      if (cfg_nav5_callback_)
        cfg_nav5_callback_(cur_nav5_settings, read_timestamp_);
      break;

    case NAV_STATUS:
            ublox_m8::NavStatus cur_nav_status;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_status, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_status_callback_)
        nav_status_callback_(cur_nav_status, read_timestamp_);
      break;

    case NAV_SOL:
            ublox_m8::NavSol cur_nav_sol;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_sol, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_sol_callback_)
        nav_sol_callback_(cur_nav_sol, read_timestamp_);
      break;

    case NAV_VELNED:
      ublox_m8::NavVelNED cur_nav_vel_ned;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_vel_ned, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_vel_ned_callback_)
        nav_vel_ned_callback_(cur_nav_vel_ned, read_timestamp_);
      break;

    case NAV_VELECEF:
    {
      ublox_m8::NavVelECEF velecef;
      memcpy(&velecef, log, LOG_PAYLOAD_LEN(log) + HDR_CHKSM_LENGTH);
      if (nav_vel_ecef_callback_)
        nav_vel_ecef_callback_(velecef, read_timestamp_);
      break;
    }

    case NAV_POSECEF:
    {
      ublox_m8::NavPosECEF pos;
      memcpy(&pos, log, LOG_PAYLOAD_LEN(log) + HDR_CHKSM_LENGTH);
      if (nav_pos_ecef_callback_)
        nav_pos_ecef_callback_(pos, read_timestamp_);
      break;
    }

    case NAV_POSLLH:
      ublox_m8::NavPosLLH cur_nav_position;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_position, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_pos_llh_callback_)
        nav_pos_llh_callback_(cur_nav_position, read_timestamp_);
      break;

    case NAV_SVINFO:
      ublox_m8::NavSVInfo cur_nav_svinfo;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      num_of_channels = (uint8_t) *(log+10);

      //std::cout << "NAV-SVINFO..." << std::endl;
      // print whole message
      //printHex((char*) log, payload_length+8);

      // Copy portion of NAV-INFO before repeated block (8 + header length)
      memcpy(&cur_nav_svinfo, log, 6+8);
      // Copy repeated block
      for(int index = 0; index < num_of_channels; index++) {
        memcpy(&cur_nav_svinfo.svinfo_repeated[index],
          log+14+(index*12), 12);
      }
      // Copy Checksum
      memcpy(&cur_nav_svinfo.checksum, log+14+(num_of_channels*12), 2);

      // Print populated structure
      //printHex((char*) &cur_nav_svinfo, sizeof(cur_nav_svinfo));
      //std::cout << std::endl;

      if (nav_sv_info_callback_)
        nav_sv_info_callback_(cur_nav_svinfo, read_timestamp_);
      break;

    case NAV_TIMEGPS:
            ublox_m8::NavTimeGPS cur_nav_gps_time;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_gps_time, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_time_gps_callback_)
        nav_time_gps_callback_(cur_nav_gps_time, read_timestamp_);
      break;

    case NAV_UTCTIME:
            ublox_m8::NavTimeUTC cur_nav_utc_time;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_utc_time, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_time_utc_callback_)
        nav_time_utc_callback_(cur_nav_utc_time, read_timestamp_);
      break;

    case NAV_DOP:
            ublox_m8::NavDOP cur_nav_dop;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_dop, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_dop_callback_)
        nav_dop_callback_(cur_nav_dop, read_timestamp_);
      break;

    case NAV_DGPS:
            ublox_m8::NavDGPS cur_nav_dgps;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_dgps, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_dgps_callback_)
        nav_dgps_callback_(cur_nav_dgps, read_timestamp_);
      break;

    case NAV_CLOCK:
            ublox_m8::NavClock cur_nav_clock;
      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      memcpy(&cur_nav_clock, log, payload_length+HDR_CHKSM_LENGTH);
      if (nav_clock_callback_)
        nav_clock_callback_(cur_nav_clock, read_timestamp_);
      break;

    case RXM_SVSI:
      // NOTE: needs to be checked!!
      ublox_m8::RxmSvsi cur_sv_status;

      payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
      num_of_svs = (uint8_t) *(log+13);

      /*
      std::cout << "number of svs following: "<<(double) num_of_svs << endl;
      std::cout << "payload length: "<<(double) payload_length << endl;
      printHex((char*) &payload_length,2);
      // print whole message
      std::cout << "RXM-SVSI..." << std::endl;
      printHex((char*) log, payload_length+8);
      std::cout <<std::endl;
      */

      // Copy portion of RXM-SVSI before repeated block (8 + header length)
      memcpy(&cur_sv_status, log, 6 + 8);
      // Copy repeated block
      for (uint16_t index = 0; index < num_of_svs; index++) {
        memcpy(&cur_sv_status.repeated_block[index],log+14+(index*6),6);
      }
      // Copy Checksum
      memcpy(&cur_sv_status.checksum, log+14+(6*num_of_svs), 2);

      /*
      // Print populated structure
      printHex((char*) &cur_sv_status, sizeof(cur_sv_status));
      std::cout << std::endl;
      */

      if (rxm_svsi_callback_)
        rxm_svsi_callback_(cur_sv_status, read_timestamp_);
      break;

    case RXM_RAWX:
    {
      ublox_m8::RxmRawX rawx;
      size_t unrep_block = sizeof(rawx) - sizeof(rawx.repeated_block) -\
        sizeof(rawx.checksum);

      // Copy portion of RXM-RAWX before repeated block
      memcpy(&rawx, log, unrep_block);
      // Copy repeated block
      for (uint16_t index = 0; index < rawx.numMeas; index++) {
        memcpy(&rawx.repeated_block[index],
          log+unrep_block+(index*sizeof(RxmRawXRepeated)),
          sizeof(RxmRawXRepeated));
      }
      // Copy Checksum
      memcpy(&rawx.checksum,
        log+unrep_block+(sizeof(RxmRawXRepeated)*rawx.numMeas),
        sizeof(rawx.checksum));


      if (rxm_rawx_callback_)
        rxm_rawx_callback_(rawx, read_timestamp_);
      break;
    }

    case RXM_SFRBX:
    {
      ublox_m8::RxmSfrbX sfrbx;
      size_t unrep_block = sizeof(sfrbx) - sizeof(sfrbx.dwrds) -\
        sizeof(sfrbx.checksum);

      // Copy portion of RXM-RAWX before repeated block
      memcpy(&sfrbx, log, unrep_block);
      // Copy repeated block
      for (uint16_t index = 0; index < sfrbx.numWords; index++) {
        memcpy(&sfrbx.dwrds[index],
          log+unrep_block+(index*sizeof(uint32_t)), sizeof(uint32_t));
      }
      // Copy Checksum
      memcpy(&sfrbx.checksum,
        log+unrep_block+(sizeof(uint32_t)*sfrbx.numWords),
        sizeof(sfrbx.checksum));


      if (rxm_sfrbx_callback_)
        rxm_sfrbx_callback_(sfrbx, read_timestamp_);
      break;
    }

    default:
      break;
    } // end switch (logID)

    /*
     * For M8L CONFIDENTIAL INTERFACE
     * If message isn't found in standard M8 protocol, it's passed to the M8L
     * interface ParseLog.
     */
    if(parse_log_callback_)
        parse_log_callback_(log, logID);

  } catch (std::exception &e) {
    std::stringstream output;
    output << "Error parsing ublox log: " << e.what();
    log_error_(output.str());
  }
} // end ParseLog()

void UbloxM8::calculateCheckSum(uint8_t* in, size_t length, uint8_t* out) {
  try {
    uint8_t a = 0;
    uint8_t b = 0;

    for (uint8_t i = 0; i < length; i++) {

      a = a + in[i];
      b = b + a;

    }

    out[0] = (a & 0xFF);
    out[1] = (b & 0xFF);
  } catch (std::exception &e) {
    std::stringstream output;
    output << "Error calculating ublox checksum: " << e.what();
    log_error_(output.str());
  }
}
