/*!
 * \file ubloxM8/ublox_m8.h
 * \author  Chris Collins <cnc0003@tigermail.auburn.edu>
 * \author  Chris Gaddes <cag0023@tigermail.auburn.edu>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 IS4S / Auburn University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for uBlox M8 GNSS receivers using
 * the UBX protocol.
 */

#ifndef UBLOXM8_H
#define UBLOXM8_H

#include "ublox_m8_structures.h"
#include <serial/serial.h>
#include <fstream>

#include <boost/function.hpp>
#include <boost/thread.hpp>
//#include <boost/bind.hpp>

namespace ublox_m8 {

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarningMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;

//! GPS Data Callbacks
//! ACK
//! AID
//! CFG
typedef boost::function<void(CfgPrt&, double&)> CfgPrtCallback;
typedef boost::function<void(CfgNav5&, double&)> CfgNav5Callback;
//! INF
//! LOG
//! MON
//! NAV
typedef boost::function<void(NavPosLLH&, double&)> NavPosLLHCallback;
typedef boost::function<void(NavSol&, double&)> NavSolCallback;
typedef boost::function<void(NavStatus&, double&)> NavStatusCallback;
typedef boost::function<void(NavVelNED&, double&)> NavVelNEDCallback;
typedef boost::function<void(NavSVInfo&, double&)> NavSVInfoCallback;
typedef boost::function<void(NavTimeGPS&, double&)> NavTimeGPSCallback;
typedef boost::function<void(NavTimeGLO&, double&)> NavTimeGLOCallback;
typedef boost::function<void(NavTimeBDS&, double&)> NavTimeBDSCallback;
typedef boost::function<void(NavTimeUTC&, double&)> NavTimeUTCCallback;
typedef boost::function<void(NavDOP&, double&)> NavDOPCallback;
typedef boost::function<void(NavDGPS&, double&)> NavDGPSCallback;
typedef boost::function<void(NavClock&, double&)> NavClockCallback;
//! MGA
//! RXM
typedef boost::function<void(RxmSvsi&, double&)> RxmSvsiCallback;
//! TIM
//! UPD

//! ParseLog callback
typedef boost::function<void(uint8_t*, size_t)> ParseLogCallback;

class UbloxM8
{
public:
	UbloxM8();
	~UbloxM8();

	/*!
	 * Connects to the uBlox receiver given a serial port.
	 *
	 * @param port Defines which serial port to connect to in serial mode.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 */
	bool Connect(std::string port, int baudrate=115200);

    /*!
     * Disconnects from the serial port
     */
    void Disconnect();

    //! Indicates if a connection to the receiver has been established.
    bool IsConnected() {return is_connected_;}

    /*!
     * Pings the GPS to determine if it is properly connected
     * This method sends a ping to the GPS and waits for a response.
     * @param num_attempts The number of times to ping the device
     * before giving up
     * @return True if the GPS was found, false if it was not.
     */
    bool Ping(int num_attempts=5);

    void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out);

    //////////////////////////////////////////////////////
    // Input Methods
    //////////////////////////////////////////////////////
    bool SendMessage(uint8_t *msg_ptr, size_t length);

    void SetCfgPrtUsb(bool ubx_input, bool ubx_output, bool nmea_input, bool nmea_output);
    // Set a specifc message to be output periodically
    bool SetCfgMsgRate(uint8_t class_id, uint8_t msg_id, uint8_t rate);
        // (rate) is relative to the event a message is registered on. For example,
        // if the rate of a navigation message is set to 2, the message is sent
        // every second navigation solution.

    bool SetCfgNav5(uint8_t dynamic_model = 3, uint8_t fix_mode = 3);

    //////////////////////////////////////////////////////
    // Command Methods
    //////////////////////////////////////////////////////
    bool Reset(uint16_t nav_bbr_mask, uint8_t reset_mode);
    bool ResetToColdStart();
    bool ResetToWarmStart();
    bool ResetToHotStart();

    //////////////////////////////////////////////////////
    // Polling Methods
    //////////////////////////////////////////////////////
    bool PollMessage(uint8_t class_id, uint8_t msg_id);
    bool PollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid);

    bool PollCfgNav5();
    void PollCfgPrt(uint8_t port_identifier = 3);
    void PollCfgPrtCurrent();
    void PollCfgPrtUsb();
    
    bool PollNavSvInfo();
    bool PollNavStatus();

    bool PollRxmSvsi();

    //////////////////////////////////////////////////////
    // Set Callback Methods
    //////////////////////////////////////////////////////
    void set_time_handler(GetTimeCallback callback) {time_handler_ = callback;};

    void set_log_info_callback(InfoMsgCallback callback){log_info_=callback;};
    void set_log_debug_callback(DebugMsgCallback callback){log_debug_=callback;};
    void set_log_warning_callback(WarningMsgCallback callback){log_warning_=callback;};
    void set_log_error_callback(ErrorMsgCallback callback){log_error_=callback;};

    void set_cfg_nav5_callback(CfgNav5Callback callback){cfg_nav5_callback_=callback;};
    void set_cfg_prt_callback(CfgPrtCallback callback){cfg_prt_callback_=callback;};

    void set_nav_status_callback(NavStatusCallback callback){nav_status_callback_=callback;};
    void set_nav_sol_callback(NavSolCallback callback){nav_sol_callback_=callback;};
    void set_nav_pos_llh_callback(NavPosLLHCallback callback){nav_pos_llh_callback_=callback;};
    void set_nav_sv_info_callback(NavSVInfoCallback callback){nav_sv_info_callback_=callback;};
    void set_nav_time_gps_callback(NavTimeGPSCallback callback){nav_time_gps_callback_=callback;};
    void set_nav_time_glo_callback(NavTimeGLOCallback callback){nav_time_glo_callback_=callback;};
    void set_nav_time_bds_callback(NavTimeBDSCallback callback){nav_time_bds_callback_=callback;};
    void set_nav_time_utc_callback(NavTimeUTCCallback callback){nav_time_utc_callback_=callback;};
    void set_nav_dop_callback(NavDOPCallback callback){nav_dop_callback_=callback;};
    void set_nav_dgps_callback(NavDGPSCallback callback){nav_dgps_callback_=callback;};
    void set_nav_clock_callback(NavClockCallback callback){nav_clock_callback_=callback;};
    void set_nav_vel_ned_callback(NavVelNEDCallback callback){nav_vel_ned_callback_=callback;};

    void set_rxm_svsi_callback(RxmSvsiCallback callback){rxm_svsi_callback_=callback;};

    void set_parse_log_callback(ParseLogCallback callback){parse_log_callback_=callback;};    
    
protected:

    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.

	// size_t header_length_;	//!< length of the current header being read
	//bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	//double parse_timestamp_;		//!< time stamp when last parse began
	// unsigned short msgID; //TODO: check if this needs to be global or can be scoped to BufferIncomingData();
    
private:

    /*!
     * Starts a thread to continuously read from the serial port.
     *
     * Starts a thread that runs 'ReadSerialPort' which constatly reads
     * from the serial port.  When valid data is received, parse and then
     *  the data callback functions are called.
     */
    void StartReading();

    /*!
     * Starts the thread that reads from the serial port
     */
    void StopReading();

    /*!
     * Method run in a seperate thread that continuously reads from the
     * serial port.  When a complete packet is received, the parse
     * method is called to process the data
     */
    void ReadSerialPort();

    //bool WaitForAck(int timeout); //!< waits for an ack from receiver (timeout in seconds)

    void BufferIncomingData(uint8_t* msg, size_t length);

    //! Function to parse logs into a usable structure
    void ParseLog(uint8_t* log, size_t logID);

        //////////////////////////////////////////////////////
    // Callbacks
    //////////////////////////////////////////////////////
    //HandleAcknowledgementCallback handle_acknowledgement_;
    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping

    //! Logging Callbacks
    DebugMsgCallback log_debug_;
    InfoMsgCallback log_info_;
    WarningMsgCallback log_warning_;
    ErrorMsgCallback log_error_;

    //! UBX Data Callbacks
    CfgPrtCallback cfg_prt_callback_;
    CfgNav5Callback cfg_nav5_callback_;

    NavPosLLHCallback nav_pos_llh_callback_;
    NavSolCallback nav_sol_callback_;
    NavStatusCallback nav_status_callback_;
    NavVelNEDCallback nav_vel_ned_callback_;
    NavSVInfoCallback nav_sv_info_callback_;
    NavTimeGPSCallback nav_time_gps_callback_;
    NavTimeGLOCallback nav_time_glo_callback_;
    NavTimeBDSCallback nav_time_bds_callback_;
    NavTimeUTCCallback nav_time_utc_callback_;
    NavDOPCallback nav_dop_callback_;
    NavDGPSCallback nav_dgps_callback_;
    NavClockCallback nav_clock_callback_;

    RxmSvsiCallback rxm_svsi_callback_;
    
    ParseLogCallback parse_log_callback_;


    //////////////////////////////////////////////////////
    // Incoming data buffers
    //////////////////////////////////////////////////////
    unsigned char data_buffer_[MAX_NOUT_SIZE];  //!< data currently being buffered to read
    // unsigned char* data_read_;       //!< used only in BufferIncomingData - declared here for speed

    bool is_connected_; //!< indicates if a connection to the receiver has been established
    size_t bytes_remaining_;    //!< bytes remaining to be read in the current message
    size_t buffer_index_;       //!< index into data_buffer_
};
}

#endif
