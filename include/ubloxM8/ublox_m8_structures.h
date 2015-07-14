// uBlox M8 UBX Data Structures
#ifndef UBLOXM8STRUCTURES_H
#define UBLOXM8STRUCTURES_H

#include "stdint.h"

namespace ublox_m8 {

#define MAX_NOUT_SIZE      (5000)   // Maximum size of a uBlox log buffer (ALMANAC logs are big!)
#define MAXCHAN		50  // Maximum number of signal channels
#define MAX_GPS_SATS 32
#define MAX_SAT 100        // Maximum number of SVs for all GNSS constellations
#define MAX_NUM_SOURCES 5
#define MAX_NUM_OSCILATORS 5

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

//! Header prepended to ubx binary messages
#define HDR_CHKSM_LENGTH 8 //(includes "sync1 sync2 classid msgid length checksum")
#define UBX_SYNC_BYTE_1 0xB5
#define UBX_SYNC_BYTE_2 0x62
    
//! UBX Protocol Class/Message ID's
#define MSG_CLASS_ACK 0X05
    #define MSG_ID_ACK_ACK 0x01
    #define MSG_ID_ACK_NAK 0x00
#define MSG_CLASS_CFG 0x06
    #define MSG_ID_CFG_ANT 0X13
    #define MSG_ID_CFG_CFG 0x09
    #define MSG_ID_CFG_DAT 0x06
    #define MSG_ID_CFG_DOSC 0x61
    #define MSG_ID_CFG_ESRC 0x60
    #define MSG_ID_CFG_GNSS 0x3E
    #define MSG_ID_CFG_INF 0x02
    #define MSG_ID_CFG_ITFM 0x39
    #define MSG_ID_CFG_LOGFILTER 0x47
    #define MSG_ID_CFG_MSG 0x01
    #define MSG_ID_CFG_NAV5 0x24
    #define MSG_ID_CFG_NAVX5 0x23
    #define MSG_ID_CFG_NMEA 0x17
    #define MSG_ID_CFG_ODO 0x1E
    #define MSG_ID_CFG_PM2 0x3B
    #define MSG_ID_CFG_PRT 0x00
    #define MSG_ID_CFG_PWR 0x57
    #define MSG_ID_CFG_RATE 0x08
    #define MSG_ID_CFG_RINV 0x34
    #define MSG_ID_CFG_RST 0x04
    #define MSG_ID_CFG_RXM 0x11
    #define MSG_ID_CFG_SBAS 0x16
    #define MSG_ID_CFG_SMGR 0x62
    #define MSG_ID_CFG_TMODE2 0x3D
    #define MSG_ID_CFG_TP5 0x31
    #define MSG_ID_CFG_TXSLOT 0x53
    #define MSG_ID_CFG_USB 0x1B
#define MSG_CLASS_INF 0x04
    #define MSG_ID_INF_DEBUG 0x04
    #define MSG_ID_INF_ERROR 0x00
    #define MSG_ID_INF_NOTICE 0x02
    #define MSG_ID_INF_TEST 0x03
    #define MSG_ID_INF_WARNING 0x01
#define MSG_CLASS_LOG 0x21
    #define MSG_ID_LOG_CREATE 0x07
    #define MSG_ID_LOG_ERASE 0x03
    #define MSG_ID_LOG_FINDTIME 0x0E
    #define MSG_ID_LOG_INFO 0x08
    #define MSG_ID_LOG_RETRIEVEPOSEXTRA 0x0F
    #define MSG_ID_LOG_RETRIEVEPOS 0x0B
    #define MSG_ID_LOG_RETRIEVESTRING 0x0d 
    #define MSG_ID_LOG_RETRIEVE 0x09
    #define MSG_ID_LOG_STRING 0x04
#define MSG_CLASS_MGA 0x13 
    #define MSG_ID_MGA_ACK 0x60
    #define MSG_ID_MGA_ANO 0x20
    #define MSG_ID_MGA_DBD 0x80
    #define MSG_ID_MGA_FLASH 0x21
    #define MSG_ID_MGA_GLO 0x06
    #define MSG_ID_MGA_GPS 0x00
    #define MSG_ID_MGA_INI 0x40
    #define MSG_ID_MGA_QZSS 0x05
#define MSG_CLASS_MON 0x0A
    #define MSG_ID_MON_GNSS 0x28
    #define MSG_ID_MON_HW2 0x0B
    #define MSG_ID_MON_HW 0x09
    #define MSG_ID_MON_IO 0x02
    #define MSG_ID_MON_MSGPP 0x06
    #define MSG_ID_MON_PATCH 0x27
    #define MSG_ID_MON_RXBUF 0x07
    #define MSG_ID_MON_RXR 0x21
    #define MSG_ID_MON_SMGR 0x2E
    #define MSG_ID_MON_TXBUF 0X08
    #define MSG_ID_MON_VER 0x04
#define MSG_CLASS_NAV 0x01
    #define MSG_ID_NAV_AOPSTATUS 0x60
    #define MSG_ID_NAV_CLOCK 0x22
    #define MSG_ID_NAV_DGPS 0x31
    #define MSG_ID_NAV_DOP 0x04
    #define MSG_ID_NAV_ODO 0x09
    #define MSG_ID_NAV_ORB 0x34
    #define MSG_ID_NAV_POSECEF 0x01
    #define MSG_ID_NAV_POSLLH 0x02
    #define MSG_ID_NAV_PVT 0x07
    #define MSG_ID_NAV_RESETODO 0x10
    #define MSG_ID_NAV_SAT 0x35
    #define MSG_ID_NAV_SBAS 0x32
    #define MSG_ID_NAV_SOL 0x06
    #define MSG_ID_NAV_STATUS 0x03
    #define MSG_ID_NAV_SVINFO 0x30
    #define MSG_ID_NAV_TIMEBDS 0x24
    #define MSG_ID_NAV_TIMEGLO 0x23
    #define MSG_ID_NAV_TIMEGPS 0x20
    #define MSG_ID_NAV_TIMEUTC 0x21
    #define MSG_ID_NAV_VELECEF 0x11
    #define MSG_ID_NAV_VELNED 0x12
#define MSG_CLASS_RXM 0x02
    #define MSG_ID_RXM_PMREQ 0x41
    #define MSG_ID_RXM_RAWX 0x15
    #define MSG_ID_RXM_SFRBX 0x13
    #define MSG_ID_RXM_SVSI 0x20
#define MSG_CLASS_TIM 0x0D
    #define MSG_ID_TIM_DOSC 0x11
    #define MSG_ID_TIM_FCHG 0x16
    #define MSG_ID_TIM_HOC 0x17
    #define MSG_ID_TIM_SMEAS 0x13
    #define MSG_ID_TIM_SVIN 0x04
    #define MSG_ID_TIM_TM2 0x03
    #define MSG_ID_TIM_TOS 0x12
    #define MSG_ID_TIM_TP 0x01
    #define MSG_ID_TIM_VCOCAL 0x15
    #define MSG_ID_TIM_VRFY 0x06
#define MSG_CLASS_UPD 0x09
    #define MSG_ID_UPD_SOS 0x14

PACK(
    struct UbloxHeader {
        uint8_t sync1;   //!< start of packet first byte (0xB5)
        uint8_t sync2;   //!< start of packet second byte (0x62)
        uint8_t message_class; //!< Class that defines basic subset of message (NAV, RXM, etc.)
        uint8_t message_id;		//!< Message ID
        uint16_t payload_length; //!< length of the payload data, excluding header and checksum
});


///////////////////////////////////////////////////////////
// UBX-CFG
///////////////////////////////////////////////////////////

/*!
* CFG-ANT Message Structure
* Antenna control settings
* ID: 0x06  0x13 Payload Length=4 bytes
*/
// TODO

/*!
 * CFG-CFG Message Structure
 * This message clears, saves, or loads novalitle memory.
 * Set masks to 0x061F to clear, save, or load all values.
 * ID: 0x06  0x09 Payload Length=12 bytes
 */
PACK(
    struct CfgCfg {
        UbloxHeader header;     //!< Ublox header
        uint32_t clear_mask;  //!< clear mask
        uint32_t save_mask;     //!< save mask
        uint32_t load_mask;           //!< load mask
        uint8_t checksum[2];      //!< Checksum
});


/*!
 * CFG-DAT Message Structure
 * Sets user-defined datum
 * ID: 0x06  0x06 Payload Length=44 bytes
 */
//TODO


/*!
 * CFG-DOSC Message Structure
 * This message allows the characteristics of the internal or 
 * external oscillator to be described to the receiver. The 
 * gainVco and gainUncertainty parameters are normally set using 
 * the calibration process initiated using UBX-TIM-VCOCAL. The 
 * behavior of the system can be badly affected by setting the wrong values, 
 * so customers are advised to only change these parameters with care.
 * ID: 0x06  0x061 Payload Length=4+32*numOsc bytes
 */
PACK(
    struct CfgDoscRepeated{
        uint8_t oscId;          // Id of oscillator. 0 - internal oscillator. 1 - external oscillator
        uint8_t reserved2;      // Reserved
        uint16_t flags;         // flags (see graphic in spec sheet)
        uint32_t freq;          // Nominal frequency of source
        int32_t phaseOffset;    // Intended phase offset of the oscillator relative to the leading edge of the time pulse
        uint32_t withTemp;      // Oscillator stability limit over operating temperature range (must be > 0)
        uint32_t withAge;       // Oscillator stability with age (must be > 0) 
        uint16_t timeToTemp;    // The minimum time that it could take for a temperature variation to move the oscillator frequency by 'withTemp' (must be > 0)
        uint8_t reserved3;      // Reserved
        int32_t gainVco;    // Oscillator control gain/slope; change of frequency per unit change in raw control change
        uint8_t gainUncertainty;    //Relative uncertainty (1 standard deviation) of oscillator control gain/slope
        uint16_t reserved4;    // Reserved       
});
PACK(
    struct CfgDosc{
        UbloxHeader header;
        uint8_t version;        // Message version (0 for this version)
        uint8_t numOsc;         // Number of oscillators to configure (affects length of this message)
        uint8_t reserved1;      // Reserved
        CfgDoscRepeated repeated_block[MAX_NUM_OSCILATORS]; // Repeated block of data for each oscillator
        uint8_t checksum[2];    //**$**Should this be included?
});


/*!
 * CFG-ESRC Message Structure
 * External time or frequency source configuration. The 
 * stability of time and frequency sources is described 
 * using different fields, see sourceType field documentation.
 * ID: 0x06  0x060 Payload Length=4+36*numSources bytes
 */
PACK(
    struct CfgEsrcRepeated{
        uint8_t extInt;        // EXTINT index of this source (0 for EXTINT0 and 1 for EXTINT1)
        uint8_t sourceType;         // Source type: 0: none
                                    // 1: frequency source; use withTemp, withAge, timeToTemp and maxDevLifeTime to describe the stability of the source
                                    // 2: time source; use offset, offsetUncertainty and jitter fields to describe the stability of the source
                                    // 3: feedback from external oscillator; stability data is taken from the external oscillator's configuration
        uint16_t flags;        // Flags (see graphic in spec sheet)
        uint32_t freq;        // Nominal frequency of source
        uint8_t reserved2;      // Reserved
        uint32_t withTemp;        // Oscillator stability limit over operating temperature range (must be > 0). Only used if sourceType is 1.
        uint32_t withAge;        // Oscillator stability with age (must be > 0). Only used if sourceType is 1.
        uint16_t timeToTemp;        // Oscillator stability with age (must be > 0). Only used if sourceType is 1.
        uint16_t maxDevLifeTime;        // Maximum frequency deviation during lifetime. (must be > 0). Only used if sourceType is 1.
        int32_t offset;        // Phase offset of signal. Only used if sourceType is 2.
        uint32_t offsetUncertainty;        // Uncertainty of phase offset (one standard deviation). Only used if sourceType is 2.
        uint32_t jitter;        // Phase jitter (must be > 0). Only used if sourceType is 2.
});
PACK(
    struct CfgEsrc{
        UbloxHeader header;
        uint8_t version;        // Message version (0 for this version)
        uint8_t numSources;        // Number of sources (affects length of this message)
        uint8_t reserved1;        // Reserved
        CfgEsrcRepeated repeated_block[MAX_NUM_SOURCES];
        uint8_t checksum[2];    //**$**Should this be included?
});



/*!
 * CFG-GNSS Message Structure
 * Gets or sets the GNSS system channel sharing configuration. 
 * The receiver will send an UBX-ACK-ACK message if the 
 * configuration is valid, an UBX-ACK-NAK if any configuration
 * parameter is invalid.
 * ID: 0x06  0x3E Payload Length=4+8*numConfigBlocks bytes
 */
PACK(
    struct CfgGnssRepeated{
        uint8_t gnssId;        // GNSS identifier (see Satellite Numbering)
        uint8_t resTrkCh;        // Number of reserved (minimum) tracking channels for this GNSS system
        uint8_t maxTrkCh;        // Maximum number of tracking channels used for this GNSS system (>=resTrkChn)
        uint8_t reserved1;        // Reserved
        uint32_t flags;        // Bitfield of flags (see graphic in datasheet)
});
PACK(
    struct CfgGnss{
        UbloxHeader header;
        uint8_t msgVer;        // Message version (=0 for this version)
        uint8_t numTrkChHw;        // Number of tracking channels available in hardware (read only)
        uint8_t numTrkChUse;        // Number of tracking channels to use (<=numTrkChHw)
        uint8_t numConfigBlocks;        // Number of configuration blocks following
        CfgGnssRepeated repeated_block;
        uint8_t checksum[2];
});

/*!
 * CFG-INF Message Structure
 * Information message configuration
 * ID: 0x06  0x02 Payload Length=0+10*N bytes
 */
// TODO


/*!
 * CFG-ITFM Message Structure
 * Jamming/Interference Monitor configuration
 * ID: 0x06  0x39 Payload Length=8 bytes
 */
// TODO


/*!
 * CFG-LOGFILTER Message Structure
 * Data Logger Configuration
 * ID: 0x06  0x47 Payload Length=12 bytes
 */
PACK(
    struct CfgLogfilter{
        UbloxHeader header;
        uint8_t version;        // The version of this message. Set to 1
        uint8_t flags;        // Flags (see graphic in spec sheet)
        uint16_t minInterval;        // Minimum time interval between logged positions (0 = not set). This is only applied in combination with the speed and/or position thresholds
        uint16_t timeThreshold;        // If the time difference is greater than the threshold then the position is logged (0 = not set).
        uint16_t speedThreshold;        // If the current speed is greater than the threshold then the position is logged (0 = not set). minInterval also applies
        uint32_t positionThreshold;        //If the 3D position difference is greater than the threshold then the position is logged (0 = not set). minInterval also applies 
        uint8_t checksum[2];
});


/*!
 * CFG-MSG Message Structure
 * This message requests a specifiable message at a given rate.
 * ID: 0x06  0x01 Payload Length=3 bytes
 */
PACK(
    struct CfgMsg {
        UbloxHeader header;     //!< Ublox header
        uint8_t message_class;  //!< class of message to request
        uint8_t message_id;     //!< id of message to request
        uint8_t rate;           //!< rate message will be sent on current port
        uint8_t checksum[2];
});

PACK(
    struct CfgMsgs {
        UbloxHeader header;     //!< Ublox header
        uint8_t message_class;  //!< class of message to request
        uint8_t message_id;     //!< id of message to request
        uint8_t rate[6];        //!< rate message will be sent on up to 6 ports
        uint8_t checksum[2];
});



/*!
* CFG-NAV5 Message Structure
* This message configures Navigation algorithm
* parameters.
* ID: 0x06  0x24 Payload Length=36 bytes
*/
PACK(
    struct CfgNav5 {
        UbloxHeader header;     //!< Ublox header
        uint16_t mask; //!< parameters bitmask (only masked params applied)
        uint8_t dynamic_model; //!< dynamic platform
        uint8_t fix_mode; //!< positioning fix mode
        int32_t fixed_altitude; //!< (scale .01) (m)
        uint32_t fixed_altitude_variance; //!< (scale .0001) (m^2)
        int8_t min_elevation; //!< (deg)
        uint8_t dead_reckoning_limit; //!< max time to perform DR w/out GPS (sec)
        uint16_t pdop; //!< (scale .1)
        uint16_t tdop; //!< (scale .1)
        uint16_t pos_accuracy_mask; //!< (m)
        uint16_t time_accuracy_mask; //!< (m)
        uint8_t static_hold_threshold; //!<
        uint8_t dgps_timeout; //!<
        uint32_t reserved2; //!< reserved (always set to zero)
        uint32_t reserved3; //!< reserved (always set to zero)
        uint32_t reserved4; //!< reserved (always set to zero)
        uint8_t checksum[2];
});


/*!
* CFG-NAVX5 Message Structure
* Navigation Engine Expert Settings
* ID: 0x06  0x23 Payload Length=40 bytes
*/
PACK(
    struct CfgNavX5 {
        UbloxHeader header;     //!< Ublox header
        uint16_t version; //!< Message version
        uint16_t mask1;
        uint32_t mask2;
        uint8_t reserved1[2];
        uint8_t min_svs;
        uint8_t max_svs;
        uint8_t min_cno;
        uint8_t reserved2;
        uint8_t iniFix3d;
        uint8_t reserved3[2];
        uint8_t ack_aiding;
        uint16_t wkn_rollover;
        uint8_t reserved4[6];
        uint8_t use_ppp;
        uint8_t aop_cfg;
        uint8_t reserved5[2];
        uint16_t aopOrbMaxErr;
        uint8_t reserved6[4];
        uint8_t reserved7[3];
        uint8_t useAdr;
        uint8_t checksum[2];
});


/*!
* CFG-NMEA Message Structure
* NMEA protocol configuration
* DEPRACTED, Use CfgNmeaV1
* ID: 0x06  0x17 Payload Length=4 bytes
*/
PACK(
    struct CfgNmeaV0{ // DEPRACTED, for backwards compatability only
        UbloxHeader header;
        uint8_t filter;        // filter flags (see graphic in spec sheet)
        uint8_t nmeaVersion;        // 0x23: NMEA version 2.3. 0x21: NMEA version 2.1
        uint8_t numSV;        //Maximum Number of SVs to report per TalkerId. 0: unlimited 8: 8 SVs 12: 12 SVs 16: 16 SVs
        uint8_t flags;        // flags (see graphic in spec sheet)
        uint32_t gnssToFilter;        // Filters out satellites based on their GNSS. If a bitfield is enabled, the corresponding satellites will be not output.
        uint8_t svNumbering;        // Configures the display of satellites that do not have an NMEA-defined value. Note: this does not apply to satellites with an unknown ID. 0: Strict - Satellites are not output 1: Extended - Use proprietary numbering (see Satellite numbering in spec sheet)
        uint8_t mainTalkerId;        // By default the main Talker ID (i.e. the Talker ID used for all messages other than GSV) is determined by the GNSS assignment of the receiver's channels (see UBX-CFG-GNSS). This field enables the main Talker ID to be overridden. 0: Main Talker ID is not overridden 1: Set main Talker ID to 'GP' 2: Set main Talker ID to 'GL' 3: Set main Talker ID to 'GN' 4: Set main Talker ID to 'GA' 5: Set main Talker ID to 'GB'
        uint8_t gsvTalkerId;        // By default the Talker ID for GSV messages is GNSS specific (as defined by NMEA). This field enables the GSV Talker ID to be overridden. 0: Use GNSS specific Talker ID (as defined by NMEA) 1: Use the main Talker ID
        uint8_t version;        // Message version (set to 0 for this version)
        uint8_t checksum[2];
});
/*!
* CFG-NMEA Message Structure
* NEWEST MESSAGE VERSION NMEA protocol configuration
* ID: 0x06  0x17 Payload Length=12 bytes
*/
PACK(
    struct CfgNmeaV1{
        UbloxHeader header;
        uint8_t filter;         // filter flags (see graphic in spec sheet)
        uint8_t nmeaVersion;    // 0x41: NMEA version 4.1. 0x40: NMEA version 4.0. 0x23: NMEA version 2.3. 0x21: NMEA version 2.1
        uint8_t numSV;          // Maximum Number of SVs to report per TalkerId. 0: unlimited. 8: 8 SVs. 12: 12 SVs. 16: 16 SVs
        uint8_t flags;          // flags (see graphic in spec sheet)
        uint32_t gnssToFilter;  // Filters out satellites based on their GNSS. If a bitfield is enabled, the corresponding satellites will be not output.
        uint8_t svNumbering;    // Configures the display of satellites that do not have an NMEA-defined value. Note: this does not apply to satellites with an unknown ID. 0: Strict - Satellites are not output 1: Extended - Use proprietary numbering (see Satellite numbering in spec sheet)
        uint8_t mainTalkerId;   // By default the main Talker ID (i.e. the Talker ID used for all messages other than GSV) is determined by the GNSS assignment of the receiver's channels (see UBX-CFG-GNSS). This field enables the main Talker ID to be overridden. 0: Main Talker ID is not overridden 1: Set main Talker ID to 'GP' 2: Set main Talker ID to 'GL' 3: Set main Talker ID to 'GN' 4: Set main Talker ID to 'GA' 5: Set main Talker ID to 'GB'
        uint8_t gsvTalkerId;    // By default the Talker ID for GSV messages is GNSS specific (as defined by NMEA). This field enables the GSV Talker ID to be overridden. 0: Use GNSS specific Talker ID (as defined by NMEA) 1: Use the main Talker ID
        uint8_t version;        // Message version (set to 0 for this version)
        uint8_t bdsTalkerId[2]; // Sets the two characters that should be used for the BeiDou Talker ID If these are set to zero, the default BeiDou TalkerId will be used
        uint8_t reserved1;      // Reserved
        uint8_t checksum[2];
});


/*!
* CFG-ODO Message Structure
* Odometer, Low-speed COG Engine Settings
* ID: 0x06  0x1E Payload Length=20 bytes
*/
PACK(
    struct CfgOdo{
        UbloxHeader header;
        uint8_t version;        // Message version (0 for this version)
        uint8_t reserved1;        // Reserved
        uint8_t flags;        // Odometer/Low-speed COG filter flags (see spec sheet)
        uint8_t odoCfg;        // Odometer filter settings (see graphic in spec sheet)
        uint8_t reserved2;        // Reserved
        uint8_t cogMaxSpeed;        // Speed below which course-over-ground (COG) is computed with the low-speed COG filter
        uint8_t cogMaxPosAcc;        // Maximum acceptable position accuracy for computing COG with the low-speed COG filter
        uint8_t reserved3;        // Reserved
        uint8_t velLpGain;        // Velocity low-pass filter level, range 0..255
        uint8_t cogLpGain;        // COG low-pass filter level (at speed < 8 m/s), range 0..255
        uint8_t reserved4;        // Reserved
        uint8_t checksum[2];
});


/*!
* CFG-PM2 Message Structure
* Extended Power Management configuration
* ID: 0x06  0x3B Payload Length=44 bytes
*/
// TODO


/*!
 * CFG-PRT Message Structure
 * This message configures a USART or USB port.
 * Use to specify input/output protocols to use
 * ID: 0x06  0x00 Payload Length=20 bytes
 */
PACK(
    struct CfgPrt {
        UbloxHeader header;     //!< Ublox header
        uint8_t port_id; //!< port identifier (0 or 1 for USART or 3 for USB)
        uint8_t reserved; //!< reserved
        uint16_t tx_ready; //!< transmit ready status
        uint32_t reserved2; //!< reserved
        uint32_t reserved3; //!< reserved
        uint16_t input_mask; //!< input protocol mask
        uint16_t output_mask; //!< output protocol mask
        uint16_t reserved4; //!< reserved
        uint16_t reserved5; //!< reserved
        uint8_t checksum[2];
});

/*!
* CFG-PRT Message Structure
* Polls the configuration of the used I/O Port
* ID: 0x06  0x00 Payload Length=0 bytes
*/
// TODO

/*!
* CFG-PRT Message Structure
* Polls the configuration for one I/O Port
* ID: 0x06  0x00 Payload Length=1 bytes
*/
// TODO


/*!
* CFG-PRT Message Structure
* Port Configuration for UART
* ID: 0x06  0x00 Payload Length=20 bytes
*/
// TODO


/*!
* CFG-PRT Message Structure
* Port Configuration for USB Port
* ID: 0x06  0x00 Payload Length=20 bytes
*/
// TODO


/*!
* CFG-PRT Message Structure
* Port Configuration for SPI Port
* ID: 0x06  0x00 Payload Length=20 bytes
*/
// TODO


/*!
* CFG-PRT Message Structure
* Port Configuration for DDC Port
* ID: 0x06  0x00 Payload Length=20 bytes
*/
// TODO


/*!
* CFG-PWR Message Structure
* Put receiver in a defined power state
* ID: 0x06  0x57 Payload Length=8 bytes
*/
PACK(
    struct CfgPwr{
        UbloxHeader header;
        uint8_t version;        // Message version (1 for this version)
        uint8_t reserved1[3];        // Reserved
        uint32_t state;        // Enter system state. 0x52554E20: GNSS running. 0x53544F50: GNSS stopped. 0x42434B50: Software Backup
        uint8_t checksum[2];    
});


/*!
* CFG-RATE Message Structure
* Navigation/Measurement Rate Settings
* NOT SUPPORTED ON FTS PRODUCT VARIANT
* ID: 0x06  0x08 Payload Length=8 bytes
*/
PACK(
    struct CfgRate
    {
        UbloxHeader header;     
        uint16_t measRate;      //< GPS measurement rate (ms)
        uint16_t navRate;       //< Navigation rate (ALWAYS = 1) (cycles)
        uint16_t timeRef;       //< Alignment to reference time (0=UTC Time, 1=GPS Time)
        uint8_t checksum[2];  
});


/*!
* CFG-RINV Message Structure
* Contents of Remote Inventory
* ID: 0x06  0x34 Payload Length=1+1*N bytes
*/
PACK(
    struct CfgRinv
    {
        UbloxHeader header;
        uint8_t flags;
        uint8_t data[30];
        uint8_t checksum[2];
});


/*!
 * CFG-RST Message Structure
 * This message allows a receiver to be reset.
 * ID: 0x06  0x04 Payload Length=4 bytes
 */
 PACK(
    struct CfgRst {
        UbloxHeader header;     //!< Ublox header
        uint16_t nav_bbr_mask;  //!< Nav data to clear: 0x0000 = hot start, 0x0001 = warm start, 0xFFFF=cold start
        uint8_t  reset_mode;     //!< Reset mode
        uint8_t  reserved;       //!< reserved
        uint8_t checksum[2];
});

/*!
 * CFG-RXM Message Structure
 * RXM configuration
 * ID: 0x06  0x11 Payload Length=2 bytes
 */


/*!
 * CFG-SBAS Message Structure
 * SBAS Configuration
 * ID: 0x06  0x16 Payload Length=8 bytes
 */


/*!
 * CFG-SMGR Message Structure
 * Synchronization manager configuration
 * ID: 0x06  0x62 Payload Length=20 bytes
 */
PACK(
    struct CfgSmgr{
        UbloxHeader header;
        uint8_t version;        // Message version (0 for this version)
        uint8_t minGNSSFix;        // Minimum number of GNSS fixes before we commit to use it as a source
        uint16_t maxFreqChangeRate;        // Maximum frequency change rate during disciplining. Must not exceed 30ppb/s
        uint16_t maxPhaseCorrRate;        // Maximum phase correction rate in coherent time pulse mode. For maximum phase correction rate in corrective time pulse mode see maxSlewRate. Note that in coherent time pulse mode phase correction is achieved by intentional frequency offset. Allowing for a high phase correction rate can result in large intentional frequency offset. Must not exceed 100ns/s
        uint8_t reserved1[2];        // Reserved
        uint16_t freqTolerance;        // Limit of possible deviation from nominal before TIM-TOS indicates that frequency is out of tolerance
        uint16_t timeTolerance;        // Limit of possible deviation from nominal before TIM-TOS indicates that time pulse is out of tolerance
        uint16_t messageCfg;        // Sync manager message configuration (see graphic below)
        uint16_t maxSlewRate;        // Maximum slew rate, the maximum time correction that shall be applied between locked pulses in corrective time pulse mode. To have no limit on the slew rate, set the flag disableMaxSlewRate to 1 For maximum phase correction rate in coherent time pulse mode see maxPhaseCorrRate.
        uint32_t flags;        // Flags (see graphic in spec sheet)
        uint8_t checksum[2];    
});


/*!
 * CFG-TMODE2 Message Structure
 * Time Mode Settings 2
 * Only available with Timing or FTS product variants
 * ID: 0x06  0x3D Payload Length=28 bytes
 */


/*!
 * CFG-TP5 Polling Message Structure
 * Poll Time Pulse Parameters
 * ID: 0x06  0x31 Payload Length=1 bytes
 */
PACK(
    struct CfgTp5Poll{
        UbloxHeader header;
        uint8_t tpIdx;        // Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
        uint32_t checksum[2];    
});


/*!
 * CFG-TP5 Message Structure
 * Time Pulse Parameters
 * ID: 0x06  0x31 Payload Length=28 bytes
 */
PACK(
    struct CfgTp5{
        UbloxHeader header;
        uint8_t tpIdx;        // Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
        uint8_t version;        // Version, 0 for this message
        uint8_t reserved1[2];        // Reserved
        int16_t antCableDelay;        // Antenna cable delay
        int16_t rfGroupDelay;        // RF group delay
        uint32_t freqPeriod;        // Frequency or period time, depending on setting of bit 'isFreq'
        uint32_t freqPeriodLock;        // Frequency or period time when locked to GPS time, only used if 'lockedOtherSet' is set
        uint32_t pulseLenRatio;        // Pulse length or duty cycle, depending on 'isLength'
        uint32_t pulseLenRatioLock;        // Pulse length or duty cycle when locked to GPS time, only used if 'lockedOtherSet' is set
        int32_t userConfigDelay;        // User configurable time pulse delay
        uint32_t flags;        // Configuration flags (see graphic in spec sheet)
        uint32_t checksum[2];    
});


/*!
 * CFG-TXSLOT Polling Message Structure
 * TX buffer time slots configuration
 * only available with FTS product variant
 * ID: 0x06  0x53 Payload Length=16 bytes
 */
PACK(
    struct CfgTxslot{
        UbloxHeader header;
        uint8_t version;        // Message version (0 for this version)
        uint8_t enable;        // Bitfield of ports for which the slots are enabled. (see graphic below)
        uint8_t refTp;        // Reference timepulse source. 0 - Timepulse. 1 - Timepulse 2
        uint8_t reserved1;        // Reserved
        uint32_t end[3];        // End of timeslot in milliseconds after time pulse
        uint8_t checksum[2];    
});


/*!
 * CFG-USB Message Structure
 * USB Configuration
 * ID: 0x06  0x1B Payload Length=108 bytes
 */


///////////////////////////////////////////////////////////
// UBX-INF
///////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////
// UBX-LOG
///////////////////////////////////////////////////////////

/*!
 * LOG-CREATE Message Structure
 * Create Log File
 * ID: 0x21  0x07 Payload Length=8 bytes
 */
PACK(
    struct LogCreate{
        UbloxHeader header;
        uint8_t version;        // The version of this message. Set to 0
        uint8_t logCfg;        // Config flags (see graphic in spec sheet)
        uint8_t reserved1;        // Reserved
        uint8_t logSize;        // Indicates the size of the log:
                                // 0 (maximum safe size): Ensures that logging will not be interupted and enough space will be left avaiable for all other uses of the filestore
                                //1 (minimum size):
                                //2 (user defined): See 'userDefinedSize' below
        uint32_t userDefinedSize;        // Sets the maximum amount of space in the filestore that can be used by the logging task. This field is only applicable if logSize is set to user defined.
        uint8_t checksum[2];    
});


/*!
 * LOG-ERASE Message Structure
 * Erase Logged Data
 * Message Type: COMMAND
 * ID: 0x21  0x03 Payload Length=0 bytes
 */
PACK(
    struct LogErase{
        UbloxHeader header;
        uint8_t checksum[2];    
});


/*!
 * LOG-FINDTIME Request Message Structure
 * Find index of the first log entry <= given time
 * Message Type: INPUT
 * ID: 0x21  0x0E Payload Length=12 bytes
 */
PACK(
    struct LogFindTimeRequest{
        UbloxHeader header;
        uint8_t version;        // Message version (=0 for this version)
        uint8_t type;        // Message type, 0 for request
        uint8_t reserved1[2];        // Reserved
        uint8_t year;        // Year (1-65635) of UTC time
        uint8_t month;        // Month (1-12) of UTC time
        uint8_t day;        // Day (1-31) of UTC time
        uint8_t hour;        // Hour (0-23) of UTC time
        uint8_t minute;        // Minute (0-59) of UTC time
        uint8_t second;        // Second (0-60) of UTC time
        uint8_t reserved2;        // Reserved
        uint8_t checksum[2];    
});

/*!
 * LOG-FINDTIME Response Message Structure
 * Find index of the first log entry <= given time
 * Message Type: OUTPUT
 * ID: 0x21  0x0E Payload Length=8 bytes
 */
PACK(
    struct LogFindTimeReqResponse{
        UbloxHeader header;
        uint8_t version;        // Message version (=1 for this version)
        uint8_t type;        // Message type, t for response
        uint8_t reserved1[2];        // Reserved
        uint32_t entryNumber;        // Index of the most recent entry with time <= specified
        uint8_t checksum[2];    
});


/*!
 * LOG-INFO Message Structure
 * Log information
 * Message Type: OUTPUT
 * ID: 0x21  0x08 Payload Length=48 bytes
 */
PACK(
    struct LogInfo{
        UbloxHeader header;
        uint8_t version;        // The version of this message. Set to 1
        uint8_t reserved1[3];        // Reserved
        uint32_t filestoreCapacity;        // The capacity of the filestore
        uint8_t reserved2[8];        // Reserved
        uint32_t currentMaxLogSize;        // The maximum size the current log is allowed to grow to
        uint32_t currentLogSize;        // Approximate amount of space in log currently occupied
        uint32_t entryCount;        // Number of entries in the log. Note: for circular logs this value will decrease when a group of entries is deleted to make space for new ones.
        uint16_t oldestYear;        // Oldest entry UTC year year (1-65635) or zero if there are no entries with known time
        uint8_t oldestMonth;        // Oldest Month (1-12)
        uint8_t oldestDay;        // Oldest day (1-31)
        uint8_t oldestHour;        // Oldest hour (0-23)
        uint8_t oldestMinute;        // Oldest minute (0-59)
        uint8_t oldestSecond;        // Oldest second (0-60)
        uint8_t reserved3;        // Reserved
        uint16_t newestYear;        // Newest year (1-65635) or zero if there are no entries with known time
        uint8_t newestMonth;        // Newest month (1-12)
        uint8_t newestDay;     // Newest day (1-31)
        uint8_t newestHour;        // Newest hour (0-23)
        uint8_t newestMinute;      // Newest minute (0-59)
        uint8_t newestSecond;      // Newest second (0-60)
        uint8_t reserved4;     // Reserved
        uint8_t status;        // Log status flags (see graphic below)
        uint8_t reserved5[3];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * LOG-RETRIEVEPOSEXTRA Message Structure
 * Odometer log entry
 * Message Type: OUTPUT
 * ID: 0x21  0x0F Payload Length=32 bytes
 */
PACK(
    struct LogRetrievePosExtra{
        UbloxHeader header;
        uint32_t entryIndex;       // The index of this log entry
        uint8_t version;       // The version of this message. Set to 0
        uint8_t reserved1;     // Reserved
        uint16_t year;     // Year (1-65635) of UTC time. Will be zero if time not  known
        uint8_t month;     // Month (1-12) of UTC time
        uint8_t day;       // Day (1-31) of UTC time
        uint8_t hour;      // Hour (0-23) of UTC time
        uint8_t minute;        // Minute (0-59) of UTC time
        uint8_t second;        // Second (0-60) of UTC time
        uint8_t reserved2[3];     // Reserved
        uint32_t distance;     // Odometer distance traveled
        uint8_t reserved3[12];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * LOG-RETRIEVEPOS Message Structure
 * Position fix log entry
 * Message Type: OUTPUT
 * ID: 0x21  0x0B Payload Length=40 bytes
 */
PACK(
    struct LogRetrievePos{
        UbloxHeader header;
        uint32_t entryIndex;    // The index of this log entry
        int32_t lon;            // Longitude (deg) [scaled *1e-7]
        int32_t lat;            // Latitude (deg) [scaled *1e-7]
        int32_t hMSL;           // Height above mean sea level
        uint32_t hAcc;          // Horizontal accuracy estimate
        uint32_t gSpeed;        // Ground speed (2-D)
        uint32_t heading;       // Heading
        uint8_t version;        // The version of this message. Set to 0
        uint8_t fixType;        // Fix type: 2: 2D-Fix 3: 3D-Fix
        uint16_t year;          // Year (1-65635) of UTC time
        uint8_t month;          // Month (1-12) of UTC time
        uint8_t day;            // Day (1-31) of UTC time
        uint8_t hour;           // Hour (0-23) of UTC time
        uint8_t minute;         // Minute (0-59) of UTC time
        uint8_t second;         // Second (0-60) of UTC time
        uint8_t reserved1;      // Reserved
        uint8_t numSV;          // Number of satellites used in the position fix
        uint8_t reserved2;      // Reserved
        uint8_t checksum[2];    
});


/*!
 * LOG-RETRIEVESTRING Message Structure
 * Position fix log entry
 * Message Type: OUTPUT
 * ID: 0x21  0x0D Payload Length=16+1*bytecount bytes
 */
 #define MAX_LOG_RETRIEVESTRING 100
PACK(
    struct LogRetrieveString{
        UbloxHeader header;
        uint32_t entryIndex;       // The index of this log entry
        uint8_t version;       // The version of this message. Set to 0
        uint8_t reserved1;     // Reserved
        uint16_t year;     // Year (1-65635) of UTC time. Will be zero if time not  known
        uint8_t month;     // Month (1-12) of UTC time
        uint8_t day;       // Day (1-31) of UTC time
        uint8_t hour;      // Hour (0-23) of UTC time
        uint8_t minute;        // Minute (0-59) of UTC time
        uint8_t second;        // Second (0-60) of UTC time
        uint8_t reserved2;     // Reserved
        uint16_t byteCount;        // Size of string in bytes
        uint8_t bytes[MAX_LOG_RETRIEVESTRING];     // The bytes of the string
        uint8_t checksum[2];    
});


/*!
 * LOG-RETRIEVE Message Structure
 * Request log data
 * Message Type: COMMAND
 * ID: 0x21  0x09 Payload Length=12 bytes
 */
PACK(
    struct LogRetrieve{
        UbloxHeader header;
        uint32_t startNumber;       // Index of first entry to be transferred
        uint32_t entryCount;        // Number of log entries to transfer. The maximum is 256
        uint8_t version;            // The version of this message. Set to 0
        uint8_t reserved1[3];       // Reserved
        uint8_t checksum[2];    
});


/*!
 * LOG-STRING Message Structure
 * Store arbitrary string in on-board flash
 * Message Type: COMMAND
 * ID: 0x21  0x04 Payload Length=0+1*N bytes
 */
PACK(
    struct LogString{
        UbloxHeader header;
        uint8_t bytes[256];     // The string of bytes to be logged (maximum 256)
        uint8_t checksum[2];    
});



///////////////////////////////////////////////////////////
// UBX-MGA
///////////////////////////////////////////////////////////

/*!
 * MGA-ACK Message Structure
 * Multi-GNSS Acknowledge message
 * Message Type: OUTPUT
 * ID: 0x13  0x60 Payload Length=8 bytes
 */
PACK(
    struct MgaAck{
        UbloxHeader header;
        uint8_t type;      // Type, 1 = ACK, 0 = NACK
        uint8_t version;       // The version of this message, always set to 0
        uint8_t errorCode;     // Indicates the reason why a NACK was returned: 0: No error occured (only if message type is ACK)
                                // The receiver doesn't know the time so can't use the data (To resolve this an BX-MGA-INI-TIME_UTC message should be supplied first)
                                // 2: The message version is not supported by the receiver
                                // 3: The message size does not match the message version
                                // 4: The message data could not be stored to the database
                                // 5: The receiver is not ready to use the message data
                                // 6: The message type is unknown 255: Undefined error occured
        uint8_t msgId;     // UBX message ID of the ack'ed message
        uint8_t msgPayloadStart[4];     // The first 4 bytes of the ack'ed message's payload
        uint8_t checksum[2];    
});


/*!
 * MGA-ANO Message Structure
 * Multi-GNSS AssistNow Offline Assistance
 * Message Type: INPUT
 * ID: 0x13  0x20 Payload Length=76 bytes
 */
PACK(
    struct MgaAno{
        UbloxHeader header;
        uint8_t type;           // message type (always 0x00)
        uint8_t version;        // message version (always 0x00)
        uint8_t svId;           // Satellite identifier (see Satellite Numbering)
        uint8_t gnssId;         // GNSS identifier (see Satellite Numbering)
        uint8_t year;           // years since the year 2000
        uint8_t month;          // month (1..12)
        uint8_t day;            // day (1..31)
        uint8_t reserved1;      // Reserved
        uint8_t data[64];       // assistance data
        uint8_t reserved2[4];   // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-DBD Message Structure
 * Navigation Database Dump Entry
 * Message Type: INPUT/OUTPUT
 * ID: 0x13  0x80 Payload Length=12+1*N bytes (max=164)
 */
PACK(
    struct MgaDbd{
        UbloxHeader header;
        uint8_t reserved1[12];      // Reserved
        uint8_t data[152];      // fw specific data
        uint8_t checksum[2];    
});

///////////////////////////////////////////////////////////
// UBX-MGA-FLASH

/*!
 * MGA-FLASH-DATA Message Structure
 * Transfer MGA-ANO data block to flash
 * Message Type: INPUT
 * ID: 0x13  0x21 Payload Length=6+1*N bytes (max=512)
 */
PACK(
    struct MgaFlashData{
        UbloxHeader header;
        uint8_t type;           // Message type. Set to 1 for this message.
        uint8_t version;        // FLASH-DATA message version (this is version 0).
        uint16_t sequence;      // Message sequence number, starting at 0 and increamenting by 1 for each MGA-FLASH-DATA message sent.
        uint16_t size;          // Payload size in bytes.
        uint8_t data[506];      // Payload data.
        uint8_t checksum[2];    
}); 


/*!
 * MGA-FLASH-STOP Message Structure
 * Finish flashing MGA-ANO data
 * Message Type: INPUT
 * ID: 0x13  0x21 Payload Length=2 bytes
 */
PACK(
    struct MgaFlashStop{
        UbloxHeader header;
        uint8_t type;           // Message type. Set to 2 for this message.
        uint8_t version;        // FLASH-STOP message version (this is version 0).
        uint8_t checksum[2];    
});


/*!
 * MGA-FLASH-ACK Message Structure
 * Acknowledge last FLASH-DATA or -STOP
 * Message Type: OUTPUT
 * ID: 0x13  0x21 Payload Length=2 bytes
 */
PACK(
    struct MgaFlashAck{
        UbloxHeader header;
        uint8_t type;           // Message type. Set to 3 for this message.
        uint8_t version;        // FLASH-ACK message version (this is version 0).
        uint8_t ack;            // Acknowledgement type. 0 - ACK: Message received and written to flash. 1 - NACK: Problem with last message, re-transmission required (this only happens while acknowledging a UBX-MGA_FLASH_DATA message). 2 - NACK: problem with last message, give up.
        uint8_t reserved1;      // Reserved
        uint16_t sequence;      // If acknowledging a UBX-MGA-FLASH-DATA message this is the Message sequence number being ack'ed. If acknowledging a UBX-MGA-FLASH-STOP message it will be set to 0xffff.
        uint8_t checksum[2];    
});


///////////////////////////////////////////////////////////
// UBX-MGA-GLO

/*!
 * MGA-GLO-EPH Message Structure
 * GLONASS Ephemeris Assistance
 * Message Type: INPUT
 * ID: 0x13  0x06 Payload Length=48 bytes
 */
 PACK(
    struct MgaGloEph{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 1 for this message (1 = Ephemeris).
        uint8_t reserved1;     // Reserved
        uint8_t svId;      // GLONASS Satellite identifier (see Satellite Numbering)
        uint8_t reserved2;     // Reserved
        uint8_t FT;        // User range accuracy
        uint8_t B;     // Health flag from string 2
        uint8_t M;     // Type of GLONASS satellite (1 indicates GLONASS-M)
        int8_t H;      // Carrier frequency number of navigation RF signal, Range=(-7 .. 6), -128 for unknown
        int32_t x;     // X component of the SV position in PZ-90.02 coordinate System
        int32_t y;     // Y component of the SV position in PZ-90.02 coordinate System
        int32_t z;     // Z component of the SV position in PZ-90.02 coordinate System
        int32_t dx;        // X component of the SV velocity in PZ-90.02 coordinate System
        int32_t dy;        // Y component of the SV velocity in PZ-90.02 coordinate System
        int32_t dz;        // Z component of the SV velocity in PZ-90.02 coordinate System
        int8_t ddx;        // X component of the SV acceleration in PZ-90.02 coordinate System
        int8_t ddy;        // Y component of the SV acceleration in PZ-90.02 coordinate System
        int8_t ddz;        // Z component of the SV acceleration in PZ-90.02 coordinate System
        uint8_t tb;        // Index of a time interval within current day according to UTC(SU)
        int16_t gamma;     // Relative carrier frequency deviation
        uint8_t E;     // Ephemeris data age indicator
        int8_t deltaTau;       // Time difference between L2 and L1 band
        int32_t tau;       // SV clock bias
        uint8_t reserved3[4];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-GLO-ALM Message Structure
 * GLONASS Ephemeris Assistance
 * Message Type: INPUT
 * ID: 0x13  0x06 Payload Length=36 bytes
 */
PACK(
    struct MgaGloAlm{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 2 for this message (2 = Almanac).
        uint8_t reserved1;     // Reserved
        uint8_t svId;      // GLONASS Satellite identifier (see Satellite Numbering)
        uint8_t reserved2;     // Reserved
        uint16_t N;        // Reference calender day number of almanac within the four-year period (from string 5)
        uint8_t M;     // Type of GLONASS satellite (1 indicates GLONASS-M)
        uint8_t C;     // Unhealthy flag at instant of almanac upload (1 indicates operability of satellite)
        int16_t tau;       // Coarse time correction to GLONASS time
        uint16_t epsilon;      // Eccentricity
        int32_t lambda;        // Longitude of the first (within the N-day) ascending node of satellite orbit in PC-90.02 coordinate system
        int32_t deltaI;        // Correction to the mean value of inclination
        uint32_t tLambda;      // Time of the first ascending node passage
        int32_t deltaT;        // Correction to the mean value of Draconian period
        int8_t deltaDT;        // Rate of change of Draconian perion
        int8_t H;      // Carrier frequency number of navigation RF signal, Range=(-7 .. 6)
        int16_t omega;     // Argument of perigee
        uint8_t reserved3[4];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-GLO-TIMEOFFSET Message Structure
 * GLONASS Auxiliary Time Offset Assistance
 * Message Type: INPUT
 * ID: 0x13  0x06 Payload Length=20 bytes
 */
PACK(
    struct MgaGloTimeOffset{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 3 for this message (3 = time offsets).
        uint8_t reserved1;     // Reserved
        uint16_t N;        // Reference calender day number within the four-year period of almanac (from string 5)
        int32_t tauC;      // Time scale correction to UTC(SU) time
        int32_t tauGps;        // Correction to GPS time relative to GLONASS time
        int16_t B1;        // Coefficient to determine delta UT1
        int16_t B2;        // Rate of change of delta UT1
        uint8_t reserved2[4];     // Reserved
        uint8_t checksum[2];    
});


///////////////////////////////////////////////////////////
// UBX-MGA-GPS

/*!
 * MGA-GPS-EPH Message Structure
 * GPS Ephemeris Assistance
 * Message Type: INPUT
 * ID: 0x13  0x00 Payload Length=68 bytes
 */
PACK(
    struct MgaGpsEph{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 1 for this message (1 = Ephemeris).
        uint8_t reserved1;     // Reserved
        uint8_t svId;      // GPS Satellite identifier (see Satellite Numbering)
        uint8_t reserved2;     // Reserved
        uint8_t fitInterval;       // Fit interval flag
        uint8_t uraIndex;      // URA index
        uint8_t svHealth;      // SV health
        int8_t tgd;        // Group delay differential
        uint16_t iodc;     // IODC
        uint16_t toc;      // Clock data reference time
        uint8_t reserved3;     // Reserved
        int8_t af2;        // Time polynomial coefficient 2
        int16_t af1;       // Time polynomial coefficient 1
        int32_t af0;       // Time polynomial coefficient 0
        int16_t crs;       // Crs
        int16_t deltaN;        // Mean motion difference from computed value
        int32_t m0;        // Mean anomaly at reference time
        int16_t cuc;       // Amplitude of cosine harmonic correction term to argument of latitude
        int16_t cus;       // Amplitude of sine harmonic correction term to argument of latitude
        uint32_t e;        // Eccentricity
        uint32_t sqrtA;        // Square root of the semi-major axis
        uint16_t toe;      // Reference time of ephemeris
        int16_t cic;       // Amplitude of cos harmonic correction term to angle of inclination
        int32_t omega0;        // Longitude of ascending node of orbit plane at weekly epoch
        int16_t cis;       // Amplitude of sine harmonic correction term to angle of inclination
        int16_t crc;       // Amplitude of cosine harmonic correction term to orbit radius
        int32_t i0;        // Inclination angle at reference time
        int32_t omega;     // Argument of perigee
        int32_t omegaDot;      // Rate of right ascension
        int16_t idot;      // Rate of inclination angle
        uint8_t reserved4[2];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-GPS-ALM Message Structure
 * GPS Almanac Assistance
 * Message Type: INPUT
 * ID: 0x13  0x00 Payload Length=36 bytes
 */
PACK(
    struct MgaGpsAlm{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 2 for this message (2 = Almanac).
        uint8_t reserved1;     // Reserved
        uint8_t svId;      // GPS Satellite identifier (see Satellite Numbering)
        uint8_t svHealth;      // SV health information
        uint16_t e;        // Eccentricity
        uint8_t almWNa;        // Reference week number of almanac (the 8 bit WNa field)
        uint8_t toa;       // Reference time of almanac
        int16_t deltaI;        // Delta inclination angle at reference time
        int16_t omegaDot;      // Rate of right ascension
        uint32_t sqrtA;        // Square root of the semi-major axis
        int32_t omega0;        // Longitude of ascending node of orbit plane
        int32_t omega;     // Argument of perigee
        int32_t m0;        // Mean anomaly at reference time
        int16_t af0;       // Time polynomial coefficient 0 (8 MSBs)
        int16_t af1;       // Time polynomial coefficient 1
        uint8_t reserved2[4];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-GPS-HEALTH Message Structure
 * GPS Health Assistance
 * Message Type: INPUT
 * ID: 0x13  0x00 Payload Length=40 bytes
 */
PACK(
    struct MgaGpsHealth{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 4 for this message (4 = health flags).
        uint8_t reserved1[3];     // Reserved
        uint8_t healthCode[32];        // Each byte represents a GPS SV (1-32). The 6 LSBs of each byte contains the 6 bit health code from subframes 4/5 page 25.
        uint8_t reserved2[4];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-GPS-UTC Message Structure
 * GPS UTC Assistance
 * Message Type: INPUT
 * ID: 0x13  0x00 Payload Length=20 bytes
 */
PACK(
    struct MgaGpsUtc{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 5 for this message (5 = Time parameters).
        uint8_t reserved1[3];     // Reserved
        int32_t utcA0;     // First parameter of UTC polynomial
        int32_t utcA1;     // Second parameter of UTC polynomial
        int8_t utcDtLS;        // Delta time due to current leap seconds
        uint8_t utcTot;        // UTC parameters reference time of week (GPS time)
        uint8_t utcWNt;        // UTC parameters reference week number (the 8 bit WNt field)
        uint8_t utcWNlsf;      // Week number at the end of which the future leap second becomes effective (the 8 bit WNLSF field)
        uint8_t utcDn;     // Day number at the end of which the future leap second becomes effective
        int8_t utcDtLSF;       // Delta time due to future leap seconds
        uint8_t reserved2[2];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-GPS-IONO Message Structure
 * GPS Ionosphere Assistance
 * Message Type: INPUT
 * ID: 0x13  0x00 Payload Length=16 bytes
 */
PACK(
    struct GPSIonAssist{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 6 for this message (6 = ionosphere parameters).
        uint8_t reserved1[3];     // Reserved
        int8_t ionoAlpha0;     // Ionospheric parameter alpha0 [s]
        int8_t ionoAlpha1;     // Ionospheric parameter alpha1 [s/semi-circle]
        int8_t ionoAlpha2;     // Ionospheric parameter alpha2 [s/semi-circle^2]
        int8_t ionoAlpha3;     // Ionospheric parameter alpha3 [s/semi-circle^3]
        int8_t ionoBeta0;      // Ionospheric parameter beta0 [s]
        int8_t ionoBeta1;      // Ionospheric parameter beta1 [s/semi-circle]
        int8_t ionoBeta2;      // Ionospheric parameter beta2 [s/semi-circle^2]
        int8_t ionoBeta3;      // Ionospheric parameter beta3 [s/semi-circle^3]
        uint8_t reserved2[4];     // Reserved
        uint8_t checksum[2];    
});


///////////////////////////////////////////////////////////
// UBX-MGA-INI

/*!
 * MGA-INI-POS_XYZ Message Structure
 * Initial Position Assistancee ECEF
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=20 bytes
 */
PACK(
    struct MgaIniPosXYZ{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0x00 for this message (0x00 = Position - ECEF - XYZ).
        uint8_t reserved1[3];     // Reserved
        int32_t ecefX;     // WGS84 ECEF X coordinate (cm)
        int32_t ecefY;     // WGS84 ECEF Y coordinate (cm)
        int32_t ecefZ;     // WGS84 ECEF Z coordinate (cm)
        uint32_t posAcc;       // Position accuracy (stddev) (cm)
        uint8_t checksum[2];    
});


/*!
 * MGA-INI-POS_LLH Message Structure
 * Initial Position Assistancee LLH
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=20 bytes
 */
PACK(
    struct MgaIniPosLLH{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0x01 for this message (0x01 = Position - ECEF - LLA).
        uint8_t reserved1[3];     // Reserved
        int32_t lat;       // WGS84 Latitude
        int32_t lon;       // WGS84 Longitude
        int32_t alt;       // WGS84 Altitude
        uint32_t posAcc;       // Position accuracy (stddev)
        uint8_t checksum[2];    
});


/*!
 * MGA-INI-TIME_UTC Message Structure
 * Initial Time Assistance
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=24 bytes
 */
PACK(
    struct MgaIniTimeUTC{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0uint8_t
        uint8_t reserved1;     // Reserved
        uint8_t ref;       // Reference to be used to set time (see graphic below)
        int8_t leapSecs;       // Number of leap seconds since 1980 (or 0x80 = -128 if unknown)
        uint16_t year;     // Year
        uint8_t month;     // Month, starting at 1
        uint8_t day;       // Day, starting at 1
        uint8_t hour;      // Hour, from 0 to 23
        uint8_t minute;        // Minute, from 0 to 59
        uint8_t second;        // Seconds, from 0 to 59
        uint8_t reserved2;     // Reserved
        uint32_t ns;       // Nanoseconds, from 0 to 999,999,999
        uint16_t tAccS;        // Seconds part of time accuracy
        uint8_t reserved3[2];     // Reserved
        uint32_t tAccNs;       // Nanoseconds part of time accuracy, from 0 to 999,999,999
        uint8_t checksum[2];    
});


/*!
 * MGA-INI-TIME_GNSS Message Structure
 * Initial Time Assistance
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=24 bytes
 */
PACK(
    struct MgaIniTimeGNSS{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0uint8_t
        uint8_t reserved1;     // Reserved
        uint8_t ref;       // Reference to be used to set time (see graphic below)
        uint8_t gnssId;        // Source of time information. Currently supported:
                                // 0:  GPS time
                                // 2:  Galileo time
                                // 3:  BeiDou time
                                // 6: GLONASS time: week = 834 + ((N4-1)*1461 + Nt)/7, tow = (((N4-1)*1461 + Nt) % 7) * 86400 + tod
        uint8_t reserved2[2];     // Reserved
        uint16_t week;     // GNSS week number
        uint32_t tow;      // GNSS time of week
        uint32_t ns;       // GNSS time of week, nanosecond part from 0 to 999,999,999
        uint16_t tAccS;        // Seconds part of time accuracy
        uint8_t reserved3[2];     // Reserved
        uint32_t tAccNs;       // Nanoseconds part of time accuracy, from 0 to 999,999,999
        uint8_t checksum[2];    
});


/*!
 * MGA-INI-CLKD Message Structure
 * Initial Clock Drift Assistance
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=12 bytes
 */
PACK(
    struct MgaIniClkD{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0x20
        uint8_t reserved1[3];     // Reserved
        int32_t clkD;      // Clock drift
        uint32_t clkDAcc;      // Clock drift accuracy
        uint8_t checksum[2];    
});


/*!
 * MGA-INI-FREQ Message Structure
 * Initial Frequency Assistance
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=12 bytes
 */
PACK(
    struct MgaIniFreq{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0uint16_t
        uint8_t reserved1[2];     // Reserved
        uint8_t flags;     // Frequency reference (see graphic below)
        int32_t freq;      // Frequency
        uint32_t freqAcc;      // Frequency accuracy
        uint8_t checksum[2];    
});


/*!
 * MGA-INI-EOP Message Structure
 * Earth Orientation Parameters Assistance
 * Message Type: INPUT
 * ID: 0x13  0x40 Payload Length=17 bytes
 */
PACK(
    struct MgaIniEop{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 0x30 for this message (0x30 = EOP).
        uint8_t reserved1[3];     // Reserved
        uint16_t d2kRef;       // reference time (days since 1.1.2000 12.00h UTC)
        uint16_t d2kMax;       // expiration time (days since 1.1.2000 12.00h UTC)
        int32_t xpP0;      // x_p t^0 polynomial term (offset)
        int32_t xpP1;      // x_p t^1 polynomial term (drift)
        int32_t ypP0;      // y_p t^0 polynomial term (offset)
        int32_t ypP1;      // y_p t^1 polynomial term (drift)
        int32_t dUT1;      // dUT1 t^0 polynomial term (offset)
        int32_t ddUT1;     // dUT1 t^1 polynomial term (drift)
        uint8_t reserved2[40];     // Reserved
        uint8_t checksum[2];    
});


///////////////////////////////////////////////////////////
// UBX-MGA-INI

/*!
 * MGA-QZSS-EPH Message Structure
 * QZSS Ephemeris Assistance
 * Message Type: INPUT
 * ID: 0x13  0x05 Payload Length=68 bytes
 */
PACK(
    struct MgaQzssEph{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 1 for this message (1 = Ephemeris).
        uint8_t reserved1;     // Reserved
        uint8_t svId;      // QZSS Satellite identifier (see Satellite Numbering), Range 1-5
        uint8_t reserved2;     // Reserved
        uint8_t fitInterval;       // Fit interval flag
        uint8_t uraIndex;      // URA index
        uint8_t svHealth;      // SV health
        int8_t tgd;        // Group delay differential
        uint16_t iodc;     // IODC
        uint16_t toc;      // Clock data reference time
        uint8_t reserved3;     // Reserved
        int8_t af2;        // Time polynomial coefficient 2
        int16_t af1;       // Time polynomial coefficient 1
        int32_t af0;       // Time polynomial coefficient 0
        int16_t crs;       // Crs
        int16_t deltaN;        // Mean motion difference from computed value
        int32_t m0;        // Mean anomaly at reference time
        int16_t cuc;       // Amp of cosine harmonic corr term to arg of lat
        int16_t cus;       // Amp of sine harmonic corr term to arg of lat
        uint32_t e;        // eccentricity
        uint32_t sqrtA;        // Square root of the semi-major axis A
        uint16_t toe;      // Reference time of ephemeris
        int16_t cic;       // Amp of cos harmonic corr term to angle of inclination
        int32_t omega0;        // Long of asc node of orbit plane at weekly epoch
        int16_t cis;       // Amp of sine harmonic corr term to angle of inclination
        int16_t crc;       // Amp of cosine harmonic corr term to orbit radius
        int32_t i0;        // Inclination angle at reference time
        int32_t omega;     // Argument of perigee
        int32_t omegaDot;      // Rate of right ascension
        int16_t idot;      // Rate of inclination angle
        uint8_t reserved4[2];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-QZSS-ALM Message Structure
 * QZSS Almanac Assistance
 * Message Type: INPUT
 * ID: 0x13  0x05 Payload Length=36 bytes
 */
PACK(
    struct MgaQzssAlm{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 2 for this message (2 = Almanac).
        uint8_t reserved1;     // Reserved
        uint8_t svId;      // QZSS Satellite identifier (see Satellite Numbering), Range 1-5
        uint8_t svHealth;      // Almanac SV health information
        uint16_t e;        // Almanac eccentricity
        uint8_t almWNa;        // Reference week number of almanac (the 8 bit WNa field)
        uint8_t toa;       // Reference time of almanac
        int16_t deltaI;        // Delta inclination angle at reference time
        int16_t omegaDot;      // Almanac rate of right ascension
        uint32_t sqrtA;        // Almanac square root of the semi-major axis A
        int32_t omega0;        // Almanac long of asc node of orbit plane at weekly
        int32_t omega;     // Almanac argument of perigee
        int32_t m0;        // Almanac mean anomaly at reference time
        int16_t af0;       // Almanac time polynomial coefficient 0 (8 MSBs)
        int16_t af1;       // Almanac time polynomial coefficient 1
        uint8_t reserved2[4];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MGA-QZSS-HEALTH Message Structure
 * QZSS Health Assistance
 * Message Type: INPUT
 * ID: 0x13  0x05 Payload Length=12 bytes
 */
PACK(
    struct QZSSHeathAssist{
        UbloxHeader header;
        uint8_t type;      // Message type. Set to 4 for this message (4 = health flags).
        uint8_t reserved1[3];     // Reserved
        uint8_t healthCode[5];        // Each byte represents a QZSS SV (1-5). The 6 LSBs of each byte contains the 6 bit health code from subframes 4/5, data ID = 3, SV ID = 51
        uint8_t reserved2[3];     // Reserved
        uint8_t checksum[2];    
});



///////////////////////////////////////////////////////////
// UBX-MON
///////////////////////////////////////////////////////////

/*!
 * MON-GNSS Message Structure
 * Information message GNSS selection
 * Message Type: OUTPUT
 * ID: 0x0A  0x28 Payload Length=8 bytes
 */
PACK(
    struct MonGnss{
        UbloxHeader header;
        uint8_t version;       // Type of the message, 1 for this type
        uint8_t Supported;     // A bit mask, saying which GNSS systems can be supported by this receiver (see graphic below)
        uint8_t Default;       // A bit mask, saying which GNSS systems are enabled in the current efuse default configuration for this receiver (see graphic below)
        uint8_t Enabled;       // A bit mask, saying which GNSS systems are currently enabled for this receiver (see graphic below)
        uint8_t Simultaneous;      // Maximum number of concurrent GNSS systems which can be supported by this receiver
        uint8_t reserved1[3];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * MON-HW2 Message Structure
 * Extended Hardware Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x0A  0x0B Payload Length=28 bytes
 */
// TODO


/*!
 * MON-HW Message Structure
 * Hardware Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x0A  0x09 Payload Length=60 bytes
 */
PACK(
    struct MonHw{
        UbloxHeader header;
        uint32_t pinSel;       // Mask of Pins Set as Peripheral/PIO
        uint32_t pinBank;      // Mask of Pins Set as Bank A/B
        uint32_t pinDir;       // Mask of Pins Set as Input/Output
        uint32_t pinVal;       // Mask of Pins Value Low/High
        uint16_t noisePerMS;       // Noise Level as measured by the GPS Core
        uint16_t agcCnt;       // AGC Monitor (counts SIGHI xor SIGLO, range 0 to 8191)
        uint8_t aStatus;       // Status of the Antenna Supervisor State Machine (0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN)
        uint8_t aPower;        // Current PowerStatus of Antenna (0=OFF, 1=ON, 2=DONTKNOW)
        uint8_t flags;     // Flags (see graphic below)
        uint8_t reserved1;     // Reserved
        uint32_t usedMask;     // Mask of Pins that are used by the Virtual Pin Manager
        uint8_t VP[17];        // Array of Pin Mappings for each of the 17 Physical Pins
        uint8_t jamInd;        // CW Jamming indicator, scaled (0 = no CW jamming, 255 = strong CW jamming)
        uint8_t reserved2[2];     // Reserved
        uint32_t pinIrq;       // Mask of Pins Value using the PIO Irq
        uint32_t pullH;        // Mask of Pins Value using the PIO Pull High Resistor
        uint32_t pullL;        // Mask of Pins Value using the PIO Pull Low Resistor
        uint8_t checksum[2];    
});


/*!
 * MON-IO Message Structure
 * I/O Subsystem Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x0A  0x02 Payload Length=0+20*N bytes
 */


/*!
 * MON-MSGPP Message Structure
 * Message Parse and Process Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x0A  0x06 Payload Length=120 bytes
 */


/*!
 * MON-PATCH Message Structure
 * Output information about installed patches.
 * Message Type: OUPUT
 * ID: 0x0A  0x27 Payload Length=4 + 16*nEntries bytes
 */
PACK(
    struct MonPatchRepeated {
        uint32_t patchInfo;        // Additional information about the patch not stated in the patch header. (see graphic below)
        uint32_t comparatorNumber;        // The number of the comparator.
        uint32_t patchAddress;     // The address that the targeted by the patch.
        uint32_t patchData;        // The data that will be inserted at the patchAddress.   
});
PACK(
    struct MonPatch {
        UbloxHeader header;
        uint16_t version;      // Type of the message. 0uint8_t
        uint16_t nEntries;     // The number of patches that is output.
        MonPatchRepeated repeated_block[20];
        uint8_t checksum[2];    
});


/*!
 * MON-RXBUF Message Structure
 * Receiver Buffer Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x0A  0x07 Payload Length=24 bytes
 */


/*!
 * MON-RXR Message Structure
 * Receiver Status Information
 * Message Type: OUTPUT
 * ID: 0x0A  0x21 Payload Length=1 bytes
 */


/*!
 * MON-SMGR Message Structure
 * Synchronization Manager Status
 * Only available with FTSproduct variant
 * Message Type: OUTPUT
 * ID: 0x0A  0x21 Payload Length=1 bytes
 */
PACK(
    struct MonSmgr{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t reserved1[3];     // Reserved
        uint32_t iTOW;     // Time of the week
        uint16_t intOsc;       // A bit mask, indicating the status of the local oscillator (see graphic below)
        uint16_t extOsc;       // A bit mask, indicating the status of the external oscillator (see graphic below)
        uint8_t discSrc;       // Disciplining source identifier: 0: internal oscillator
                                // 1: GNSS
                                // 2: EXTINT0
                                // 3: EXTINT1
                                // 4: internal oscillator measured by the host 5: external oscillator measured by the host
        uint8_t gnss;      // A bit mask, indicating the status of the GNSS (see graphic below)
        uint8_t extInt0;       // A bit mask, indicating the status of the external input 0 (see graphic below)
        uint8_t extInt1;       // A bit mask, indicating the status of the external input 1 (see graphic below)
        uint8_t checksum[2];    
});


/*!
 * MON-TXBUF Message Structure
 * Transmitter Buffer Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x0A  0x08 Payload Length=28 bytes
 */


/*!
 * MON-VER Message Structure
 * Poll Receiver/Software Version
 * Message Type: POLLED
 * ID: 0x0A  0x04 Payload Length=40+30*N bytes
 */
 PACK(
    struct MonVerRepeated {
        uint8_t extension[30];
});
PACK(
    struct MonVer {
        UbloxHeader header;
        uint8_t swVersion[30];
        uint8_t hwVersion[10];
        MonVerRepeated repeated_block[10];
        uint8_t checksum[2];    
});


///////////////////////////////////////////////////////////
// UBX-NAV

/*!
 * NAV-AOPSTATUS Message Structure
 * AssistNow Autonomous Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x01  0x60 Payload Length=16 bytes
 */
PACK(
    struct NavAopStatus {
        UbloxHeader header;
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint8_t aopCfg;        // AssistNow Autonomous configuration (see graphic below)
        uint8_t status;        // AssistNow Autonomous subsystem is idle (0) or running (not 0)
        uint8_t reserved1[10];     // Reserved
        uint8_t checksum[2];    
});


/*!
* NAV-CLOCK Message Structure
* Clock Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x22  Payload Length= 20 bytes
*/
PACK(
    struct NavClock{
        UbloxHeader header;
        uint32_t iTOW;
        int32_t clkbias;    // clock bias in nanoseconds
        int32_t clkdrift;   // clock drift in ns/s
        uint32_t tacc;      // time accuracy estimate (ns)
        uint32_t facc;      // frequency accuracy estimate (ps/s)
        uint8_t checksum[2];
});


/*!
* NAV-DGPS Message Structure
* DGPS Data Used for NAV
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x31  Payload Length= (16 + 12*numChannels bytes)
*/
PACK(
    struct NavDGPSReap{
        uint8_t svid;
        uint8_t flags;  // bitfield containing channel each sv is on and DGPS status
        uint16_t agecorr;   // age of latest correction data (ms)
        float prcorr;   // psuedorange correction   (m)
        float prrcorr;  // psuedorange rate correction (m/sec)
});

PACK(
    struct NavDGPS{
        UbloxHeader header;
        uint32_t iTOW;  // GPS ms time of week
        int32_t age;    // age of newest correction data (ms)
        int16_t baseID; // DGPS base station ID
        int16_t basehealth; // DGPS base station health
        uint8_t numchan;    // nomber of channels for which correction data is following
        uint8_t status; // DGPS correction type status
        uint16_t reserved;  // reserved
        NavDGPSReap nav_dgps_reap;  // repeated portion of NAV-DGPS message
        uint8_t checksum[2];
});


/*!
* NAV-DOP Message Structure
* This message outputs various DOPs. All 
* DOP values are scaled by a factor of 100.
* Ex. If gdop contains a value 156, the true
* value is 1.56
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x04  Payload Length= 18 bytes
*/
PACK(
    struct NavDOP{
        UbloxHeader header;
        uint32_t iTOW;  // GPS ms time of week (ms)
        uint16_t gdop;  // Geometric DOP
        uint16_t pdop;  // Position DOP
        uint16_t tdop;  // Time DOP
        uint16_t vdop;  // Vertical DOP
        uint16_t hdop;  // Horizontal DOP
        uint16_t ndop;  // Northing DOP
        uint16_t edop;  // Easting DOP
        uint8_t checksum[2];
});


/*!
* NAV-ODO Message Structure
* Odometer Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x09  Payload Length= 20 bytes
*/
PACK(
    struct NavOdo{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t reserved1[3];     // Reserved
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint32_t distance;     // Ground distance since last reset
        uint32_t totalDistance;        // Total cumulative ground distance
        uint32_t distanceStd;      // Ground distance accuracy (1-sigma)
        uint8_t checksum[2];    
});


/*!
* NAV-ORB Message Structure
* GNSS Orbit Database Info
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x34  Payload Length= 8 + 6*numSv bytes
*/
PACK(
    struct NavOrbRepeated{
        uint8_t gnssId;        // GNSS ID
        uint8_t svId;      // Satellite ID
        uint8_t svFlag;        // Information Flags (see graphic below)
        uint8_t eph;       // Ephemeris data (see graphic below)
        uint8_t alm;       // Almanac data (see graphic below)
        uint8_t otherOrb;      // Other orbit data available (see graphic below)   
});
PACK(
    struct NavOrb{
        UbloxHeader header;
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint8_t version;       // Message version (0, for this version)
        uint8_t numSv;     // Number of SVs in the database
        uint8_t reserved1[2];       // Reserved
        NavOrbRepeated repeated_block[MAX_SAT];
        uint8_t checksum[2];    
});


/*!
* NAV-POSECEF Message Structure
* Position Solution in ECEF
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x01  Payload Length= 20 bytes
*/
PACK(
    struct NavPosECEF{
        UbloxHeader header;
        uint32_t iTOW;
        int32_t ecefX;
        int32_t ecefY;
        int32_t ecefZ;
        uint32_t pAcc;
        uint8_t checksum[2];
});


/*!
* NAV-POSLLH Message Structure
* This message outputs the Geodetic position in
* the currently selected Ellipsoid. The default is
* the WGS84 Ellipsoid, but can be changed with the
* message CFG-DAT.
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x02  Payload Length=28 bytes
*/
PACK(
    struct NavPosLLH{
        UbloxHeader header;     //!< Ublox header
        uint32_t iTOW;          //!< GPS millisecond time of week
        int32_t longitude_scaled; //!< longitude in degrees. Scaling 1e-7
        int32_t latitude_scaled; //!< latitude in degrees. Scaling 1e-7
        int32_t height;          //!< height above ellipsoid [mm]
        int32_t height_mean_sea_level; //!< height above mean sea level [mm]
        uint32_t horizontal_accuracy; //!< horizontal accuracy estimate [mm]
        uint32_t vertical_accuracy; //!< vertical accuracy estimate [mm]
        uint8_t checksum[2];
});


/*!
* NAV-PVT Message Structure
* Navigation Position Velocity Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x07  Payload Length=92 bytes
*/
PACK(
    struct NavPVT{
        UbloxHeader header;
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint16_t year;     // Year (UTC)
        uint8_t month;     // Month, range 1..12 (UTC)
        uint8_t day;       // Day of month, range 1..31 (UTC)
        uint8_t hour;      // Hour of day, range 0..23 (UTC)
        uint8_t min;       // Minute of hour, range 0..59 (UTC)
        uint8_t sec;       // Seconds of minute, range 0..60 (UTC)
        uint8_t valid;     // Validity Flags (see graphic below)
        uint32_t tAcc;     // Time accuracy estimate (UTC)
        int32_t nano;      // Fraction of second, range -1e9 .. 1e9 (UTC)
        uint8_t fixType;       // GNSSfix Type, range 0..5 0x00 = No Fix
                                // 0x01 = Dead Reckoning only 0x02 = 2D-Fix
                                // 0x03 = 3D-Fix
                                // 0x04 = GNSS + dead reckoning combined 0x05 = Time only fix
                                // 0x06..0xff: reserved
        uint8_t flags;     // Fix Status Flags (see graphic below)
        uint8_t reserved1;     // Reserved
        uint8_t numSV;     // Number of satellites used in Nav Solution
        int32_t lon;       // Longitude
        int32_t lat;       // Latitude
        int32_t height;        // Height above ellipsoid
        int32_t hMSL;      // Height above mean sea level
        uint32_t hAcc;     // Horizontal accuracy estimate
        uint32_t vAcc;     // Vertical accuracy estimate
        int32_t velN;      // NED north velocity
        int32_t velE;      // NED east velocity
        int32_t velD;      // NED down velocity
        int32_t gSpeed;        // Ground Speed (2-D)
        int32_t headMot;       // Heading of motion (2-D)
        uint32_t sAcc;     // Speed accuracy estimate
        uint32_t headAcc;      // Heading accuracy estimate (both motion and vehicle)
        uint16_t pDOP;     // Position DOP
        uint8_t reserved2[6];     // Reserved
        int32_t headVeh;       // Heading of vehicle (2-D)
        uint8_t reserved3[4];     // Reserved
        uint8_t checksum[2];    
});


/*!
* NAV-RESETODO Message Structure
* Reset odometer
* Message Type: COMMAND
* ID: 0x01  0x10  Payload Length=0 bytes
*/
PACK(
    struct NavResetOdo{
        UbloxHeader header;
        uint8_t checksum[2];    
});


/*!
* NAV-SAT Message Structure
* Satellite Information
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x35  Payload Length=8 + 12*numSvs bytes
*/
PACK(
    struct NavSat{
        UbloxHeader header;
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint8_t version;       // Message version (1 for this version)
        uint8_t numSvs;        // Number of satellites
        uint8_t reserved1[2];     // Reserved
        uint8_t gnssId;        // GNSS identifier (see Satellite numbering) for assignment
        uint8_t svId;      // Satellite identifier (see Satellite numbering) for assignment
        uint8_t cno;       // Carrier to noise ratio (signal strength)
        int8_t elev;       // Elevation (range: +/-90), unknown if out of range
        int16_t azim;      // Azimuth (range +/-180), unknown if elevation is out of range
        int16_t prRes;     // Pseudo range residual
        uint32_t flags;        // Bitmask (see graphic below)
        uint8_t checksum[2];    
});


/*!
* NAV-SBAS Message Structure
* SBAS Status Data
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x32  Payload Length=12 + 12*cnt bytes
*/


/*!
* NAV-SOL Message Structure
* This message combines Position, velocity and
* time solution in ECEF, including accuracy figures.
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x06  Payload Length=52 bytes
*/
#define NAVSOL_FLAG_GPSFIX_VALID 0b0001
#define NAVSOL_FLAG_DGPS_USED_FOR_FIX 0b0010
#define NAVSOL_FLAG_WEEK_NUM_VALID 0b0100
#define NAVSOL_FLAG_TOW_VALID 0b1000

PACK(
    struct NavSol{
        UbloxHeader header;
        uint32_t iTOW;
        int32_t fTOW;
        int16_t week;
        uint8_t gpsFix;
        int8_t flags;
        int32_t ecefX;
        int32_t ecefY;
        int32_t ecefZ;
        uint32_t pAcc;
        int32_t ecefVX;
        int32_t ecefVY;
        int32_t ecefVZ;
        uint32_t sAcc;
        uint16_t pDop;
        uint8_t reserved1;
        uint8_t numSV;
        uint32_t reserved2;
        uint8_t checksum[2];
});


/*!
 * NAV-STATUS Message Structure
 * Receiver Navigation Status
 * Message Type: PERIODIC/POLLED
 * ID: 0x01 0x03 Payload Length=16 bytes
 */
PACK(
    struct NavStatus {
        UbloxHeader header;
        uint32_t iTOW;      // Time of Week (ms)
        uint8_t fixtype;    // no fix=0x00, deadreckoning only=0x01, 2D=0x02, 3D=0x03, deadreck+GPS=0x04, time fix only=0x05, reserved=0x06..0xff
        uint8_t flags;
        uint8_t fixstat;
        uint8_t flags2;
        uint32_t ttff;      // TTFF (ms)
        uint32_t msss;      // Milliseconds since startup/reset
        uint8_t checksum[2];

});


/*!
* NAV-SVINFO Message Structure
* This message outputs info about SVs each 
* channel is tracking
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x30  Payload Length= (8+12*NumChannels bytes)
*/
PACK(
    struct SVInfoReapBlock{
        uint8_t ch_num;     //!< Channel Number (255 if SV isn't assigned to channel)
        uint8_t svid;       // Satellite ID number
        uint8_t flags;      // bitfield (description of contents follows)
        uint8_t quality;    // signal quality indicator bitfield
        uint8_t cno;        // carrier to noise ratio (dbHz)
        int8_t elev;        // elevation (deg)
        int16_t azim;       // azimuth (deg)
        int32_t prRes;      // Psuedorange residual (centimeters)
});

PACK(
    struct NavSVInfo{
        UbloxHeader header;     //!< Ublox header
        uint32_t iTOW;  // GPS time of week (ms)
        uint8_t numch;  //! number of channels following
        uint8_t global_flags;   // Chip and Hardware Generation
        uint16_t reserved2;
        SVInfoReapBlock svinfo_reap[MAXCHAN]; // NOTE: TODO: True max needs to be confirmed
        uint8_t checksum[2];
});
// Description of flags bitfield
#define NAV_SVINFO_FLAGS_USED4NAV 0B00000001 // SV used in NAV sol
#define NAV_SVINFO_FLAGS_DGPS_AVAIL 0B00000010 // DGPS corr data available for SV
#define NAV_SVINFO_FLAGS_ORBIT_INFO_AVAIL 0B00000100 // Ephemeris of Almanac orbit info available for SV
#define NAV_SVINFO_FLAGS_EPHEMS_AVAIL 0B00001000 // Ephemeris orbit info available for SV
#define NAV_SVINFO_FLAGS_SV_UNHEALTHY 0B00010000 // SV unhealthy and not used
#define NAV_SVINFO_FLAGS_ALMPLUS_AVAIL 0B00100000 // Almanac Plus orbit info used
#define NAV_SVINFO_FLAGS_ASSNOW_AUTO 0B01000000 // AssistNow Autonomous orbit info used
#define NAV_SVINFO_FLAGS_PR_SMOOTHED 0B10000000 // Carrier Smoothed pseudorange used (PPP)


/*!
* NAV-TIMEBDS Message Structure
* Bedou Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x24  Payload Length= 20 bytes
*/
PACK(
    struct NavTimeBDS{
        UbloxHeader header;
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint32_t SOW;      // BDS time of week (rounded to seconds)
        int32_t fSOW;      // Fractional part of SOW (range: +/-500000000). The precise BDS time of week in seconds is: SOW + fSOW * 1e-9
        int16_t week;      // BDS week number of the navigation epoch
        int8_t leapS;      // BDS leap seconds (BDS-UTC)
        uint8_t valid;     // Validity Flags (see graphic below)
        uint32_t tAcc;     // Time Accuracy Estimate
        uint8_t checksum[2];    
});


/*!
* NAV-TIMEGLO Message Structure
* GLONASS Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x23  Payload Length= 20 bytes
*/
PACK(
    struct NavTimeGLO{
        UbloxHeader header;
        uint32_t iTOW;     // GPS time of week of the navigation epoch. See the description of iTOW for details.
        uint32_t TOD;      // GLONASS time of day (rounded to integer seconds)
        int32_t fTOD;      // Fractional part of TOD (range: +/-500000000). The precise GLONASS time of day in seconds is: TOD + fTOD * 1e-9
        uint16_t Nt;       // Current date (range: 1-1461), starting at 1 from the 1st Jan of the year indicated by N4 and ending at 1461 at the 31st Dec of the third year after that indicated by N4
        uint8_t N4;        // Four-year interval number starting from 1996 (1=1996, 2=2000, 3=2004...)
        uint8_t valid;     // Validity flags (see graphic below)
        uint32_t tAcc;     // Time Accuracy Estimate
        uint8_t checksum[2];    
});


/*!
* NAV-TIMEGPS Message Structure
* GPS Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x20  Payload Length= 16 bytes
*/
PACK(
    struct NavTimeGPS{
        UbloxHeader header;
        uint32_t iTOW;  // GPS ms time of week
        int32_t ftow;   // fractional nanoseconds remainder
        int16_t week;   // GPS week
        int8_t leapsecs;// GPS UTC leap seconds
        uint8_t valid;  // validity flags
        uint32_t tacc;  // time accuracy measurement (nanosecs)
        uint8_t checksum[2];
});


/*!
* NAV-TIMEUTC Message Structure
* UTC Time Solution
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x21  Payload Length= 20 bytes
*/
PACK(
    struct NavTimeUTC{
        UbloxHeader header;
        uint32_t iTOW;  // GPS time of week (msec)
        uint32_t tacc;  // time accuracy measurement
        int32_t nano;   // Nanoseconds of second
        uint16_t year;  // year
        uint8_t month;  // month
        uint8_t day;    // day
        uint8_t hour;   // hour
        uint8_t min;    // minute
        uint8_t sec;    // second
        uint8_t valid;  // validity flags
        uint8_t checksum[2];
});


/*!
* NAV-VELECEF Message Structure
* Velocity Solution in ECEF
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x11  Payload Length=20 bytes
*/


/*!
* NAV-VELNED Message Structure
* Velocity Solution in NED
* Message Type: PERIODIC/POLLED
* ID: 0x01  0x12  Payload Length=36 bytes
*/
PACK(
    struct NavVelNED{
        UbloxHeader header;     //!< Ublox header
        uint32_t iTOW;
        int32_t velocity_north; //!< north velocity [cm/s]
        int32_t velocity_east; //!< east velocity [cm/s]
        int32_t velocity_down; //!< down velocity [cm/s]
        uint32_t speed; //!< 3D speed [cm/s]
        uint32_t ground_speed; //!< 2D (ground) speed [cm/s]
        int32_t heading_scaled; //!< heading [deg]. Scaling 1e-5
        uint32_t speed_accuracy; //!< speed accuracy estimate [cm/s]
        uint32_t heading_accuracy; //!< course/heading accuracy estimate [deg]. Scaling 1e-5
        uint8_t checksum[2];
});


///////////////////////////////////////////////////////////
// UBX-RXM

/*!
* RXM-PMREQ Message Structure
* Requests a Power Management task
* Message Type: COMMAND
* ID: 0x02  0x41  Payload Length=8 bytes
*/

/*!
* RXM-RAWX Message Structure
* Multi-GNSS Raw Measurement Data
* Message Type: PERIODIC/POLLED
* ID: 0x02  0x15  Payload Length=16 + 32*numMeas bytes
*/
PACK(
    struct RxmRawXRepeated{
        double prMes;      // Pseudorange measurement [m]. GLONASS inter frequency channel delays are compensated with an internal calibration table.
        double cpMes;      // Carrier phase measurement [cycles]. The carrier phase initial ambiguity is initialized using an approximate value to make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code measurements in accordance with the RINEX specification.
        float doMes;      // Doppler measurement (positive sign for approaching satellites) [Hz]
        uint8_t gnssId;        // GNSS identifier (see Satellite Numbering for a list of identifiers)
        uint8_t svId;      // Satellite identifier (see Satellite Numbering)
        uint8_t reserved2;     // Reserved
        uint8_t freqId;        // Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
        uint16_t locktime;     // Carrier phase locktime counter (maximum 64500ms)
        uint8_t cno;       // Carrier-to-noise density ratio (signal strength) [dB-Hz]
        uint8_t prStdev;       // Estimated pseudorange measurement standard deviation (see graphic below)
        uint8_t cpStdev;       // Estimated carrier phase measurement standard deviation (note a raw value of 0x0F indicates the value is invalid) (see graphic below)
        uint8_t doStdev;       // Estimated Doppler measurement standard deviation. (see graphic below)
        uint8_t trkStat;       // Tracking status bitfield (see graphic below)
        uint8_t reserved3;     // Reserved
   
});
PACK(
    struct RxmRawX{
        UbloxHeader header;
        double rcvTow;     // Measurement time of week in receiver local  time approximately aligned to the GPS time system. The receiver local time of week, week number and leap second information can be used to translate the time to other time systems. More information about the difference in time systems can be found in RINEX 3 documentation. For a receiver operating in GLONASS only mode, UTC time can be determined by subtracting the leapS field from GPS time regardless of whether the GPS leap seconds are valid.
        uint16_t week;     // GPS week number in receiver local time.
        int8_t leapS;      // GPS leap seconds (GPS-UTC). This field represents the receiver's best knowledge of the leap seconds offset. A flag is given in the recStat bitfield to indicate if the leap seconds are known.
        uint8_t numMeas;       // Number of measurements to follow
        uint8_t recStat;       // Receiver tracking status bitfield (see graphic below)
        uint8_t reserved1[3];     // Reserved
        RxmRawXRepeated repeated_block[MAX_SAT];
        uint8_t checksum[2];    
});


/*!
* RXM-SFRBX Message Structure
* Raw Subframe Data. Thus the message contains preamble, 
* parity bits and all protocol specific overhead and is
* sent out when new data is received from the transmitter.
* Message Type: APERIODIC
* ID: 0x02  0x15  Payload Length=16 + 32*numMeas bytes
*/
PACK(
    struct RxmSfrbX{
        UbloxHeader header;
        uint8_t gnssId;        // GNSS identifier (see Satellite Numbering)
        uint8_t svId;      // Satellite identifier (see Satellite Numbering)
        uint8_t reserved1;     // Reserved
        uint8_t freqId;        // Only used for GLONASS: This is the frequency slot + 7 (range from 0 to 13)
        uint8_t numWords;      // The number of data words contained in this message (0..16)
        uint8_t reserved2;     // Reserved
        uint8_t version;       // Message version, (=1 for this version)
        uint8_t reserved3;     // Reserved
        uint32_t dwrd;     // The data words
        uint8_t checksum[2];    
});


/*!
 * RXM-SVSI Message Structure
 * SV Status Info
 * Message Type: PERIODIC/POLLED
 * ID: 0x02 0x20 Payload Length = (8 + 6*#SVs) bytes
 */
PACK(
    struct RxmSvsiRepeated{
        uint8_t svid;       // Satellite ID
        uint8_t svflag;     // Information Flag
        int16_t azim;       // Azimuth
        int8_t elev;        // Elevation
        uint8_t age;        // Age of almanac and ephemeris

});
PACK(
    struct RxmSvsi{
        UbloxHeader header;
        int32_t iTow;       // ms - Time of Week
        int16_t week;       // weeks - GPS Week
        uint8_t numvis;     // Number of visible SVs
        uint8_t numSV;      // # of SVs following
        RxmSvsiRepeated repeated_block[MAX_SAT]; // NOTE: TODO: Find the max repititions possible for this!! max thus far: (71)
        uint8_t checksum[2];
});


///////////////////////////////////////////////////////////
// UBX-TIM

/*!
 * TIM-DOSC Message Structure
 * Disciplined oscillator control
 * Only available with FTS product variant
 * Message Type: OUTPUT
 * ID: 0x0D 0x11 Payload Length = 8 bytes
 */
PACK(
    struct TimDosc{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t reserved1[3];     // Reserved
        uint32_t value;        // The raw value to be applied to the DAC controlling the external oscillator. The least significant bits should be written to the DAC, with the higher bits being ignored.
        uint8_t checksum[2];    
});


/*!
 * TIM-FCHG Message Structure
 * Oscillator frequency changed notification
 * Only available with FTS product variant
 * Message Type: NOTIFICATION
 * ID: 0x0D 0x16 Payload Length = 32 bytes
 */
PACK(
    struct TimFchg{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t reserved1[3];     // Reserved
        uint32_t iTOW;     // GPS time of week of the navigation epoch from which the sync manager obtains the GNSS specific data. Like for the NAV message, the iTOW can be used to group messages of a single sync manager run together (See the description of iTOW for details)
        int32_t intDeltaFreq;      // Frequency increment of the internal oscillator
        uint32_t intDeltaFreqUnc;     // Uncertainty of the internal oscillator frequency increment
        uint32_t intRaw;       // Current raw DAC setting commanded to the internal oscillator
        int32_t extDeltaFreq;      // Frequency increment of the external oscillator
        uint32_t extDeltaFreqUnc;     // Uncertainty of the external oscillator frequency increment
        uint32_t extRaw;       // Current raw DAC setting commanded to the external oscillator
        uint8_t checksum[2];    
});


/*!
 * TIM-HOC Message Structure
 * Host oscillator control
 * Only available with FTS product variant
 * Message Type: INPUT
 * ID: 0x0D 0x17 Payload Length = 8 bytes
 */
PACK(
    struct TimHoc{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t oscId;     // Id of oscillator:
                            // 0: internal oscillator
                            // 1:  external oscillator
        uint8_t flags;     // Flags (see graphic below)
        uint8_t reserved1;     // Reserved
        int32_t value;     // Required frequency offset or raw output, depending on the flags
        uint8_t checksum[2];    
});


/*!
 * TIM-SMEAS Message Structure
 * Source measurement
 * Only available with FTS product variant
 * Message Type: INPUT/OUTPUT
 * ID: 0x0D 0x13 Payload Length = 12 + 24*numMeas bytes
 */
PACK(
    struct TimSmeasRepeated{
        uint8_t sourceId;      // Index of source. SMEAS can provide six measurement sources. The first four sourceId values represent measurements made by the receiver and sent to the host. The first of these with a sourceId value of 0 is a measurement of the internal oscillator against the current receiver time-and-frequency estimate. The internal oscillator is being disciplined against that estimate and this result represents the current offset between the actual and desired internal oscillator states. The next three sourceId values represent frequency and time measurements made by the receiver against the internal oscillator. sourceId 1 represents the GNSS-derived frequency and time compared with the internal oscillator frequency and time. sourceId2 give measurements of a signal  coming in on EXTINT0. sourceId 3 corresponds to a similar measurement on EXTINT1. The remaining two of these measurements (sourceId 4 and 5) are made by the host and sent to the receiver. A measurement with sourceId 4 is a measurement by the host of the internal oscillator and sourceId 5 indicates a host measurement of the external oscillator.
        uint8_t flags;     // Flags (see graphic below)
        int8_t phaseOffsetFrac;     // Sub-nanosecond phase offset; the total offset is the sum of phaseOffset and phaseOffsetFrac
        uint8_t phaseUncFrac;      // Sub-nanosecond phase uncertainty
        int32_t phaseOffset;       // Phase offset, positive if the source lags accurate phase and negative if the source is early
        uint32_t phaseUnc;     // Phase uncertainty (one standard deviation)
        uint8_t reserved3[4];     // Reserved
        int32_t freqOffset;        // Frequency offset, positive if the source frequency is too high, negative if the frequency is too low.
        uint32_t freqUnc;      // Frequency uncertainty (one standard deviation)   
});
PACK(
    struct TimSmeas{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t numMeas;       // Number of measurements in repeated block
        uint8_t reserved1[2];     // Reserved
        uint32_t iTOW;     // Time of the week
        uint8_t reserved2[4];     // Reserved
        TimSmeasRepeated repeated_block[100];
        uint8_t checksum[2];    
});


/*!
 * TIM-SVIN Message Structure
 * Source measurement
 * Only available with FTS product variant
 * Message Type: PERIODIC/POLLED
 * ID: 0x0D 0x04 Payload Length = 28 bytes
 */


/*!
 * TIM-TM2 Message Structure
 * Time mark data
 * Message Type: PERIODIC/POLLED
 * ID: 0x0D 0x03 Payload Length = 28 bytes
 */


/*!
 * TIM-TOS Message Structure
 * Time Pulse Time and Frequency Data
 * Only available with FTS product variant
 * Message Type: PERIODIC
 * ID: 0x0D 0x12 Payload Length = 56 bytes
 */
PACK(
    struct TimTos{
        UbloxHeader header;
        uint8_t version;       // Message version (0 for this version)
        uint8_t gnssId;        // GNSS system used for reporting GNSS time (seeSatellite Numbering)
        uint8_t reserved1[2];     // Reserved
        uint32_t flags;        // Flags (see graphic below)
        uint16_t year;     // Year of UTC time
        uint8_t month;     // Month of UTC time
        uint8_t day;       // Day of UTC time
        uint8_t hour;      // Hour of UTC time
        uint8_t minute;        // Minute of UTC time
        uint8_t second;        // Second of UTC time
        uint8_t utcStandard;       // UTC standard identifier:
                                    // 0: unknown
                                    // 3: UTC as operated by the U.S. Naval Observatory (USNO)
                                    // 6: UTC as operated by the former Soviet Union 7: UTC as operated by the National Time Service Center, China
        int32_t utcOffset;     // Time offset between the preceding pulse and UTC top of second
        uint32_t utcUncertainty;      // Uncertainty of utcOffset
        uint32_t week;     // GNSS week number
        uint32_t TOW;      // GNSS time of week
        int32_t gnssOffset;        // Time offset between the preceding pulse and GNSS top of second
        uint32_t gnssUncertainty;     // Uncertainty of gnssOffset
        int32_t intOscOffset;      // Internal oscillator frequency offset
        uint32_t intOscUncertainty;       // Internal oscillator frequency uncertainty
        int32_t extOscOffset;      // External oscillator frequency offset
        uint32_t extOscUncertainty;       // External oscillator frequency uncertainty
        uint8_t checksum[2];    
});


/*!
 * TIM-TP Message Structure
 * Time Pulse Timedatat
 * Message Type: PERIODIC/POLLED
 * ID: 0x0D 0x01 Payload Length = 16 bytes
 */


/*!
 * TIM-VCOCAL Command Message Structure
 * VCO calibration extended command
 * Only available with FTS product variant
 * Message Type: COMMAND
 * ID: 0x0D 0x15 Payload Length = 12 bytes
 */
PACK(
    struct TimVcoCalCommand{
        UbloxHeader header;
        uint8_t type;      // Message type (2 for this message)
        uint8_t version;       // Message version (0 for this version)
        uint8_t oscId;     // Oscillator to be calibrated:
                            // 0: internal oscillator
                            // 1: external oscillator
        uint8_t srcId;     // Reference source: 0: internal oscillator
                            // 1: GNSS
                            // 2: EXTINT0
                            // 3: EXTINT1
                            // Option 0 should be used when calibrating the external oscillator. Options 1-3 should be used when calibrating the internal oscillator.
        uint8_t reserved1[2];     // Reserved
        uint16_t raw0;     // First value used for calibration
        uint16_t raw1;     // Second value used for calibration
        uint16_t maxStepSize;      // Maximum step size to be used
        uint8_t checksum[2];    
});


/*!
 * TIM-VCOCAL Results Message Structure
 * Results of the calibration
 * Only available with FTS product variant
 * Message Type: COMMAND
 * ID: 0x0D 0x15 Payload Length = 12 bytes
 */
PACK(
    struct TimVcoCalResults{
        UbloxHeader header;
        uint8_t type;      // Message type (3 for this message)
        uint8_t version;       // Message version (0 for this version)
        uint8_t oscId;     // Id of oscillator:
                            // 0: internal oscillator
                            // 1: external oscillator
        uint8_t reserved1[3];     // Reserved
        uint16_t gainUncertainty;     // Relative gain uncertainty after calibration, 0 if calibration failed
        int32_t gainVco;       // Calibrated gain or 0 if calibration failed
        uint8_t checksum[2];    
});


/*!
 * TIM-VRFY Message Structure
 * Sourced Time Verification
 * Message Type: POLLED/ONCE
 * ID: 0x0D 0x06 Payload Length = 20 bytes
 */
// TODO


///////////////////////////////////////////////////////////
// UBX-UPD
///////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
// UBX-UPD-SOS

/*!
 * UPD-SOS Poll Message Structure
 * Poll Backup File Restore Status
 * Message Type: POLL REQUEST
 * ID: 0x09 0x14 Payload Length = 0 bytes
 */
PACK(
    struct UpdSosPoll{
        UbloxHeader header;
        uint8_t checksum[2];    
});


/*!
 * UPD-SOS Create Backup Message Structure
 * Create Backup File in Flash
 * Message Type: INPUT
 * ID: 0x09 0x14 Payload Length = 4 bytes
 */
PACK(
    struct UpdSosCreateBackup{
        UbloxHeader header;
        uint8_t cmd;       // Command (must be 0)
        uint8_t reserved1[3];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * UPD-SOS Clear Backup Message Structure
 * Create Backup File in Flash
 * Message Type: INPUT
 * ID: 0x09 0x14 Payload Length = 4 bytes
 */
PACK(
    struct UpdSosClearBackup{
        UbloxHeader header;
        uint8_t cmd;       // Command (must be 1)
        uint8_t reserved1[3];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * UPD-SOS File Creation ACK Message Structure
 * Backup File Creation Acknowledge
 * Message Type: OUTPUT
 * ID: 0x09 0x14 Payload Length = 8 bytes
 */
PACK(
    struct UpdSosCreateBackupAck{
        UbloxHeader header;
        uint8_t cmd;       // Command (must be 2)
        uint8_t reserved1[3];     // Reserved
        uint8_t response;      // 0: Not acknowledged
                                // 1: Acknowledged
        uint8_t reserved2[3];     // Reserved
        uint8_t checksum[2];    
});


/*!
 * UPD-SOS File Creation ACK Message Structure
 * System Restored from Backup
 * Message Type: OUTPUT
 * ID: 0x09 0x14 Payload Length = 8 bytes
 */
PACK(
    struct UpsSosSystemRestoredFromBackup{
        UbloxHeader header;
        uint8_t cmd;       // Command (must be 3)
        uint8_t reserved1[3];     // Reserved
        uint8_t response;      // 0:  Unknown
                                // 1:  Failed restoring from backup file 2:  Restored from backup file
                                // 3:  Not restored (no backup)
        uint8_t reserved2[3];     // Reserved
        uint8_t checksum[2];    
});


enum Message_ID
{
    ACK_ACK = 0x0501,
    ACK_NAK = 0x0500,
    AID_AOP = 0x0B33,
    AID_REQ = 2816,                 // (ID 0x0B 0x00) Receiver Requests Aiding data if not present at startup
    AID_EPH = 2865,                 // (ID 0x0B 0x31) Ephemerides
    AID_ALM = 2864,                 // (ID 0x0B 0x30) Almanac
    AID_HUI = 2818,                 // (ID 0x0B 0x02) GPS Health, Ionospheric, UTC
    AID_INI = 2817,                 // (ID 0x0B 0x01) Position, Time, Frequency, Clock Drift
    CFG_PRT = 1536,                 // (ID 0x06 0x00) I/O Protocol Settings
    CFG_NAV5 = 1572,                // (ID 0x06 0x24) Navigation Algorithm Parameter Settings
    CFG_ANT = 0x0613,
    CFG_CFG = 0x0609,
    CFG_DOSC = 0x0661,
    CFG_ESRC = 0x0660,
    CFG_GNSS = 0x063E,
    CFG_INF = 0x0602,
    CFG_LOGFILTER = 0x0647,
    CFG_MSG = 0x0601,
    CFG_NAVX5 = 0x0623,
    CFG_NMEA = 0x0617,
    CFG_ODO = 0x061E,
    CFG_PM2 = 0x063B,
    CFG_RATE = 0x0608,
    CFG_RINV = 0x0634,
    CFG_RXM = 0x0611,
    CFG_SBAS = 0x0616,
    CFG_SMGR = 0x0662,
    CFG_TMODE2 = 0x063D,
    CFG_TP5 = 0x0631,
    CFG_USB = 0x061B,
    INF_DEBUG = 0x0404,
    INF_ERROR = 0x0400,
    INF_NOTICE = 0x0402,
    INF_TEST = 0x0403,
    INF_WARNING = 0x0401,
    LOG_FINDTIME = 0x210E,
    LOG_INFO = 0x2108,
    LOG_RETRIEVEPOSEXTRA = 0x210f,
    LOG_RETRIEVEPOS = 0x210b,
    LOG_RETRIEVESTRING = 0x210d,
    LOG_RETRIEVE = 0x2109,
    MGA_ACK = 0x1360,
    MGA_DBD = 0x1380,
    MGA_FLASH = 0x1321,
    MON_GNSS = 0x0A28,
    MON_HW2 = 0x0A0B,
    MON_HW = 0x0A09,
    MON_IO = 0x0A02,
    MON_MSGPP = 0x0A06,
    MON_PATCH = 0x0A27,
    MON_RXBUF = 0x0A07,
    MON_RXR = 0x0A21,
    MON_SMGR = 0x0A2E,
    MON_TXBUF = 0x0A08,
    MON_VER = 0x0A04,
    NAV_AOPSTATUS = 0x0160,
    NAV_ODO = 0x0109,
    NAV_ORB = 0x0134,
    NAV_POSECEF = 0x0101,
    NAV_PVT = 0x0107,
    NAV_SAT = 0x0135,
    NAV_SBAS = 0x0132,
    NAV_TIMEBDS = 0x0124,
    NAV_TIMEGLO = 0x0123,
    NAV_TIMEUTC = 0x0121,
    NAV_VELECEF = 0x0111,
    NAV_STATUS = 259,               // (ID 0x01 0x03) TTFF, GPS Fix type, time since startup/reset
    NAV_SOL = 262,                  // (ID 0x01 0x06) ECEF Pos,Vel, TOW, Accuracy,
    NAV_VELNED = 274,               // (ID 0x01 0x12) Vel (North, East, Down), Speed, Ground Speed
    NAV_POSLLH = 258,               // (ID 0x01 0x02) Pos (Lat,Long,Height)
    NAV_SVINFO = 304,               // (ID 0x01 0x30) Info on Channels and the SVs they're tracking
    NAV_TIMEGPS = 288,              // (ID 0x01 0x20) GPS Time
    NAV_DGPS = 305,                 // (ID 0x01 0x31) Outputs correction data used for the nav solution
    NAV_DOP = 260,                  // (ID 0x01 0x04) Various Dilution of Precisions
    NAV_UTCTIME = 289,              // (ID 0x01 0x21) UTC Time
    NAV_CLOCK = 290,                  // (ID 0x01 0x22) Clock information
    RXM_RAWX = 0x0215,
    RXM_SFRBX = 0x0213,
    RXM_SVSI = 544,                 // (ID 0x02 0x20) SV Status Info
    TIM_DOSC = 0x0D11,
    TIM_FCHG = 0x0D16,
    TIM_SMEAS = 0x0D13,
    TIM_SVIN = 0x0D04,
    TIM_TM2 = 0x0D03,
    TIM_TOS = 0x0D12,
    TIM_TP = 0x0D01,
    TIM_VCOCAL = 0x0D15,
    TIM_VRFY = 0x0D06,
};

} // end namespace
#endif
