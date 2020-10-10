// MIT License
//
// Copyright(c) 2019 ZVISION. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef DEFINE_H_
#define DEFINE_H_

#include <memory>
#if defined _WIN32
#include <windows.h>
#include <winsock.h>
#else
#define closesocket close
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <vector>

namespace zvision
{
#ifdef _WIN32

#else
    #define sscanf_s sscanf
    #define sprintf_s sprintf
#endif

#ifndef POINT_CLOUD_UDP_LEN
#define POINT_CLOUD_UDP_LEN 1304
#endif

    typedef struct FirmwareVersion
    {
        unsigned char kernel_version[4];
        unsigned char boot_version[4];
    } FirmwareVersion;

    typedef enum TimestampType
    {
        TimestampPtp = 0,
        TimestampPpsGps,
        TimestampUnknown,
    }TimestampType;

    typedef enum RetroMode
    {
        RetroDisable = 0,
        RetroEnable = 1,
        RetroUnknown,
    }RetroMode;

    typedef enum EchoMode
    {
        EchoSingleFirst = 0, //First echo
        EchoSingleStrongest = 1, //Strongest echo
        EchoSingleLast = 2, //Last echo
        EchoDoubleFirstStrongest = 3, //First and strongest
        EchoDoubleFirstLast = 4, //First and last
        EchoDoubleStrongestLast = 5, //Strongest and last
        EchoUnknown,
    }EchoMode;

    typedef enum DeviceType
    {
        LidarML30B1,
        LidarML30SA1,
        LidarMLX,
        LidarUnknown,
    }DeviceType;

    typedef struct DeviceConfigurationInfo
    {
        DeviceType device;
        std::string serial_number;
        FirmwareVersion version;

        std::string device_mac;
        std::string device_ip;
        std::string subnet_mask;
        
        std::string destination_ip;
        int destination_port;

        TimestampType time_sync;
        RetroMode retro_enable;

        uint32_t phase_offset; // 5ns
        EchoMode echo_mode;

    } DeviceConfigurationInfo;

    typedef struct CalibrationData
    {
        DeviceType device_type;

		/** \brief Store every point's calibration data in azimuth elevation point by point.
		* For the order's example
		* F0-P0-ath F0-P0-ele F1-P0-ath F1-P0-ele F2-P0-ath F2-P0-ele
		* F0-P1-ath F0-P1-ele F1-P1-ath F1-P1-ele F2-P1-ath F2-P0-ele
		* ...........................................................
		* F0-Pn-ath F0-Pn-ele F1-Pn-ath F1-Pn-ele F2-Pn-ath F2-Pn-ele
		*/
        std::vector<float> data;
    }CalibrationData;

    typedef struct CalibrationDataSinCos
    {
        float sin_ele;
        float cos_ele;
        float sin_azi;
        float cos_azi;

    }CalibrationDataSinCos;

    typedef struct CalibrationDataSinCosTable
    {
        DeviceType device_type;

		/** \brief Store every point's calibration data in the format of sin-cos point by point.
		* The data is orderd by points' order in the udp package.
		* For the order's example
		* P0
		* P1
		* ...
		* Pn
		*/
        std::vector<CalibrationDataSinCos> data;
    }CalibrationDataSinCosTable;

    typedef struct Point
    {
        float x = 0;
        float y = 0;
        float z = 0;
        int reflectivity;
        int fov;
        int number;
        int valid = 0;
        double timestamp;
    }Point;

    /** \brief Set of return code. */
    typedef enum ReturnCode
    {
        Success,
        Failure,
        Timeout,

        InvalidParameter,
        NotSupport,

        InitSuccess,
        InitFailure,
        NotInit,

        OpenFileError,
        ReadFileError,
        InvalidContent,
        EndOfFile,

        NotMatched,
        BufferOverflow,

        Unknown,

    }ReturnCode;

    /* 4 bytes IP address */
    typedef int ip_address;

    /* IPv4 header */
    typedef struct ip_header {
        u_char  ver_ihl;        // Version (4 bits) + Internet header length (4 bits)
        u_char  tos;            // Type of service 
        u_short tlen;           // Total length 
        u_short identification; // Identification
        u_short flags_fo;       // Flags (3 bits) + Fragment offset (13 bits)
        u_char  ttl;            // Time to live
        u_char  proto;          // Protocol
        u_short crc;            // Header checksum
        ip_address  saddr;      // Source address
        ip_address  daddr;      // Destination address
        u_int   op_pad;         // Option + Padding
    }ip_header;

    /* UDP header*/
    typedef struct udp_header {
        u_short sport;          // Source port
        u_short dport;          // Destination port
        u_short len;            // Datagram length
        u_short crc;            // Checksum
    }udp_header;

    /* pcap packet header*/
    typedef struct pcap_packet_header {
        u_short sport;          // Source port
        u_short dport;          // Destination port
        u_short len;            // Datagram length
        u_short crc;            // Checksum
    }pcap_packet_header;

    /** \brief DeviceType to string
    * \param[in] tp      the DeviceType
    * \return string.
    */
    std::string get_device_type_string(DeviceType tp);

    /** \brief TimestampType to string
    * \param[in] tp      the TimestampType
    * \return string.
    */
    std::string get_time_sync_type_string(TimestampType tp);

    /** \brief RetroMode to string
    * \param[in] tp      the RetroMode
    * \return string.
    */
    std::string get_retro_mode_string(RetroMode tp);

    /** \brief ReturnCode to string
    * \param[in] tp      the ReturnCode
    * \return string.
    */
    std::string get_return_code_string(ReturnCode tp);

    /** \brief Echo mode to string
    * \param[in] mode    the EchoMode
    * \return string.
    */
    std::string get_echo_mode_string(EchoMode mode);

    /** \brief config info to string
    * \param[in] info    the DeviceConfigurationInfo
    * \return string.
    */
    std::string get_cfg_info_string(DeviceConfigurationInfo& info);
}

#endif //end DEFINE_H_