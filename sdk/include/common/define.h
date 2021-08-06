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

    typedef enum PhaseOffsetMode
    {
        PhaseOffsetDisable = 0,
        PhaseOffsetEnable = 1,
        PhaseOffsetUnknown,
    }PhaseOffsetMode;

    typedef enum ReturnType
    {
        FirstReturn = 1, //First return
        LastReturn = 2, //Last return
        StrongestReturn = 3, //Strongest return
        SecondStrongestReturn = 4, //Second strongest return
        UnknownReturn,
    }ReturnType;

    typedef enum EchoMode
    {
        EchoSingleFirst = 1, //First echo
        EchoSingleStrongest = 2, //Strongest echo
        EchoSingleLast = 3, //Last echo
        EchoDoubleFirstStrongest = 4, //First and strongest
        EchoDoubleFirstLast = 5, //First and last
        EchoDoubleStrongestLast = 6, //Strongest and last
        EchoUnknown,
    }EchoMode;

    typedef enum CalSendMode
    {
        CalSendDisable = 0,
        CalSendEnable = 1,
        CalSendUnknown,
    }CalSendMode;

    typedef enum DownsampleMode
    {
        DownsampleNone = 0, // No downsample
        Downsample_1_2 = 1, // 1/2 downsample
        Downsample_1_4 = 2, // 1/4 downsample
        DownsampleUnknown,
    }DownSampleMode;

    typedef enum DeviceType
    {
        LidarML30B1,
        LidarML30SA1,
        LidarML30SA1_2,
        LidarML30SB1,
        LidarML30SB2,
        LidarMLX,
        LidarMLYA,
        LidarMLYB,
        LidarUnknown,
    }DeviceType;

    typedef enum ScanMode
    {
        ScanML30B1_100,
        ScanML30SA1_160,
        ScanML30SA1_160_1_2,
        ScanML30SA1_160_1_4,
        ScanML30SA1_190,
        ScanMLX_160,
        ScanMLX_190,
        ScanMLXS_180,
        ScanMLYA_190,
        ScanMLYB_190,
        ScanUnknown,
    }ScanMode;

    typedef struct DeviceConfigurationInfo
    {
        DeviceType device;
        std::string serial_number;
        FirmwareVersion version;
        FirmwareVersion backup_version;

        std::string device_mac;
        std::string device_ip;
        std::string subnet_mask;
        
        std::string destination_ip;
        int destination_port;

        TimestampType time_sync;
        RetroMode retro_enable;

        uint32_t phase_offset; // 5ns
        PhaseOffsetMode phase_offset_mode;
        EchoMode echo_mode;

        CalSendMode cal_send_mode;
        DownsampleMode downsample_mode;

        int retro_param_1_ref_min; // 0-255
        int retro_param_2_point_percent; // 0-100

    } DeviceConfigurationInfo;

    using CalibrationPackets = std::vector<std::string>;

    typedef struct CalibrationData
    {
        DeviceType device_type;
        ScanMode scan_mode;
        std::string description;

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
        float ele;
        float azi;
        float sin_ele;
        float cos_ele;
        float sin_azi;
        float cos_azi;

    }CalibrationDataSinCos;

    typedef struct CalibrationDataSinCosTable
    {
        DeviceType device_type;
        ScanMode scan_mode;
        std::string description;

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
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        int reflectivity = -1;       // [0-255]
        int fov = -1;                // [0,3) for ML30B1, [0-8) for ML30S(A/B)
        int point_number = -1;       // [0, max fires) for single echo, [0, max fires x 2] for dual echo
        int fire_number = -1;        // [0, max fires)
        int valid = 0;               // if this points is resolved in udp packet, this points is valid
        int echo_num = 0;            // 0 for first, 1 for second
        ReturnType return_type = ReturnType::UnknownReturn;
        uint64_t timestamp_ns = 0;   // nano second. UTC time for GPS mode, others for PTP mode
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
        NoEnoughResource,

        NotEnoughData,

        ItemNotFound,

        TcpConnTimeout,
        TcpSendTimeout,
        TcpRecvTimeout,

        DevAckError,

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

    /** \brief ScanMode to type string
    * \param[in] sm      the ScanMode
    * \return string.
    */
    std::string get_device_type_string_by_mode(ScanMode sm);

    /** \brief DeviceType to string
    * \param[in] sm      the ScanMode
    * \return string.
    */
    std::string get_scan_mode_string(ScanMode sm);

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

    /** \brief phase offset mode to string
    * \param[in] mode    the PhaseOffsetMode
    * \return string.
    */
    std::string get_phase_offset_mode_string(PhaseOffsetMode mode);

    /** \brief calibration send mode to string
    * \param[in] mode    the CalSendMode
    * \return string.
    */
    std::string get_cal_send_mode_string(CalSendMode mode);

    /** \brief downsample mode to string
    * \param[in] mode    the DownsampleMode
    * \return string.
    */
    std::string get_downsample_mode_string(DownsampleMode mode);
}

#endif //end DEFINE_H_