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
#include <string>

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
    #define Sleep(x) usleep(x * 1000)
    #endif

    #ifndef EZ6_B2_POINT_CLOUD_LEN  
    #define EZ6_B2_POINT_CLOUD_LEN 1058
    #endif

    #ifndef NZ1_A2_POINT_CLOUD_LEN  
    #define NZ1_A2_POINT_CLOUD_LEN 1132
    #endif

    #ifndef NZ5_B5_POINT_CLOUD_LEN  
    #define NZ5_B5_POINT_CLOUD_LEN 1068
    #endif

    #ifndef NZ1_A2_TRANS_LEN  
    #define NZ1_A2_TRANS_LEN 5794
    #endif

    #ifndef NZ5_B5_TRANS_LEN  
    #define NZ5_B5_TRANS_LEN 11554
    #endif

    #ifndef NZ1_A2_ANGLE_LEN  
    #define NZ1_A2_ANGLE_LEN 1314
    #endif

    #ifndef NZ1_A2_IMU_LEN
    #define NZ1_A2_IMU_LEN  68
    #endif 

    #ifndef NZ1_A2_ANGLE_SIZE 
    #define NZ1_A2_ANGLE_SIZE 46080 
    #endif

    #ifndef NZ5_B5_ANGLE_SIZE 
    #define NZ5_B5_ANGLE_SIZE 92160 
    #endif

    #ifndef NZ5_XPRO_ANGLE_SIZE 
    #define NZ5_XPRO_ANGLE_SIZE 53760 
    #endif

    // MRZ16 (EE FF protocol) on-wire frame lengths
    #ifndef MRZ16_POINT_CLOUD_LEN
    #define MRZ16_POINT_CLOUD_LEN 80
    #endif

    #ifndef MRZ16_IMU_LEN
    #define MRZ16_IMU_LEN 34
    #endif

    #ifndef MRZ16_CHANNEL_COUNT
    #define MRZ16_CHANNEL_COUNT 16
    #endif

    const float SPEED_US = 0.299710218 / 2;

    typedef struct FirmwareVersion
    {
        std::string kernel_version;
        std::string boot_version;
    } FirmwareVersion;

    typedef enum DeviceType {
      LidarEZ6_B2,
      LidarNZ1_A2,
      LidarMRZ16,
      LidarUnknown,
    } DeviceType;

    typedef enum ScanMode:uint8_t
    {
        ScanEZ6_B2_192,
        ScanNZ1_A2_96,
        ScanUnknown,
    }ScanMode;

    typedef struct DeviceConfigurationInfo
    {
        DeviceType device;
        std::string serial_number;
        std::string factory_mac;
        FirmwareVersion version;
        FirmwareVersion backup_version;

		std::string config_mac;
        std::string device_mac;
        std::string device_ip;
        std::string subnet_mask;
        
        std::string destination_ip;
        int destination_port;
		std::string gateway_addr;

        int frame_switch;
        int frame_offset;

        int work_mode;
    } DeviceConfigurationInfo;

    using CalibrationPackets = std::vector<std::string>;

    typedef struct angle_comp
    {
        DeviceType device_type;
        ScanMode scan_mode;
        std::string description;

        std::vector<float> azi;
        std::vector<float> ele;
    }angle_comp_t;

    typedef struct Point
    {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float distance = 0.0;
        int reflectivity = -1;       // [0-255]
        int reflectivity_13bits = -1;// 13bits value
        int fov = -1;                // [0,3) for ML30B1, [0-8) for ML30S(A/B)
        int point_number = -1;       // [0, max fires) for single echo, [0, max fires x 2] for dual echo
        int fire_number = -1;        // [0, max fires)
        int groupid = -1;            // fovs group id
        int line_id = -1;            // scan line id
        int valid = 0;               // if this points is resolved in udp packet, this points is valid
        int echo_num = 0;            // 0 for first, 1 for second
        float ele = 0;               // ele (rad)
        float azi = 0;               // azi (rad)
        uint16_t channel = 0;        // channel id
        uint64_t timestamp_us = 0;   // micro second. UTC time for PTP mode

        int idx = -1;
        int row;
        int col;
        int area;
        bool retro_flag = false;
        int bom_num = -1;
        int sea_num = -1;
        int retro_num = -1;

        float azimuth = 0.0;
        float elevation = 0.0;

        int pointid = 0;
        uint8_t mirrornum = 0;

        uint16_t rowCnt = 0;
        uint16_t colCnt = 0;
        int dirtyValue = 0;
        bool dirty_ = 0;
    }Point;

    typedef struct imu_data
    {
        float acc_x = 0.0;      //unit: m/s^2
        float acc_y = 0.0;    
        float acc_z = 0.0;

        float gyro_x = 0.0;     //unit: deg/s
        float gyro_y = 0.0;
        float gyro_z = 0.0;

        uint64_t timestamp_us = 0;
    }imu_data_t;

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

	/*****************************************************************/

    /** \brief Get sdk version
    * \param[in] tp      the DeviceType
    * \return string.
    */
    std::string get_sdk_version_string();

    /** \brief DeviceType to string
    * \param[in] tp      the DeviceType
    * \return string.
    */
    std::string get_device_type_string(DeviceType tp);

    /** \brief ReturnCode to string
    * \param[in] tp      the ReturnCode
    * \return string.
    */
    std::string get_return_code_string(ReturnCode tp);
}

#endif //end DEFINE_H_
