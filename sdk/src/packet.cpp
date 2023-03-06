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


#include "define.h"
#include "packet.h"
#include "point_cloud.h"
#include <cstring>
#ifdef WIN32
#define timegm _mkgmtime
#endif

namespace zvision
{

    bool MarkedPacket::IsValidMarkedPacket(std::string& pkt)
    {
        std::string tag = zvision::get_lidar_frame_mark_string();
        if (tag.size() != pkt.size())
            return false;

        return tag.compare(0, pkt.size(), pkt) == 0;
    }

    bool CalibrationPacket::IsValidPacket(std::string& packet)
    {
        if (PACKET_LEN != packet.size())
            return false;
        return ((packet[0] == 'C') && (packet[1] == 'A') && (packet[2] == 'L'));
    }

    ScanMode CalibrationPacket::GetScanMode(std::string& packet)
    {
        std::string dev_code(packet.c_str() + 3, 6);
        if (0 == dev_code.compare("30_B1 "))
        {
            return ScanMode::ScanML30B1_100;
        }
        else if (0 == dev_code.compare("30S_A1"))
        {
            unsigned char downsample_flag = packet[13];
            if(0x00 == downsample_flag)
                return ScanMode::ScanML30SA1_160;
            else if(0x01 == downsample_flag)
                return ScanMode::ScanML30SA1_160_1_2;
            else if(0x02 == downsample_flag)
                return ScanMode::ScanML30SA1_160_1_4;
            else
                return ScanMode::ScanML30SA1_160;
        }
        else if (0 == dev_code.compare("30S_B1"))
        {
            return ScanMode::ScanML30SA1_160;
        }
        else if (0 == dev_code.compare("MLX   "))
        {
            return ScanMode::ScanMLX_160;
        }
        else if (0 == dev_code.compare("XS    "))
        {
            return ScanMode::ScanMLXS_180;
        }
		else if (0 == dev_code.compare("30S+A1"))
		{
			//unsigned char downsample_flag = packet[13];
			//if (0x00 == downsample_flag)
			//	return ScanMode::ScanML30SA1Plus_160;
			//else if (0x01 == downsample_flag)
			//	return ScanMode::ScanML30SA1Plus_160_1_2;
			//else if (0x02 == downsample_flag)
			//	return ScanMode::ScanML30SA1Plus_160_1_4;
			//else
				return ScanMode::ScanML30SA1Plus_160;
		}
        else
        {
            return ScanMode::ScanUnknown;
        }
    }

    int CalibrationPacket::GetPacketSeq(std::string& packet)
    {
        unsigned char* cal_data_ = (unsigned char*)packet.c_str();
        uint16_t* seq = (uint16_t*)(cal_data_ + 9);
        return ntohs(*seq);;
    }

    int CalibrationPacket::GetMaxSeq(zvision::ScanMode& mode)
    {
        int packets = 0;
        switch (mode)
        {
        case ScanMode::ScanML30B1_100:
            packets = 235;// 10000 * 3 * 2 * 4 / 1024
            break;
        case ScanMode::ScanML30SA1_160:
		case ScanMode::ScanML30SA1Plus_160:
            packets = 400;// 6400 * 8 * 2 * 4 / 1024
            break;
        case ScanMode::ScanML30SA1_160_1_2:
		case ScanMode::ScanML30SA1Plus_160_1_2:
            packets = 200;// 3200 * 8 * 2 * 4 / 1024
            break;
        case ScanMode::ScanML30SA1_160_1_4:
		case ScanMode::ScanML30SA1Plus_160_1_4:
            packets = 100;// 1600 * 8 * 2 * 4 / 1024
            break;
        case ScanMode::ScanML30SA1_190:
            packets = 450;// 7200 * 8 * 2 * 4 / 1024
            break;
        case ScanMode::ScanMLX_160:
            packets = 750;// 32000 *3 * 2 * 4 / 1024
            break;
        case ScanMode::ScanMLXS_180:
            packets = 844;// 36000 *3 * 2 * 4 / 1024
            break;
        default:
            break;
        }
        return packets - 1;
    }

    DeviceType CalibrationPacket::GetDeviceType()
    {
        std::string dev_code(cal_data_ + 3, 6);
        if (0 == dev_code.compare("30_B1 "))
        {
            return DeviceType::LidarML30B1;
        }
        else if ((0 == dev_code.compare("30S_A1")) || (0 == dev_code.compare("30S_B1")))
        {
            return DeviceType::LidarML30SA1;
        }
        else if (0 == dev_code.compare("MLX   "))
        {
            return DeviceType::LidarMLX;
        }
        else
        {
            return DeviceType::LidarUnknown;
        }
    }

    int CalibrationPacket::GetPacketSeq()
    {
        return (unsigned char)(this->cal_data_[9]) + (((unsigned char)cal_data_[10] & 0xF) << 8);
    }

    bool PointCloudPacket::IsValidPacket(std::string& packet)
    {
        if (PACKET_LEN != packet.size())
            return false;
        uint16_t flag = *(uint16_t*)packet.c_str();
        return ((flag == 0xAAAA) || (flag == 0xBBBB) || (flag == 0xCCCC));
    }

    DeviceType PointCloudPacket::GetDeviceType(std::string& packet)
    {
        if (packet.size() != POINT_CLOUD_UDP_LEN)
        {
            return LidarUnknown;
        }

        unsigned char value = packet[1303];
        if (0xFF == value)
        {
            return LidarML30B1;
        }
        else if ((0x04 == value) || (0x80 == value))
        {
            return LidarML30SA1;
        }
        else if (0x05 == value)
        {
            return LidarMLX;
        }
        else if (0x06 == value)
        {
            return LidarMLYA;
        }
        else if (0x07 == value)
        {
            return LidarMLYB;
        }
        else
        {
            return LidarUnknown;
        }
    }

    ScanMode PointCloudPacket::GetScanMode(std::string& packet)
    {
        if (packet.size() != POINT_CLOUD_UDP_LEN)
        {
            return ScanMode::ScanUnknown;
        }

        unsigned char type_field = packet[POINT_CLOUD_UDP_LEN - 1];

        if (0xFF == type_field)
        {
            return ScanMode::ScanML30B1_100;
        }
        else if ((0x04 == type_field) || (0x08 == type_field))
        {
            return ScanMode::ScanML30SA1_160;
        }
        else if (0x80 == type_field)
        {
            unsigned char downsample_flag = packet[POINT_CLOUD_UDP_LEN - 2];
            if (0x00 == downsample_flag)
                return ScanMode::ScanML30SA1_160;
            else if (0x01 == downsample_flag)
                return ScanMode::ScanML30SA1_160_1_2;
            else if (0x02 == downsample_flag)
                return ScanMode::ScanML30SA1_160_1_4;
            else
                return ScanMode::ScanML30SA1_160;
        }
        else if ((0x09 == type_field) || (0x0a == type_field))
        {
            return ScanMode::ScanML30SA1_190;
		}
		else if ((0x0b == type_field) || (0x0c == type_field)) {
			//unsigned char downsample_flag = packet[POINT_CLOUD_UDP_LEN - 2];
			//if (0x00 == downsample_flag)
			//	return ScanMode::ScanML30SA1Plus_160;
			//else if (0x01 == downsample_flag)
			//	return ScanMode::ScanML30SA1Plus_160_1_2;
			//else if (0x02 == downsample_flag)
			//	return ScanMode::ScanML30SA1Plus_160_1_4;
			//else
				return ScanMode::ScanML30SA1Plus_160;
		}
        else if (0x05 == type_field)
        {
            //return ScanMode::ScanMLX_190;
            return ScanMode::ScanMLXS_180;
        }
        else if (0x06 == type_field)
        {
            return ScanMode::ScanMLYA_190;
        }
        else if (0x07 == type_field)
        {
            return ScanMode::ScanMLYB_190;
        }
        else
        {
            return ScanMode::ScanUnknown;
        }
    }

    int PointCloudPacket::GetPacketSeq(std::string& packet)
    {
        return (unsigned char)(packet[3]) + (((unsigned char)packet[2] & 0xF) << 8);
    }

	int PointCloudPacket::GetPacketCount(std::string& packet) {

		int cnt = -1;
		zvision::ScanMode scan_mode = zvision::PointCloudPacket::GetScanMode(packet);
		if (scan_mode == ScanMode::ScanUnknown)
			return cnt;

		int echo_cnt = (packet[2] >> 6) & 0x3;
		switch (scan_mode)
		{
        case ScanMode::ScanML30B1_100: {
            cnt = 125 * echo_cnt;       // 125 = 30000/(80*3)
        }break;
		case ScanMode::ScanMLX_160: {
			cnt = 400 * echo_cnt;	    // 400 = 96000/(80*3)
		}break;
		case ScanMode::ScanML30SA1_160:
        case ScanMode::ScanML30SA1Plus_160:{
			cnt = 160 * echo_cnt;	    // 160 = 51200/(320 points in one udp)
		}break;
		case ScanMode::ScanML30SA1_160_1_2:
        case ScanMode::ScanML30SA1Plus_160_1_2: {
			cnt = 80 * echo_cnt;		// 80 = 51200/2/(320 points in one udp)
		}break;
		case ScanMode::ScanML30SA1_160_1_4:
        case ScanMode::ScanML30SA1Plus_160_1_4: {
			cnt = 40 * echo_cnt;		// 40 = 51200/4/(320 points in one udp)
		}break;
		case ScanMode::ScanML30SA1_190: {
			cnt = 190 * echo_cnt;		// 190 = 60800/(320 points in one udp)
		}break;
		case ScanMode::ScanMLX_190: {
			cnt = 475 * echo_cnt;		// 475 = 114000/(80*3)
		}break;
		case ScanMode::ScanMLYA_190: {
			cnt = 475 * echo_cnt;		// 475 = 114000/(80*3)
		}break;
		case ScanMode::ScanMLYB_190: {
			cnt = 60 * echo_cnt;		// 60 = 14400/(80*3)
		}break;
		case ScanMode::ScanMLXS_180: {
			cnt = 450 * echo_cnt;		// 450 = 108000/(80*3)
		}break;

		default:
			break;
		}

		return cnt;
	}

    int PointCloudPacket::GetEchoCount(std::string& packet)
    {
        return (packet[2] >> 6) & 0x3;
    }

    double PointCloudPacket::GetTimestamp(std::string& packet)
    {
        unsigned char* data = (unsigned char*)(packet.data());

        if (data[2] & 0x20) // ptp mode
        {
            uint64_t seconds = 0;
            seconds += ((uint64_t)data[1304 - 20 + 0] << 40);
            seconds += ((uint64_t)data[1304 - 20 + 1] << 32);
            seconds += ((uint64_t)data[1304 - 20 + 2] << 24);
            seconds += ((uint64_t)data[1304 - 20 + 3] << 16);
            seconds += ((uint64_t)data[1304 - 20 + 4] << 8);
            seconds += ((uint64_t)data[1304 - 20 + 5]);

            uint32_t ms = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
            uint32_t us = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

            return (double)seconds + (double)ms / 1000.0 + (double)us / 1000000.0;
        }
        else// GPS mode
        {
            struct tm tm_;
            tm_.tm_year = data[1304 - 20 + 0];
            tm_.tm_mon = data[1304 - 20 + 1] - 1;
            tm_.tm_mday = data[1304 - 20 + 2];
            tm_.tm_hour = data[1304 - 20 + 3];
            tm_.tm_min = data[1304 - 20 + 4];
            tm_.tm_sec = data[1304 - 20 + 5];
            tm_.tm_isdst = 0;

            time_t seconds = timegm(&tm_);
            uint32_t ms = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
            uint32_t us = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

            return (double)seconds + (double)ms / 1000.0 + (double)us / 1000000.0;
        }
    }

    uint64_t PointCloudPacket::GetTimestampNS(std::string& packet)
    {
        unsigned char* data = (unsigned char*)(packet.data());

        if (data[2] & 0x20) // ptp mode
        {
            uint64_t seconds = 0;
            seconds += ((uint64_t)data[1304 - 20 + 0] << 40);
            seconds += ((uint64_t)data[1304 - 20 + 1] << 32);
            seconds += ((uint64_t)data[1304 - 20 + 2] << 24);
            seconds += ((uint64_t)data[1304 - 20 + 3] << 16);
            seconds += ((uint64_t)data[1304 - 20 + 4] << 8);
            seconds += ((uint64_t)data[1304 - 20 + 5]);

            uint32_t ms = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
            uint32_t us = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

            return seconds * 1000000000 + ms * 1000000 + us * 1000;
        }
        else// GPS mode
        {
            struct tm tm_;
            tm_.tm_year = data[1304 - 20 + 0];
            tm_.tm_mon = data[1304 - 20 + 1] - 1;
            tm_.tm_mday = data[1304 - 20 + 2];
            tm_.tm_hour = data[1304 - 20 + 3];
            tm_.tm_min = data[1304 - 20 + 4];
            tm_.tm_sec = data[1304 - 20 + 5];
            tm_.tm_isdst = 0;

            time_t seconds = timegm(&tm_);
            uint32_t ms = (int)(data[1304 - 20 + 6] << 8) + data[1304 - 20 + 7];
            uint32_t us = (int)(data[1304 - 20 + 8] << 8) + data[1304 - 20 + 9];

            return seconds * 1000000000 + ms * 1000000 + us * 1000;
        }
    }

    int PointCloudPacket::ProcessPacket(std::string& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud, LidarPointsFilter* filter)
    {
        unsigned char* data = const_cast<unsigned char*>((unsigned char*)packet.c_str());

        //pkt content len: 42 + 1304
        if (packet.size() != 1304)
        {
            return InvalidContent;
        }

        //find lidar type
        ScanMode scan_mode = PointCloudPacket::GetScanMode(packet);
        //type = DeviceType::LidarMLYB;

        if (scan_mode != cal_lut.scan_mode)
        {
            //std::cout << "Scan mode is " << scan_mode << " calibration mode is " << cal_lut.scan_mode << std::endl;
            return NotMatched;
        }

        //device is unknown or device type and calibration data are not matched
        if (scan_mode == ScanUnknown)
            return NotSupport;

        //udp seq
        int seq = PointCloudPacket::GetPacketSeq(packet);

        //echo mode
        int echo_cnt = PointCloudPacket::GetEchoCount(packet);

        //timestamp
        uint64_t udp_timestamp = PointCloudPacket::GetTimestampNS(packet);

        //fire in one swap for azimuth and elevation index
        int fires = 0;

        //UDP packet parameters
        unsigned int groups_in_one_udp = 0;
        unsigned int points_in_one_group = 0;
        unsigned int point_position_in_group = 0;
        unsigned int group_len = 0;
        unsigned int point_len = 4;
        unsigned int group_position_in_packet = 4;
        int distance_bit = 19;
        int reflectivity_bit = 13;

        // fov index in one group
        int fov_index_ml30b1_single_echo[3] = { 0, 1, 2 };
        int fov_index_ml30b1_dual_echo[6] = { 0, 1, 2 };
        int fov_index_ml30sa1_single_echo[8] = { 0, 6, 1, 7, 2, 4, 3, 5 };
        int fov_index_ml30sa1_dual_echo[16] = { 0, 6, 0, 6, 1, 7, 1, 7, 2, 4, 2, 4, 3, 5, 3, 5 };
        int fov_index_mlx_single_echo[3] = { 2, 1, 0 };
        int fov_index_mlx_dual_echo[3] = { 2, 1, 0 };
		int fov_index_ml30sa1_plus_H_single_echo[4] = { 0, 1, 2, 3 };
		int fov_index_ml30sa1_plus_L_single_echo[4] = { 4, 5, 6, 7 };
		int fov_index_ml30sa1_plus_H_dual_echo[8] = { 0, 0, 1, 1, 2, 2, 3, 3 };
		int fov_index_ml30sa1_plus_L_dual_echo[8] = { 4, 4, 5, 5, 6, 6, 7, 7 };
        int fov_index_ml30sa1_plus_ep_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int fov_index_ml30sa1_plus_ep_dual_echo[16] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7 };

        int *fov_index = fov_index_ml30sa1_single_echo;

        // fire index in one group
        int fire_index_ml30b1_single_echo[3] = { 0, 1, 2 };
        int fire_index_ml30b1_dual_echo[6] = { 0, 1, 2 };
        int fire_index_ml30sa1_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int fire_index_ml30sa1_dual_echo[16] = { 0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7 };
        int fire_index_mlx_single_echo[3] = { 0, 1, 2 };
        int fire_index_mlx_dual_echo[3] = { 0, 1, 2 };
		int fire_index_ml30sa1_plus_single_echo[4] = { 0, 1, 2, 3 };
		int fire_index_ml30sa1_plus_dual_echo[8] = { 0, 0, 1, 1, 2, 2, 3, 3 };
        int fire_index_ml30sa1_plus_ep_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int fire_index_ml30sa1_plus_ep_dual_echo[16] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7 };
        int *fire_index = fire_index_ml30sa1_single_echo;

        // point number index in one group
        int number_index_ml30b1_single_echo[3] = { 0, 1, 2 };
        int number_index_ml30b1_dual_echo[6] = { 0, 1, 2 };
        int number_index_ml30sa1_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int number_index_ml30sa1_dual_echo[16] = { 0, 2, 1, 3, 4, 6, 5, 7, 8, 10, 9, 11, 12, 14, 13, 15 };
        int number_index_mlx_single_echo[3] = { 0, 1, 2 };
        int number_index_mlx_dual_echo[3] = { 0, 1, 2 };
		int number_index_ml30sa1_plus_single_echo[4] = { 0, 1, 2, 3 };
		int number_index_ml30sa1_plus_dual_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int number_index_ml30sa1_plus_ep_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int number_index_ml30sa1_plus_ep_dual_echo[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
        int *number_index = number_index_ml30sa1_single_echo;

        //fire interval
        double fire_interval_us = 0.0;

        if (ScanML30B1_100 == scan_mode)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 4;
            group_len = 16;
            fires = 30000;
            fov_index = fov_index_ml30b1_single_echo;
            fire_index = fire_index_ml30b1_single_echo;
            number_index = number_index_ml30b1_single_echo;
            fire_interval_us = 1.5625;
            if (2 == echo_cnt)
            {
                fov_index = fov_index_ml30b1_dual_echo;
                fire_index = fire_index_ml30b1_dual_echo;
                number_index = number_index_ml30b1_dual_echo;
            }
        }
        else if ((ScanML30SA1_160 == scan_mode) || (ScanML30SA1_190 == scan_mode) || (ScanML30SA1_160_1_2 == scan_mode) || (ScanML30SA1_160_1_4 == scan_mode))
        {
            if (ScanML30SA1_160 == scan_mode)
            {
                fires = 51200;
                fire_interval_us = 1.5625 / 2.0; // 0.00000078125
            }
            else if (ScanML30SA1_160_1_2 == scan_mode)
            {
                fires = 51200 / 2;
                fire_interval_us = 1.5625 / 2.0 / 2.0; // 0.000000390625
            }
            else if (ScanML30SA1_160_1_4 == scan_mode)
            {
                fires = 51200 / 4;
                fire_interval_us = 1.5625 / 2.0; // not smooth
            }
            else // 190 lines
            {
                fires = 60800;
                fire_interval_us = 1.5625 / 2.0; // 0.00000078125
            }

            if (1 == echo_cnt)
            {
                groups_in_one_udp = 40;
                points_in_one_group = 8;
                point_position_in_group = 0;
                group_len = 32;
                fov_index = fov_index_ml30sa1_single_echo;
                fire_index = fire_index_ml30sa1_single_echo;
                number_index = number_index_ml30sa1_single_echo;
            }
            else
            {
                groups_in_one_udp = 20;
                points_in_one_group = 16;
                point_position_in_group = 0;
                group_len = 64;
                fov_index = fov_index_ml30sa1_dual_echo;
                fire_index = fire_index_ml30sa1_dual_echo;
                number_index = number_index_ml30sa1_dual_echo;
            }
        }
        else if (ScanMLX_190 == scan_mode)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 0;
            group_len = 16;
            fires = 114000;
            fire_interval_us = 2.5 / 3.0; // 0.00000083333
            fov_index = fov_index_mlx_single_echo;
            fire_index = fire_index_mlx_single_echo;
            number_index = number_index_mlx_single_echo;
            if (2 == echo_cnt)
            {
                fov_index = fov_index_mlx_dual_echo;
                fire_index = fire_index_mlx_dual_echo;
                number_index = number_index_mlx_dual_echo;
            }
        }
        else if (ScanMLYA_190 == scan_mode)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 4;
            group_len = 16;
            fires = 114000;
            fov_index = fov_index_ml30b1_single_echo;
            fire_index = fire_index_ml30b1_single_echo;
            number_index = number_index_ml30b1_single_echo;
            distance_bit = 19;
            reflectivity_bit = 10;
            if (2 == echo_cnt)
            {
                fov_index = fov_index_ml30b1_dual_echo;
                fire_index = fire_index_ml30b1_dual_echo;
                number_index = number_index_ml30b1_dual_echo;
            }
        }
        else if (ScanMLYB_190 == scan_mode)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 4;
            group_len = 16;
            fires = 14400;
            fov_index = fov_index_ml30b1_single_echo;
            fire_index = fire_index_ml30b1_single_echo;
            number_index = number_index_ml30b1_single_echo;
            distance_bit = 22;
            reflectivity_bit = 10;
            if (2 == echo_cnt)
            {
                fov_index = fov_index_ml30b1_dual_echo;
                fire_index = fire_index_ml30b1_dual_echo;
                number_index = number_index_ml30b1_dual_echo;
            }
        }
        else if (ScanMLXS_180 == scan_mode)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 4;
            group_len = 16;
            fires = 108000;
            fov_index = fov_index_ml30b1_single_echo;
            fire_index = fire_index_ml30b1_single_echo;
            number_index = number_index_ml30b1_single_echo;
            distance_bit = 19;
            reflectivity_bit = 10;
            if (2 == echo_cnt)
            {
                fov_index = fov_index_ml30b1_dual_echo;
                fire_index = fire_index_ml30b1_dual_echo;
                number_index = number_index_ml30b1_dual_echo;
            }
        }
		else if (ScanML30SA1Plus_160 == scan_mode || ScanML30SA1Plus_160_1_2 == scan_mode || ScanML30SA1Plus_160_1_4 == scan_mode) {
			fires = 51200;
			fire_interval_us = 1.5625;
			if (ScanML30SA1Plus_160 == scan_mode)
			{
				fires = 51200;
				fire_interval_us = 1.5625 / 2.0;
			}
			else if (ScanML30SA1Plus_160_1_2 == scan_mode)
			{
				fires = 51200 / 2;
				fire_interval_us = 1.5625 / 2.0 / 2.0;
			}
			else if (ScanML30SA1Plus_160_1_4 == scan_mode)
			{
				fires = 51200 / 4;
				fire_interval_us = 1.5625 / 2.0;
			}

			if (1 == echo_cnt)
			{
#ifdef ZVISION_ML30sPlus_b1_ep_mode
                groups_in_one_udp = 40;
                points_in_one_group = 8;
                point_position_in_group = 0;
                group_len = 32;
                fov_index = fov_index_ml30sa1_plus_ep_single_echo;
                fire_index = fire_index_ml30sa1_plus_ep_single_echo;
                number_index = number_index_ml30sa1_plus_ep_single_echo;
#else
				groups_in_one_udp = 80;
				points_in_one_group = 4;
				point_position_in_group = 0;
				group_len = 16;
				int udp_cnt = fires / points_in_one_group / groups_in_one_udp;
				if (seq < udp_cnt / 2)
					fov_index = fov_index_ml30sa1_plus_H_single_echo;
				else
					fov_index = fov_index_ml30sa1_plus_L_single_echo;

				fire_index = fire_index_ml30sa1_plus_single_echo;
				number_index = number_index_ml30sa1_plus_single_echo;
#endif
			}
			else
			{
#ifdef ZVISION_ML30sPlus_b1_ep_mode
                groups_in_one_udp = 20;
                points_in_one_group = 16;
                point_position_in_group = 0;
                group_len = 64;
                fov_index = fov_index_ml30sa1_plus_ep_dual_echo;
                fire_index = fire_index_ml30sa1_plus_ep_dual_echo;
                number_index = number_index_ml30sa1_plus_ep_dual_echo;
#else
				groups_in_one_udp = 40;
				points_in_one_group = 8;
				point_position_in_group = 0;
				group_len = 32;
				int udp_cnt = fires / points_in_one_group / groups_in_one_udp * 2;
				if (seq < udp_cnt / 2)
					fov_index = fov_index_ml30sa1_plus_H_dual_echo;
				else
					fov_index = fov_index_ml30sa1_plus_L_dual_echo;

				fire_index = fire_index_ml30sa1_plus_dual_echo;
				number_index = number_index_ml30sa1_plus_dual_echo;
#endif
			}
		}
        else
        {
            return NotSupport;
        }

        int points = fires * echo_cnt;

        //realloc memory
        if (points != (int)cloud.points.size())
        {
            cloud.points.resize(points);
        }

        // downsample
        DownsampleMode downsample = DownsampleMode::DownsampleUnknown;
        if (filter) {
            if ((ScanML30SA1_160 == scan_mode || ScanML30SA1Plus_160 == scan_mode) && (fires == 51200) && \
                (filter->GetScanMode() == ScanML30SA1_160 || filter->GetScanMode() == ScanML30SA1Plus_160))
            {
                downsample = filter->GetDownsampleMode(scan_mode);
            }
        }

        for (uint32_t gp = 0; gp < groups_in_one_udp; ++gp)/*every groups*/
        {
            // 1/2 or 1/4 downsample mode
            if (downsample == DownsampleMode::Downsample_1_2) {
                if (gp % 2 != 0) {
                    for (uint32_t pt = 0; pt < points_in_one_group; ++pt)
                    {
                        int point_number = seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group + number_index[pt];
                        if (point_number > ((int)cloud.points.size()) - 1) return -1;
                        cloud.points[point_number].valid = 0;
                    }
                    continue;
                }
            }
            else if (downsample == DownsampleMode::Downsample_1_4) {
                int pre_group[8] = { 0,0,1,1,1,1,0,0 };
                if ((gp % 4) != pre_group[seq % 8]) {
                    for (uint32_t pt = 0; pt < points_in_one_group; ++pt)
                    {
                        int point_number = seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group + number_index[pt];
                        if (point_number > ((int)cloud.points.size()) - 1) return -1;
                        cloud.points[point_number].valid = 0;
                    }
                    continue;
                }
            }

            unsigned char* first_point_pos_in_group = data + group_position_in_packet + group_len * gp + point_position_in_group;
            ReturnType return_type = ReturnType::UnknownReturn;
            float distance = 0.0;
            int reflectivity = 0;
            for (uint32_t pt = 0; pt < points_in_one_group; ++pt)
            {
                int fire_number = (seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group) / echo_cnt + fire_index[pt];
                // manu downsample mode
                if (downsample == DownsampleMode::Downsample_cfg_file) {
                    if (filter->IsLidarPointCovered(fire_number)) {
                        continue;
                    }
                }

                int point_number = seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group + number_index[pt];
                int fov_number = fov_index[pt];

                unsigned char* point_pos = first_point_pos_in_group + point_len * pt;
                PointCloudPacket::ResolvePoint(point_pos, return_type, distance, reflectivity, distance_bit, reflectivity_bit);

                int echo = 0;//first echo
                if ((2 == echo_cnt) && (number_index[pt] % 2))
                    echo = 1;//second echo

                //orientation index 
                int orientation_index = fire_number;

                if (orientation_index > ((int)cal_lut.data.size()) - 1)
                    return -1;
                if (point_number > ((int)cloud.points.size()) - 1)
                    return -1;

                CalibrationDataSinCos& point_cal = cal_lut.data[orientation_index];
                Point& point_data = cloud.points[point_number];
                point_data.distance = distance;
                point_data.x = distance * point_cal.cos_ele * point_cal.sin_azi;/*x*/
                point_data.y = distance * point_cal.cos_ele * point_cal.cos_azi;/*y*/
                point_data.z = distance * point_cal.sin_ele;/*z*/
                point_data.ele = point_cal.ele;
                point_data.azi = point_cal.azi;
                point_data.reflectivity_13bits = reflectivity & 0x1FFF;
                point_data.reflectivity = reflectivity & 0xFF;
                point_data.point_number = point_number;
                point_data.fire_number = fire_number;
                point_data.fov = fov_number;
                point_data.valid = 1;
                point_data.echo_num = echo;
                point_data.return_type = return_type;
                int fire_number_in_udp = ((gp * points_in_one_group) / echo_cnt) + fire_index[pt];
                point_data.timestamp_ns = udp_timestamp + (uint64_t)((fire_interval_us * fire_number_in_udp) * 1000);
            }
        }

        return 0;

    }

    int PointCloudPacket::ProcessPacket(PointCloudPacket& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud)
    {
        std::string pkt(packet.data_, sizeof(packet.data_) / sizeof(char));
        return PointCloudPacket::ProcessPacket(pkt, cal_lut, cloud);
    }

    void PointCloudPacket::ResolvePoint(const unsigned char* point, ReturnType& return_type, float& dis, int& ref, int dis_bit, int ref_bit)
    {
        uint32_t data = ntohl(*(uint32_t*)(point));
        dis = float(int(data >> (32 - dis_bit)) * 0.0015);
        ref = (int)data & ((0x1 << ref_bit) - 1);

        uint32_t tp_field = (data & 0x300) >> 8;
        if (0x00 == tp_field)
            return_type = ReturnType::FirstReturn;
        else if (0x01 == tp_field)
            return_type = ReturnType::StrongestReturn;
        else if (0x02 == tp_field)
            return_type = ReturnType::LastReturn;
        else if (0x03 == tp_field)
            return_type = ReturnType::SecondStrongestReturn;
        else
            return_type = ReturnType::UnknownReturn;

    }

    void CalibrationPacket::ExtractData(std::vector<float>& cal)
    {
        int network_data = 0;
        int host_data = 0;
        float* pfloat_data = reinterpret_cast<float *>(&host_data);
        for (int i = 0; i < 128 * 2; i++)
        {
            memcpy(&network_data, cal_data_ + i * 4 + 16, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
            host_data = ntohl(network_data);
            cal.push_back(*pfloat_data);
        }
    }

    void CalibrationPacket::ExtractData(std::string& packet, std::vector<float>& cal)
    {
        int network_data = 0;
        int host_data = 0;
        float* pfloat_data = reinterpret_cast<float *>(&host_data);
        char* pkt = const_cast<char *>(packet.c_str());
        for (int i = 0; i < 128 * 2; i++)
        {
            memcpy(&network_data, pkt + i * 4 + 16, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
            host_data = ntohl(network_data);
            cal.push_back(*pfloat_data);
        }
    }

    // New architecture protocol
    const float InternalPacket::DISTANCE_UNITS = 0.0015f;
    void InternalPacket::GetPacketType(const std::string& packet, PacketType& type, ScanMode& mode)
    {
        int pkt_len = packet.size();
        type = Tp_PacketUnknown;
        mode = ScanUnknown;

        // check header len
        if (pkt_len <= PACKET_HEADER_LEN)
            return;

        uint8_t* data = (unsigned char*)(packet.data());
        uint16_t value = (*(data + 0) << 8) | *(data + 1);
        switch (value)
        {
        case 0x1111:
            mode = ScanML30SA1_160;
            break;
        case 0x2222:
            mode = ScanMLXS_180;
            break;
        default:
            break;
        }

        value = (*(data + 22) << 8) | *(data + 23);
        switch (value)
        {
        case 0xAAAA:
        case 0xCCCC:
        {
            // TODO  pointcloud...
            break;
        }
        case 0xAA01:
        case 0xCC01:
        {
            if (pkt_len == BloomingPacket::PACKET_LEN)
                type = PacketType::Tp_BloomingPacket;
            break;
        }
        case 0xAA02:
        case 0xCC02:
        {
            // TODO  apd channel...
            break;
        }
        case 0xAA03:
        case 0xCC03:
        {
            // TODO  intensity distance cali ...
            break;
        }
        case 0xAA04:
        case 0xCC04:
        {
            // TODO  adc original...
            break;
        }
        case 0xAA05:
        case 0xCC05:
        {
            // TODO  original data...
            break;
        }

        // ...


        case 0xAA21:
        case 0xCC21:
        {
            // TODO  Block Debug...
            break;
        }

        default:
            break;
        }

    }

    int InternalPacket::GetPacketSeq(const std::string& packet)
    {
        // check header len
        if (packet.size() <= PACKET_HEADER_LEN)
            return -1;

        uint8_t* data = (unsigned char*)(packet.data());
        return (*(data + 24) << 8) | *(data + 25);
    }

    double InternalPacket::GetTimestamp(const std::string& packet)
    {
        int len = packet.size();
        const int chk_sum_len = 2;
        const int reserved_len = 2;
        const int stamp_len = 10;
        int stamp_offset = len - reserved_len - chk_sum_len - stamp_len;
        unsigned char* data = (unsigned char*)(packet.data());

        uint64_t seconds = 0;
        seconds += ((uint64_t)data[stamp_offset + 0] << 40);
        seconds += ((uint64_t)data[stamp_offset + 1] << 32);
        seconds += ((uint64_t)data[stamp_offset + 2] << 24);
        seconds += ((uint64_t)data[stamp_offset + 3] << 16);
        seconds += ((uint64_t)data[stamp_offset + 4] << 8);
        seconds += ((uint64_t)data[stamp_offset + 5]);

        uint32_t ms = (int)(data[stamp_offset + 6] << 8) + data[stamp_offset + 7];
        uint32_t us = (int)(data[stamp_offset + 8] << 8) + data[stamp_offset + 9];
        return (double)seconds + (double)ms / 1000.0 + (double)us / 1000000.0;
    }

    uint64_t InternalPacket::GetTimestampNS(const std::string& packet)
    {
        int len = packet.size();
        const int chk_sum_len = 2;
        const int reserved_len = 2;
        const int stamp_len = 10;
        int stamp_offset = len - reserved_len - chk_sum_len - stamp_len;
        unsigned char* data = (unsigned char*)(packet.data());

        uint64_t seconds = 0;
        seconds += ((uint64_t)data[stamp_offset + 0] << 40);
        seconds += ((uint64_t)data[stamp_offset + 1] << 32);
        seconds += ((uint64_t)data[stamp_offset + 2] << 24);
        seconds += ((uint64_t)data[stamp_offset + 3] << 16);
        seconds += ((uint64_t)data[stamp_offset + 4] << 8);
        seconds += ((uint64_t)data[stamp_offset + 5]);

        uint32_t ms = (int)(data[stamp_offset + 6] << 8) + data[stamp_offset + 7];
        uint32_t us = (int)(data[stamp_offset + 8] << 8) + data[stamp_offset + 9];
        return (uint64_t)(seconds * 1000000000) + (uint64_t)(ms * 1000000) + (uint64_t)(us * 1000);
    }

    bool InternalPacket::GetFrameResolveInfo(const std::string& packet, InternalPacketHeader& header)
    {
        header.valid = false;
        // get packet type
        GetPacketType(packet, header.packet_type, header.scan_mode);
        if(header.packet_type == Tp_PacketUnknown || header.scan_mode == ScanUnknown)
            return false;

        uint8_t* pdata = (uint8_t*)packet.data();
        // product version
        header.product_version = (*(pdata + 2) << 8) | (*(pdata + 3));

        // get seq
        header.seq = GetPacketSeq(packet);
        if (header.seq < 0)
            return false;

        int offset = 0;
        int length = 0;
        // serial number
        {
            offset = 4;
            length = 18;
            header.serial_number = std::string(packet.data() + offset, length);
            if (header.serial_number.empty())
                return false;
        }

        auto& info = header.resolve_info;

        // is dual? 1 or 2
        info.echo = 1;

        // downsample mode    ds value
        // none                 1
        // 1/2                  2
        // 1/4                  4
        uint8_t ds = 1;
        switch (header.scan_mode)
        {
        case ScanML30SA1_160:
        case ScanML30SA1_160_1_2:
        case ScanML30SA1_160_1_4:
        {
            if (header.scan_mode == ScanML30SA1_160_1_2)     ds = 2;
            else if(header.scan_mode == ScanML30SA1_160_1_4) ds = 4;

            info.fovs = 8;
            info.lines = 80;
            info.points_per_line = 80 * info.echo / ds;
            if (info.echo == 2)   info.fov_id_in_group = std::vector<int>{ 0,6,0,6,1,7,1,7,2,4,2,4,3,5,3,5 };
            else                info.fov_id_in_group = std::vector<int>{ 0,6,1,7,2,4,3,5 };

            info.points_per_group = 8 * info.echo;
            info.points_offset_bytes_in_group.resize(info.points_per_group, 0);
            info.groups_per_udp = 40 / info.echo;

            info.udp_count = 160 * info.echo / ds;
            info.npoints = 51200 * info.echo / ds;
            break;
        }
        case ScanML30SA1Plus_160:
        case ScanML30SA1Plus_160_1_2:
        case ScanML30SA1Plus_160_1_4:
        {
            if (header.scan_mode == ScanML30SA1Plus_160_1_2)      ds = 2;
            else if (header.scan_mode == ScanML30SA1Plus_160_1_4) ds = 4;

            info.fovs = 8;
            info.lines = 80;
            info.points_per_line = 80 * info.echo / ds;
            if (info.echo == 2)   info.fov_id_in_group = std::vector<int>{ 0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7 };
            else                info.fov_id_in_group = std::vector<int>{ 0,1,2,3,4,5,6,7 };

            info.points_per_group = 4 * info.echo;
            info.points_offset_bytes_in_group.resize(info.points_per_group, 0);
            info.groups_per_udp = 80 / info.echo;

            info.udp_count = 160 * info.echo / ds;
            info.npoints = 51200 * info.echo / ds;
            break;
        }
        case ScanMLXS_180:
        {
            info.fovs = 3;
            info.lines = 180;
            info.points_per_line = 200 * info.echo / ds;
            if (info.echo == 2)
            {
                info.fov_id_in_group = std::vector<int>{ 0,0,1,1,2,2 };
                info.points_offset_bytes_in_group = std::vector<int>{ 4,4,4,8,8,8};
            }
            else
            {
                info.fov_id_in_group = std::vector<int>{ 0,1,2 };
                info.points_offset_bytes_in_group = std::vector<int>{ 4,4,4 };
            }

            info.points_per_group = 3 * info.echo;
            info.groups_per_udp = 80 / info.echo;

            info.udp_count = 450 * info.echo / ds;
            info.npoints = 108000 * info.echo / ds;
            break;
        }
        default:
            return false;
        }

        switch (header.packet_type)
        {
        case Tp_BloomingPacket:
        {
            info.point_size = 12;
            break;
        }

        default:
            return false;
        }



        header.valid = true;
        return true;
    }

    int BloomingPacket::ProcessPacket(const std::string& packet, const zvision::CalibrationDataSinCosTable& cal_lut, BloomingPoints& cloud, InternalPacketHeader* header)
    {
        int ret = -1;
        InternalPacketHeader header_;
        if (header)
            header_ = *header;
        else
        {
            if (!InternalPacket::GetFrameResolveInfo(packet, header_))
                return -1;
        }

        if (!header_.valid)
            return ret;

        const auto& info = header_.resolve_info;
        if (cloud.size() < info.npoints)
            cloud.resize(info.npoints);

        // resolve
        uint8_t* pdata = (uint8_t*)packet.c_str() + PACKET_HEADER_LEN;
        for (int g = 0; g< info.groups_per_udp; g++)
        {
            for (int p = 0; p< info.points_per_group; p++)
            {
                int group_id = header_.seq * info.groups_per_udp + g;
                int fire_id = (group_id * info.points_per_group + p) / info.echo ;
                int point_id = (header_.seq * info.groups_per_udp + g) * info.points_per_group + p;
                int pos = g * (info.points_per_group * info.point_size + info.points_offset_bytes_in_group.at(p)) + info.points_offset_bytes_in_group.at(p) + p * info.point_size;

                BloomingPoint point;
                // get point descrption
                point.fov_id = info.fov_id_in_group.at(p);
                point.group_id = group_id;
                point.fire_id = fire_id;
                point.point_id = point_id;
                point.echo_id = info.echo - 1;

                // get point data
                uint8_t* point_ptr = pdata + pos;
                point.peak = *point_ptr;
                point.pulse_width = static_cast<float>(*(point_ptr + 1)) + static_cast<float>(*(point_ptr + 2)) / 256.0f;
                point.gain = *(point_ptr + 3);
                uint32_t d19_r13 = (*(point_ptr + 4) << 24) + (*(point_ptr + 5) << 16) + (*(point_ptr + 6) << 8) + (*(point_ptr + 7));
                point.distance = DISTANCE_UNITS * static_cast<int>(d19_r13 >> (32 - DISTANCE_BITS));
                point.reserved = (d19_r13 & 0x1F00) >> 8;
                point.reflectivity = d19_r13 & 0xFF;
                point.noisy = static_cast<float>(*(point_ptr + 9)) + static_cast<float>(*(point_ptr + 8)) / 256.0f;
                point.noisy = static_cast<float>(*(point_ptr + 11)) + static_cast<float>(*(point_ptr + 10)) / 256.0f;

                point.x = point.distance * cal_lut.data.at(fire_id).cos_ele * cal_lut.data.at(fire_id).sin_azi;
                point.y = point.distance * cal_lut.data.at(fire_id).cos_ele * cal_lut.data.at(fire_id).cos_azi;
                point.z = point.distance * cal_lut.data.at(fire_id).sin_ele;
                point.valid = true;
                cloud.at(point_id) = point;
            }
        }

        return 0;
    }

}
