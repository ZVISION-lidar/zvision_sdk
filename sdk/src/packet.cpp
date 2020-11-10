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
    DeviceType CalibrationPacket::GetDeviceType()
    {
        std::string dev_code(cal_data_ + 3, 6);
        if (0 == dev_code.compare("30_B1 "))
        {
            return DeviceType::LidarML30B1;
        }
        else if (0 == dev_code.compare("30S_A1"))
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
        else if (0x04 == value)
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

    int PointCloudPacket::GetPacketSeq(std::string& packet)
    {
        return (unsigned char)(packet[3]) + (((unsigned char)packet[2] & 0xF) << 8);
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


    int PointCloudPacket::ProcessPacket(std::string& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud)
    {
        unsigned char* data = const_cast<unsigned char*>((unsigned char*)packet.c_str());

        //pkt content len: 42 + 1304
        if (packet.size() != 1304)
        {
            return InvalidContent;
        }

        //find lidar type
        DeviceType type = PointCloudPacket::GetDeviceType(packet);
        //type = DeviceType::LidarMLYB;

        if (type != cal_lut.device_type)
        {
            return NotMatched;
        }

        //device is unknown or device type and calibration data are not matched
        if (type == LidarUnknown)
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
        int *fov_index = fov_index_ml30sa1_single_echo;

        // fire index in one group
        int fire_index_ml30b1_single_echo[3] = { 0, 1, 2 };
        int fire_index_ml30b1_dual_echo[6] = { 0, 1, 2 };
        int fire_index_ml30sa1_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int fire_index_ml30sa1_dual_echo[16] = { 0, 1, 0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7 };
        int fire_index_mlx_single_echo[3] = { 0, 1, 2 };
        int fire_index_mlx_dual_echo[3] = { 0, 1, 2 };
        int *fire_index = fire_index_ml30sa1_single_echo;

        // point number index in one group
        int number_index_ml30b1_single_echo[3] = { 0, 1, 2 };
        int number_index_ml30b1_dual_echo[6] = { 0, 1, 2 };
        int number_index_ml30sa1_single_echo[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
        int number_index_ml30sa1_dual_echo[16] = { 0, 2, 1, 3, 4, 6, 5, 7, 8, 10, 9, 11, 12, 14, 13, 15 };
        int number_index_mlx_single_echo[3] = { 0, 1, 2 };
        int number_index_mlx_dual_echo[3] = { 0, 1, 2 };
        int *number_index = number_index_ml30sa1_single_echo;

        //fire interval
        double fire_interval_us = 0.0;

        if (LidarML30B1 == type)
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
        else if (LidarML30SA1 == type)
        {
            fires = 51200;
            fire_interval_us = 1.5625 / 2.0; // 0.00000078125
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
        else if (LidarMLX == type)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 0;
            group_len = 16;
            fires = 96000;
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
        else if (LidarMLYA == type)
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
        else if (LidarMLYB == type)
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

        for (uint32_t gp = 0; gp < groups_in_one_udp; ++gp)/*every groups*/
        {
            unsigned char* first_point_pos_in_group = data + group_position_in_packet + group_len * gp + point_position_in_group;
            float distance = 0.0;
            int reflectivity = 0;
            for (uint32_t pt = 0; pt < points_in_one_group; ++pt)
            {
                unsigned char* point_pos = first_point_pos_in_group + point_len * pt;

                int point_number = seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group + number_index[pt];
                int fire_number = (seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group) / echo_cnt + fire_index[pt];
                int fov_number = fov_index[pt];

                PointCloudPacket::ResolvePoint(point_pos, distance, reflectivity, distance_bit, reflectivity_bit);

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
                point_data.x = distance * point_cal.cos_ele * point_cal.sin_azi;/*x*/
                point_data.y = distance * point_cal.cos_ele * point_cal.cos_azi;/*y*/
                point_data.z = distance * point_cal.sin_ele;/*z*/
                point_data.reflectivity = reflectivity & 0xFF;
                point_data.point_number = point_number;
                point_data.fire_number = fire_number;
                point_data.fov = fov_number;
                point_data.valid = 1;
                point_data.echo_num = echo;

                Point& first_point_in_sweep = cloud.points[0];
                uint64_t first_fire_timestamp_in_sweep = first_point_in_sweep.timestamp_ns;
                if (0 == fire_number)
                {
                    point_data.timestamp_ns = udp_timestamp;
                }
                else
                {
                    point_data.timestamp_ns = first_fire_timestamp_in_sweep + (uint64_t)((fire_interval_us * fire_number) * 1000);
                }
                //point_data.timestamp = timestamp + fire_interval * (fire_index[pt] + points_in_one_group / echo_cnt * gp);
            }
        }

        return 0;

    }

    int PointCloudPacket::ProcessPacket(PointCloudPacket& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud)
    {
        std::string pkt(packet.data_, sizeof(packet.data_) / sizeof(char));
        return PointCloudPacket::ProcessPacket(pkt, cal_lut, cloud);
    }

    void PointCloudPacket::ResolvePoint(const unsigned char* point, float& dis, int& ref, int dis_bit, int ref_bit)
    {
        uint32_t data = ntohl(*(uint32_t*)(point));
        dis = float(int(data >> (32 - dis_bit)) * 0.0015);
        ref = (int)data & ((0x1 << ref_bit) - 1);
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
}
