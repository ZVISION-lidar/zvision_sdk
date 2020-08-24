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
        else
        {
            return LidarUnknown;
        }
    }

    int PointCloudPacket::GetPacketSeq(std::string& packet)
    {
        return (unsigned char)(packet[3]) + (((unsigned char)packet[2] & 0xF) << 8);
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
        if (type != cal_lut.device_type)
        {
            return NotMatched;
        }

        //device is unknown or device type and calibration data are not matched
        if (type == LidarUnknown)
            return NotSupport;

        int seq = PointCloudPacket::GetPacketSeq(packet);

        //UDP packet parameters
        unsigned int groups_in_one_udp = 0;
        unsigned int points_in_one_group = 0;
        unsigned int point_position_in_group = 0;
        unsigned int group_len = 0;
        unsigned int point_len = 4;
        unsigned int group_position_in_packet = 4;

        if (LidarML30B1 == type)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 4;
            group_len = 16;
            if (30000 != cloud.points.size())
                cloud.points.resize(30000);
        }
        else if (LidarML30SA1 == type)
        {
            groups_in_one_udp = 40;
            points_in_one_group = 8;
            point_position_in_group = 0;
            group_len = 32;
            if (51200 != cloud.points.size())
                cloud.points.resize(51200);
        }
        else if (LidarMLX == type)
        {
            groups_in_one_udp = 80;
            points_in_one_group = 3;
            point_position_in_group = 0;
            group_len = 16;
            if (96000 != cloud.points.size())
                cloud.points.resize(96000);
        }
        else
        {
            return NotSupport;
        }
        for (uint32_t gp = 0; gp < groups_in_one_udp; ++gp)/*every groups*/
        {
            unsigned char* first_point_pos_in_group = data + group_position_in_packet + group_len * gp + point_position_in_group;
            uint32_t dis_low, dis_high, int_low, int_high;/*dis*/
            float distance = 0.0;
            int reflectivity = 0;
            Point p;
            for (uint32_t pt = 0; pt < points_in_one_group; ++pt)
            {
                unsigned char* point_pos = first_point_pos_in_group + point_len * pt;
                dis_high = point_pos[0];
                dis_low = point_pos[1];
                int_high = point_pos[2];
                int_low = point_pos[3];

                distance = static_cast<int>((((dis_high << 8) + dis_low) << 3) + (int)((int_high & 0xE0) >> 5));
                distance = distance * 0.0015f;
                reflectivity = (((int_high & 0x1F) << 8) + (int_low));
                reflectivity = (int)reflectivity & 0x3FF;

                int point_number = seq * points_in_one_group * groups_in_one_udp + gp * points_in_one_group + pt;
                CalibrationDataSinCos& point_cal = cal_lut.data[point_number];
                Point& point_data = cloud.points[point_number];
                point_data.x = distance * point_cal.cos_ele * point_cal.sin_azi;/*x*/
                point_data.y = distance * point_cal.cos_ele * point_cal.cos_azi;/*y*/
                point_data.z = distance * point_cal.sin_ele;/*z*/
                point_data.reflectivity = reflectivity;
                point_data.number = point_number;
                point_data.fov = 0;
                point_data.valid = true;
            }
        }

        return 0;

    }

    int PointCloudPacket::ProcessPacket(PointCloudPacket& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud)
    {
        std::string pkt(packet.data_, sizeof(packet.data_) / sizeof(char));
        return PointCloudPacket::ProcessPacket(pkt, cal_lut, cloud);
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
