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
#include <cstdint>
#include <cstring>
#include <math.h>
#include <cmath>
#include <iomanip>
#include "loguru.hpp"
#include "packet_ez6_b2.h"

namespace zvision
{
    /**
    *@ brief Verify if the data packet is a valid EZ6_S2 point cloud packet
    *The original data packet string input by param packet
    *@ return true means legal, false means illegal
    */
    bool pkg_parse_ez6_b2::IsValidPacket(std::string& packet)
    {
        if ((EZ6_B2_POINT_CLOUD_LEN != packet.size()) ||
            (packet.substr(0,6) != "EZ06B2"))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    /**
    *@ brief Get device type
    *The input data packet of param packet
    *@ return returns the device type (fixed as LidarEZ6_S2)
    */
    DeviceType pkg_parse_ez6_b2::GetDeviceType(std::string& packet)
    {
        return LidarEZ6_B2;
    }

    /**
    *@ brief Get frame number
    *The input data packet of param packet
    *@ return frame number (integer)
    */
    int pkg_parse_ez6_b2::GetFrameNum(std::string& packet)
    {
        const uint8_t *data = (uint8_t *)packet.c_str();
        int frame_num = (data[8]<<8)+(data[9]<<0);

        return frame_num;
    }
    /**
    *@ brief Get scanning mode
    *The input data packet of param packet
    *@ return Scan mode (fixed as ScanEZ6_S2/192)
    */
    ScanMode pkg_parse_ez6_b2::GetScanMode(std::string& packet)
    {
        return ScanEZ6_B2_192;
    }
    /**
    *@ brief Get package number
    *The input data packet of param packet
    *@ return Package Number
    */
    int pkg_parse_ez6_b2::GetPacketSeq(std::string& packet)
    {
        return (unsigned char)(packet[13]) + (((unsigned char)packet[12] & 0xF) << 8);
    }
    /**
    *@ brief Get timestamp (in microseconds)
    *The input data packet of param packet
    *@ return timestamp (unit: microseconds)
    */
    uint64_t pkg_parse_ez6_b2::GetTimestamp(uint8_t *data)
    {
        uint64_t seconds = 0;
        seconds += ((uint64_t)data[0] << 40);
        seconds += ((uint64_t)data[1] << 32);
        seconds += ((uint64_t)data[2] << 24);
        seconds += ((uint64_t)data[3] << 16);
        seconds += ((uint64_t)data[4] << 8);
        seconds += ((uint64_t)data[5]);

        uint32_t ms = (int)(data[6] << 8) + data[7];
        uint32_t us = (int)(data[8] << 8) + data[9];

        return (seconds*1000000 + ms*1000 + us);
    }
    /**
    *@ brief deals with a point cloud data packet
    *The input data packet of param packet
    *@ paramangle_comp angle compensation data
    *Point cloud object output from paramcloud
    *@ return 0 represents normal package, 1 represents frame end, 2 represents new frame start
    */
    int pkg_parse_ez6_b2::ProcessPacket(
        std::string &packet,
        angle_comp_t *angle_comp,
        PointCloud &cloud
        ) 
    {
        const uint8_t *data = (uint8_t *)packet.c_str();
        uint16_t current_frame = (data[8]<<8)+(data[9]<<0);
        uint16_t current_pkg = (data[12]<<8)+(data[13]<<0);

        /* pkg in new frame, old frame finish, should run processpacket again */
        if(last_frame != current_frame)
        {
            last_frame = current_frame;
            return 2;
        }

        /* last pkg in this frame */
        if((current_pkg == 599))
        {
            Pkg2Points(packet, angle_comp->azi, angle_comp->ele, cloud);
            return 1;
        }
        /* normal pkg in this frame */
        else
        {
            Pkg2Points(packet, angle_comp->azi, angle_comp->ele, cloud);
            return 0;
        }
    }
    /**
    *@ brief parses the data packet into a point cloud
    *The input data packet of param packet
    *@ param v'azi_comp azimuth compensation array
    *@ param v-ele_comp elevation angle compensation array
    *Point cloud object output from paramcloud
    */
    void pkg_parse_ez6_b2::Pkg2Points(std::string &packet, std::vector<float>& v_azi_comp, std::vector<float>& v_ele_comp, PointCloud &cloud)
    {
        unsigned char *pdata = const_cast<unsigned char *>((unsigned char *)packet.c_str());

        int head_len = 18;

        uint8_t block_num = *((uint8_t*)(pdata + head_len + 2));

        uint16_t row = ntohs(*((uint16_t*)(pdata + head_len + 4)));
        uint16_t column = ntohs(*((uint16_t*)(pdata + head_len + 6)));

        for (int i = 0; i < block_num; i++)
        {
            int blockhead_len = head_len + 8;

            uint16_t slot_id = ntohs(*((uint16_t*)(pdata + blockhead_len + 0)));
            uint8_t point_cnt = *((uint8_t*)(pdata + blockhead_len + 4));

            float fov_h_group[8] = { 0 };
            for (int h = 0; h < 8; h++)
            {
                uint8_t sign = ((*(uint8_t*)(pdata + blockhead_len + 16 + h * 2)) & 0x80) >> 7;
                uint8_t data_int = *(uint8_t*)(pdata + blockhead_len + 16 + h * 2) & 0x7F;
                uint8_t data_dec = *(uint8_t*)(pdata + blockhead_len + 17 + h * 2) & 0xFF;

                float data_dec_f = data_dec / 256.0;
                float fov_h = data_int + data_dec_f;
                if (sign)
                    fov_h = -fov_h;

                fov_h_group[h] = fov_h;
            }

            uint8_t sign = ((*(uint8_t*)(pdata + blockhead_len + 48)) & 0x80) >> 7;
            uint8_t data_int = *(uint8_t*)(pdata + blockhead_len + 48) & 0x7F;
            uint8_t data_dec = *(uint8_t*)(pdata + blockhead_len + 49) & 0xFF;

            float data_dec_f = data_dec / 256.0;
            float fov_v = data_int + data_dec_f;
            if (sign)
                fov_v = -fov_v;

            uint8_t side_flag = *((uint8_t*)(pdata + blockhead_len + 50));

            for (int pt = 0; pt < point_cnt; pt++)
            {
                uint16_t distance = ntohs(*((uint16_t*)(pdata + blockhead_len + 64 + pt * 5)));
                uint8_t reflectivity = *((uint8_t*)(pdata + blockhead_len + 66 + pt * 5));
                uint8_t Flag = *((uint8_t*)(pdata + blockhead_len + 68 + pt * 5));

                int retro_flag = Flag & 0x1;
                int dirty_flag = (Flag >> 1) & 0x1;
                int dirty_grade = (Flag >> 2) & 0x3;

                int area_num = pt / 24;

                float distance_f = (float)distance / 16.0 * speed_light;
                float fov_h_f = fov_h_group[area_num];

                float fov_v_f = fov_v;

                if ((v_azi_comp.size() != 0) && (v_ele_comp.size() != 0))
                {
                    fov_h_f += v_azi_comp.at(pt);
                    fov_v_f += v_ele_comp.at(pt);
                }

                float azi = fov_h_f / 180.0 * 3.1416;
                float ele = fov_v_f / 180.0 * 3.1416;

                Point point_data;

                point_data.x = (distance_f * cos(ele) * sin(azi));
                point_data.y = (distance_f * cos(ele) * cos(azi));
                point_data.z = distance_f * sin(ele);

                point_data.azimuth = fov_h_f;
                point_data.elevation = fov_v_f;
                point_data.reflectivity = reflectivity;
                point_data.distance = distance_f;

                point_data.col = slot_id;
                point_data.row = pt;
                point_data.pointid = point_data.col * row + point_data.row;

                point_data.groupid = area_num;
                point_data.mirrornum = side_flag;
                point_data.retro_flag = retro_flag;

                point_data.rowCnt = row;
                point_data.colCnt = column;
                point_data.dirtyValue = dirty_grade;
                point_data.dirty_ = dirty_flag;

                point_data.timestamp_us = GetTimestamp(pdata+blockhead_len+6);
                cloud.stamp_us =  point_data.timestamp_us;
                cloud.dev_type = GetDeviceType(packet); 
                cloud.points.push_back(point_data);
            }
        }
    }

    bool pkg_parse_ez6_b2::IsValidAnglePacket(std::string& packet)
    {
        /* TODO: add the real check */
        return false;
    }


    int pkg_parse_ez6_b2::ParseAnglePkg(std::string &packet, angle_comp_t &angle_comp)
    {
        /* TODO: add the real parse */
        return 0;
    }

    bool pkg_parse_ez6_b2::IsValidImuPacket(std::string& packet)
    {
        /* TODO: add the real check */
        return false;
    }

    int pkg_parse_ez6_b2::ParseImuPkg(std::string &packet, imu_data_t &imu_data)
    {
        /* TODO: add the real parse */
        return 0;
    }
}
