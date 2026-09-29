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
#include "packet_nz1_a2.h"

#define BLOCK_SIZE 544 //5*96+64
#define NZ1_ROW     96
#define NZ1_COL     480    

namespace zvision
{
    /**
    *@ brief Verify if the data packet is a valid NZ1_A2 point cloud packet
    *The original data packet string input by param packet
    *@ return true means legal, false means illegal
    */
    bool pkg_parse_nz1_a2::IsValidPacket(std::string& packet)
    {
        if (((NZ1_A2_POINT_CLOUD_LEN != packet.size()) && (NZ1_A2_TRANS_LEN != packet.size()) && (NZ5_B5_POINT_CLOUD_LEN != packet.size()) && (NZ5_B5_TRANS_LEN != packet.size())) ||
            ((packet.substr(0,9) != "ZVSNZ1_A2")&&(packet.substr(0,9) != "ZVSNZ1_B1")&&(packet.substr(0,9) != "ZVSNZ3_B3")&&(packet.substr(0,9) != "ZVSNZ5_B5") && (packet.substr(0,9) != "ZVSNZ5_MT") && (packet.substr(0,9) != "ZVSNZ5_XP")) ||
            ((packet.substr(12,5) != "POINT") && (packet.substr(12,5) != "TRANS")))
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
    *@ return returns the device type (fixed as LidarNZ1-A2)
    */
    DeviceType pkg_parse_nz1_a2::GetDeviceType(std::string& packet)
    {
        return LidarNZ1_A2;
    }
    /**
    *@ brief Get frame number
    *The input data packet of param packet
    *@ return frame number (integer)
    */
    int pkg_parse_nz1_a2::GetFrameNum(std::string& packet)
    {
        const uint8_t *data = (uint8_t *)packet.c_str();
        int frame_num = (data[19]<<8)+(data[20]<<0);

        return frame_num;
    }

    /**
    *@ brief Get scanning mode
    *The input data packet of param packet
    *@ return scanning mode (fixed as ScanNZ1_A2_96)
    */
    ScanMode pkg_parse_nz1_a2::GetScanMode(std::string& packet)
    {
        return ScanNZ1_A2_96;
    }
    /**
    *@ brief Get package number
    *The input data packet of param packet
    *@ return Package Number
    */
    int pkg_parse_nz1_a2::GetPacketSeq(std::string& packet)
    {
        return ((uint8_t)packet[21]<<8)+((uint8_t)packet[22]<<0);
    }
    /**
    *@ brief Get timestamp (in microseconds)
    *The input data packet of param packet
    *@ return timestamp (unit: microseconds)
    */
    uint64_t pkg_parse_nz1_a2::GetTimestamp(uint8_t *data)
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
    int pkg_parse_nz1_a2::ProcessPacket(
        std::string &packet,
        angle_comp_t *angle_comp,
        PointCloud &cloud
        ) 
    {
        const uint8_t *data = (uint8_t *)packet.c_str();
        uint16_t current_frame = (data[19]<<8)+(data[20]<<0);
        uint16_t current_pkg = (data[21]<<8)+(data[22]<<0);
        uint16_t pkg_max;

        int lidar_type = data[5] - '0';
        int lidar_subtype = data[7];

        /* process the normal point */
        if(packet.size() == NZ1_A2_POINT_CLOUD_LEN)
        {
            pkg_max = ((data[34]<<8)+(data[35]<<0))/2 - 1;
        }
        else if(packet.size() == NZ5_B5_POINT_CLOUD_LEN)
        {
            pkg_max = ((data[34]<<8)+(data[35]<<0)) - 1;
        }
        /* process the trans point */
        else
        {
            if(lidar_subtype == 'M')
            {
                pkg_max = 240-1;
            }
            else if(lidar_subtype == 'X')
            {
                pkg_max = 280-1;
            }
            else
            {
                pkg_max = 480-1;
            }
        }

        /* pkg in new frame, old frame finish, should run processpacket again */
        if(last_frame != current_frame)
        {
            last_frame = current_frame;
            return 2;
        }

        /* last pkg in this frame */
        if(current_pkg == pkg_max)
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
    void pkg_parse_nz1_a2::Pkg2Points(std::string &packet, std::vector<float>& v_azi_comp, std::vector<float>& v_ele_comp, PointCloud &cloud)
    {
        static uint16_t area_offset1[2][4] = {{43,63,58,45},{40,62,59,48}}; 
        static uint16_t area_offset3[2][8] = {{68,73,88,92,38,45,59,65},{57,62,39,43,89,93,70,76}}; 
        static uint16_t area_offset5[2][8] = {{59,63,52,56,61,65,54,58},{52,56,59,63,54,58,61,65}}; 

        uint8_t *pdata = const_cast<uint8_t *>((uint8_t *)packet.c_str());
        int head_len = 30;

        int frame_num;
        int angle_index = 0;
        int row_96_or_192 = 96;
        int lidar_type = pdata[5] - '0';
        int lidar_subtype = pdata[7];
        int half_slots = 240;
        /* nz1 b1 */
        if((lidar_type == 1)||(lidar_type == 3))
        {
            if(v_azi_comp.size() == NZ1_A2_ANGLE_SIZE*2)
            {
                frame_num = GET_UINT16(pdata+19);
                /* the second angle para */
                if((frame_num%2) == 1)
                {
                    angle_index = NZ1_A2_ANGLE_SIZE;
                }
            }
        }
        else if(lidar_type == 5)
        {
            frame_num = GET_UINT16(pdata+19);
            /* the second angle para */
            if((frame_num%2) == 1)
            {
                angle_index = v_azi_comp.size()/2;
            }
            row_96_or_192 = 192;

            if(lidar_subtype == 'M')
            {
                half_slots = 120;
            }
            else if(lidar_subtype == 'X')
            {
                half_slots = 140;
            }
        }

        /* normal point */
        if((packet.size() == NZ1_A2_POINT_CLOUD_LEN)||(packet.size() == NZ5_B5_POINT_CLOUD_LEN))
        {
            uint8_t block_num = *((uint8_t*)(pdata + head_len + 0));
            uint16_t row = ntohs(*((uint16_t*)(pdata + head_len + 2)));
            uint16_t column = ntohs(*((uint16_t*)(pdata + head_len + 4)));

            for (int i = 0; i < block_num; i++)
            {
                int blockhead_len = head_len + 6 + i*BLOCK_SIZE;

                uint16_t slot_id = ntohs(*((uint16_t*)(pdata + blockhead_len + 0)));
                uint8_t point_cnt = *((uint8_t*)(pdata + blockhead_len + 4));
                int offset_index = slot_id/half_slots;

                uint8_t side_flag = *((uint8_t*)(pdata + blockhead_len + 50));

                int fov_offset = (slot_id/240)*4;
                for (int pt = 0; pt < point_cnt; pt++)
                {
                    uint16_t distance = ntohs(*((uint16_t*)(pdata + blockhead_len + 64 + pt * 5)));
                    uint8_t reflectivity = *((uint8_t*)(pdata + blockhead_len + 66 + pt * 5));
                    uint8_t Flag = *((uint8_t*)(pdata + blockhead_len + 68 + pt * 5));

                    int retro_flag = Flag & 0x1;
                    int dirty_flag = (Flag >> 1) & 0x1;
                    int dirty_grade = (Flag >> 2) & 0x3;

                    int area_num;
                    if((lidar_type == 1)||(lidar_type == 5))
                    {
                        area_num = pt / 24;
                    }
                    else if(lidar_type == 3)
                    {
                        area_num = pt / 12;
                    }
                    else
                    {
                        area_num = 0;
                    }

                    float distance_f = (float)distance / 16.0 * speed_light;

                    if((distance_f < 0.001)&&(!cloud.points.empty()))
                    {
                        continue;
                    }

                    float azi_degree = v_azi_comp[angle_index+slot_id*row_96_or_192+pt];
                    float ele_degree = v_ele_comp[angle_index+slot_id*row_96_or_192+pt];

                    float azi = azi_degree / 180.0 * 3.1416;
                    float ele = ele_degree / 180.0 * 3.1416; 

                    Point point_data;

                    point_data.x = (distance_f * cos(ele) * sin(azi));
                    point_data.y = (distance_f * cos(ele) * cos(azi));
                    point_data.z = distance_f * sin(ele);

                    point_data.azimuth = azi_degree;
                    point_data.elevation = ele_degree;
                    point_data.reflectivity = reflectivity;
                    point_data.distance = distance_f;

                    point_data.col = slot_id;
                    point_data.row = pt;
                    point_data.pointid = point_data.col * row + point_data.row;

                    point_data.groupid = area_num;
                    point_data.mirrornum = side_flag;
                    point_data.retro_flag = retro_flag;

                    point_data.fov = 3-(point_data.row/24)+fov_offset;
                    point_data.line_id = slot_id;

                    point_data.rowCnt = row;
                    point_data.colCnt = column;
                    point_data.dirtyValue = dirty_grade;
                    point_data.dirty_ = dirty_flag;

                    if(lidar_type == 1)
                    {
                        point_data.timestamp_us = GetTimestamp(pdata + blockhead_len + 6)-143+area_offset1[offset_index][area_num];
                    }
                    else if(lidar_type == 3)
                    {
                        point_data.timestamp_us = GetTimestamp(pdata + blockhead_len + 6)-143+area_offset3[offset_index][area_num];
                    }
                    else if(lidar_type == 5)
                    {
                        /* MT or XPRO */
                        if((lidar_subtype == 'M')||(lidar_subtype == 'X'))
                        {
                            point_data.timestamp_us = GetTimestamp(pdata + blockhead_len + 6)-273+area_offset5[offset_index][area_num]*2;
                        }
                        else
                        {
                            point_data.timestamp_us = GetTimestamp(pdata + blockhead_len + 6)-273+area_offset5[offset_index][area_num];
                        }
                    }
                    else
                    {
                        point_data.timestamp_us = GetTimestamp(pdata + blockhead_len + 6);
                    }
                    
                    cloud.stamp_us =  point_data.timestamp_us;
                    cloud.dev_type = GetDeviceType(packet); 
                    cloud.points.push_back(point_data);
                }
            }
        }
        /* trans point */
        else
        {
            uint32_t point_head[9];
            for(int i=0; i<9; i++)
            {
                point_head[i] = GET_UINT32(pdata+head_len+4*i);
            }

            // uint8_t frame_id = head[0]&0xff;
            uint16_t col_id = (point_head[0]>>8)&0xfff;
            // uint16_t row_id = (head[0]>>20)&0x3ff;

            uint32_t timestamp_low = (point_head[3]>>16) + ((point_head[4]<<16)&0xffff0000);
            uint32_t timestamp_mid = (point_head[4]>>16);
            uint32_t timestamp_high = point_head[5];

            uint16_t ts_ms,ts_us,ts_sl;
	        uint64_t ts_sh;

            ts_ms = (timestamp_low>>2)/1000000;
			ts_us = ((timestamp_low>>2)%1000000)/1000;
			ts_sl = timestamp_mid;
			ts_sh = timestamp_high;

            uint64_t time_us = ((ts_sh<<16)+ts_sl)*1000000+ts_ms*1000+ts_us;

            uint16_t slot_id = col_id;
            uint8_t point_cnt = row_96_or_192;
            uint8_t side_flag = point_head[7]&0x0f;

            int fov_offset = (slot_id/240)*4;
            for (int pt = 0; pt < point_cnt; pt++)
            {
                uint32_t pt_data = GET_UINT32(pdata+head_len+60*pt+44); 
                uint16_t distance = pt_data&0xffff;
                uint8_t reflectivity = (pt_data>>16)&0xff;

                float distance_f = (float)distance / 16.0 * speed_light;

                if((distance_f < 0.001)&&(!cloud.points.empty()))
                {
                    continue;
                }

                int area_num = pt / 24;
                float azi_degree = v_azi_comp[angle_index+slot_id*row_96_or_192+pt];
                float ele_degree = v_ele_comp[angle_index+slot_id*row_96_or_192+pt];

                float azi = azi_degree / 180.0 * 3.1416;
                float ele = ele_degree / 180.0 * 3.1416; 

                Point point_data;

                point_data.x = (distance_f * cos(ele) * sin(azi));
                point_data.y = (distance_f * cos(ele) * cos(azi));
                point_data.z = distance_f * sin(ele);

                point_data.azimuth = azi_degree;
                point_data.elevation = ele_degree;
                point_data.reflectivity = reflectivity;
                point_data.distance = distance_f;

                point_data.col = slot_id;
                point_data.row = pt;
                point_data.pointid = point_data.col * point_cnt + point_data.row;

                point_data.groupid = area_num;
                point_data.mirrornum = side_flag;

                point_data.fov = 3-(point_data.row/24)+fov_offset;
                point_data.line_id = slot_id;

                point_data.rowCnt = point_cnt;
                point_data.colCnt = NZ1_COL;

                point_data.timestamp_us = time_us;
                cloud.stamp_us =  point_data.timestamp_us;
                cloud.dev_type = GetDeviceType(packet); 
                cloud.points.push_back(point_data);
            }
        }
    }
    /**
    *@ brief Verify if it is a valid angle compensation package
    *The input data packet of param packet
    *@ return true means legal, false means illegal
    */
    bool pkg_parse_nz1_a2::IsValidAnglePacket(std::string& packet)
    {
        if((packet.size() != NZ1_A2_ANGLE_LEN) ||
           ((packet.substr(0,9) != "ZVSNZ1_A2") && (packet.substr(0,9) != "ZVSNZ1_B1") && (packet.substr(0,9) != "ZVSNZ3_B3") && (packet.substr(0,9) != "ZVSNZ5_B5") && (packet.substr(0,9) != "ZVSNZ5_MT") && (packet.substr(0,9) != "ZVSNZ5_XP")) ||
           (packet.substr(12,5)!= "ANGLE"))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    /**
    *@ brief parsing angle compensation package
    *The input data packet of @ param packet
    *Angle compensation data output from paramangle_comp
    *@ return 0 indicates success, non-zero indicates failure
    */
    int pkg_parse_nz1_a2::ParseAnglePkg(std::string &packet, angle_comp_t &angle_comp)
    {
        static int index = 0;
        uint8_t *header = (uint8_t *)packet.data();
        uint8_t *data = (uint8_t *)packet.data()+30;

        int angle_size,angle_pkg;
        if((packet.substr(7,2) == "B1")||(packet.substr(7,2) == "B3"))
        {
            angle_size = NZ1_A2_ANGLE_SIZE*2;
            angle_pkg = 576;
        }
        else if(packet.substr(4,2) == "Z5")
        {
            if(packet.substr(7,2) == "MT")
            {
                angle_size = NZ5_B5_ANGLE_SIZE;
                angle_pkg = 576;
            }
            else if(packet.substr(7,2) == "XP")
            {
                angle_size = NZ5_XPRO_ANGLE_SIZE*2;
                angle_pkg = 672;
            }
            else
            {
                angle_size = NZ5_B5_ANGLE_SIZE*2;
                angle_pkg = 1152;
            }
        }
        else
        {
            angle_size = NZ1_A2_ANGLE_SIZE;
            angle_pkg = 288;
        }

        if(angle_comp.azi.size() != angle_size)
        {
            index = 0;
            angle_comp.azi.resize(angle_size);
            angle_comp.ele.resize(angle_size);
        }

        uint16_t pkg_num = GET_UINT16(header+19);

        if(index == pkg_num)
        {
            for(int i=0; i<160; i++)
            {
                angle_comp.azi[index*160+i] = get_float(data+8*i);
                angle_comp.ele[index*160+i] = get_float(data+8*i+4);
            }

            /* get whole angle file */
            if(index == (angle_pkg-1))
            {
                index = 0;
                return 1;
            }
            
            index++;
        }

        return 0;
    }
    /**
    *@ brief Verify if the IMU data packet is valid
    *The input data packet of param packet
    *@ return true means legal, false means illegal
    */
    bool pkg_parse_nz1_a2::IsValidImuPacket(std::string& packet)
    {
        if(((packet.size() != NZ1_A2_IMU_LEN)&&(packet.size() != (NZ1_A2_IMU_LEN+4))) ||
           ((packet.substr(0,9) != "ZVSNZ1_A2") && (packet.substr(0,9) != "ZVSNZ1_B1") && (packet.substr(0,9) != "ZVSNZ3_B3") && (packet.substr(0,9) != "ZVSNZ5_B5") && (packet.substr(0,9) != "ZVSNZ5_MT") && (packet.substr(0,9) != "ZVSNZ5_XP")) ||
           (packet.substr(12,3)!= "IMU"))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    /**
    *@ brief parsing IMU data packet
    *The input data packet of @ param packet
    *IMU data output from @ paramimu_data
    *@ return 0 indicates success, non-zero indicates failure
    */
    int pkg_parse_nz1_a2::ParseImuPkg(std::string &packet, imu_data_t &imu_data)
    {
        uint8_t *data = (uint8_t *)(packet.data());

        imu_data.acc_x = get_float(data+30);
        imu_data.acc_y = get_float(data+34);
        imu_data.acc_z = get_float(data+38);

        imu_data.gyro_x = get_float(data+42);
        imu_data.gyro_y = get_float(data+46);
        imu_data.gyro_z = get_float(data+50);

        uint64_t seconds = 0;
        uint32_t offset = 54;
        seconds += ((uint64_t)data[offset + 0] << 40);
        seconds += ((uint64_t)data[offset + 1] << 32);
        seconds += ((uint64_t)data[offset + 2] << 24);
        seconds += ((uint64_t)data[offset + 3] << 16);
        seconds += ((uint64_t)data[offset + 4] << 8);
        seconds += ((uint64_t)data[offset + 5]);

        uint32_t ms = (int)(data[offset + 6] << 8) + data[offset + 7];
        uint32_t us = (int)(data[offset + 8] << 8) + data[offset + 9];

        imu_data.timestamp_us = seconds*1000000+ms*1000+us;

        return 0;
    }
}
