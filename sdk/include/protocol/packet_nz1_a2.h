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


#ifndef PACKET_NZ1_A2_H_
#define PACKET_NZ1_A2_H_
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>

#include "define.h"
#include "packet.h"

namespace zvision
{
    class PointCloud;

    float get_float(uint8_t * buffer);

    class pkg_parse_nz1_a2 : public PointCloudPacket  
    {
    public:

        /** \brief Packet is valid pointcloud udp packet or not.
        * \return true for yes, false for no.
        */
        bool IsValidPacket(std::string& packet) override;

        /** \brief Get device type from the pointcloud packet.
        * \return DeviceType.
        */
        DeviceType GetDeviceType(std::string& packet) override;

        /** \brief Get frame num from the pointcloud packet.
        * \return frame Num.
        */
        int GetFrameNum(std::string& packet) override;

        /** \brief Get scan mode from the pointcloud packet.
        * \return DeviceType.
        */
        ScanMode GetScanMode(std::string& packet) override;

        /** \brief Get the udp sequence number from the pointcloud packet.
        * \return udp sequence number.
        */
        int GetPacketSeq(std::string& packet) override;

        /** \brief Get the timestamp from the exciton pointcloud sampleB  packet.
         * \return timestamp in second.
         */
        uint64_t GetTimestamp(uint8_t *data) override;

        int ProcessPacket(
            std::string &packet,
            angle_comp_t *angle_comp,
            PointCloud &cloud
        ) override;

        void Pkg2Points(std::string &packet, std::vector<float>& v_azi_comp, std::vector<float>& v_ele_comp, PointCloud &cloud) override;

        /** \brief Packet is valid angle comp packet or not.
        * \return true for yes, false for no.
        */
        bool IsValidAnglePacket(std::string& packet) override; 

        /** \brief Parse the angle comp package data to angle_comp.
        */
        int ParseAnglePkg(std::string &packet, angle_comp_t &angle_comp) override;

        /** \brief Packet is valid imu packet or not.
        * \return true for yes, false for no.
        */
        bool IsValidImuPacket(std::string& packet) override;

        /** \brief Parse the imu package data to imu_data.
        */
        int ParseImuPkg(std::string &packet, imu_data_t &imu_data) override;
    };
}

#endif //end PACKET_H_
