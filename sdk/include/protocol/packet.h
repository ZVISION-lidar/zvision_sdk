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


#ifndef PACKET_H_
#define PACKET_H_
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>

#include "define.h"

#define  GET_UINT32(x) ntohl(*((uint32_t *)(x)))
#define  GET_UINT16(x) ntohs(*((uint16_t *)(x)))

namespace zvision
{
    class PointCloud;

    struct LidarUdpPacket
    {
        std::string data;
        int ip;
        int ip_dst = 0xffffffff;
    };

    class PointCloudPacket
    {
    public:

        virtual ~PointCloudPacket() = default;

        /** \brief Packet is valid pointcloud udp packet or not.
        * \return true for yes, false for no.
        */
        virtual bool IsValidPacket(std::string& packet) = 0;

        /** \brief Get device type from the pointcloud packet.
        * \return DeviceType.
        */
        virtual DeviceType GetDeviceType(std::string& packet) = 0;

        /** \brief Get frame num from the pointcloud packet.
        * \return frame Num.
        */
        virtual int GetFrameNum(std::string& packet) = 0;

        /** \brief Get scan mode from the pointcloud packet.
        * \return DeviceType.
        */
        virtual ScanMode GetScanMode(std::string& packet) = 0;

        /** \brief Get the udp sequence number from the pointcloud packet.
        * \return udp sequence number.
        */
        virtual int GetPacketSeq(std::string& packet) = 0;

        /** \brief Get the timestamp from the exciton pointcloud sampleB  packet.
         * \return timestamp in second.
         */
        virtual uint64_t GetTimestamp(uint8_t *data) = 0;

        /** \brief process the data depend the lidar type.
         * \return success or not.
         */
        virtual int ProcessPacket(
            std::string &packet,
            angle_comp_t *angle_comp,
            PointCloud &cloud
        ) = 0;

        /** \brief Reset any cross-packet parser state (e.g. revolution detection).
         *  Default no-op; parsers that accumulate state across packets override it.
         */
        virtual void ResetParserState() {}

        /** \brief Get the azimuth (degrees) of a point-cloud packet cheaply,
         *         without decoding the full point cloud. Used by offline indexers
         *         to detect revolution boundaries. Default returns false.
         */
        virtual bool GetPkgAngle(std::string& packet, double* az_deg)
        {
            (void)packet;
            (void)az_deg;
            return false;
        }

        /** \brief parse the pkg data to points data .
        */
        virtual void Pkg2Points(std::string &packet, std::vector<float>& v_azi_comp, std::vector<float>& v_ele_comp, PointCloud &cloud) = 0;

        /** \brief Packet is valid angle comp packet or not.
        * \return true for yes, false for no.
        */
        virtual bool IsValidAnglePacket(std::string& packet) = 0;

        /** \brief Parse the angle comp package data to angle_comp.
        */
        virtual int ParseAnglePkg(std::string &packet, angle_comp_t &angle_comp) = 0;

        /** \brief Emit the given per-channel angle table as an embedded pcap
         *  meta-record. The table is owned by PointCloudProducer and passed in, the
         *  same way ProcessPacket()/ParseAnglePkg() receive it, so a parser keeps no
         *  angle state of its own. Default no-op; MRZ16 overrides it so an offline
         *  replay can carry angles without an external calibration file. */
        virtual bool GetAngleMetaRecord(const angle_comp_t& angle_comp, std::vector<uint8_t>& out) const
        {
            (void)angle_comp;
            (void)out;
            return false;
        }

        /** \brief Packet is valid imu packet or not.
        * \return true for yes, false for no.
        */
        virtual bool IsValidImuPacket(std::string& packet) = 0;

        /** \brief Parse the imu package data to imu_data.
        */
        virtual int ParseImuPkg(std::string &packet, imu_data_t &imu_data) = 0;

    protected:
        int last_frame = -1; 
    };
}

#endif //end PACKET_H_
