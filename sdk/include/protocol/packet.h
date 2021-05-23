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

namespace zvision
{
    class PointCloud;

    class CalibrationPacket
    {
    public:

        static const int PACKET_LEN = 1040;

        /** \brief Packet is valid calibration udp packet or not.
        * \return true for yes, false for no.
        */
        static bool IsValidPacket(std::string& packet);

        /** \brief Get scan mode from the cal packet.
        * \return ScanMode.
        */
        static ScanMode GetScanMode(std::string& packet);

        /** \brief Get the udp sequence number from the calibration packet.
        * \return udp sequence number.
        */
        static int GetPacketSeq(std::string& packet);

        /** \brief Get the max udp sequence number by scan mode.
        * \return udp sequence number.
        */
        static int GetMaxSeq(zvision::ScanMode& mode);

        /** \brief Get device type from the cal packet.
        * \return DeviceType.
        */
        DeviceType GetDeviceType();

        /** \brief Get the udp sequence number from the calibration packet.
        * \return udp sequence number.
        */
        int GetPacketSeq();

        /** \brief Process the raw cal udp packet to float(azimuth and elevation in degree format).
        * \param[out] cal             calibration data
        */
        void ExtractData(std::vector<float>& cal);

        /** \brief Process the raw cal udp packet to float(azimuth and elevation in degree format).
        * \param[out] cal             calibration data
        */
        static void ExtractData(std::string& packet, std::vector<float>& cal);

        /** \brief cal upd packet data.
        */
        char cal_data_[1040];

    };

    class PointCloudPacket
    {
    public:

        static const int PACKET_LEN = 1304;

        /** \brief Packet is valid pointcloud udp packet or not.
        * \return true for yes, false for no.
        */
        static bool IsValidPacket(std::string& packet);

        /** \brief Get device type from the pointcloud packet.
        * \return DeviceType.
        */
        static DeviceType GetDeviceType(std::string& packet);

        /** \brief Get scan mode from the pointcloud packet.
        * \return DeviceType.
        */
        static ScanMode GetScanMode(std::string& packet);

        /** \brief Get the udp sequence number from the pointcloud packet.
        * \return udp sequence number.
        */
        static int GetPacketSeq(std::string& packet);

        /** \brief Get the echo count from the pointcloud packet.
        * \return eho count( 1 for single echo and 2 for dual echo ).
        */
        static int GetEchoCount(std::string& packet);

        /** \brief Get the timestamp from the pointcloud packet.
        * \return timestamp in second.
        */
        static double GetTimestamp(std::string& packet);

        /** \brief Get the timestamp from the pointcloud packet.
        * \return timestamp in nano second.
        */
        static uint64_t GetTimestampNS(std::string& packet);

        /** \brief Process a pointcloud udp packet to points.
        * \param[in] packet          udp data packet
        * \param[in] cal_lut         points' cal data in sin-cos format
        * \param[in] cloud           to store the pointcloud
        * \return 0 for ok, others for failure.
        */
        static int ProcessPacket(std::string& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud);

        /** \brief Process a pointcloud udp packet to points.
        * \param[in] packet          class PointCloudPacket
        * \param[in] cal_lut         points' cal data in sin-cos format
        * \param[in] cloud           to store the pointcloud
        * \return 0 for ok, others for failure.
        */
        static int ProcessPacket(PointCloudPacket& packet, CalibrationDataSinCosTable& cal_lut, PointCloud& cloud);

        /** \brief Convert a point data(4 bytes) to x y z.
        * \param[in]  point           pointer to 4 bytes's point data
        * \param[out] dis             points' distance
        * \param[out] ref             points' reflectivity
        * \param[in]  dis_bit         how many bits used for distance
        * \param[in]  ref_bit         how many bits used for reflectivity
        * \return None.
        */
        static void ResolvePoint(const unsigned char* point, ReturnType& return_type, float& dis, int& ref, int dis_bit, int ref_bit);

        /** \brief pointcloud upd packet data.
        */
        char data_[1304];

    };
}

#endif //end PACKET_H_
