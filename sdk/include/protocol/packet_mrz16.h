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


#ifndef PACKET_MRZ16_H_
#define PACKET_MRZ16_H_

#include <cstdint>
#include <string>
#include <vector>

#include "define.h"
#include "packet.h"

namespace zvision
{
    ////////////////////////////////////////////////////////////////////////////////
    // MRZ16 protocol definitions. 
    // One point-cloud column frame is 80 bytes on the wire, each frame carries
    // 16 channels at one azimuth; IMU frame is 34 bytes.

    constexpr int mrz16_channel_count = MRZ16_CHANNEL_COUNT;
    constexpr int mrz16_point_cloud_frame_len = MRZ16_POINT_CLOUD_LEN;  // 80
    constexpr int mrz16_imu_frame_len = MRZ16_IMU_LEN;                  // 34

    // Raw value scales (user manual)
    constexpr double mrz16_distance_scale_m = 0.004;    // distance raw * 4 mm -> m
    constexpr double mrz16_azimuth_scale_deg = 0.01;    // azimuth raw * 0.01 deg

    // Number of azimuth columns of one revolution (360 deg / 600 = 0.6 deg per
    // column). This is a property of the lidar: it stays 600 even when some
    // columns of a revolution are lost.
    constexpr uint32_t mrz16_columns_per_revolution = 600;

    // Optical center offsets (m)
    constexpr double mrz16_x_offset_m = 0.00700;
    constexpr double mrz16_y_offset_m = 0.01386;
    constexpr double mrz16_z_offset_m = 0.00504;

    // IMU raw scales: acc LSB = (1/8192) g -> m/s^2; gyro LSB = (1/32.8) deg/s
    constexpr double mrz16_imu_acc_scale = 9.80665 / 8192.0;
    constexpr double mrz16_imu_gyro_scale = 1.0 / 32.8;

    #pragma pack(push, 1)
    struct mrz16_packet_header
    {
        uint8_t head0xEE;
        uint8_t head0xFF;
        uint8_t protocol_major;
        uint8_t protocol_minor;
        uint8_t reserved;
        uint8_t data_type;  // 0 = point cloud, 1 = IMU
    };

    struct mrz16_data_time
    {
        uint8_t year;   // year - 1900
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    };

    struct mrz16_data_header
    {
        mrz16_data_time utc_time;
        uint32_t timestamp_us;
    };

    struct mrz16_channel_data
    {
        uint16_t distance;     // * 4 mm
        uint8_t reflectivity;  // * 1%
    };

    struct mrz16_point_cloud_body
    {
        uint16_t azimuth;  // 0.01 deg
        mrz16_channel_data channels[mrz16_channel_count];
        uint8_t window_contamination[4];
        uint8_t lidar_state;
        uint8_t reserved_id;
        uint8_t reserved_info[2];
        uint16_t udp_sequence;
    };

    struct mrz16_point_cloud_packet
    {
        mrz16_packet_header header;
        mrz16_data_header data_header;
        mrz16_point_cloud_body body;
        uint32_t tail;  // crc
    };

    struct mrz16_imu_body
    {
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
        uint16_t imu_sequence;
    };

    struct mrz16_imu_packet
    {
        mrz16_packet_header header;
        mrz16_data_header data_header;
        mrz16_imu_body body;
        uint32_t tail;  // crc
    };
    #pragma pack(pop)

    static_assert(sizeof(mrz16_point_cloud_packet) == mrz16_point_cloud_frame_len,
                  "mrz16 point cloud frame must be 80 bytes");
    static_assert(sizeof(mrz16_imu_packet) == mrz16_imu_frame_len,
                  "mrz16 imu frame must be 34 bytes");

    /** \brief per-channel vertical/horizontal angles (degree) */
    struct mrz16_channel_angles
    {
        double vertical_deg[mrz16_channel_count]{};
        double horizontal_deg[mrz16_channel_count]{};
        int count = 0;
    };

    class SerialClient;

    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief pkg_parse_mrz16 parse EE FF point-cloud / IMU frames of the MRZ16 lidar.
    *
    * A MRZ16 revolution (one complete 360 deg scan) is assembled from consecutive
    * 80-byte point-cloud frames. The parser detects the 0 <-> 360 deg wrap of the
    * azimuth counter and reports it with the same return convention used by the
    * EZ6/NZ1 parsers inside PointCloudProducer::ProcessLidarPacket:
    *   - 0 : this frame belongs to the frame being accumulated
    *   - 1 : (unused)
    *   - 2 : this frame starts a new revolution; the previously accumulated
    *         cloud is a complete frame (already finalized) and should be flushed.
    */
    class pkg_parse_mrz16 : public PointCloudPacket
    {
    public:
        pkg_parse_mrz16() = default;
        virtual ~pkg_parse_mrz16() = default;

        // ---------------- MRZ16 specific (used by the serial producer path) ----------------

        /** \brief Load per-channel angles from a CSV angle file.
        *
        * Format: one row per channel, '<horizontal_deg>,<vertical_deg>'.
        * The file carries no channel-number column; the zero-based row index
        * is the channel number.
        * \param[out] out  filled with the parsed table; the caller (the producer)
        *                  owns it, so the parser holds no angle state.
        * \return true when the file was parsed successfully.
        */
        bool LoadChannelAnglesFile(const std::string& path, angle_comp_t* out, std::string* err);

        /** \brief Ask the lidar for its per-channel angles through $LDCMD / $LDACK
        * over the dual serial transport.
        * \param[out] out  filled with the received table; the caller (the producer)
        *                  owns it.
        * \return true when an $LDACK with 16 channels was received.
        */
        bool FetchChannelAnglesOverSerial(zvision::SerialClient* serial, angle_comp_t* out, std::string* err);

        /** \brief Serialize the given per-channel angle table into an embedded pcap
         *         meta-record. Returns false when no angles are available. */
        bool GetAngleMetaRecord(const angle_comp_t& angle_comp, std::vector<uint8_t>& out) const override;

        /** \brief Pop the next complete EE FF frame (80-byte point cloud or
        * 34-byte IMU) from a raw serial byte stream. Resyncs when the stream is
        * corrupted. \return true when *frame holds a complete frame.
        */
        static bool ExtractNextFrame(std::string& stream, std::string* frame);

        // ---------------- PointCloudPacket interface ----------------

        bool IsValidPacket(std::string& packet) override;

        DeviceType GetDeviceType(std::string& packet) override;

        int GetFrameNum(std::string& packet) override;

        /** \brief Cheap azimuth (deg) read of a point-cloud packet, used by the
         *         offline indexer to detect revolution boundaries without decoding
         *         the full point cloud. */
        bool GetPkgAngle(std::string& packet, double* az_deg) override;

        ScanMode GetScanMode(std::string& packet) override;

        int GetPacketSeq(std::string& packet) override;

        uint64_t GetTimestamp(uint8_t* data) override;

        int ProcessPacket(std::string& packet, angle_comp_t* angle_comp,
                          PointCloud& cloud) override;

        /** \brief convert one point-cloud packet (one rotating column) into points of cloud.
         *
         *  Used by ProcessPacket for every received frame, same as the other lidars.
         *  MRZ16 geometry is driven by the per-channel angles carried in angle_comp_t
         *  (v_azi_comp = horizontal, v_ele_comp = vertical) plus the frame azimuth from
         *  the packet. The column index of every point is derived from the packet
         *  sequence of the frame (see col_base_seq_).
         */
        void Pkg2Points(std::string& packet, std::vector<float>& v_azi_comp,
                        std::vector<float>& v_ele_comp, PointCloud& cloud) override;

        bool IsValidAnglePacket(std::string& packet) override;

        int ParseAnglePkg(std::string& packet, angle_comp_t& angle_comp) override;

        bool IsValidImuPacket(std::string& packet) override;

        int ParseImuPkg(std::string& packet, imu_data_t& imu_data) override;

        /** \brief Reset revolution-detection state so the next frame is treated
         *         as the start of a new revolution. */
        void ResetParserState() override;

        /** \brief Build the embedded pcap angle meta-record bytes from angle_comp_t. */
        static void BuildAngleMetaRecord(const angle_comp_t& ac, std::vector<uint8_t>& out);

    private:

        /** \brief Clear revolution-detection state (azimuth / column origin) so the
         *         next frame is treated as the start of a new revolution; called by
         *         ResetParserState(). */
        void ResetRevolutionState();

        /** \brief azimuth (deg) of the last point-cloud frame. */
        double last_azimuth_deg_ = -1.0;

        /** \brief packet sequence (body.udp_sequence) of the first frame of the current
         *         revolution; column indices are measured against it, so they do not
         *         depend on how many frames were actually received. */
        uint16_t col_base_seq_ = 0;

        /** \brief distance filtering range (m). */
        double min_distance_m_ = 0.04;
        double max_distance_m_ = 200.0;
    };
}

#endif // end PACKET_MRZ16_H_
