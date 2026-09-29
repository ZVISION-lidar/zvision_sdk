//
// MIT License
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


#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <deque>
#include <unordered_map>
#include <condition_variable>
#include "define.h"
#include "protocol/packet.h"

namespace zvision
{
    extern float speed_light;

    class UdpReceiver;
    class SerialClient;

    template <typename T>
    class SynchronizedQueue;

    class PointCloud
    {
    public:
        PointCloud()
            :dev_type(DeviceType::LidarUnknown)
            , scan_mode(ScanMode::ScanUnknown)
            , stamp_us(0)
        {}

        //  device type
        DeviceType dev_type;
        std::vector<Point> points;
        zvision::ScanMode scan_mode;
        uint64_t stamp_us;

        size_t Size() const { return points.size(); }
        float GetX(size_t index) const { return points[index].x; }
        float GetY(size_t index) const { return points[index].y; }
        float GetZ(size_t index) const { return points[index].z; }
    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief PointCloudProducer for get lidar's pointcloud data.
    * \author zvision
    */
    class PointCloudProducer
    {
    public:

        typedef std::function<void(PointCloud&, int& status)> PointCloudCallback;

        /** \brief zvision PointCloudProducer constructor.
        * \param[in] data_port       lidar udp destination port.
        * \param[in] lidar_ip        lidar's ip address
        * \param[in] cal_filename    lidar's calibration file name, if empty string, using online calibration data
        * \param[in] multicast_en    enbale join multicast group
        * \param[in] mc_group_ip     multicast group ip address (224.0.0.0 -- 239.255.255.255)
        */
        PointCloudProducer(int pc_dst_port, std::string lidar_ip, std::string angle_comp_filename, bool multicast_en, std::string mc_group_ip, DeviceType tp);

        /** \brief zvision PointCloudProducer constructor for the MRZ16 serial lidar.
        * \param[in] port_send          MRZ16 command serial port (send "$LDCMD", e.g. /dev/ttyUSB0)
        * \param[in] port_recv          MRZ16 point cloud serial port (e.g. /dev/ttyUSB1)
        * \param[in] angle_comp_filename MRZ16 channel angle file (csv, optional;
        *                  rows "<horizontal_deg>,<vertical_deg>", no channel column)
        * \param[in] tp                 device type, must be LidarMRZ16
        * \param[in] baud_send          baud rate of the command port
        * \param[in] baud_recv          baud rate of the point cloud port
        */
        PointCloudProducer(std::string port_send, std::string port_recv, std::string angle_comp_filename, DeviceType tp, int baud_send = 9600, int baud_recv = 3125000);

        PointCloudProducer() = delete;


        /** \brief zvision PointCloudSource destructor.
        */
        ~PointCloudProducer();


        /** \brief register pointcloud callback.
        * \param[in] cb              callback function
        */
        void RegisterPointCloudCallback(PointCloudCallback cb);

        /** \brief enable or disable saving the lidar pointcoud data .
        * \param[in] en              enable or disable
        */
        void SetPointcloudBufferEnable(bool en);

        /*start the udp handler(ThreadLoop) thread, pop up udp data in queue and process*/
        int Start();

        /*stop the udp handler(ThreadLoop) thread**/
        void Stop();

        /** \brief get pointcloud.
        * \param[out] points          to store the pointcloud data
        * \param[in]  timeout_ms      timeout to waiting for the pointcloud
        * \return 0 for success, others for failure.
        */
        int GetPointCloud(PointCloud& points, int timeout_ms);

        int GetImuData(imu_data_t &imu_data, int timeout_ms);

        /** \brief Emit this producer's per-channel angle table as an embedded pcap
         *         meta-record (MRZ16 only; delegates to pkg_parse). Returns false when
         *         unavailable. */
        bool GetAngleMetaRecord(std::vector<uint8_t>& out) const;

        /** \brief Register a callback that receives every frame exactly as it comes
         *  off the wire (MRZ16 serial: one complete 80-byte EE FF point-cloud frame
         *  or one 34-byte IMU frame per call, invoked on the producer thread). Lets an
         *  external writer persist the raw serial stream, e.g. by wrapping each frame
         *  into a UDP record of a pcap for offline replay. */
        void RegisterRawFrameCallback(std::function<void(const std::string&)> cb);

    protected:

        /** \brief Check the connection to device, if connection is not established, try to connect.
        * \return true for ok, false for failure.
        */
        bool CheckInit();

        /** \brief CheckInit for the MRZ16 serial lidar: open the dual serial ports
        * and resolve the per-channel angles (angle file first, $LDCMD fallback).
        * \return true for ok, false for failure.
        */
        bool CheckInitSerial();

        /** \brief CheckInit for offline replay: build the packet parser for the
        * configured device type and, when a calibration file is configured, load the
        * per-channel angles from it. Neither the network nor the serial ports are
        * touched, so replaying a pcap never depends on a lidar being reachable.
        * The angle table embedded in the recording itself is applied later through
        * update_angle_comp().
        * \return true for ok, false for failure.
        */
        bool CheckInitOffline();

        void reset_points();

        /** \brief Reset the parser's cross-packet state (revolution detection, etc.). */
        void reset_parser_state();

        void update_angle_comp(angle_comp_t &angle_comp);

        /** \brief Process lidar pointcloud udp packet to pointcloud.
        * \param[in]  packet          lidar pointcloud udp packet
        */
        void ProcessLidarPacket(LidarUdpPacket& packet);

        /** \brief Call this function to notify to handle the new pointcloud data(store the data and notify the callback function).
        */
        void ProcessOneFrame();

        /*thread function: get lidar udp packet*/
        void Producer();

        /*thread function: get packet and process to pointcloud**/
        void Consumer();


    private:
        
        /** \brief store the angle comp file name */
        std::string angle_comp_filename_;

        /** \brief store the angle comp */
        std::shared_ptr<angle_comp_t> angle_comp_;

        /** \brief store the lidar udp pcaket*/
        std::shared_ptr<SynchronizedQueue<LidarUdpPacket> > packets_;

        /** \brief store the lidar pointcoud data*/
        std::shared_ptr<PointCloud> points_;

        /** \brief store the lidar pointcoud data for request*/
        std::deque<std::shared_ptr<PointCloud>> pointclouds_;

        /** \brief store the lidar imu data for request*/
        std::deque<std::shared_ptr<imu_data_t>> imu_datas_;

        /** \brief Thread: handle the lidar packet and get the poingcloud*/
        std::shared_ptr<std::thread> consumer_;

        /** \brief Thread: receive lidar packet*/
        std::shared_ptr<std::thread> producer_;

        /** \brief receive udp data packet */
        std::shared_ptr<UdpReceiver> receiver_;

        /** \brief lidar_ip */
        std::string lidar_ip_;

        /** \brief point cloud udp dest ip */
        std::string pc_dst_ip_;

        /** \brief point cloud udp dest port */
        uint16_t pc_dst_port_;

        /** \brief device type define by user */
        DeviceType device_type_usr_;

        /** \brief join the multicast group */
        bool join_multicast_;

        /** \brief device type and scan mode from udp package */
        DeviceType device_type_;
        ScanMode scan_mode_;
        
        bool use_pointcloud_buffer_;
        unsigned int filter_ip_;

        int last_seq_;
        bool init_ok_;
        bool need_stop_;

        PointCloudCallback pointcloud_cb_;

        /** \brief raw-frame sink (MRZ16 serial only), see RegisterRawFrameCallback(). */
        std::function<void(const std::string&)> raw_frame_cb_;

        mutable std::mutex mutex_;
        std::condition_variable cond_;

        mutable std::mutex imu_mutex_;
        std::condition_variable imu_cond_;

        /** \brief pointcloud cache size.*/
        unsigned int max_pointcloud_count_;

        /** \brief imu_data cache size.*/
        unsigned int max_imudata_count_;

        std::unique_ptr<zvision::PointCloudPacket> pkg_parse;

        ////////////////////////////////////////////////////////////////////////
        // MRZ16 serial members
        ////////////////////////////////////////////////////////////////////////

        /** \brief dual serial transport (command port + point cloud port) */
        std::shared_ptr<SerialClient> serial_client_;

        /** \brief command serial port name. */
        std::string serial_port_send_;

        /** \brief point cloud serial port name. */
        std::string serial_port_recv_;

        /** \brief baud rates. */
        int serial_baud_send_ = 9600;
        int serial_baud_recv_ = 3125000;

        /** \brief byte-stream buffer used to slice EE FF frames out of the raw
        * serial input (producer thread only). */
        std::string serial_rx_stream_;

        /** \brief close and reopen the serial ports until the hardware returns
         *  (or need_stop_). True once reconnected, false if the thread must exit. */
        bool ReconnectSerial();
    };
}

#endif //end POINT_CLOUD_H_
