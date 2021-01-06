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


#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>
#include "define.h"


namespace zvision
{
    class UdpReceiver;
    class PcapUdpSource;

    template <typename T>
    class SynchronizedQueue;

    class PointCloud
    {
    public:
        std::vector<Point> points;
        
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
        PointCloudProducer(int data_port, std::string lidar_ip, std::string cal_filename = "", bool multicast_en = false, std::string mc_group_ip = "");


        PointCloudProducer() = delete;


        /** \brief zvision PointCloudSource destructor.
        */
        ~PointCloudProducer();


        /** \brief register pointcloud callback.
        * \param[in] cb              callback function
        */
        void RegisterPointCloudCallback(PointCloudCallback cb);

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

    protected:

        /** \brief Check the connection to device, if connection is not established, try to connect.
        * \return true for ok, false for failure.
        */
        bool CheckInit();

        /** \brief Process lidar pointcloud udp packet to pointcloud.
        * \param[in]  packet          lidar pointcloud udp packet
        */
        void ProcessLidarPacket(std::string& packet);

        /** \brief Call this function to notify to handle the new pointcloud data(store the data and notify the callback function).
        */
        void ProcessOneSweep();

        /*thread function: get lidar udp packet*/
        void Producer();

        /*thread function: get packet and process to pointcloud**/
        void Consumer();


    private:

        /** \brief Calibration data. */
        std::shared_ptr<CalibrationData> cal_;

        /** \brief store the points' calibration data(elevation and azimuth's sin cos)*/
        std::shared_ptr<CalibrationDataSinCosTable> cal_lut_;

        /** \brief store the lidar udp pcaket*/
        std::shared_ptr<SynchronizedQueue<std::string*> > packets_;

        /** \brief store the lidar pointcoud data*/
        std::shared_ptr<PointCloud> points_;

        /** \brief store the lidar pointcoud data for request*/
        std::deque<std::shared_ptr<PointCloud>> pointclouds_;

        /** \brief Thread: handle the lidar packet and get the poingcloud*/
        std::shared_ptr<std::thread> consumer_;

        /** \brief Thread: receive lidar packet*/
        std::shared_ptr<std::thread> producer_;

        /** \brief receive udp data packet */
        std::shared_ptr<UdpReceiver> receiver_;
        std::string device_ip_;
        std::string cal_filename_;
        DeviceType device_type_;
        ScanMode scan_mode_;
        int last_seq_;

        /** \brief end of a full pointcloud's udp seq. It depends on lidar type*/
        int end_seq_;

        unsigned int filter_ip_;
        int data_port_;

        /** \brief lidar data udp destination ip address */
        std::string data_dst_ip_;
        bool join_multicast_;

        bool init_ok_;

        bool need_stop_;

        PointCloudCallback pointcloud_cb_;

        mutable std::mutex mutex_;
        std::condition_variable cond_;

        /** \brief pointclopud cache size.*/
        unsigned int max_pointcloud_count_;

    };


    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief OfflinePointCloudProducer for get lidar's pointcloud data from offline pcap file.
    * \author zvision
    */
    class OfflinePointCloudProducer
    {
    public:

        typedef std::function<void(PointCloud&, int& status)> PointCloudCallback;


        /** \brief zvision PointCloudProducer constructor.
        * \param[in] pcap_filename   offline pcap file name
        * \param[in] cal_filename    lidar's calibration file name
        * \param[in] lidar_ip        lidar's ip address
        * \param[in] data_port       lidar udp destination port.
        */
        OfflinePointCloudProducer(std::string pcap_filename, std::string cal_filename, std::string lidar_ip, int data_port);


        OfflinePointCloudProducer() = delete;


        /** \brief zvision OfflinePointCloudProducer destructor.
        */
        ~OfflinePointCloudProducer();

        /** \brief read file and analysis the pointcloud info.
        * \param[out] size            how many pointcloud in the file
        * \param[out] type            lidar type
        * \return 0 for success, others for failure.
        */
        int GetPointCloudInfo(int& size, DeviceType& type);


        /** \brief get pointcloud.
        * \param[in]  frame_number    pointcloud frame number in the pcap file
        * \param[out] points          to store the pointcloud data
        * \return 0 for success, others for failure.
        */
        int GetPointCloud(int frame_number, PointCloud& points);

    private:

        /*offline pcap filename */
        std::string pcap_filename_;

        std::shared_ptr<PcapUdpSource> packet_source_;

        /*store the points' calibration data(elevation and azimuth's sin cos)*/
        std::shared_ptr<CalibrationDataSinCosTable> cal_lut_;

        /*store the lidar pointcoud data*/
        std::shared_ptr<PointCloud> points_;

        std::string cal_filename_;
        DeviceType device_type_;
        int count_;

        std::string device_ip_;
        int last_seq_;

        int filter_ip_;
        int data_port_;

        bool init_ok_;

        PointCloudCallback pointcloud_cb_;


    };

}

#endif //end POINT_CLOUD_H_
