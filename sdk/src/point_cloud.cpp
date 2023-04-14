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


#include "client.h"
#include "point_cloud.h"
#include "lidar_tools.h"
#include "packet.h"
#include "packet_source.h"
#include "print.h"
#include "loguru.hpp"
#include <iostream>
#include <functional>
#include <fstream>
#include <thread>
#include <queue>
#include <mutex>
#include <cmath>
#include <math.h>

namespace zvision
{
    template <typename T>
    class SynchronizedQueue/*store the lidar udp packet*/
    {
    public:
        SynchronizedQueue() :
            queue_(), 
            mutex_(), 
            cond_(), 
            request_to_end_(false), 
            enqueue_data_(true)
        {
        }

        bool enqueue(const T& data)
        {
            std::unique_lock<std::mutex> lock(mutex_);

            if (enqueue_data_)
            {
                queue_.push(data);
                cond_.notify_one();
                return true;
            }
            else
            {
                return false;
            }
        }

        bool dequeue(T& result)
        {
            std::unique_lock<std::mutex> lock(mutex_);

            while (queue_.empty() && (!request_to_end_))
            {
                cond_.wait(lock);
            }

            if (request_to_end_)
            {
                doEndActions();
                return false;
            }

            result = queue_.front();
            queue_.pop();

            return true;
        }

        void stopQueue()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            request_to_end_ = true;
            cond_.notify_one();
        }

        unsigned int size()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            return static_cast<unsigned int>(queue_.size());
        }

        bool isEmpty() const
        {
            std::unique_lock<std::mutex> lock(mutex_);
            return (queue_.empty());
        }

    private:
        void doEndActions()
        {
            enqueue_data_ = false;

            while (!queue_.empty())
            {
                queue_.pop();
            }
        }

        std::queue<T> queue_;            //udp packet queue
        mutable std::mutex mutex_;     //data access 
        std::condition_variable cond_; // The condition to wait for

        bool request_to_end_;
        bool enqueue_data_;
    };

    LidarPointsFilter::LidarPointsFilter()
        :downsample_(zvision::DownSampleMode::DownsampleUnknown)
        , scan_mode_(zvision::ScanMode::ScanUnknown)
        , cfg_file_path_("")
        , init_(false)
        , uncover_cnt_(-1)
    {}

    LidarPointsFilter::~LidarPointsFilter()
    {}

    void LidarPointsFilter::FilterBucklingPointCloud(PointCloud& cloud)
    {        
        if (cloud.scan_mode != ScanMode::ScanML30SA1Plus_160)
            return;
        bool is_ml30splus_b1_ep_mode = zvision::is_ml30splus_b1_ep_mode_enable();
        // for ml30s+b1 ep1 fov0-fov7
        int npoints = 51200;
        int points_per_group = 8;
        int points_per_line = 80;
        if (!is_ml30splus_b1_ep_mode)
        {
            points_per_group = 4;
        }
        static const int blks = 16;
        static float RMS[4][3] = { 0.0433094, -0.016089,  0.0151864,
                                    0.0398198, -0.0177786, -0.0101136,
                                   -0.0398198, -0.0177786, -0.0101136,
                                   -0.0433094, -0.016089,  0.0151864 };
        static float rms_mod[4] = { 0 };
        if (rms_mod[0] < 1e-5)
        {
            for (int i = 0; i < 4; i++)
            {
                rms_mod[i] = std::sqrt(std::pow(RMS[i][0], 2) + std::pow(RMS[i][1], 2) + std::pow(RMS[i][2], 2));
            }
        }

        static int fov_rm_id[8] = { 0,1,2,3,0,1,2,3 };
        if (cloud.points.size() == npoints)
        {
            int groups = npoints / points_per_group;
            for (int g = 0; g < groups; g++)
            {
                for (int p = 0; p < points_per_group; p++)
                {
                    int id = g * points_per_group + p;
                    auto& point = cloud.points.at(id);

                    if (point.distance < 3)
                        continue;

                    // calculate
                    int rm_id = fov_rm_id[p];
                    float mult_p_rm = RMS[rm_id][0] * point.x \
                        + RMS[rm_id][1] * point.y \
                        + RMS[rm_id][2] * point.z;

                    float theta = std::acos((mult_p_rm) / (rms_mod[rm_id] * point.distance));
                    float delta_azi = 0;
                    float delta_ele = 0;
                    {
                        float sig = 1.0f;
                        if (rm_id >= 2)
                            sig = -1.0f;

                        delta_azi = sig * rms_mod[rm_id] * (1.0 / 2 - 1.0 / point.distance) * std::sin(theta) * std::cos(point.ele);
                        delta_ele = sig * rms_mod[rm_id] * (1.0 / 2 - 1.0 / point.distance) * std::sin(theta) * std::sin(point.ele);

                        // repair layer
                        if (point.fov == 1 || point.fov == 5 || point.fov == 2 || point.fov == 6)
                        {
                            float ratio = 1.0f;
                            static float blk_ratio[blks] = { 0.0f, 0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
                            // direct_right: 0 <- , 1 ->
                            int direct_right = (g / points_per_line) % 2;
                            int pos = g % points_per_line;
                            if (direct_right)
                                pos = points_per_line - 1 - pos;

                            int blk = pos / 5;
                            if (point.fov == 1 || point.fov == 5)
                            {
                                ratio = blk_ratio[blk];
                            }
                            else if (point.fov == 2 || point.fov == 6)
                            {
                                blk = blks - 1 - blk;
                                ratio = blk_ratio[blk];
                            }

                            delta_azi = delta_azi * ratio;
                            delta_ele = delta_ele * ratio;
                        }

                        // update pointcloud data
                        point.ele = point.ele + delta_ele;
                        point.azi = point.azi + delta_azi;
                        point.x = point.distance * std::cos(point.ele) * std::sin(point.azi);
                        point.y = point.distance * std::cos(point.ele) * std::cos(point.azi);
                        point.z = point.distance * std::sin(point.ele);
                    }
                }
            }
        }
    }

    zvision::DownSampleMode LidarPointsFilter::GetDownsampleMode(zvision::ScanMode mode)
    {
        if (downsample_ == zvision::Downsample_cfg_file)
        {
            if (mode != scan_mode_)
            {
                return zvision::DownsampleUnknown;
            }
        }

        return downsample_;
    }

    void LidarPointsFilter::Init(zvision::DownSampleMode mode, std::string cfg_path, bool is_ml30sp_b1_ep) {

        init_ = true;
        if (mode != zvision::DownSampleMode::Downsample_cfg_file) {
            downsample_ = mode;
            scan_mode_ = zvision::ScanML30SA1_160;
            return;
        }

        if (cfg_path.empty())
            return;

        // load cfg file data
        const int table_size = 51200;
        const int total_lines = 640;
        const int bytes_per_line = 10;
        const int lines_per_fov = 80;
        const int points_in_fov = 6400;

        try {
            // for ml30s/ml30splus device, update cover table
            cover_table_.resize(table_size, 1);
            uncover_cnt_ = table_size;

            // get file type
            int header_lines = 0;
            {
                std::ifstream in(cfg_path.c_str(), std::ios::in);
                if (!in.is_open())
                    return;
                std::string line;
                std::string ml30s_tag = "Mode ML30S_160";
                std::string ml30sp_tag = "Mode ML30SPlus_160";
                while (std::getline(in, line))
                {
                    header_lines++;
                    if (line.compare(0, ml30s_tag.size(), ml30s_tag.c_str(), 0, ml30s_tag.size()) == 0)
                    {
                        scan_mode_ = zvision::ScanML30SA1_160;
                        break;
                    }
                    else if (line.compare(0, ml30sp_tag.size(), ml30sp_tag.c_str(), 0, ml30sp_tag.size()) == 0)
                    {
                        scan_mode_ = zvision::ScanML30SA1Plus_160;
                        break;
                    }
                }
                in.close();
                if (scan_mode_ == zvision::ScanUnknown)
                    header_lines = 0;
            }


            std::ifstream in(cfg_path.c_str(), std::ios::in);
            if (!in.is_open())
                return;

            uint8_t flg = 0x80;
            uint8_t fov_in_group[8] = { 0, 2, 4, 6, 5, 7, 1, 3 };
            uint8_t fov_in_group_30sp_b1_ep[8] = { 0, 1, 2, 3, 4, 5, 6, 7};
            std::string line;
            int id = 0;
            int header_id = 0;
            while (std::getline(in, line))
            {
                if ((header_lines > 0) && (header_id < header_lines))
                {
                    header_id++;
                    continue;
                }

                // check
                if (line.size() == 0)
                    continue;
                if (line.at(0) == '#')
                    continue;

                if (line.size() < 20)
                    break;

                if (id >= total_lines)
                    break;

                // get value
                uint8_t masks[10] = { 0xFF };
                for (int b = 0; b < 10; b++) {
                    char h = line.at(b * 2);
                    char l = line.at(b * 2 + 1);
                    masks[b] = hex2uint8(h) << 4 | hex2uint8(l);
                }

                // update
                for (int j = 0; j < bytes_per_line; j++) {
                    for (int k = 0; k < 8; k++) {
                        flg = 0x80 >> k;
                        if ((flg & masks[j]) != flg) {
                            int fov = id / lines_per_fov;
                            int point_id = -1;
                            if (scan_mode_ == zvision::ScanML30SA1Plus_160)
                            {
                                if (is_ml30sp_b1_ep)
                                {
                                    int group = (id * bytes_per_line * 8 + j * 8 + k) % points_in_fov;
                                    point_id = group * 8 + fov_in_group_30sp_b1_ep[fov];
                                }
                                else
                                {
                                    int group = (id * bytes_per_line * 8 + j * 8 + k) % points_in_fov;
                                    if (fov < 4)
                                        point_id = group * 4 + fov;
                                    else
                                        point_id = group * 4 + table_size / 2 + fov - 4;
                                }
                            }
                            else
                            {
                                int group = (id * bytes_per_line * 8 + j * 8 + k) % points_in_fov;
                                point_id = group * 8 + fov_in_group[fov];
                            }
                            if (point_id >= table_size || point_id < 0)
                                continue;
                            cover_table_.at(point_id) = 0;
                            uncover_cnt_--;
                        }
                    }
                }
                id++;
            }

            in.close();
            if (id != total_lines) {
                cover_table_.resize(table_size, 1);
                uncover_cnt_ = table_size;
            }
            else
            {
                // not found version tag, default ScanML30SA1_160
                if (scan_mode_ == zvision::ScanUnknown)
                    scan_mode_ = zvision::ScanML30SA1_160;
            }
        }
        catch (std::exception e)
        {
            cover_table_.resize(table_size, 1);
            uncover_cnt_ = table_size;
            return;
        }

        downsample_ = mode;
        cfg_file_path_ = cfg_path;
    }

    zvision::ScanMode LidarPointsFilter::GetScanMode() {
        return scan_mode_;
    }

    int LidarPointsFilter::GetPointsCoutFromCfgFile(int& cnt) {
        if (init_ && uncover_cnt_ > 0)
            cnt = uncover_cnt_;
        return 0;
    }

    bool LidarPointsFilter::IsLidarPointCovered(uint32_t id) {
        if (!init_ || id >= cover_table_.size())
            return false;

        // get point cover state
        return cover_table_.at(id) == 0;
    }

    uint8_t LidarPointsFilter::hex2uint8(char c) {

        uint8_t val = 0x0F;
        if (c >= 'A' && c <= 'Z')
            val = c - 'A' + 10;
        else if (c >= 'a' && c <= 'z')
            val = c - 'a' + 10;
        else if (c >= '0' && c <= '9')
            val = c - '0';
        return val;
    }

    PointCloudProducer::PointCloudProducer(int data_port, std::string lidar_ip, std::string cal_filename, bool multicast_en, std::string mc_group_ip, DeviceType tp) :
        cal_(new CalibrationData()),
        points_(new PointCloud()),
        device_ip_(lidar_ip),
        cal_filename_(cal_filename),
        device_type_(LidarUnknown),
		device_type_usr_(tp),
        scan_mode_(ScanUnknown),
        last_seq_(-1),
        data_port_(data_port),
        data_dst_ip_(mc_group_ip),
        join_multicast_(multicast_en),
        init_ok_(false),
        need_stop_(false),
        pointcloud_cb_(nullptr),
        mutex_(),
        cond_(),
        max_pointcloud_count_(200)
    {
    }

    PointCloudProducer::~PointCloudProducer()
    {
        Stop();
    }

    bool PointCloudProducer::CheckInit()
    {
        if (!init_ok_)
        {
            if (!StringToIp(device_ip_, filter_ip_))
            {
                return false;
            }

            LidarTools tool(this->device_ip_, 5000, 5000, 5000);
            DeviceConfigurationInfo cfg;

            if (!StringToIp(device_ip_, filter_ip_))
            {
                return false;
            }

			int ret = -1;
            // if port is negative or auto join multicast, we need to get the cfg from lidar by tcp connection
            if ((data_port_ < 0) || (join_multicast_ && (!data_dst_ip_.size())))
            {
                // get the cfg from lidar by tcp

				if (device_type_usr_ == DeviceType::LidarMl30SA1Plus)
                {
					ret = tool.QueryML30sPlusDeviceConfigurationInfo(cfg);
				}
                else if (device_type_usr_ == DeviceType::LidarMl30SB1Plus)
                {
                    ret = tool.QueryML30sPlusB1DeviceConfigurationInfo(cfg);
                }
				else {
					ret = tool.QueryDeviceConfigurationInfo(cfg);
				}

                if (ret)
                {
                    LOG_F(ERROR, "Query device configuration info failed.");
                    if (data_port_ < 0)
                        LOG_F(ERROR, "Please specify the data port and retry.");
                    if (join_multicast_)
                        LOG_F(ERROR, "No multicast group is joined.");

                    return false;
                }
                else
                {
                    if (data_port_ < 0)
                    {
                        data_port_ = cfg.destination_port;
                        LOG_F(INFO, "Query device destination port ok, port is %d.", data_port_);
                    }
                    if (join_multicast_ && (!data_dst_ip_.size()))
                    {
                        data_dst_ip_ = cfg.destination_ip;
                        LOG_F(INFO, "Query device multicast address ok, group is %s.", data_dst_ip_.c_str());
                    }
                }
            }

            if (cal_filename_.size())
            {
                if (LidarTools::ReadCalibrationData(cal_filename_, *(this->cal_.get())))
                {
                    LOG_F(ERROR, "Load calibration file error, %s", cal_filename_.c_str());
                    return false;
                }
            }
            else
            {
				if (device_type_usr_ == DeviceType::LidarMl30SA1Plus)
                {
					zvision::JsonConfigFileParam param;
					ret = tool.RunML30sPlusDeviceManager(zvision::EML30SPlusCmd::read_cali_packets, &param);
					if (ret != 0) {
						LOG_F(ERROR, "Get lidar[%s]`s calibration packets error", device_ip_.c_str());
						return false;
					}

					zvision::CalibrationPackets cal_pkts = param.temp_recv_packets;
					if (0 != (ret = LidarTools::GetDeviceCalibrationData(cal_pkts, *(this->cal_.get())))) {
						LOG_F(ERROR, "Convert lidar[%s]`s calibration packets error", device_ip_.c_str());
						return false;
					}
				}
                else if (device_type_usr_ == DeviceType::LidarMl30SB1Plus)
                {
                    zvision::CalibrationPackets cal_pkts;
                    ret = tool.GetML30sPlusB1DeviceCalibrationPackets(cal_pkts);
                    if (ret != 0) {
                        LOG_F(ERROR, "Get lidar[%s]`s calibration packets error", device_ip_.c_str());
                        return false;
                    }
                    if (0 != (ret = LidarTools::GetDeviceCalibrationData(cal_pkts, *(this->cal_.get())))) {
                        LOG_F(ERROR, "Convert lidar[%s]`s calibration packets error", device_ip_.c_str());
                        return false;
                    }
                }
				else {
					if (tool.GetDeviceCalibrationData(*(this->cal_.get())))
						return false;
				}
            }

            if (!this->cal_lut_)
                this->cal_lut_.reset(new CalibrationDataSinCosTable());
            LidarTools::ComputeCalibrationSinCos(*(this->cal_.get()), *(this->cal_lut_.get()));
            init_ok_ = true;
            return true;    
        }

        return true;
    }

    void PointCloudProducer::ProcessLidarPacket(std::string& packet)
    {
        //pkt content len: 42 + 1304
        if (packet.size() != 1304)
        {
            return;
        }

        //find lidar type
        if (this->scan_mode_ == ScanUnknown)
        {
            this->device_type_ = PointCloudPacket::GetDeviceType(packet);
            this->scan_mode_ = PointCloudPacket::GetScanMode(packet);
            //if (ScanML30B1_100 == this->scan_mode_)
            //{
            //    //this->last_seq_ = 124;
            //}
            //else if ((ScanMode::ScanML30SA1_160 == this->scan_mode_) || (ScanMode::ScanML30SA1_160_1_2 == this->scan_mode_) || (ScanMode::ScanML30SA1_160_1_4 == this->scan_mode_))
            //{
            //    //this->last_seq_ = 159;
            //}
            //else if (ScanMode::ScanMLX_160 == this->scan_mode_)
            //{
            //    //this->last_seq_ = 399;
            //}
            //else if (ScanMode::ScanMLX_190 == this->scan_mode_)
            //{
            //    //this->last_seq_ = 474;
            //}
            //else
            //{
            //    LOG_F(ERROR, "Unknown scan mode %d.", this->scan_mode_);
            //    return;
            //}
        }

        //scan mode is unknown or scan mode and calibration data are not matched
        if ((this->scan_mode_ == ScanUnknown) || (this->scan_mode_ != this->cal_->scan_mode))
            return;

        int seq = PointCloudPacket::GetPacketSeq(packet);
        if (((-1 != this->last_seq_) && (0 != seq)) && (seq != (this->last_seq_ + 1))) //packet loss
        {
            LOG_F(ERROR, "Packet loss, last seq [%3d], current seq[%3d].", this->last_seq_, seq);
        }

        if (ScanMode::ScanML30SA1Plus_160 == this->scan_mode_)
        {
            if (((seq < this->last_seq_) && (this->last_seq_ != 159)) || (159 == seq))
            {
                int ret = PointCloudPacket::ProcessPacket(packet, *(this->cal_lut_), *points_, &points_filter_);
                if (0 != ret)
                    LOG_F(WARNING, "ProcessPacket error, %d.", ret);

                this->ProcessOneSweep();
                this->last_seq_ = seq;
                return;
            }

        }
        // we get last packet by seq, but 10Hz has a 50ms delay issues
        else if (seq < this->last_seq_)/*we have get one total frame*/
        {
            this->ProcessOneSweep();
        }

        int ret = PointCloudPacket::ProcessPacket(packet, *(this->cal_lut_), *points_, &points_filter_);
        if(0 != ret)
            LOG_F(WARNING, "ProcessPacket error, %d.", ret);

        this->last_seq_ = seq;
    }

    void PointCloudProducer::ProcessOneSweep()
    {
        // manu downsample
        std::shared_ptr<PointCloud> ds_points;
        if ((points_filter_.GetDownsampleMode(this->scan_mode_) == Downsample_1_2 || \
            points_filter_.GetDownsampleMode(this->scan_mode_) == Downsample_1_4 || \
            points_filter_.GetDownsampleMode(this->scan_mode_) == Downsample_cfg_file) && \
            ((ScanML30SA1_160 == this->scan_mode_ || ScanML30SA1Plus_160 == this->scan_mode_) && (this->points_->points.size() == 51200)))
        {
            ds_points.reset(new PointCloud());
            if (points_filter_.GetDownsampleMode(this->scan_mode_) == Downsample_1_2)
                ds_points->points.resize(51200 / 2);
            else if (points_filter_.GetDownsampleMode(this->scan_mode_) == Downsample_1_4)
                ds_points->points.resize(51200 / 4);
            else if (points_filter_.GetDownsampleMode(this->scan_mode_) == Downsample_cfg_file) {
                int cnt = 51200;
                points_filter_.GetPointsCoutFromCfgFile(cnt);
                ds_points->points.resize(cnt);
            }

            int id = 0;
            for (int i = 0; i < this->points_->points.size(); i++) {
                if (this->points_->points[i].valid == 1) {
                    if (id >= ds_points->points.size())
                        break;

                    ds_points->points[id] = this->points_->points[i];
                    id++;
                }
            }
        }
        else {
            ds_points = points_;
        }

        // manu filter
        LidarPointsFilter::FilterBucklingPointCloud(*(ds_points.get()));


        //If a new pointcloud processed done, call the callback function.
        int ret = 0;
        if (pointcloud_cb_)
        {
            (pointcloud_cb_)(*ds_points, ret);
        }

        //If a new pointcloud processed done, push back the deque.
        std::unique_lock<std::mutex> lock(this->mutex_);
        {
            if (this->max_pointcloud_count_ > 0)
            {
                while (this->pointclouds_.size() >= this->max_pointcloud_count_)
                {
                    this->pointclouds_.pop_front();
                }
            }
            this->pointclouds_.push_back(ds_points);
        }

        //Allocate a new pointcloud for next one.
        this->points_.reset(new PointCloud());

        //Notify a new pointcloud available.
        cond_.notify_one();
        return;

    }

    void PointCloudProducer::Producer()
    {
        if (this->receiver_)
        {
            uint32_t ip;
            int len;
            int ret = 0;
            while (!need_stop_)
            {
                std::string data(2048, '0');
                ret = receiver_->SyncRecv(data, len, ip);
                if (ret >= 0)
                {
                    if ((len > 0) && (ip == this->filter_ip_))
                    {
                        //std::string* packet = new std::string(data.c_str(), len);
                        std::string packet = std::string(data.c_str(), len);
                        this->packets_->enqueue(packet);
                    }
                }
                else
                {
                    return;
                }
            }
        }
    }

    void PointCloudProducer::InternalProducer()
    {
        
    
    
    }
    
    void PointCloudProducer::Consumer()
    {
        std::string packet;
        while (this->packets_->dequeue(packet))
        {
            this->ProcessLidarPacket(packet);

            // process blooming packet

        }
    }

    void PointCloudProducer::RegisterPointCloudCallback(PointCloudCallback cb)
    {
        this->pointcloud_cb_ = cb;
    }


    int PointCloudProducer::GetPointCloud(PointCloud& points, int timeout_ms)
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);

            if (this->pointclouds_.empty())
            {
                //wait_for bug on vs2015&vs2017: https://developercommunity.visualstudio.com/content/problem/438027/unexpected-behaviour-with-stdcondition-variablewai.html
                if (std::cv_status::timeout == cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms)))
                {
                    LOG_F(WARNING, "Wait for pointcloud timeout.");
                    return Timeout;
                }
                else
                {
                    if (this->pointclouds_.empty())
                    {
                        return Unknown;
                    }
                }
            }
        }

        std::unique_lock<std::mutex> lock(mutex_);
        {
            points = *(this->pointclouds_.front());
            this->pointclouds_.pop_front();
        }

        // match blooming pointcloud





        return 0;
    }

    void PointCloudProducer::setDownsampleMode(zvision::DownSampleMode mode, std::string cfg_path) {
        points_filter_.Init(mode, cfg_path, zvision::is_ml30splus_b1_ep_mode_enable());
    }

    int  PointCloudProducer::Start()/*start the thread which will handle the udp packet one by one*/
    {
        if (!CheckInit())
            return InitFailure;

        if (!this->packets_)
        {
            this->packets_.reset(new SynchronizedQueue<std::string>);
        }

        if (!this->receiver_)
        {
            this->receiver_.reset(new UdpReceiver(this->data_port_, 1000));

            if (join_multicast_ && data_dst_ip_.size())
            {
                unsigned int dst_ip_int = 0;
                if (StringToIp(data_dst_ip_, dst_ip_int))
                {
                    if ((dst_ip_int & 0xF0000000) == 0xE0000000)
                    {
                        this->receiver_->JoinMulticastGroup(data_dst_ip_);
                        LOG_F(INFO, "Join multicast group %s.", data_dst_ip_.c_str());
                    }
                    else
                    {
                        //LOG_F(WARNING, "Invalid multicast group ip %s.", data_dst_ip_.c_str());
                    }
                }
                else
                {
                    LOG_F(ERROR, "Resolve destination ip address error, %s.", data_dst_ip_.c_str());
                }
            }
        }

        if (!this->consumer_)
        {
            this->consumer_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&PointCloudProducer::Consumer, this)));
        }

        if (!this->producer_)
        {
            this->producer_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&PointCloudProducer::Producer, this)));
        }
        return 0;
    }

    void PointCloudProducer::Stop()/*start the thread*/
    {
        this->need_stop_ = true;

        if (this->packets_)
        {
            this->packets_->stopQueue();
        }

        if (this->consumer_)
        {
            this->consumer_->join();
            this->consumer_.reset();
        }

        if (this->producer_)
        {
            this->producer_->join();
            this->producer_.reset();
        }

        if (this->receiver_)
        {
            this->receiver_.reset();
        }
    }


    OfflinePointCloudProducer::OfflinePointCloudProducer(std::string pcap_filename, std::string cal_filename, std::string lidar_ip, int data_port):
        pcap_filename_(pcap_filename),
        cal_lut_(new CalibrationDataSinCosTable()),
        cal_filename_(cal_filename),
        device_type_(LidarUnknown),
        count_(0),
        device_ip_(lidar_ip),
        ///last_seq_(-1),
        data_port_(data_port),
        init_ok_(false),
        pointcloud_cb_(nullptr)
    {

    }

    OfflinePointCloudProducer::~OfflinePointCloudProducer()
    {

    }

    int OfflinePointCloudProducer::GetPointCloudInfo(int& size, DeviceType& type)
    {
        if (init_ok_)
        {
            type = this->device_type_;
            size = this->count_;
            return 0;
        }

        this->packet_source_.reset(new PcapUdpSource(this->device_ip_, this->data_port_, this->pcap_filename_));

        int ret = 0;
        if (0 != (ret = this->packet_source_->ReadFrameInfo(count_, type)))
            return ret;

        if (cal_filename_.size())
        {
            CalibrationData cal;
            if (0 != (ret = LidarTools::ReadCalibrationData(cal_filename_, cal)))
                return ret;
            LidarTools::ComputeCalibrationSinCos(cal, *(this->cal_lut_.get()));
        }

        size = count_;
        this->device_type_ = type;
        init_ok_ = true;
        return 0;
    }

    int OfflinePointCloudProducer::GetPointCloud(int frame_number, PointCloud& points)
    {
        if (!init_ok_)
            return NotInit;

        //get one full pointcloud's udp packets
        std::vector<PointCloudPacket> packets;
        int ret = 0;
        if (0 != (ret = this->packet_source_->GetPointCloudPackets(frame_number, packets)))
            return ret;

        //process udp packets and generate poincloud
        for (unsigned int i = 0; i < packets.size(); ++i)
        {
            if (0 != (ret = PointCloudPacket::ProcessPacket(packets[i], *(this->cal_lut_.get()), points)))
            {
                return ret;
            }
        }

        // manu filter
        LidarPointsFilter::FilterBucklingPointCloud(points);

        // try to get blooming data
        if (points.is_ptp_mode) 
        {

            std::vector<BloomingPacket> blo_pkts;
            // get matched blooming packets and generate blooming pointcloud
            this->packet_source_->GetBloomingPackets(frame_number, points.timestamp, blo_pkts);

            // process packet
            if (blo_pkts.size())
            {
                if (!points.blooming_frame)
                    points.blooming_frame = std::make_shared<BloomingFrame>();
                
                InternalPacketHeader header;
                std::string pkt_0((char*)(blo_pkts[0].data), BloomingPacket::PACKET_LEN);
                if (InternalPacket::GetFrameResolveInfo(pkt_0, header))
                {
                    for (unsigned int i = 0; i < blo_pkts.size(); ++i)
                    {
                        std::string pkt((char*)(blo_pkts[i].data), BloomingPacket::PACKET_LEN);
                        if (0 != (ret = BloomingPacket::ProcessPacket(pkt, *(this->cal_lut_.get()), *points.blooming_frame, &header)))
                        {
                            break;
                        }

                        if (i == 0) 
                        {
                            int seq = BloomingPacket::GetPacketSeq(pkt);
                            points.blooming_frame->sys_stamp = blo_pkts[0].sys_stamp - 1e-6 * BloomingPacket::DELTA_PACKRT_US * seq;
                        }
                    }

                    points.use_blooming = true;
                }
            }
        }
        return 0;
    }

    int OfflinePointCloudProducer::GetCalibrationDataSinCosTable(zvision::CalibrationDataSinCosTable& cal)
    {
       if (!cal_lut_)
            return -1;
        cal = *(cal_lut_.get());
        return 0;
    }
}
