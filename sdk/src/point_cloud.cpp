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
#include <iostream>
#include <functional>
#include <fstream>
#include <thread>
#include <queue>
#include <mutex>

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

    PointCloudProducer::PointCloudProducer(int data_port, std::string lidar_ip, std::string cal_filename = ""):
        cal_(new CalibrationData()),
        points_(new PointCloud()),
        device_ip_(lidar_ip),
        cal_filename_(cal_filename),
        device_type_(LidarUnknown),
        last_seq_(-1),
        data_port_(data_port),
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

            // if port is negative, we need to get the port from lidar by tcp connection
            if (data_port_ < 0)
            {
                // get the port from lidar by tcp
                if (tool.QueryDeviceConfigurationInfo(cfg))
                    return false;
                else
                    data_port_ = cfg.destination_port;

            }

            if (cal_filename_.size())
            {
                if (LidarTools::ReadCalibrationData(cal_filename_, *(this->cal_.get())))
                {
                    LOG_ERROR("Load calibration file error, %s\n", cal_filename_.c_str());
                    return false;
                }
            }
            else
            {
                if (tool.GetDeviceCalibrationData(*(this->cal_.get())))
                    return false;
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
        if (this->device_type_ == LidarUnknown)
        {
            this->device_type_ = PointCloudPacket::GetDeviceType(packet);
            if (LidarML30B1 == this->device_type_)
            {
                //this->last_seq_ = 124;
            }
            else if (LidarML30SA1 == this->device_type_)
            {
                //this->last_seq_ = 159;
            }
            else if (LidarMLX == this->device_type_)
            {
                //this->last_seq_ = 399;
            }
            else
            {
                return;
            }
        }

        //device is unknown or device type and calibration data are not matched
        if ((this->device_type_ == LidarUnknown) || (this->device_type_ != this->cal_->device_type))
            return;

        int seq = PointCloudPacket::GetPacketSeq(packet);
        if (((-1 != this->last_seq_) && (0 != seq)) && (seq != (this->last_seq_ + 1))) //packet loss
        {
            LOG_ERROR("Packet loss, last seq [%3d], current seq[%3d]\n", this->last_seq_, seq);
        }

        // we get last packet by seq, but 10Hz has a 50ms delay issues
        if (seq < this->last_seq_)/*we have get one total frame*/
        {
            this->ProcessOneSweep();
        }

        PointCloudPacket::ProcessPacket(packet, *(this->cal_lut_), *points_);

        this->last_seq_ = seq;
    }

    void PointCloudProducer::ProcessOneSweep()
    {
        //If a new pointcloud processed done, call the callback function.
        int ret = 0;
        if (pointcloud_cb_)
        {
            (pointcloud_cb_)(*(this->points_), ret);
        }

        //If a new pointcloud processed done, push back the deque.
        std::unique_lock<std::mutex> lock(this->mutex_);
        if (this->max_pointcloud_count_ > 0)
        {
            while (this->pointclouds_.size() >= this->max_pointcloud_count_)
            {
                this->pointclouds_.pop_front();
            }
        }
        this->pointclouds_.push_back(this->points_);

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
                        std::string* packet = new std::string(data.c_str(), len);
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
    
    void PointCloudProducer::Consumer()
    {
        std::string* packet = 0;
        while (this->packets_->dequeue(packet))
        {
            this->ProcessLidarPacket(*packet);
			if (packet)
				delete packet;
        }
    }

    void PointCloudProducer::RegisterPointCloudCallback(PointCloudCallback cb)
    {
        this->pointcloud_cb_ = cb;
    }


    int PointCloudProducer::GetPointCloud(PointCloud& points, int timeout_ms)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if (this->pointclouds_.empty())
        {
            //wait_for bug on vs2015&vs2017: https://developercommunity.visualstudio.com/content/problem/438027/unexpected-behaviour-with-stdcondition-variablewai.html
            if (std::cv_status::timeout == cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms)))
            {
                LOG_WARN("Wait for pointcloud timeout.\n");
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
        points = *(this->pointclouds_.front());
        this->pointclouds_.pop_front();
        return 0;
    }

    int  PointCloudProducer::Start()/*start the thread which will handle the udp packet one by one*/
    {
        if (!CheckInit())
            return InitFailure;

        if (!this->packets_)
        {
            this->packets_.reset(new SynchronizedQueue<std::string*>);
        }

        if (!this->receiver_)
        {
            this->receiver_.reset(new UdpReceiver(this->data_port_, 1000));
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
        last_seq_(-1),
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
                return ret;
        }

        return 0;
    }
}
