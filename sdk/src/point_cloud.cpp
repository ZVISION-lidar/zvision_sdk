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
#include "define.h"
#include "packet.h"
#include "point_cloud.h"
#include "tcp_ez6_b2.h"
#include "tcp_nz1_a2.h"
#include "tcp_tools.h"
#include "packet_ez6_b2.h"
#include "packet_nz1_a2.h"
#include "packet_mrz16.h"
#include "serial_client.h"
#include "print.h"
#include "loguru.hpp"
#include <iostream>
#include <functional>
#include <fstream>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <cmath>
#include <math.h>

namespace zvision
{
    float speed_light = SPEED_US;

    /**
    *@ brief thread safe queue, used to store LiDAR UDP packets
    *The data types stored in the tparam T queue
    */
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
        /**
        *@ brief Join the team operation, put the data into the queue
        *@ param data: Data waiting to join the team
        *@ return true indicates successful joining of the queue, false indicates the queue has stopped receiving data
        */
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
        /**
        *@ brief team out operation, retrieve an element from the queue
        *Location of data storage for param result team departure
        *@ return true indicates successful data retrieval, false indicates queue has stopped
        */
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
        /**
        *@ brief Stop queue, wake up waiting thread and clear queue
        */
        void stopQueue()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            request_to_end_ = true;
            cond_.notify_one();
        }
        /**
        *@ brief Get the current size of the queue
        *The number of elements in the return queue
        */
        unsigned int size()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            return static_cast<unsigned int>(queue_.size());
        }
        /**
        *@ brief Check if the queue is empty
        *@ return true means the queue is empty, false means it is not empty
        */
        bool isEmpty() const
        {
            std::unique_lock<std::mutex> lock(mutex_);
            return (queue_.empty());
        }

    private:
        /**
        *Clean up operation at the end of the @ brief queue
        */
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

    PointCloudProducer::PointCloudProducer(int pc_dst_port, std::string lidar_ip, std::string angle_comp_filename, bool multicast_en, std::string mc_group_ip, DeviceType tp) :
        angle_comp_filename_(angle_comp_filename),
        angle_comp_(new angle_comp_t()),
        points_(new PointCloud()),
        lidar_ip_(lidar_ip),
        pc_dst_ip_(mc_group_ip),
        pc_dst_port_(pc_dst_port),
        device_type_usr_(tp),
        join_multicast_(multicast_en),
        device_type_(LidarUnknown),
        scan_mode_(ScanUnknown),
        use_pointcloud_buffer_(true),
        last_seq_(-1),
        init_ok_(false),
        need_stop_(false),
        pointcloud_cb_(nullptr),
        mutex_(),
        cond_(),
        imu_mutex_(),
        imu_cond_(),
        max_pointcloud_count_(200),
        max_imudata_count_(200)
    {
    }

    PointCloudProducer::~PointCloudProducer()
    {
        Stop();
    }

    PointCloudProducer::PointCloudProducer(std::string port_send, std::string port_recv,
                                           std::string angle_comp_filename, DeviceType tp,
                                           int baud_send, int baud_recv) :
        angle_comp_filename_(angle_comp_filename),
        angle_comp_(new angle_comp_t()),
        points_(new PointCloud()),
        lidar_ip_(),
        pc_dst_ip_(),
        pc_dst_port_(0),
        device_type_usr_(tp),
        join_multicast_(false),
        device_type_(LidarUnknown),
        scan_mode_(ScanUnknown),
        use_pointcloud_buffer_(true),
        last_seq_(-1),
        init_ok_(false),
        need_stop_(false),
        pointcloud_cb_(nullptr),
        mutex_(),
        cond_(),
        imu_mutex_(),
        imu_cond_(),
        max_pointcloud_count_(200),
        max_imudata_count_(200),
        serial_client_(),
        serial_port_send_(port_send),
        serial_port_recv_(port_recv),
        serial_baud_send_(baud_send),
        serial_baud_recv_(baud_recv),
        serial_rx_stream_()
    {
    }
    /**
    *@ brief initialization check, establish TCP connection and obtain device configuration
    *@ return true indicates successful initialization, false indicates failure
    */
    bool PointCloudProducer::CheckInit()
    {
        if (!init_ok_)
        {
            // MRZ16 works over dual serial ports, no network / tcp_tool involved.
            if (device_type_usr_ == LidarMRZ16)
            {
                return CheckInitSerial();
            }

            if (!StringToIp(lidar_ip_, filter_ip_))
            {
                LOG_F(INFO, "stringtoip error\n");
                return false;
            }
            
            std::unique_ptr<zvision::tcp_tools> tcp_tool;
            if(device_type_usr_ == LidarEZ6_B2)
            {
                tcp_tool = std::make_unique<zvision::tcp_ez6_b2>(lidar_ip_, 5000, 5000, 5000);
            }
            else if(device_type_usr_ == LidarNZ1_A2)
            {
                tcp_tool = std::make_unique<zvision::tcp_nz1_a2>(lidar_ip_, 5000, 5000, 5000);
            }
            else
            {
                LOG_F(INFO, "device type error");
                return false;
            }   

            DeviceConfigurationInfo cfg;
			int ret = -1;

            // if port is negative or auto join multicast, we need to get the cfg from lidar by tcp connection
            if ((pc_dst_port_ < 0) || (join_multicast_ && (!pc_dst_ip_.size())))
            {
                ret = tcp_tool->get_basic_info(cfg);
                if (ret)
                {
                    LOG_F(ERROR, "get network info failed.");
                    if (pc_dst_port_ < 0)
                        LOG_F(ERROR, "Please specify the pc dest port and retry.");
                    if (join_multicast_)
                        LOG_F(ERROR, "No multicast group is joined.");

                    return false;
                }
                else
                {
                    if (pc_dst_port_ < 0)
                    {
                        pc_dst_port_ = cfg.destination_port;
                        LOG_F(INFO, "get lidar destination port ok, port is %d.", pc_dst_port_);
                    }
                    if (join_multicast_ && (!pc_dst_ip_.size()))
                    {
                        pc_dst_ip_ = cfg.destination_ip;
                        LOG_F(INFO, "get lidar multicast address ok, group is %s.", pc_dst_ip_.c_str());
                    }
                }
            }

            if (angle_comp_filename_.size())
            {
                if (tcp_tool->read_comp_from_csv(angle_comp_filename_,*(this->angle_comp_.get())))
                {
                    LOG_F(ERROR, "Load calibration file error, %s", angle_comp_filename_.c_str());
                    return false;
                }
                else
                {
                    if(device_type_usr_ == LidarEZ6_B2)
                    {
                        LOG_F(INFO, "angle 0 = %f %f",angle_comp_->azi[0],angle_comp_->ele[0]);
                        LOG_F(INFO, "angle 192 = %f %f",angle_comp_->azi[191],angle_comp_->ele[191]);
                    }
                    else if(device_type_usr_ == LidarNZ1_A2)
                    {
                        LOG_F(INFO, "angle 0 = %f %f",angle_comp_->azi[0],angle_comp_->ele[0]);
                        LOG_F(INFO, "angle 46080 = %f %f",angle_comp_->azi[46079],angle_comp_->ele[46079]);
                    }
                }
            }
            else
            {
                ret = tcp_tool->get_angle_comp(*(this->angle_comp_.get()));
                if(ret != 0)
                {
                    LOG_F(ERROR,"Get lidar[%s]`s comp data error, use default", lidar_ip_.c_str());
                    return false;
                }
            }

            if(device_type_usr_ == LidarEZ6_B2)
            {
                pkg_parse = std::make_unique<zvision::pkg_parse_ez6_b2>(); 
            }
            else if(device_type_usr_ == LidarNZ1_A2)
            {
                pkg_parse = std::make_unique<zvision::pkg_parse_nz1_a2>(); 
            }

            init_ok_ = true;
            return true;
        }

        return true;
    }
    /**
    *@ brief initialization check for offline replay.
    * Creates the packet parser for the configured device type and, when a calibration
    * file is given, loads the per-channel angles from it. This is the hardware-free
    * counterpart of CheckInit(): that one opens the MRZ16 serial ports or a TCP
    * connection to the lidar, and returns false (leaving pkg_parse empty) whenever the
    * hardware is absent - which is always the case when replaying a pcap.
    * The angle table embedded in the recording is applied later through
    * update_angle_comp().
    * @ return true indicates successful initialization, false indicates failure
    */
    bool PointCloudProducer::CheckInitOffline()
    {
        if (init_ok_)
        {
            return true;
        }

        if (device_type_usr_ == LidarEZ6_B2)
        {
            pkg_parse = std::make_unique<zvision::pkg_parse_ez6_b2>();
            if (angle_comp_filename_.size())
            {
                // read_comp_from_csv() is pure file I/O: the tcp_tools instance only
                // supplies the CSV format, no connection is attempted.
                zvision::tcp_ez6_b2 csv_reader(lidar_ip_, 5000, 5000, 5000);
                if (csv_reader.read_comp_from_csv(angle_comp_filename_, *angle_comp_) != 0)
                {
                    LOG_F(ERROR, "offline replay: load calibration file %s failed",
                          angle_comp_filename_.c_str());
                }
            }
        }
        else if (device_type_usr_ == LidarNZ1_A2)
        {
            pkg_parse = std::make_unique<zvision::pkg_parse_nz1_a2>();
            if (angle_comp_filename_.size())
            {
                zvision::tcp_nz1_a2 csv_reader(lidar_ip_, 5000, 5000, 5000);
                if (csv_reader.read_comp_from_csv(angle_comp_filename_, *angle_comp_) != 0)
                {
                    LOG_F(ERROR, "offline replay: load calibration file %s failed",
                          angle_comp_filename_.c_str());
                }
            }
        }
        else if (device_type_usr_ == LidarMRZ16)
        {
            auto parse = std::make_unique<zvision::pkg_parse_mrz16>();
            if (angle_comp_filename_.size())
            {
                std::string err;
                if (!parse->LoadChannelAnglesFile(angle_comp_filename_, angle_comp_.get(), &err))
                {
                    LOG_F(ERROR, "offline replay: load MRZ16 channel angles %s failed: %s",
                          angle_comp_filename_.c_str(), err.c_str());
                }
            }
            pkg_parse = std::move(parse);
        }
        else
        {
            LOG_F(ERROR, "offline replay: unsupported device type %d",
                  static_cast<int>(device_type_usr_));
            return false;
        }

        init_ok_ = true;
        return true;
    }
    /**
    *@ brief CheckInit for the MRZ16 serial lidar.
    * Open both serial ports, then resolve the per-channel angles:
    * an angle file wins when provided, otherwise a $LDCMD/$LDACK exchange.
    * @ return true indicates successful initialization, false indicates failure
    */
    bool PointCloudProducer::CheckInitSerial()
    {
        if (!this->serial_client_)
        {
            this->serial_client_.reset(new zvision::SerialClient(1000, 100));
        }

        if (this->serial_client_->Connect(serial_port_send_, serial_port_recv_,
                                          serial_baud_send_, serial_baud_recv_) != 0)
        {
            LOG_F(ERROR, "open MRZ16 serial ports failed. cmd=%s@%d data=%s@%d",
                  serial_port_send_.c_str(), serial_baud_send_,
                  serial_port_recv_.c_str(), serial_baud_recv_);
            this->serial_client_.reset();
            return false;
        }
        LOG_F(INFO, "open MRZ16 serial ports ok, cmd=%s@%d data=%s@%d",
              serial_port_send_.c_str(), serial_baud_send_,
              serial_port_recv_.c_str(), serial_baud_recv_);

        auto parse = std::make_unique<zvision::pkg_parse_mrz16>();

        // The per-channel table lives in this producer, exactly like the NZ1/EZ6 TCP
        // path (read_comp_from_csv / get_angle_comp) and the offline pcap path
        // (Offline_update_angle_comp): the loaders write straight into angle_comp_, so
        // "load succeeded" and "table available for rendering" cannot diverge.
        angle_comp_t* angles = this->angle_comp_.get();

        std::string err;
        bool angles_ok = false;
        if (angle_comp_filename_.size())
        {
            if (parse->LoadChannelAnglesFile(angle_comp_filename_, angles, &err))
            {
                angles_ok = true;
            }
            else
            {
                LOG_F(WARNING, "load MRZ16 channel angles file %s failed: %s, fallback to $LDCMD",
                      angle_comp_filename_.c_str(), err.c_str());
            }
        }

        if (!angles_ok)
        {
            if (!parse->FetchChannelAnglesOverSerial(this->serial_client_.get(), angles, &err))
            {
                LOG_F(ERROR, "get MRZ16 channel angles from $LDCMD failed: %s, continue with flat angles",
                      err.c_str());
            }
        }

        pkg_parse = std::move(parse);
        init_ok_ = true;
        return true;
    }
    /**
    *@ brief updates angle compensation data
    *@ paramangle_comp new angle compensation data
    */
    void PointCloudProducer::update_angle_comp(angle_comp_t &angle_comp)
    {
        angle_comp_->azi = angle_comp.azi;
        angle_comp_->ele = angle_comp.ele;
    }
    /**
    *@ brief Reset point cloud cache
    */
    void PointCloudProducer::reset_points()
    {
        points_.reset(new zvision::PointCloud);
    }

    void PointCloudProducer::reset_parser_state()
    {
        if (pkg_parse)
        {
            pkg_parse->ResetParserState();
        }
    }
    /**
    *@ brief handles LiDAR UDP packets
    *@ param packet Input LiDAR UDP packet
    */
    void PointCloudProducer::ProcessLidarPacket(LidarUdpPacket& packet)
    {
        if (!pkg_parse)
        {
            // No parser means CheckInit()/CheckInitOffline() failed; dropping the packet
            // keeps the caller alive instead of dereferencing a null parser.
            return;
        }

        zvision::PointCloud cloud;

        /* process point pkg */
        int ret = pkg_parse->IsValidPacket(packet.data);
        if(ret == true)
        {
            ret = pkg_parse->ProcessPacket(packet.data, angle_comp_.get(),*points_);
            if(ret == 1)
            {
                this->ProcessOneFrame();
                this->points_.reset(new zvision::PointCloud);
            }
            else if(ret == 2)
            {
                if (points_->points.size() != 0) 
                {
                    this->ProcessOneFrame(); 
                }
                this->points_.reset(new zvision::PointCloud);
                pkg_parse->ProcessPacket(packet.data, angle_comp_.get(),*points_);
            }
            else
            {}        

            return;                                                      
        }
        /* process imu pkg */
        ret = pkg_parse->IsValidImuPacket(packet.data);
        if (ret == true)
        {
            auto imu_ptr = std::make_shared<imu_data_t>();
            ret = pkg_parse->ParseImuPkg(packet.data, *imu_ptr);

            std::unique_lock<std::mutex> lock(this->imu_mutex_);
             {
                if (this->max_imudata_count_ > 0)
                {
                    while (this->imu_datas_.size() >= this->max_imudata_count_)
                    {
                        this->imu_datas_.pop_front();
                    }
                }

                this->imu_datas_.push_back(imu_ptr);
            }

            imu_cond_.notify_one();
        }

        return;
    }
    /**
    *@ brief processes a frame of point cloud data, triggers a callback, and stores it in the cache
    */
    void PointCloudProducer::ProcessOneFrame()
    {
        std::shared_ptr<PointCloud> ds_points = points_;

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

            if (use_pointcloud_buffer_)
                this->pointclouds_.push_back(ds_points);
        }

        //Allocate a new pointcloud for next one.
        this->points_.reset(new PointCloud());

        //Notify a new pointcloud available.
        cond_.notify_one();
        return;
    }
    bool PointCloudProducer::ReconnectSerial()
    {
        // The external serial link dropped. Tear down the stale connection and
        // keep trying to reopen the same ports so the viewer recovers as soon as
        // the hardware is plugged back in. A transient link loss must never kill
        // the producer thread.
        LOG_F(WARNING, "MRZ16 serial link lost, waiting to reconnect...");
        if (this->serial_client_)
        {
            this->serial_client_->Close();
        }
        this->serial_rx_stream_.clear();

        while (!need_stop_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (this->serial_client_ &&
                this->serial_client_->Connect(serial_port_send_, serial_port_recv_,
                                              serial_baud_send_, serial_baud_recv_) == 0)
            {
                LOG_F(INFO, "MRZ16 serial reconnected.");
                return true;
            }
            LOG_F(INFO, "MRZ16 serial reconnect failed, will retry...");
        }
        return false;
    }

    /**
    *The @ brief producer thread function receives UDP data and joins the queue
    */
    void PointCloudProducer::Producer()
    {
        // MRZ16 reads a continuous EE FF byte stream from the serial data port.
        // Raw frames are sliced here so that every enqueued item is exactly one
        // complete 80-byte point-cloud frame or one 34-byte IMU frame, i.e. the
        // same "one packet per queue item" contract as the UDP path.
        if (device_type_usr_ == LidarMRZ16)
        {
            if (!this->serial_client_)
            {
                return;
            }

            std::string data;
            int len = 0;
            while (!need_stop_)
            {
                data.clear();
                const int ret = this->serial_client_->SyncRecv(data, len);
                if (ret < 0)
                {
                    if (!ReconnectSerial())
                    {
                        return;
                    }
                    continue;
                }
                if (len <= 0)
                {
                    continue;
                }

                this->serial_rx_stream_.append(data.data(), static_cast<size_t>(len));

                std::string frame;
                while (zvision::pkg_parse_mrz16::ExtractNextFrame(this->serial_rx_stream_, &frame))
                {
                    // 原始帧先交给外部录制器（如封装成 UDP 记录写入 pcap），再入队解析。
                    if (this->raw_frame_cb_)
                    {
                        this->raw_frame_cb_(frame);
                    }
                    LidarUdpPacket packet;
                    packet.data = std::move(frame);
                    packet.ip = 0;
                    this->packets_->enqueue(packet);
                }
            }
            return;
        }

        if (this->receiver_)
        {
            uint32_t ip;
            int len;
            int ret = 0;
            while (!need_stop_)
            {
                std::string data(12000, '0');
                ret = receiver_->SyncRecv(data, len, ip);

                if (ret >= 0)
                {
                    if ((len > 0) && (ip == this->filter_ip_))
                    {
                        LidarUdpPacket packet;
                        packet.data = std::string(data.c_str(), len);
                        packet.ip = ip;
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
    
    /**
    *The @ brief consumer thread function retrieves data from the queue and parses it
    */
    void PointCloudProducer::Consumer()
    {
        LidarUdpPacket packet;
        while (this->packets_->dequeue(packet))
        {
            this->ProcessLidarPacket(packet);
        }
    }
    /**
    *@ brief Register Point Cloud callback function
    *@ paramcb callback function pointer
    */
    void PointCloudProducer::RegisterPointCloudCallback(PointCloudCallback cb)
    {
        this->pointcloud_cb_ = cb;
    }
    /**
    *@ brief Register a raw-frame sink (one complete frame per call, MRZ16 serial only)
    *@ paramcb callback function pointer
    */
    void PointCloudProducer::RegisterRawFrameCallback(std::function<void(const std::string &)> cb)
    {
        this->raw_frame_cb_ = std::move(cb);
    }
    /**
    *@ brief Get a frame of point cloud data
    *@ param points Output point cloud data
    *@ paramtimeouts timeout (milliseconds)
    *@ return 0 indicates success, Timeout indicates timeout, Unknown indicates unknown error
    */
    int PointCloudProducer::GetPointCloud(PointCloud& points, int timeout_ms)
    {
        {
            std::unique_lock<std::mutex> lock(mutex_);

            // std::cout << " Su -------------------> this->pointclouds_.empty() = " << this->pointclouds_.empty() << std::endl;
            if (this->pointclouds_.empty())
            {
                //wait_for bug on vs2015&vs2017: https://developercommunity.visualstudio.com/content/problem/438027/unexpected-behaviour-with-stdcondition-variablewai.html
                if (std::cv_status::timeout == cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms)))
                {
                  //  LOG_F(WARNING, "Wait for pointcloud timeout.");
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

        // disable blooming frame
        std::unique_lock<std::mutex> lock(mutex_);

        points = *(this->pointclouds_.front());
        this->pointclouds_.pop_front();
        return 0;
    }
    /**
    *@ brief Get one frame of IMU data
    *@ paramimu_data outputs IMU data
    *@ paramtimeouts timeout (milliseconds)
    *@ return 0 indicates success, Timeout indicates timeout, Unknown indicates unknown error
    */
    int PointCloudProducer::GetImuData(imu_data_t &imu_data, int timeout_ms)
    {
        {
            std::unique_lock<std::mutex> lock(imu_mutex_);

            if (this->imu_datas_.empty())
            {
                //wait_for bug on vs2015&vs2017: https://developercommunity.visualstudio.com/content/problem/438027/unexpected-behaviour-with-stdcondition-variablewai.html
                if (std::cv_status::timeout == imu_cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms)))
                {
                   // LOG_F(WARNING, "Wait for imudata timeout.");
                    return Timeout;
                }
                else
                {
                    if (this->imu_datas_.empty())
                    {
                        return Unknown;
                    }
                }
            }
        }

        // disable blooming frame
        std::unique_lock<std::mutex> lock(imu_mutex_);

        imu_data = *(this->imu_datas_.front());
        this->imu_datas_.pop_front();
        return 0;
    }

    bool PointCloudProducer::GetAngleMetaRecord(std::vector<uint8_t>& out) const
    {
        if (pkg_parse)
        {
            return pkg_parse->GetAngleMetaRecord(*angle_comp_, out);
        }
        return false;
    }

    /**
    *@ brief Set whether point cloud caching is enabled
    *@ param en true means enabled, false means disabled
    */
    void PointCloudProducer::SetPointcloudBufferEnable(bool en)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        this->use_pointcloud_buffer_ = en;
    }
    /**
    *@ brief Start point cloud producer, create producer and consumer threads
    *@ return 0 indicates success, InitFailure indicates initialization failure
    */
    int  PointCloudProducer::Start()/*start the thread which will handle the udp packet one by one*/
    {
        //std::cout << " CheckInit error " << std::endl;
        if (!CheckInit())       
        {
            return InitFailure;  
        } 
            
        if (!this->packets_)
        {
            this->packets_.reset(new SynchronizedQueue<LidarUdpPacket>);
        }

        if ((!this->receiver_) && (device_type_usr_ != LidarMRZ16))
        {
            this->receiver_.reset(new UdpReceiver(this->pc_dst_port_, 1000, 12000));

            if (join_multicast_ && pc_dst_ip_.size())
            {
                unsigned int dst_ip_int = 0;
                if (StringToIp(pc_dst_ip_, dst_ip_int))
                {
                    
                    if ((dst_ip_int & 0xF0000000) == 0xE0000000)
                    {
                        this->receiver_->JoinMulticastGroup(pc_dst_ip_);
                        LOG_F(INFO, "Join multicast group %s.", pc_dst_ip_.c_str());
                    }
                    else
                    {
                        //LOG_F(WARNING, "Invalid multicast group ip %s.", data_dst_ip_.c_str());
                    }
                }
                else
                {
                    LOG_F(ERROR, "Resolve destination ip address error, %s.", pc_dst_ip_.c_str());
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
    /**
    *@ brief Stop point cloud producer, close threads and receivers
    */
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

        if (this->serial_client_)
        {
            this->serial_client_.reset();
        }
        this->serial_rx_stream_.clear();
    }
}
