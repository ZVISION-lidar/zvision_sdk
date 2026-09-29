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


#ifdef USING_PCL_VISUALIZATION
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <boost/filesystem.hpp>
#endif

#include "print.h"
#include "tcp_ez6_b2.h"
#include "point_cloud.h"
#include "loguru.hpp"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <map>
#include <chrono>


class ParamResolver
{
public:

    static int GetParameters(int argc, char* argv[], std::map<std::string, std::string>& paras, std::string& appname)
    {
        paras.clear();
        if (argc >= 1)
            appname = std::string(argv[0]);

        std::string key;
        std::string value;
        for (int i = 1; i < argc; i++)
        {
            std::string str(argv[i]);
            if ((str.size() > 1) && ('-' == str[0]))
            {
                key = str;
                if (i == (argc - 1))
                    value = "";
                else
                {
                    value = std::string(argv[i + 1]);
                    if ('-' == value[0])
                    {
                        value = "";
                    }
                    else
                    {
                        i++;
                    }
                }
                paras[key] = value;
            }
        }
        return 0;
    }

};

#ifdef USING_PCL_VISUALIZATION
/*to pcl pointcloud*/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_convert(zvision::PointCloud& in_point)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (auto& p : in_point.points)
    {
        pcl::PointXYZRGBA pcl_p;
        pcl_p.x = p.x;
        pcl_p.y = p.y;
        pcl_p.z = p.z;

#if 0
        pcl_p.r = p.reflectivity % 255;
        pcl_p.g = p.reflectivity % 255;
        pcl_p.b = 255 - (p.reflectivity % 255);
#else
        float t = p.reflectivity / 255.0f; // Normalize reflectivity to [0, 1]
        uint8_t r = 0, g = 0, b = 0;
        float fraction = 0.0f;

        if (t < 0.2f) { // Blue to Cyan, [0, 51) 
            fraction = t / 0.2f;
            g = static_cast<uint8_t>(255 * fraction); // Green increases from 0 to 255
            b = 255;
        } else if (t < 0.4f) { // Cyan to Green,  [51, 102)
            fraction = (t - 0.2f) / 0.2f;
            g = 255;
            b = static_cast<uint8_t>(255 * (1 - fraction)); // Blue decreases from 255 to 0
        } else if (t < 0.6f) { // Green to Yellow, [102, 153)
            fraction = (t - 0.4f) / 0.2f;
            r = static_cast<uint8_t>(255 * fraction); // Red increases from 0 to 255
            g = 255;
        } else if (t < 0.8f) { // Yellow to Orange,  [153, 204)
            fraction = (t - 0.6f) / 0.2f;
            r = 255;
            g = static_cast<uint8_t>(255 * (1 - fraction)); // Green decreases from 255 to 0
        } else { // Orange to Red,  [204, 255]
            r = 255;
        }

        pcl_p.r = r;
        pcl_p.g = g;
        pcl_p.b = b;
#endif

        // printf("pcl_p.r = %d,  pcl_p.g = %d,  pcl_p.b = %d \n", pcl_p.r, pcl_p.g, pcl_p.b);

        pcl_p.a = 255;
        cloud->push_back(pcl_p);
    }

    return cloud;
}


#endif

struct PlayParam 
{
    /** bref
    param: ip                       lidar ipaddress
    param: port                     lidar pointcloud udp destination port
    param: angle_comp_name          (optional) angle comp filename, if pcapfilename is empty, online cal will be used.
    param: mc_enable                enable to join multicast group, if enable, mc_ip will be used.
    param: mcg_ip                   multicast group ip address.If you dont't known the mc_ip, set to "" , we get the mc_ip by tcp connection.
    param: tp                       lidar type
    */
    PlayParam(std::string ip, int port, std::string angle_comp_name, bool mc_enable, std::string mcg_ip, zvision::DeviceType tp):
        lidar_ip_(ip),
        port_(port),
        angle_comp_name_(angle_comp_name),
        mc_en_(mc_enable),
        mc_ip_(mcg_ip),
        tp_(tp)
    {}

    PlayParam() {
        lidar_ip_ = "192.168.10.108";
        port_ = 2368;
        angle_comp_name_ = "";
        mc_en_ = false;
        mc_ip_ = "";
        tp_ = zvision::DeviceType::LidarUnknown;
    }

    std::string lidar_ip_;
    int port_;
    std::string angle_comp_name_;
    bool mc_en_;
    std::string mc_ip_;
    zvision::DeviceType tp_;
};

//Pointcloud callback function.
void sample_pointcloud_callback(zvision::PointCloud& pc, int& status)
{
     LOG_F(2, "PointCloud callback, size %ld status %d.", pc.points.size(), status);
}

std::string usToUTC(uint64_t microseconds) {
    // 将 microsecond 转换为 system_clock::time_point
    auto tp = std::chrono::system_clock::time_point() +
              std::chrono::microseconds(microseconds);

    // 转换为 time_t（以秒为单位）
    std::time_t t = std::chrono::system_clock::to_time_t(tp);

    // 获取 tm 结构（UTC 时间）
    const struct std::tm* utc_tm = std::gmtime(&t);

    // 拼接日期时间字符串
    std::ostringstream oss;
    oss << std::put_time(utc_tm, "%Y-%m-%d %H:%M:%S");

    // 添加微秒部分
    int us = static_cast<int>(microseconds % 1000000);
    oss << "." << std::setw(6) << std::setfill('0') << us;

    return oss.str();
}

//Common pointcloud fetch loop shared by the online (UDP) and MRZ16 (serial) samples.
void run_pointcloud_demo_loop(zvision::PointCloudProducer& player, int imu_support);

//sample 0 : get online pointcloud. You can get poincloud from online device.
//parameter param   lidar play parameters
void sample_online_pointcloud(const PlayParam& param,int imu_support)
{
    //Step 1 : Init a online player.
    //If you want to specify the calibration file for the pointcloud, cal_filename is used to load the calibtation data.
    //Otherwise, the PointCloudProducer will connect to lidar and get the calibtation data by tcp connection.
    zvision::PointCloudProducer player(param.port_, param.lidar_ip_, param.angle_comp_name_, param.mc_en_, param.mc_ip_, param.tp_);

    //Step 2 (Optioncal): Regist a callback function.
    //If a callback function registered, the callback function will be called when a new pointcloud is ready.
    //Otherwise, you can call PointCloudProducer's member function "GetPointCloud" to get the pointcloud.
    player.RegisterPointCloudCallback(sample_pointcloud_callback);
    
    int ret = player.Start();
    if(ret != 0)
    {
        LOG_F(ERROR, "Start online pointcloud player failed. ip=%s port=%d", param.lidar_ip_.c_str(), param.port_);
        return;
    }

    run_pointcloud_demo_loop(player, imu_support);
}

//sample mrz16 : get pointcloud from the MRZ16 serial lidar.
//The lidar streams over two UARTs: a command port (9600 baud, "$LDCMD" is sent
//here) and a point-cloud port (custom baud, default 3125000).
void sample_mrz16_serial_pointcloud(const std::string& cmd_port, const std::string& data_port,
                                    const std::string& angle_comp_name,
                                    int baud_cmd, int baud_data, int imu_support)
{
    //Step 1 : Init the MRZ16 serial player with the PointCloudProducer serial ctor.
    //When angle_comp_name is empty (or cannot be loaded), the producer fetches the
    //per-channel angles automatically through a $LDCMD / $LDACK exchange.
    zvision::PointCloudProducer player(cmd_port, data_port, angle_comp_name,
                                       zvision::DeviceType::LidarMRZ16,
                                       baud_cmd, baud_data);

    //Step 2 (Optional): Regist a callback function.
    player.RegisterPointCloudCallback(sample_pointcloud_callback);

    int ret = player.Start();
    if (ret != 0)
    {
        LOG_F(ERROR, "Start MRZ16 serial pointcloud player failed. cmd=%s data=%s",
              cmd_port.c_str(), data_port.c_str());
        return;
    }

    LOG_F(INFO, "MRZ16 serial producer started. cmd=%s@%d data=%s@%d angle_file=%s",
          cmd_port.c_str(), baud_cmd, data_port.c_str(), baud_data,
          angle_comp_name.empty() ? "<none, auto via $LDCMD>" : angle_comp_name.c_str());

    run_pointcloud_demo_loop(player, imu_support);
}

//Common pointcloud fetch loop shared by the online (UDP) and MRZ16 (serial) samples.
void run_pointcloud_demo_loop(zvision::PointCloudProducer& player, int imu_support)
{
#ifdef USING_PCL_VISUALIZATION
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif

    while (1)
    {
        int ret = 0;

	    /* get and parse point cloud */
        zvision::PointCloud cloud;
        //Step 3 : Wait the pointcloud for 200 ms. this function return when get poincloud ok or timeout. 
        ret = player.GetPointCloud(cloud, 1);
        if (ret != 0)
        {
            ;
        }
        else
        {
            LOG_F(2, "GetPointCloud ok.");

#if 0 
	    uint64_t last_pkg_time = cloud.points.back().timestamp_us/1000;
        auto now = std::chrono::system_clock::now();
        auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()
        );

        // print point cloud process time, ms 
        std::cout << ms_since_epoch.count()-last_pkg_time  << std::endl;
#endif


#ifdef USING_PCL_VISUALIZATION
            // auto start_time = std::chrono::high_resolution_clock::now();            
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pcl_cloud = point_cloud_convert(cloud);
            // 计算并打印经过的时间
            // auto end_time = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            // std::cout << "Time taken to convert and visualize the point cloud: "
            //         << duration.count() << " microseconds." << std::endl;

            if (!(viewer->updatePointCloud(pcl_cloud, "cloud")))
            {
                viewer->addPointCloud(pcl_cloud, "cloud", 0);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            }
            viewer->spinOnce(10);
#endif
        }

	    /*** get and parse imu data ***/
        if(imu_support == 1)
        {
            zvision::imu_data_t imu_data;
            ret = player.GetImuData(imu_data, 1);
            if (ret != 0)
            {
                ;
            }
            else
            {
                std::string timestamp = usToUTC(imu_data.timestamp_us);
                LOG_F(INFO, "get imu ok, acc: %f %f %f, gyro: %f %f %f, time = %s\n",imu_data.acc_x,imu_data.acc_y,imu_data.acc_z,
                imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z,timestamp.c_str());
            }	
        }
    }
    getchar();
}

void init_log(int argc, char* argv[])
{
    loguru::init();
    loguru::add_file("Log/log.txt", loguru::Truncate, loguru::Verbosity_INFO);
    loguru::g_stderr_verbosity = 0;
    loguru::g_preamble_date = 0;
    loguru::g_preamble_time = 0;
    loguru::g_preamble_uptime = 0;
    loguru::g_preamble_thread = 0;
    loguru::g_preamble_file = 0;
    loguru::g_preamble_verbose = 0;
    loguru::g_preamble_pipe = 0;
}

int main(int argc, char** argv)
{
    init_log(argc, argv);
    using Param = std::map<std::string, std::string>;
    std::map<std::string, std::string> paras;
    std::string appname = "";
    ParamResolver::GetParameters(argc, argv, paras, appname);

    Param::iterator online = paras.find("-online");

    int imu_support = 0;
    
    if (online != paras.end())// play online sensor
    {
        Param::iterator find = paras.find("-ip");// lidar ip
        if (find != paras.end())
        {
            std::string ip = find->second;
            std::string angle_comp_name = "";
            bool mc_enable = false;
            std::string mcg_ip = "";
            int port = -1;
			zvision::DeviceType tp = zvision::DeviceType::LidarEZ6_B2;
            if (paras.end() != (find = paras.find("-p")))// pointcloud udp port
            {
                port = std::atoi(find->second.c_str());
            }
            if (paras.end() != (find = paras.find("-c")))// angle comp file
            {
                angle_comp_name = find->second;
            }
            if (paras.end() != (find = paras.find("-j")))// join multicast group
            {
                mc_enable = true;
            }
            if (paras.end() != (find = paras.find("-g")))// multicast group ip address
            {
                mcg_ip = find->second;
            }
            if (paras.end() != (find = paras.find("-ez6_b2"))) 
            {
                tp = zvision::DeviceType::LidarEZ6_B2;
            }
            if (paras.end() != (find = paras.find("-nz1_a2"))) 
            {
                tp = zvision::DeviceType::LidarNZ1_A2;
            }

            if (paras.end() != (find = paras.find("-imu"))) 
            {
                imu_support = 1;
            }

            PlayParam param(ip, port, angle_comp_name, mc_enable, mcg_ip, tp);

            sample_online_pointcloud(param,imu_support);
            return 0;
        }
        else
        {
             LOG_F(ERROR, "Invalid parameters, no device ip address found.");
        }
    }   

    Param::iterator serial = paras.find("-mrz16");// MRZ16 (EE FF series) serial lidar
    if (serial != paras.end())
    {
        // command (send) port and point-cloud (recv) port defaults follow the
        // MRZ16 reference bring-up ports.
        std::string cmd_port = "/dev/ttyACM2";
        std::string data_port = "/dev/ttyACM0";
        std::string angle_comp_name = "";
        int baud_cmd = 9600;
        int baud_data = 3125000;

        Param::iterator find;
        if (paras.end() != (find = paras.find("-cmd")))// command serial port
        {
            cmd_port = find->second;
        }
        if (paras.end() != (find = paras.find("-data")))// point cloud serial port
        {
            data_port = find->second;
        }
        if (paras.end() != (find = paras.find("-c")))// channel angle file (csv)
        {
            angle_comp_name = find->second;
        }
        if (paras.end() != (find = paras.find("-baud_cmd")))// command port baud rate
        {
            baud_cmd = std::atoi(find->second.c_str());
        }
        if (paras.end() != (find = paras.find("-baud_data")))// point cloud port baud rate
        {
            baud_data = std::atoi(find->second.c_str());
        }
        if (paras.end() != (find = paras.find("-imu")))
        {
            imu_support = 1;
        }

        sample_mrz16_serial_pointcloud(cmd_port, data_port, angle_comp_name,
                                       baud_cmd, baud_data, imu_support);
        return 0;
    }

    std::cout
    << "############################# USER GUIDE "
        "################################\n\n"
    << "Online sample param:\n"
    << "        -online (required)\n"
    << "        -ip lidar_ip_address(required)\n"
    << "        -p  pointcloud_udp_port(optional)\n"
    << "        -c  angle_comp_file_name(optional)\n"
    << "        -j  (optional for online)\n"
    << "        -g  multicast_group_ip_address(optional, valid when -j "
        "is set)\n"
    << "        -ez6_b2 or -nz1_a2 must be set\n"
    << "\n"
    << "        -imu should be set if you want to enable imu, only nz1 a2 support\n"
    << "\n"
    << "Online sample 1 : -online -ip 192.168.10.108 -p 2368 -nz1_a2 -imu\n"
    << "\n"
    << "MRZ16 serial sample param (dual UART, EE FF protocol):\n"
    << "        -mrz16 (required)\n"
    << "        -cmd      command serial port, optional, default /dev/ttyACM2\n"
    << "        -data     point cloud serial port, optional, default /dev/ttyACM0\n"
    << "        -c        channel angle file (csv), optional;\n"
    << "                  csv rows: '<horizontal_deg>,<vertical_deg>' (no channel-number\n"
    << "                  column, row index = channel number); if empty/broken the\n"
    << "                  producer fetches angles via $LDCMD/$LDACK\n"
    << "        -baud_cmd   command port baud, optional, default 9600\n"
    << "        -baud_data  point cloud port baud, optional, default 3125000\n"
    << "        -imu      optional, enable imu output\n"
    << "\n"
    << "MRZ16 serial sample : -mrz16 -cmd /dev/ttyACM2 -data /dev/ttyACM0\n"
    << "\n"

    << "############################# END  GUIDE "
        "################################\n\n";
    getchar();

    return zvision::InvalidParameter; 
}
