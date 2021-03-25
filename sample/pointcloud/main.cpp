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
#endif

#include "print.h"
#include "lidar_tools.h"
#include "point_cloud.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <map>

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
        pcl_p.r = p.reflectivity % 255;
        pcl_p.g = p.reflectivity % 255;
        pcl_p.b = 255 - (p.reflectivity % 255);
        pcl_p.a = 255;
        cloud->push_back(pcl_p);
    }

    return cloud;
}

#endif


//Pointcloud callback function.
void sample_pointcloud_callback(zvision::PointCloud& pc, int& status)
{
    LOG_INFO("PointCloud callback, size %d status %d.\n", pc.points.size(), status);
}

//sample 0 : get online pointcloud. You can get poincloud from online device.
//parameter lidar_ip              lidar ipaddress
//parameter port                  lidar pointcloud udp destination port
//parameter calfilename(optional) calibration filename, if pcapfilename is empty, online cal will be used.
//parameter mc_en                 enable to join multicast group, if enable, mc_ip will be used.
//parameter mc_ip                 multicast group ip address. If you dont't known the mc_ip, set to "" , we get the mc_ip by tcp connection.
void sample_online_pointcloud(std::string lidar_ip = "192.168.10.108", int port = 2368, std::string calfilename = "", bool mc_en = false, std::string mc_ip = "")
{
    //Step 1 : Init a online player.
    //If you want to specify the calibration file for the pointcloud, cal_filename is used to load the calibtation data.
    //Otherwise, the PointCloudProducer will connect to lidar and get the calibtation data by tcp connection.
    zvision::PointCloudProducer player(port, lidar_ip, calfilename, mc_en, mc_ip);

    //Step 2 (Optioncal): Regist a callback function.
    //If a callback function registered, the callback function will be called when a new pointcloud is ready.
    //Otherwise, you can call PointCloudProducer's member function "GetPointCloud" to get the pointcloud.
    player.RegisterPointCloudCallback(sample_pointcloud_callback);

    //Step 3 : Start to receive the pointcloud packet and process it.
    int ret = player.Start();
#ifdef USING_PCL_VISUALIZATION
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif

    while (1)
    {
        int ret = 0;
        zvision::PointCloud cloud;

        //Step 3 : Wait the pointcloud for 200 ms. this function return when get poincloud ok or timeout. 
        if (ret = player.GetPointCloud(cloud, 200))
            LOG_ERROR("GetPointCloud error, ret = %d.\n", ret);
        else
        {
            LOG_INFO("GetPointCloud ok.\n");
#ifdef USING_PCL_VISUALIZATION
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pcl_cloud = point_cloud_convert(cloud);
            if (!(viewer->updatePointCloud(pcl_cloud, "cloud")))
            {
                viewer->addPointCloud(pcl_cloud, "cloud", 0);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
            }
            viewer->spinOnce(10);
#endif
        }
    }
    getchar();
}

//sample 1 : get offline pointcloud. You can get poincloud from pcap file.
//parameter lidar_ip                lidar ipaddress
//parameter port                    lidar pointcloud udp destination port
//parameter calfilename             calibration filename
//parameter pcapfilename(required)  pcap filename
void sample_offline_pointcloud(std::string lidar_ip = "192.168.10.108", int port = 2368, std::string calfilename = "", std::string pcapfilename = "")
{
    int ret = 0;

    //Step 1 : Specify a pcap file, which contain the lidar's pointcloud packet.
    std::string pcap_filename = pcapfilename;

    //Specify a calibration file, which contain the lidar's calibration data.
    std::string cal_filename = calfilename;

    //Step 2 : Specify the pcap file, calibtation file to play.
    //The ip address and udp destination port is used to filter the pcap file to play the special lidar data. 
    zvision::OfflinePointCloudProducer player(pcap_filename, cal_filename, lidar_ip, port);

    int size = 0;
    zvision::DeviceType type = zvision::LidarUnknown;

    //Step 3 : Read pointcloud info from file.
    if (ret = player.GetPointCloudInfo(size, type))
    {
        LOG_ERROR("OfflinePointCloudProducer GetPointCloudInfo failed, ret = %d.\n", ret);
    }
    else
    {
        LOG_INFO("OfflinePointCloudProducer GetPointCloudInfo ok, count is %d, type is %d\n", size, type);

        if (0 == size)
        {
            LOG_ERROR("No pointcloud found for lidar %s:%d.\n", lidar_ip.c_str(), port);
            return;
        }
#ifdef USING_PCL_VISUALIZATION
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif
        //Step 4: Iterate the pointcloud.
        for (int i = 0; i < size; ++i)
        {
            zvision::PointCloud pointcloud;
            if (ret = player.GetPointCloud(i, pointcloud))
            {
                LOG_ERROR("GetPointCloud error, frame number is %d, ret = %d.", i, ret);
            }
            else
            {
                int point_valid = 0;
                for (auto& n : pointcloud.points)
                {
                    if (n.valid)
                        point_valid++;
                }
                LOG_INFO("GetPointCloud ok, frame number is %d, valid points %d.\n", i, point_valid);
#ifdef USING_PCL_VISUALIZATION
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  pcl_cloud = point_cloud_convert(pointcloud);
                if (!(viewer->updatePointCloud(pcl_cloud, "cloud")))
                {
                    viewer->addPointCloud(pcl_cloud, "cloud", 0);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                }
                viewer->spinOnce(50);
#endif
            }
        }
    }
    getchar();
}

static void export_point_cloud(zvision::PointCloud& points, std::string filename)
{
    std::ofstream out(filename, std::ios::binary);
    if (out.is_open())
    {
        out.setf(std::ios::fixed, std::ios::floatfield);
        out.precision(3);
        for (int i = 0; i < points.points.size(); ++i)
        {
            zvision::Point pt = points.points[i];
            out << pt.x << " " << pt.y << " " << pt.z << " " << std::endl;
        }
        out.close();
    }
}

int main(int argc, char** argv)
{
    using Param = std::map<std::string, std::string>;
    std::map<std::string, std::string> paras;
    std::string appname = "";
    ParamResolver::GetParameters(argc, argv, paras, appname);

    Param::iterator online = paras.find("-online");
    Param::iterator offline = paras.find("-offline");

    if (online != paras.end())// play online sensor
    {
        Param::iterator find = paras.find("-ip");// device ip
        if (find != paras.end())
        {
            std::string ip = find->second;
            std::string calibration = "";
            bool mc_enable = false;
            std::string mcg_ip = "";
            int port = -1;
            if (paras.end() != (find = paras.find("-p")))// pointcloud udp port
            {
                port = std::atoi(find->second.c_str());
            }
            if (paras.end() != (find = paras.find("-c")))// calibration file
            {
                calibration = find->second;
            }
            if (paras.end() != (find = paras.find("-j")))// join multicast group
            {
                mc_enable = true;
            }
            if (paras.end() != (find = paras.find("-g")))// multicast group ip address
            {
                mcg_ip = find->second;
            }
            sample_online_pointcloud(ip, port, calibration, mc_enable, mcg_ip);
            return 0;
        }
        else
        {
            LOG_ERROR("Invalid parameters, no device ip address found.\n");
        }
    }   
    else if (offline != paras.end())// play online sensor
    {
        Param::iterator ip_find = paras.find("-ip");// device ip
        Param::iterator port_find = paras.find("-p");// pointcloud udp port
        Param::iterator pcap_find = paras.find("-f");// pointcloud udp port
        Param::iterator calibration_find = paras.find("-c");// pointcloud udp port

        if ((ip_find != paras.end()) && (port_find != paras.end()) && (pcap_find != paras.end()) && (calibration_find != paras.end()))
        {
            std::string ip = ip_find->second;
            int port = std::atoi(port_find->second.c_str());
            std::string calibration = calibration_find->second;
            std::string filename = pcap_find->second;
            sample_offline_pointcloud(ip, port, calibration, filename);
            return 0;
        }
        else
        {
            if(ip_find == paras.end())
                LOG_ERROR("Invalid parameters, device ip address not found.\n");
            if (port_find == paras.end())
                LOG_ERROR("Invalid parameters, port not found.\n");
            if (pcap_find == paras.end())
                LOG_ERROR("Invalid parameters, filename not found.\n");
            if (calibration_find == paras.end())
                LOG_ERROR("Invalid parameters, calibration file name not found.\n");
        }
    }
    else
    {

    }

    std::cout << "############################# USER GUIDE ################################\n\n"
        << "Online sample param:\n"
        << "        -online (required)\n"
        << "        -ip lidar_ip_address(required)\n"
        << "        -p  pointcloud_udp_port(optional)\n"
        << "        -c  calibration_file_name(optional)\n"
        << "        -j  (optional for online)\n"
        << "        -g  multicast_group_ip_address(optional, valid when -j is set)\n"
        << "\n"
        << "Online sample 1 : -online -ip 192.168.10.108\n"
        << "Online sample 2 : -online -ip 192.168.10.108 -p 2368\n"
        << "Online sample 3 : -online -ip 192.168.10.108 -c xxxx.cal\n"
        << "Online sample 4 : -online -ip 192.168.10.108 -p 2368 -c xxxx.cal\n"
        << "Online sample 5 : -online -ip 192.168.10.108 -j\n"
        << "Online sample 6 : -online -ip 192.168.10.108 -j 239.0.0.1\n"
        << "\n"

        << "Offline sample param:\n"
        << "        -offline (required)\n"
        << "        -ip lidar_ip_address(required)\n"
        << "        -p  pointcloud_udp_port(required)\n"
        << "        -f  pcap_file_name(required)\n"
        << "        -c  calibration_file_name(required)\n"
        << "\n"
        << "Offline sample 1 : -offline -ip 192.168.10.108 -p 2368 -f xxxx.pcap -c xxxx.cal\n"

        << "############################# END  GUIDE ################################\n\n"
        ;
    getchar();

    return zvision::InvalidParameter; 
}

#if 0// test code
int main()
{
    //sample 0 : get online pointcloud. You can get poincloud from online device.
    //sample_online_pointcloud();

    //sample 1 : get offline pointcloud. You can get poincloud from pcap file.
    sample_offline_pointcloud();
    return 0;
}
#endif
