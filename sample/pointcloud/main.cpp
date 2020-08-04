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
void sample_online_pointcloud(std::string lidar_ip = "192.168.10.108", int port = 2368, std::string calfilename = "")
{
    //Step 1 : Init a online player.
    //If you want to specify the calibration file for the pointcloud, cal_filename is used to load the calibtation data.
    //Otherwise, the PointCloudProducer will connect to lidar and get the calibtation data by tcp connection.
    zvision::PointCloudProducer player(port, lidar_ip, calfilename);

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
    if (argc < 4)
    {
        std::cout << "############################# USER GUIDE ################################\n\n"
            << "Sample 0 : play online pointcloud\n"
            << "Format: -online lidar_ip lidar_port calfilename(if use online cal, ignore this parameter)\n"
            << "Demo:   -online 192.168.10.108 2368\n\n"

            << "Sample 1 : play offline pointcloud\n"
            << "Format: -offline lidar_ip lidar_port pcapfilename calfilename\n"
            << "Demo:   -offline 192.168.10.108 2368 xxxx.pcap xxxx.cal\n\n"

            << "############################# END  GUIDE ################################\n\n"
            ;
        getchar();
        return 0;
    }
    std::string lidar_ip = std::string(argv[2]);
    int port = std::atoi(argv[3]);
    std::string cal = "";
    std::string pcapfilename = "";

    if (0 == std::string(argv[1]).compare("-online"))
    {
        if (argc == 4)
            ;
        else if (argc == 5)
            cal = std::string(argv[4]);
        else
        {
            LOG_ERROR("Invalid parameter.\n");
            return -1;
        }
        sample_online_pointcloud(lidar_ip, port, cal);
    }
    else if (0 == std::string(argv[1]).compare("-offline") && argc == 6)
    {
        cal = std::string(argv[5]);
        pcapfilename = std::string(argv[4]);
        sample_offline_pointcloud(lidar_ip, port, cal, pcapfilename);
    }
    else
    {
        LOG_ERROR("Invalid parameters\n.");
        return zvision::InvalidParameter;
    }
    return 0;
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
