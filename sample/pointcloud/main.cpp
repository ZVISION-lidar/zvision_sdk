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
void sample_online_pointcloud()
{
    //If you want to specify the calibration file for the pointcloud, cal_filename is used to load the calibtation data.
    //Otherwise, the PointCloudProducer will connect to lidar and get the calibtation data by tcp connection.
    zvision::PointCloudProducer player(2368, "192.168.10.108", "");

    //If a callback function registered, the callback function will be called when a new pointcloud is ready.
    //Otherwise, you can call PointCloudProducer's member function "GetPointCloud" to get the pointcloud.
    player.RegisterPointCloudCallback(sample_pointcloud_callback);

    //Start to receive the pointcloud packet and process it.
    player.Start();

#ifdef USING_PCL_VISUALIZATION
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif

    while (1)
    {
        int ret = 0;
        zvision::PointCloud cloud;

        // Wait the pointcloud for 100 ms. this function return when get poincloud ok or timeout. 
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
void sample_offline_pointcloud()
{
    int ret = 0;

    //Specify a pcap file, which contain the lidar's pointcloud packet.
    std::string pcap_filename = "xxxx.pcap";

    //Specify a calibration file, which contain the lidar's calibration data.
    std::string cal_filename = "xxxx.cal";

    //Specify the pcap file, calibtation file to play.
    //The ip address and udp destination port is used to filter the pcap file to play the special lidar data. 
    zvision::OfflinePointCloudProducer player(pcap_filename, cal_filename, "192.168.100.120", 3500);

    int size = 0;
    zvision::DeviceType type = zvision::LidarUnknown;

    //Read pointcloud info from file.
    if (ret = player.GetPointCloudInfo(size, type))
    {
        LOG_ERROR("OfflinePointCloudProducer GetPointCloudInfo failed, ret = %d.\n", ret);
    }
    else
    {
        LOG_INFO("OfflinePointCloudProducer GetPointCloudInfo ok, count is %d, type is %d\n", size, type);

#ifdef USING_PCL_VISUALIZATION
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        viewer.reset(new pcl::visualization::PCLVisualizer("cloudviewtest"));
#endif
        //Iterate the pointcloud.
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

void main()
{
    //sample 0 : get online pointcloud. You can get poincloud from online device.
    //sample_online_pointcloud();

    //sample 1 : get offline pointcloud. You can get poincloud from pcap file.
    sample_offline_pointcloud();

}
