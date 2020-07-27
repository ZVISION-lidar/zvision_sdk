#if 1
#include "print.h"
#include "lidar_tools.h"
#include "tcp_client.h"
#include "point_cloud.h"
#include "packet.h"
#include "packet_source.h"
#include <windows.h>
#include <stdio.h>
#include <iostream>
#include <iostream>           // std::cout
#include <thread>             // std::thread
#include <chrono>             // std::chrono::seconds
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable, std::cv_status


void test_config()
{
    std::string device_ip = "192.168.10.108";
    zvision::LidarTools tools(device_ip, 1000, 1000, 1000);

    std::string sn = "";
    if (tools.QueryDeviceSnCode(sn))
        std::cout << "read sn code error" << std::endl;
    else
        std::cout << "read sn code ok, " << sn << std::endl;

    zvision::FirmwareVersion version;
    if (tools.QueryDeviceFirmwareVersion(version))
        std::cout << "read version info error" << std::endl;
    else
    {
        std::cout << "read version ok, \n"
            << "boot version : "
            << version.boot_version[0] << "."
            << version.boot_version[1] << "."
            << version.boot_version[2] << "."
            << version.boot_version[3] << ""
            << "kernel version : "
            << version.kernel_version[0] << "."
            << version.kernel_version[1] << "."
            << version.kernel_version[2] << "."
            << version.kernel_version[3] << std::endl;
    }

    std::string ip_to_set = "192.168.10.108";
    if(tools.SetDeviceStaticIpAddress(ip_to_set))
        std::cout << "set device ip error" << std::endl;
    else
    {
        std::cout << "set device ip ok, "
            << ip_to_set << std::endl;
    }

    std::string dst_ip_to_set = "192.168.10.255";
    if (tools.SetDeviceUdpDestinationIpAddress(dst_ip_to_set))
        std::cout << "set device udp destination ip error" << std::endl;
    else
    {
        std::cout << "set device udp destination ip ok, "
            << dst_ip_to_set << std::endl;
    }

    std::string subnet_mask = "255.255.255.0";
    if (tools.SetDeviceSubnetMask(subnet_mask))
        std::cout << "set device subnet mask error" << std::endl;
    else
    {
        std::cout << "set device udp destination ip ok, "
            << dst_ip_to_set << std::endl;
    }

#if 1

    float ps, pl;
    if (tools.QueryDeviceTemperature(ps, pl))
        std::cout << "ger device temperature error" << std::endl;
    else
    {
        std::cout << "ger device temperature ok, ps: " << ps << "pl: " << pl << std::endl;
    }
#endif

    zvision::DeviceConfigurationInfo info;
    if (tools.QueryDeviceConfigurationInfo(info))
        std::cout
            << "#####################################\n"
            << "ger device configuration error" << std::endl;
    else
    {
        std::cout
            << "#####################################" << std::endl
            << "get device configuration ok, " << std::endl
            << "version: boot, " << (UINT32)info.version.boot_version[0] << (UINT32)info.version.boot_version[1] << (UINT32)info.version.boot_version[2] << (UINT32)info.version.boot_version[3] << std::endl
            << "version: kernel, " << (UINT32)info.version.kernel_version[0] << (UINT32)info.version.kernel_version[1] << (UINT32)info.version.kernel_version[2] << (UINT32)info.version.kernel_version[3] << std::endl
            << "device ip: " << info.device_ip << std::endl
            << "serial number: " << info.serial_number << std::endl
            << "device type:" << info.device << std::endl
            << "device mac: " << info.device_mac << std::endl
            << "device subnet mask: " << info.subnet_mask << std::endl
            << "destination ip: " << info.destination_ip << std::endl
            << "destination port: " << info.destination_port << std::endl
            << "timestamp syn mode: " << info.time_sync << std::endl
            << "retro enbale: " << info.retro_enable << std::endl;
    }

    std::string output_filename = "ml30b1cal.cal";
    if (tools.GetDeviceCalibrationDataToFile(output_filename))
        std::cout << "ger device calibration data to file error" << std::endl;
    else
    {
        std::cout
            << "#####################################" << std::endl
            << "ger device calibration data to file ok, "
            << output_filename << std::endl;
    }

    zvision::UdpReceiver rv(3500, 1000);
    while (1)
    {
        std::string data;
        int len = 0;
        int ip = 0;
        if (!rv.SyncRecv(data, len, ip))
        {
            static int counter = 0;
            if(0 == ((counter++) % 1000))
                printf("Recv data: %d\n",data.length());
        }
    }
    getchar();
}

void cb(zvision::PointCloud& pc, int& status)
{
    printf("point size %d status %d\n", pc.points.size(), status);
}

void test_pointcloud_cb()
{
    //zvision::PointCloudProducer pro(3500, "192.168.10.108", "");
    zvision::PointCloudProducer pro(2368, "192.168.10.108", "F:/1_Project/ML30/Beta0012/ML30_LiDAR_012_B1_port_8000.cal");

    pro.RegisterPointCloudCallback(cb);
    pro.Start();
    while (1)
    {
        Sleep(1);
    }
}

std::condition_variable cv;

int value;

void read_value() {
    std::cin >> value;
    cv.notify_one();
}

int test_notify()
{
    std::cout << "Please, enter an integer (I'll be printing dots): \n";
    std::thread th(read_value);

    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    while (cv.wait_for(lck, std::chrono::seconds(1)) == std::cv_status::timeout) {
        std::cout << '.' << std::endl;
    }
    std::cout << "You entered: " << value << '\n';

    th.join();

    return 0;
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

void test_pointcloud_req()
{
    zvision::PointCloudProducer pro(2368, "192.168.10.108", "F:/1_Project/ML30/Beta0012/ML30_LiDAR_012_B1_port_8000.cal");

    pro.Start();
    while (1)
    {
        //Sleep(1);
        int ret = 0;
        zvision::PointCloud pointcloud;
        if (ret = pro.GetPointCloud(pointcloud, 1000))
        {
            std::cout << "get point cloud error , " << ret << std::endl;
        }
        else
        {
            //std::cout << "get point cloud ok " << std::endl;
            zvision::Point pt = pointcloud.points[10005];
            //std::cout << "GetPointCloud, size is " << pointcloud.points.size() << std::endl;
            //std::cout << "First point is, " << pt.x << " " << pt.y << " " << pt.z << ", point number is " << pt.number<< std::endl;
            static bool write_to_file = 1;
            if (write_to_file)
            {
                write_to_file = 0;
                export_point_cloud(pointcloud, "out_put_cloud.txt");
            }
        }
    }
}

void test_pcap_resolver()
{
    int ret = 0;
    zvision::PcapUdpSource pcap("192.168.100.120", 3500, "F:/1_Project/06_Test/01_before/20200702-5m-3-lidardata.pcap");
    std::string pkt;
    int len = 0;
    while (!(ret = pcap.ReadOne(pkt, len)))
    {
        std::cout << "Ok, pkt length is " << len << "data content [0] is " << int(pkt[0]) << std::endl;
    }
    std::cout << "PcapUdpSource ReadOne failed, error code is " << ret << std::endl;

    getchar();

}

void test_pcap_read_frame_info()
{
    int ret = 0;
    zvision::PcapUdpSource pcap("192.168.100.120", 3500, "F:/1_Project/06_Test/01_before/20200702-5m-3-lidardata.pcap");
    int size = 0;
    zvision::DeviceType type = zvision::LidarUnknown;

    if (ret = pcap.ReadFrameInfo(size, type))
    {
        std::cout << "PcapUdpSource ReadFrameInfo failed, error code is " << ret << std::endl;
    }
    else
    {
        std::cout << "PcapUdpSource ReadFrameInfo ok count is " << size << " type is " << type << std::endl;
    }

    getchar();

}

void test_pcap_play()
{
    int ret = 0;
    zvision::OfflinePointCloudProducer player("F:/1_Project/06_Test/01_before/20200702-5m-3-lidardata.pcap", 
        "F:/1_Project/30S/00_30SA1/04_SampleData/30s_12/12-May-2020-ML30SA1012-LiDAR.cal", 
        "192.168.100.120", 3500);

    int size = 0;
    zvision::DeviceType type = zvision::LidarUnknown;
    if (ret = player.GetPointCloudInfo(size, type))
    {
        std::cout << "OfflinePointCloudProducer GetPointCloudInfo failed, error code is " << ret << std::endl;
    }
    else
    {
        std::cout << "OfflinePointCloudProducer GetPointCloudInfo ok, count is " << size << " type is " << type << std::endl;
        for(int i = 0;i < size; ++i)
        {
            zvision::PointCloud pointcloud;
            if (ret = player.GetPointCloud(i, pointcloud))
            {
                std::cout << "Error, GetPointCloud frame number is " << i << ", error code is " << ret << std::endl;
            }
            else
            {
                std::cout << "Ok, GetPointCloud frame number is " << i << " point counter is " << pointcloud.points.size() << std::endl;
            }
        }
        std::cout << "OfflinePointCloudProducer ok " << ret << std::endl;
    }

    getchar();

}

void Test_QueryDeviceInfo(std::string ip)
{
    std::string device_ip = ip;
    zvision::LidarTools tools(device_ip);
    zvision::DeviceConfigurationInfo info;
    std::cout << "\n################QueryDeviceInfo##################\n" << std::endl;

    if (tools.QueryDeviceConfigurationInfo(info))
        std::cout
        << "#####################################\n"
        << "QueryDeviceConfigurationInfo error" << std::endl;
    else
    {
        std::cout
            << "#####################################" << std::endl
            << "get device configuration ok, " << std::endl
            << "version: boot, " << (UINT32)info.version.boot_version[0] << (UINT32)info.version.boot_version[1] << (UINT32)info.version.boot_version[2] << (UINT32)info.version.boot_version[3] << std::endl
            << "version: kernel, " << (UINT32)info.version.kernel_version[0] << (UINT32)info.version.kernel_version[1] << (UINT32)info.version.kernel_version[2] << (UINT32)info.version.kernel_version[3] << std::endl
            << "device ip: " << info.device_ip << std::endl
            << "serial number: " << info.serial_number << std::endl
            << "device type:" << info.device << std::endl
            << "device mac: " << info.device_mac << std::endl
            << "device subnet mask: " << info.subnet_mask << std::endl
            << "destination ip: " << info.destination_ip << std::endl
            << "destination port: " << info.destination_port << std::endl
            << "timestamp syn mode: " << info.time_sync << std::endl
            << "retro enbale: " << info.retro_enable << std::endl;
    }
    std::cout << "################ End QueryDeviceInfo #############\n" << std::endl;
}

void test_mac_set()
{
    {
        int ret = 0;
        std::string device_ip = "192.168.10.106";
        //{
        //    zvision::LidarTools tools(device_ip);
        //    ret += tools.SetDeviceMacAddress("00-66-66-66-66-66");
        //    ret += tools.SetDeviceSubnetMask("255.255.0.0");
        //    ret += tools.SetDeviceUdpDestinationIpAddress("192.168.255.255");
        //    std::cout << "set device " << device_ip << " config done, ret is " << ret << std::endl;
        //}
        Test_QueryDeviceInfo(device_ip);

        //getchar();
        //return;
    }

    {
        int ret = 0;
        std::string device_ip = "192.168.10.107";
        //{
        //    zvision::LidarTools tools(device_ip);
        //    tools.SetDeviceMacAddress("00-77-77-77-77-77");
        //    tools.SetDeviceSubnetMask("255.255.0.0");
        //    tools.SetDeviceUdpDestinationIpAddress("192.168.255.255");
        //    std::cout << "set device " << device_ip << " config done, ret is " << ret << std::endl;
        //}
        Test_QueryDeviceInfo(device_ip);
    }

    {
        int ret = 0;
        std::string device_ip = "192.168.10.108";
        //{
        //    zvision::LidarTools tools(device_ip);
        //    tools.SetDeviceMacAddress("00-88-88-88-88-88");
        //    tools.SetDeviceSubnetMask("255.255.0.0");
        //    tools.SetDeviceUdpDestinationIpAddress("192.168.255.255");
        //    std::cout << "set device " << device_ip << " config done, ret is " << ret << std::endl;
        //}

        Test_QueryDeviceInfo(device_ip);
    }

    getchar();

}

void main()
{
    //test_pointcloud();
    //test_config();
    //test_pcap_resolver();
    //test_pcap_read_frame_info();
    //test_pcap_play();
    LOG_ERROR("123\n");
    LOG_INFO("000\n");
    LOG_DEBUG("111\n");
    LOG_WARN("456\n");
    test_pointcloud_req();
    //test_notify();
    //test_mac_set();
}

#else
#ifndef UNICODE
#define UNICODE
#endif

#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>

// Link with ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

int main()
{

    int iResult = 0;

    WSADATA wsaData;

    SOCKET RecvSocket;
    struct sockaddr_in RecvAddr;

    unsigned short Port = 3500;

    char RecvBuf[1304];
    int BufLen = 1305;

    struct sockaddr_in SenderAddr;
    int SenderAddrSize = sizeof(SenderAddr);

    //-----------------------------------------------
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR) {
        wprintf(L"WSAStartup failed with error %d\n", iResult);
        return 1;
    }
    //-----------------------------------------------
    // Create a receiver socket to receive datagrams
    RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (RecvSocket == INVALID_SOCKET) {
        wprintf(L"socket failed with error %d\n", WSAGetLastError());
        return 1;
    }
    //-----------------------------------------------
    // Bind the socket to any address and the specified port.
    RecvAddr.sin_family = AF_INET;
    RecvAddr.sin_port = htons(Port);
    RecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    iResult = bind(RecvSocket, (SOCKADDR *)& RecvAddr, sizeof(RecvAddr));
    if (iResult != 0) {
        wprintf(L"bind failed with error %d\n", WSAGetLastError());
        return 1;
    }
    //-----------------------------------------------
    // Call the recvfrom function to receive datagrams
    // on the bound socket.
    wprintf(L"Receiving datagrams...\n");
    iResult = recvfrom(RecvSocket,
        RecvBuf, BufLen, 0, (SOCKADDR *)& SenderAddr, &SenderAddrSize);
    if (iResult == SOCKET_ERROR) {
        wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
    }

    //-----------------------------------------------
    // Close the socket when finished receiving datagrams
    wprintf(L"Finished receiving. Closing socket.\n");
    iResult = closesocket(RecvSocket);
    if (iResult == SOCKET_ERROR) {
        wprintf(L"closesocket failed with error %d\n", WSAGetLastError());
        return 1;
    }

    //-----------------------------------------------
    // Clean up and exit.
    wprintf(L"Exiting.\n");
    WSACleanup();
    return 0;
}
#endif