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


#include "print.h"
#include "lidar_tools.h"
#include "define.h"
#include <stdio.h>
#include <fstream>
#include <iostream>

//Callback function for progress notify
void print_current_progress(int percent)
{
    LOG_INFO("# Current progress %3d.\n", percent);
}

//Sample code 0 : Set lidar's mac address
int sample_config_lidar_mac_address(std::string lidar_ip, std::string mac)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceMacAddress(mac))
        LOG_ERROR("Set device [%s]'s MAC address to [%s] failed, ret = %d.\n", lidar_ip.c_str(), mac.c_str(), ret);
    else
        LOG_INFO("Set device [%s]'s MAC address to [%s] ok.\n", lidar_ip.c_str(), mac.c_str());
    return ret;
}

//Sample code 1 : Set lidar's static ip address
int sample_config_lidar_ip(std::string lidar_ip, std::string new_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceStaticIpAddress(new_ip))
        LOG_ERROR("Set device [%s]'s IP address to [%s] failed, ret = %d.\n", lidar_ip.c_str(), new_ip.c_str(), ret);
    else
        LOG_INFO("Set device [%s]'s IP address to [%s] ok.\n", lidar_ip.c_str(), new_ip.c_str());
    return ret;
}

//Sample code 2 : Set lidar's subnet mask
int sample_config_lidar_subnet_mask(std::string lidar_ip, std::string subnetmask)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceSubnetMask(subnetmask))
        LOG_ERROR("Set device [%s]'s subnet mask to [%s] failed, ret = %d.\n", lidar_ip.c_str(), subnetmask.c_str(), ret);
    else
        LOG_INFO("Set device [%s]'s subnet mask to [%s] ok.\n", lidar_ip.c_str(), subnetmask.c_str());
    return ret;
}

//Sample code 3 : Set lidar's udp destination ip address
int sample_config_lidar_udp_destination_ip(std::string lidar_ip, std::string dst_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceUdpDestinationIpAddress(dst_ip))
        LOG_ERROR("Set device [%s]'s UDP destination ip to [%s] failed, ret = %d.\n", lidar_ip.c_str(), dst_ip.c_str(), ret);
    else
        LOG_INFO("Set device [%s]'s UDP destination ip to [%s] ok.\n", lidar_ip.c_str(), dst_ip.c_str());
    return ret;
}

//Sample code 4 : Set lidar's udp destination port
int sample_config_lidar_udp_destination_port(std::string lidar_ip, int dst_port)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceUdpDestinationPort(dst_port))
        LOG_ERROR("Set device [%s]'s UDP destination port to [%d] failed, ret = %d.\n", lidar_ip.c_str(), dst_port, ret);
    else
        LOG_INFO("Set device [%s]'s UDP destination port to [%d] ok.\n", lidar_ip.c_str(), dst_port);
    return ret;
}

//Sample code 5 : Set lidar's retro function
int sample_config_lidar_retro_enable(std::string lidar_ip, bool enable)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceRetroEnable(enable))
        LOG_ERROR("Set device [%s]'s retro to [%d] failed, ret = %d.\n", lidar_ip.c_str(), enable, ret);
    else
        LOG_INFO("Set device [%s]'s retro to [%d] ok.\n", lidar_ip.c_str(), enable);
    return ret;
}

//Sample code 6 : Set lidar's time sync mode
int sample_config_lidar_time_sync(std::string lidar_ip, zvision::TimestampType type)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceTimestampType(type))
        LOG_ERROR("Set device [%s]'s timestamp type to [%d] failed, ret = %d.\n", lidar_ip.c_str(), type, ret);
    else
        LOG_INFO("Set device [%s]'s timestamp type to [%d] ok.\n", lidar_ip.c_str(), type);
    return ret;
}

//Sample code 7 : Query lidar's firmware version
int sample_query_lidar_firmware_version(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    std::string boot_version, kernel_version;
    zvision::FirmwareVersion version;
    if (ret = config.QueryDeviceFirmwareVersion(version))
        LOG_ERROR("Query device [%s]'s firmware version failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Query device [%s]'s firmware version ok.\n", lidar_ip.c_str());
        LOG_INFO("Boot   version: %u.%u.%u\n", version.boot_version[0], version.boot_version[1], version.boot_version[2], version.boot_version[3]);
        LOG_INFO("Kernel version: %u.%u.%u\n", version.kernel_version[0], version.kernel_version[1], version.kernel_version[2], version.kernel_version[3]);
    }
    return ret;
}

//Sample code 8 : Query lidar's serial number
int sample_query_lidar_serial_number(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    std::string serial_number;
    if (ret = config.QueryDeviceSnCode(serial_number))
        LOG_ERROR("Query device [%s]'s serial number failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Query device [%s]'s serial number ok.\n", lidar_ip.c_str());
        LOG_INFO("Serial number: %s\n", serial_number.c_str());
    }
    return ret;
}

//Sample code 9 : Query lidar's hardware temperature
int sample_query_lidar_hardware_temperature(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    float temperature0 = 0.0, temperature1 = 0.0;
    if (ret = config.QueryDeviceTemperature(temperature0, temperature1))
        LOG_ERROR("Query device [%s]'s hardware temperature failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Query device [%s]'s hardware temperature ok.\n", lidar_ip.c_str());
        LOG_INFO("Temperature 0: %.3f\n", temperature0);
        LOG_INFO("Temperature 1: %.3f\n", temperature1);
    }
    return ret;
}

//Sample code 10 : Query lidar's configurature
int sample_query_lidar_configuration(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    zvision::DeviceConfigurationInfo info;
    if (ret = config.QueryDeviceConfigurationInfo(info))
        LOG_ERROR("Query device [%s]'s configuration info failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        std::string info_str = zvision::get_cfg_info_string(info);
        LOG_INFO("Query device [%s]'s configuration info ok.\n", lidar_ip.c_str());
        LOG_INFO("%s\n", info_str.c_str());
    }
    return ret;
}

//Sample code 11 : Get lidar's calibration file by tcp connection.
int sample_get_lidar_calibration(std::string lidar_ip, std::string savefilename)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.GetDeviceCalibrationDataToFile(savefilename))
        LOG_ERROR("Get device [%s]'s calibration data failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Get device [%s]'s calibration data ok.\n", lidar_ip.c_str());
        LOG_INFO("Calibration data save to file %s.\n", savefilename.c_str());
    }
    return ret;
}

//Sample code 12 : Firmware update.
int sample_firmware_update(std::string lidar_ip, std::string filename)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.FirmwareUpdate(filename, print_current_progress))
        LOG_ERROR("Update device [%s]'s firmware %s failed, ret = %d.\n", lidar_ip.c_str(), filename.c_str(), ret);
    else
    {
        LOG_INFO("Update device [%s] fireware %s ok.\n", lidar_ip.c_str(), filename.c_str());
    }
    return ret;
}

//Sample code 13 : Reboot lidar by tcp connection.
int sample_reboot_lidar(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.RebootDevice())
        LOG_ERROR("Reboot device [%s] failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Reboot device [%s] ok.\n", lidar_ip.c_str());
    }
    return ret;
}


//Sample code 14 : Scan lidar on the heart beat port
//Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
int sample_scan_lidar_on_heat_beat_port(int seconds)
{
    int ret = 0;
    std::vector<zvision::DeviceConfigurationInfo> devices;
    
    if (ret = zvision::LidarTools::ScanDevice(devices, seconds))
        LOG_ERROR("Scan device on heart beat port failed, ret = %d.\n", ret);
    else
    {
        LOG_INFO("Scan device on heart beat port ok, total %d found.\n", devices.size());

        for (int i = 0; i < devices.size(); ++i)
        {
            zvision::DeviceConfigurationInfo& info = devices[i];
            LOG_INFO("####################### Device number %3d #######################\n", i);
            LOG_INFO("Serial number: %s\n", info.serial_number.c_str());
            LOG_INFO("Device ip: %s\n", info.device_ip.c_str());
            LOG_INFO("Device subnet mask: %s\n", info.subnet_mask.c_str());
            LOG_INFO("Device mac: %s\n", info.device_mac.c_str());
            LOG_INFO("Destination ip: %s\n", info.destination_ip.c_str());
            LOG_INFO("Destination port: %d\n", info.destination_port);
            LOG_INFO("Timestamp syn mode: %d\n", info.time_sync);
            LOG_INFO("Retro enbale: %d\n", info.retro_enable);
            LOG_INFO("Boot   version: %u.%u.%u\n", info.version.boot_version[0], info.version.boot_version[1], info.version.boot_version[2], info.version.boot_version[3]);
            LOG_INFO("Kernel version: %u.%u.%u\n", info.version.kernel_version[0], info.version.kernel_version[1], info.version.kernel_version[2], info.version.kernel_version[3]);
            LOG_INFO("Device type: %d\n", info.device);
            LOG_INFO("#################################################################\n\n");
        }
    }
    return ret;
}

int main(int argc, char** argv)
{
    if (argc <= 2)
    {
        std::cout << "############################# USER GUIDE ################################\n\n"
            << "Sample 0 : config mac address\n"
            << "Format: -config_mac lidar_ip mac_address\n"
            << "Demo:   -config_mac 192.168.10.108 66-66-66-66-66-66\n\n"
            
            << "Sample 1 : config static ip address\n"
            << "Format: -config_static_ip old_ip new_ip\n"
            << "Demo:   -config_static_ip 192.168.10.108 192.168.10.107\n\n"

            << "Sample 2 : config subnet mask\n"
            << "Format: -config_subnet_mask lidar_ip subnet_mask\n"
            << "Demo:   -config_subnet_mask 192.168.10.108 255.255.255.0\n\n"

            << "Sample 3 : config udp destination ip address\n"
            << "Format: -config_dst_ip lidar_ip dst_ip\n"
            << "Demo:   -config_dst_ip 192.168.10.108 192.168.10.255\n\n"

            << "Sample 4 : config udp destination port\n"
            << "Format: -config_dst_port lidar_ip port\n"
            << "Demo:   -config_dst_port 192.168.10.108 2368\n\n"

            << "Sample 5 : config retro mode\n"
            << "Format: -config_retro lidar_ip mode(0 for disable, 1 for enable)\n"
            << "Demo:   -config_retro 192.168.10.108 0\n\n"

            << "Sample 6 : config time sync mode\n"
            << "Format: -config_time_sync lidar_ip mode(0 for ptp, 1 for gpspps)\n"
            << "Demo:   -config_time_sync 192.168.10.108 0\n\n"

            << "Sample 7 : query firmware version\n"
            << "Format: -query_version lidar_ip\n"
            << "Demo:   -query_version 192.168.10.108\n\n"

            << "Sample 8 : query serial number\n"
            << "Format: -query_sn lidar_ip\n"
            << "Demo:   -query_sn 192.168.10.108\n\n"

            << "Sample 9 : query hardware temperature\n"
            << "Format: -query_temp lidar_ip\n"
            << "Demo:   -query_temp 192.168.10.108\n\n"

            << "Sample 10 : query configuration\n"
            << "Format: -query_cfg lidar_ip\n"
            << "Demo:   -query_cfg 192.168.10.108\n\n"

            << "Sample 11 : get calibration data to file\n"
            << "Format: -get_cal lidar_ip savefilename\n"
            << "Demo:   -get_cal 192.168.10.108 device.cal\n\n"

            << "Sample 12 : firmware update\n"
            << "Format: -firmware_update lidar_ip filename\n"
            << "Demo:   -firmware_update 192.168.10.108 firmware_name.pack\n\n"

            << "Sample 13 : reboot\n"
            << "Format: -reboot lidar_ip\n"
            << "Demo:   -reboot 192.168.10.108\n\n"

            << "Sample 14 : scan device\n"
            << "Format: -scan_device scan_time(s)\n"
            << "Demo:   -scan_device 5\n\n"

            << "############################# END  GUIDE ################################\n\n"
            ;
        getchar();
        return 0;
    }
    std::string lidar_ip = std::string(argv[2]);

    if (0 == std::string(argv[1]).compare("-config_mac") && argc == 4)
        //Sample code 0 : Set lidar's mac address
        sample_config_lidar_mac_address(lidar_ip, std::string(argv[3]));

    else if (0 == std::string(argv[1]).compare("-config_static_ip") && argc == 4)
        //Sample code 1 : Set lidar's static ip address
        sample_config_lidar_ip(lidar_ip, std::string(argv[3]));

    else if (0 == std::string(argv[1]).compare("-config_subnet_mask") && argc == 4)
        //Sample code 2 : Set lidar's subnet mask
        sample_config_lidar_subnet_mask(lidar_ip, std::string(argv[3]));

    else if (0 == std::string(argv[1]).compare("-config_dst_ip") && argc == 4)
        //Sample code 3 : Set lidar's udp destination ip address
        sample_config_lidar_udp_destination_ip(lidar_ip, std::string(argv[3]));

    else if (0 == std::string(argv[1]).compare("-config_dst_port") && argc == 4)
        //Sample code 4 : Set lidar's udp destination port
        sample_config_lidar_udp_destination_port(lidar_ip, std::atoi(argv[3]));

    else if (0 == std::string(argv[1]).compare("-config_retro") && argc == 4)
        //Sample code 5 : Set lidar's retro function
        sample_config_lidar_retro_enable(lidar_ip, std::atoi(argv[3]));

    else if (0 == std::string(argv[1]).compare("-config_time_sync") && argc == 4)
        //Sample code 6 : Set lidar's time sync mode
        sample_config_lidar_time_sync(lidar_ip, zvision::TimestampType(std::atoi(argv[3])));

    else if (0 == std::string(argv[1]).compare("-query_version") && argc == 3)
        //Sample code 7 : Query lidar's firmware version
        sample_query_lidar_firmware_version(lidar_ip);

    else if (0 == std::string(argv[1]).compare("-query_sn") && argc == 3)
        //Sample code 8 : Query lidar's serial number
        sample_query_lidar_serial_number(lidar_ip);

    else if (0 == std::string(argv[1]).compare("-query_temp") && argc == 3)
        //Sample code 9 : Query lidar's hardware temperature
        sample_query_lidar_hardware_temperature(lidar_ip);

    else if (0 == std::string(argv[1]).compare("-query_cfg") && argc == 3)
        //Sample code 10 : Query lidar's configurature
        sample_query_lidar_configuration(lidar_ip);

    else if (0 == std::string(argv[1]).compare("-get_cal") && argc == 4)
        //Sample code 11 : Get lidar's calibration file by tcp connection.
        sample_get_lidar_calibration(lidar_ip, std::string(argv[3]));

    else if (0 == std::string(argv[1]).compare("-firmware_update") && argc == 4)
        //Sample code 12 : Firmware update.
        sample_firmware_update(lidar_ip, std::string(argv[3]));

    else if (0 == std::string(argv[1]).compare("-reboot") && argc == 3)
        //Sample code 13 : Reboot lidar by tcp connection.
        sample_reboot_lidar(lidar_ip);

    else if (0 == std::string(argv[1]).compare("-scan_device") && argc == 3)
        //Sample code 14 : Scan lidar on the heart beat port
        //Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
        sample_scan_lidar_on_heat_beat_port(std::atoi(argv[2]));
    else
    {
        LOG_ERROR("Invalid parameters\n.");
        return zvision::InvalidParameter;
    }

    return 0;
}


#if 0// test code
int main(int argc, char** argv)
{
    std::string lidar_ip = "192.168.10.108";

    //Sample code 0 : Set lidar's mac address
    sample_config_lidar_mac_address(lidar_ip, "76-66-66-66-66-66");

    //Sample code 1 : Set lidar's static ip address
    sample_config_lidar_ip(lidar_ip, "192.168.10.108");

    //Sample code 2 : Set lidar's subnet mask
    sample_config_lidar_subnet_mask(lidar_ip, "255.255.255.0");

    //Sample code 3 : Set lidar's udp destination ip address
    sample_config_lidar_udp_destination_ip(lidar_ip, "192.168.10.255");

    //Sample code 4 : Set lidar's udp destination port
    sample_config_lidar_udp_destination_port(lidar_ip, 2368);

    //Sample code 5 : Set lidar's retro function
    sample_config_lidar_retro_enable(lidar_ip, false);

    //Sample code 6 : Set lidar's time sync mode
    sample_config_lidar_time_sync(lidar_ip, zvision::TimestampType::TimestampPtp);

    //Sample code 7 : Query lidar's firmware version
    sample_query_lidar_firmware_version(lidar_ip);

    //Sample code 8 : Query lidar's serial number
    sample_query_lidar_serial_number(lidar_ip);

    //Sample code 9 : Query lidar's hardware temperature
    sample_query_lidar_configuration(lidar_ip);

    //Sample code 10 : Query lidar's configurature
    sample_query_lidar_hardware_temperature(lidar_ip);

    //Sample code 11 : Get lidar's calibration file by tcp connection.
    sample_get_lidar_calibration(lidar_ip);

    //Sample code 12 : Firmware update.
    sample_firmware_update(lidar_ip);

    //Sample code 13 : Reboot lidar by tcp connection.
    sample_reboot_lidar(lidar_ip);

    //Sample code 14 : Scan lidar on the heart beat port
    //Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
    sample_scan_lidar_on_heat_beat_port();

    return 0;
}

#endif
