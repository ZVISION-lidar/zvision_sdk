#include "print.h"
#include "lidar_tools.h"
#include <stdio.h>
#include <fstream>
#include <iostream>

//Sample code 0 : Set lidar's mac address
int sample_config_lidar_mac_address(std::string lidar_ip, std::string mac)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
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
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
    zvision::DeviceConfigurationInfo info;
    if (ret = config.QueryDeviceConfigurationInfo(info))
        LOG_ERROR("Query device [%s]'s configuration info failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Query device [%s]'s configuration info ok.\n", lidar_ip.c_str());
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
    }
    return ret;
}

//Sample code 11 : Get lidar's calibration file by tcp connection.
int sample_get_lidar_calibration(std::string lidar_ip)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 1000, 500, 500);
    std::string savefilename = "sample_get_lidar_calibration.cal";
    if (ret = config.GetDeviceCalibrationDataToFile(savefilename))
        LOG_ERROR("Get device [%s]'s calibration data failed, ret = %d.\n", lidar_ip.c_str(), ret);
    else
    {
        LOG_INFO("Get device [%s]'s calibration data ok.\n", lidar_ip.c_str());
        LOG_INFO("Calibration data save to file %s.\n", savefilename.c_str());
    }
    return ret;
}

//Sample code 12 : Scan lidar on the heart beat port
//Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
int sample_scan_lidar_on_heat_beat_port()
{
    int ret = 0;
    std::vector<zvision::DeviceConfigurationInfo> devices;
    
    if (ret = zvision::LidarTools::ScanDevice(devices, 5))
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

int main()
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

    //Sample code 12 : Scan lidar on the heart beat port
    //Notice, this function is supported by the lidar's new firmware kernel version, at least 0.1.20
    sample_scan_lidar_on_heat_beat_port();

    return 0;
}
