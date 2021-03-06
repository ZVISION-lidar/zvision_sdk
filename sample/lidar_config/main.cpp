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
#include <map>
#include <set>
#include <thread>

/* Progress bar defination */
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 50
/* Progress information */
static std::map<std::string, int> g_lidars_percent;

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

/* String split */
void strSplit(const std::string& s, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
	std::string::size_type lastPos = s.find_first_not_of(delimiters, 0);
	std::string::size_type pos = s.find_first_of(delimiters, lastPos);
	while (std::string::npos != pos || std::string::npos != lastPos) {
		tokens.push_back(s.substr(lastPos, pos - lastPos));
		//use emplace_back after C++11
		lastPos = s.find_first_not_of(delimiters, pos);
		pos = s.find_first_of(delimiters, lastPos);
	}
}

/* Verify ip */
bool AssembleIpString(const std::string& ip)
{
	try
	{
		/* ip format: xxx.xxx.xxx.xxx */
		if (ip.size() > 15)
			return false;

		std::vector<std::string> vals;
		strSplit(ip, vals, ".");
		if (vals.size() != 4)
			return false;

		for (auto s : vals) {
			for (auto c : s) {
				if (c<'0' || c>'9')
					return false;
			}
		}
	}
	catch (const std::exception& e)
	{
		return false;
	}
	return true;
}

//Callback function for progress notify
void print_current_progress(std::string ip, int percent)
{
    LOG_INFO("#%s:Current progress %3d.\n", ip.c_str(), percent);
}

/* Callback function for progress notifiy with progress bar */
void print_current_progress_multi(int percent, std::string ip)
{
	g_lidars_percent[ip] = percent;
	//get min percent
	int min_percent = percent;
	for (auto it : g_lidars_percent)
		if (it.second < min_percent) min_percent = it.second;

	int lpad = (int)(1.0f * min_percent / 100 * PBWIDTH);
	int rpad = PBWIDTH - lpad;
	int idx = 0;
	for (auto it : g_lidars_percent) {

		if (idx == 0)
			printf("\r\r #%s:[%3d%%] ", it.first.c_str(), it.second);
		else
			printf("#%s:[%3d%%] ", it.first.c_str(), it.second);

		if (idx == (g_lidars_percent.size() - 1))
			printf(" [%.*s%*s]", lpad, PBSTR, rpad, "");
		idx++;
	}
	fflush(stdout);
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
        LOG_INFO("FPGA   version: %u.%u.%u\n", version.boot_version[0], version.boot_version[1], version.boot_version[2], version.boot_version[3]);
        LOG_INFO("Embedded version: %u.%u.%u\n", version.kernel_version[0], version.kernel_version[1], version.kernel_version[2], version.kernel_version[3]);
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
    if (ret = config.FirmwareUpdate(filename, print_current_progress_multi))
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
            std::string cfg_desp = zvision::get_cfg_info_string(info);
            LOG_INFO("####################### Device number %3d #######################\n", i);
            LOG_INFO("%s\n", cfg_desp.c_str());
            LOG_INFO("#################################################################\n\n");
        }
    }
    return ret;
}

//Sample code 15 : Config lidar retro parameter 1(min ref, [0,100])
int sample_config_lidar_retro_param_min_ref(std::string lidar_ip, int ref)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceRetroParam1MinRef(ref))
        LOG_ERROR("Set device [%s]'s retro param 1 to [%d] failed, ret = %d.\n", lidar_ip.c_str(), ref, ret);
    else
        LOG_INFO("Set device [%s]'s retro param 1 to [%d] ok.\n", lidar_ip.c_str(), ref);
    return ret;
}

//Sample code 16 : Config lidar retro parameter 2(point percentage, [0,100])
int sample_config_lidar_retro_param_point_percentage(std::string lidar_ip, int percent)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDeviceRetroParam2PointPercentage(percent))
        LOG_ERROR("Set device [%s]'s retro param 2 to [%d] failed, ret = %d.\n", lidar_ip.c_str(), percent, ret);
    else
        LOG_INFO("Set device [%s]'s retro param 2 to [%d] ok.\n", lidar_ip.c_str(), percent);
    return ret;
}

//Sample code 17 : Config lidar phaseoffset enable
int sample_config_lidar_phase_offset_enable(std::string lidar_ip, bool en)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDevicePhaseOffsetEnable(en))
        LOG_ERROR("Set device [%s]'s phase offset enable to [%d] failed, ret = %d.\n", lidar_ip.c_str(), en, ret);
    else
        LOG_INFO("Set device [%s]'s phase offset enable to [%d] ok.\n", lidar_ip.c_str(), en);
    return ret;
}

//Sample code 18 : Config lidar phaseoffset value
int sample_config_lidar_phase_offset_value(std::string lidar_ip, int value_5ns)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDevicePhaseOffset(value_5ns))
        LOG_ERROR("Set device [%s]'s phase offset value to [%d]x5ns failed, ret = %d.\n", lidar_ip.c_str(), value_5ns, ret);
    else
        LOG_INFO("Set device [%s]'s phase offset value to [%d]x5ns ok.\n", lidar_ip.c_str(), value_5ns);
    return ret;
}

//Sample code 19 : Config lidar ptp configuration file
int sample_config_lidar_ptp_configuration_file(std::string lidar_ip, std::string filename)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.SetDevicePtpConfiguration(filename))
        LOG_ERROR("Set device [%s]'s ptp configuration file to [%s] failed, ret = %d.\n", lidar_ip.c_str(), filename.c_str(), ret);
    else
        LOG_INFO("Set device [%s]'s ptp configuration file to [%s] ok.\n", lidar_ip.c_str(), filename.c_str());
    return ret;
}

//Sample code 20 : Get lidar ptp configuration file
int sample_get_lidar_ptp_configuration_to_file(std::string lidar_ip, std::string filename)
{
    int ret = 0;
    zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
    if (ret = config.GetDevicePtpConfigurationToFile(filename))
        LOG_ERROR("Get device [%s]'s ptp configuration file to [%s] failed, ret = %d.\n", lidar_ip.c_str(), filename.c_str(), ret);
    else
        LOG_INFO("Get device [%s]'s ptp configuration file to [%s] ok.\n", lidar_ip.c_str(), filename.c_str());
    return ret;
}

//Sample code 21 : Config lidar calibration file broadcast enable
int sample_config_lidar_cali_file_broadcast_mode(std::string lidar_ip, bool en)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::CalSendMode mode = zvision::CalSendMode::CalSendDisable;
	if (en)
		mode = zvision::CalSendMode::CalSendEnable;

	if (ret = config.SetDeviceCalSendMode(mode))
		LOG_ERROR("Set device [%s]'s calibration file broadcast enable to [%d] failed, ret = %d.\n", lidar_ip.c_str(), en, ret);
	else
		LOG_INFO("Set device [%s]'s calibration file broadcast enable to [%d] ok.\n", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 22 : Config lidar downsample mode
int sample_config_lidar_downsample_mode(std::string lidar_ip, std::string mode)
{
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DownsampleMode dm = zvision::DownsampleMode::DownsampleUnknown;
	if (mode.compare("none") == 0)
		dm = zvision::DownsampleNone;
	else if (mode.compare("1/2") == 0)
		dm = zvision::Downsample_1_2;
	else if (mode.compare("1/4") == 0)
		dm = zvision::Downsample_1_4;
	else {
		LOG_ERROR("Set device [%s]'s downsample mode to [%s] failed, invalid parameters input.\n", lidar_ip.c_str(), mode.c_str());
		return ret;
	}

	if (ret = config.SetDeviceDownsampleMode(dm))
		LOG_ERROR("Set device [%s]'s downsample mode to [%s] failed, ret = %d.\n", lidar_ip.c_str(), mode.c_str(), ret);
	else
		LOG_INFO("Set device [%s]'s downsample mode to [%s] ok.\n", lidar_ip.c_str(), mode.c_str());
	return ret;
}

//Sample code 23 : Config lidar retro parameter
int sample_set_lidar_retro_parameters(std::string lidar_ip,int id, std::string val) {

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	switch (id)
	{
	case 2:
		ret = config.SetDeviceRetroParam(zvision::RetroDisThres,std::atoi(val.c_str()));break;
	case 3:
		ret = config.SetDeviceRetroParam(zvision::RetroLowRangeThres, (unsigned short)std::atoi(val.c_str())); break;
	case 4:
		ret = config.SetDeviceRetroParam(zvision::RetroHighRangeThres, (unsigned short)std::atoi(val.c_str())); break;
	case 1:
		ret = config.SetDeviceRetroParam(zvision::RetroMinGrayNum, (unsigned char)std::atoi(val.c_str())); break;
	case 5:
		ret = config.SetDeviceRetroParam(zvision::RetroDelGrayThres, (unsigned char)std::atoi(val.c_str())); break;
	case 6:
		ret = config.SetDeviceRetroParam(zvision::RetroDelRatioGrayLowThres, (unsigned char)std::atoi(val.c_str())); break;
	case 7:
		ret = config.SetDeviceRetroParam(zvision::RetroDelRatioGrayHighThres, (unsigned char)std::atoi(val.c_str())); break;
	case 8:
		ret = config.SetDeviceRetroParam(zvision::RetroMinGray, (unsigned char)std::atoi(val.c_str())); break;
	default:
		ret = -1; break;
	}

	if (ret != 0)
		LOG_ERROR("Set device [%s]'s retro parameters failed, ret = %d.\n", lidar_ip.c_str(), ret);
	else
		LOG_INFO("Set device [%s]'s retro parameters ok.\n", lidar_ip.c_str());

	return ret;
}

//Sample code 24 : Config lidar adhesion parameter
int sample_set_lidar_adhesion_parameters(std::string lidar_ip, int id, std::string val) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	switch (id)
	{
	case 1:
		ret = config.SetDeviceAdhesionParam(zvision::MinimumHorizontalAngleRange, std::atoi(val.c_str())); break;
	case 2:
		ret = config.SetDeviceAdhesionParam(zvision::MaximumHorizontalAngleRange, std::atoi(val.c_str())); break;
	case 3:
		ret = config.SetDeviceAdhesionParam(zvision::MinimumVerticalAngleRange, std::atoi(val.c_str())); break;
	case 4:
		ret = config.SetDeviceAdhesionParam(zvision::MaximumVerticalAngleRange, std::atoi(val.c_str())); break;
	case 5:
		ret = config.SetDeviceAdhesionParam(zvision::HorizontalAngleResolution, (float)std::atof(val.c_str())); break;
	case 6:
		ret = config.SetDeviceAdhesionParam(zvision::VerticalAngleResolution, (float)std::atof(val.c_str())); break;
	case 7:
		ret = config.SetDeviceAdhesionParam(zvision::DeletePointThreshold, (float)std::atof(val.c_str())); break;
	case 8:
		ret = config.SetDeviceAdhesionParam(zvision::MaximumProcessingRange, (float)std::atof(val.c_str())); break;
	case 9:
		ret = config.SetDeviceAdhesionParam(zvision::NearFarPointDiff, (float)std::atof(val.c_str())); break;
	default:
		ret = -1; break;
	}

	if (ret != 0)
		LOG_ERROR("Set device [%s]'s adhesion parameters failed, ret = %d.\n", lidar_ip.c_str(), ret);
	else
		LOG_INFO("Set device [%s]'s adhesion parameters ok.\n", lidar_ip.c_str());

	return ret;
}

//Sample code 25 : Get lidar algorithm parameter(retro and adhesion)
int sample_get_lidar_algorithm_parameters(std::string lidar_ip) {

	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	zvision::DeviceAlgoParam param;
	if (ret = config.QueryDeviceAlgoParam(param))
		LOG_ERROR("Get device [%s]'s algorithm parameters failed, ret = %d.\n", lidar_ip.c_str(), ret);
	else {
		LOG_INFO("Get device [%s]'s algorithm parameters ok.\n", lidar_ip.c_str());
		std::string msg;
		msg += "------  Retro  -------\n";
		msg += "min gray num: " + std::to_string(param.retro_min_gray_num) + "\n";
		msg += "dis thres: " + std::to_string(param.retro_dis_thres) + "\n";
		msg += "low range thres: " + std::to_string(param.retro_low_range_thres) + "\n";
		msg += "high range thres: " + std::to_string(param.retro_high_range_thres) + "\n";
		msg += "del gray thres: " + std::to_string(param.retro_del_gray_thres) + "\n";
		msg += "del ratio gray low thres: " + std::to_string(param.retro_del_ratio_gray_low_thres) + "\n";
		msg += "del ratio gray high thres: " + std::to_string(param.retro_del_ratio_gray_high_thres) + "\n";
		msg += "min gray: " + std::to_string(param.retro_min_gray) + "\n";
		msg += "-----  Adhesion  -----\n";
		msg += "angle hor min: " + std::to_string(param.adhesion_angle_hor_min) +"\n";
		msg += "angle hor max: " + std::to_string(param.adhesion_angle_hor_max) +"\n";
		msg += "angle ver min: " + std::to_string(param.adhesion_angle_ver_min) +"\n";
		msg += "angle ver max: " + std::to_string(param.adhesion_angle_ver_max) +"\n";
		msg += "angle hor res: " + std::to_string(param.adhesion_angle_hor_res) +"\n";
		msg += "angle ver res: " + std::to_string(param.adhesion_angle_ver_res) +"\n";
		msg += "diff thres: " + std::to_string(param.adhesion_diff_thres) + "\n";
		msg += "dis limit: " + std::to_string(param.adhesion_dis_limit) + "\n";
		msg += "min diff: " + std::to_string(param.adhesion_min_diff) + "\n";
		LOG_INFO(msg.c_str());
	}

	return ret;
}

//Sample code 26 : Config lidar delete close points enable
int sample_config_lidar_delete_points(std::string lidar_ip, bool en) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.SetDeviceAlgorithmEnable(zvision::AlgoType::AlgoDeleteClosePoints,en))
		LOG_ERROR("Set device [%s]'s delete close points enable to [%d] failed, ret = %d.\n", lidar_ip.c_str(), en, ret);
	else
		LOG_INFO("Set device [%s]'s delete close points enable to [%d] ok.\n", lidar_ip.c_str(), en);
	return ret;
}

//Sample code 27 : Config lidar adhesion enable
int sample_config_lidar_adhesion(std::string lidar_ip, bool en) {
	int ret = 0;
	zvision::LidarTools config(lidar_ip, 5000, 5000, 5000);
	if (ret = config.SetDeviceAlgorithmEnable(zvision::AlgoType::AlgoAdhesion, en))
		LOG_ERROR("Set device [%s]'s adhesion enable to [%d] failed, ret = %d.\n", lidar_ip.c_str(), en, ret);
	else
		LOG_INFO("Set device [%s]'s adhesion enable to [%d] ok.\n", lidar_ip.c_str(), en);
	return ret;
}

/* Handling multiple lidars */
//Sample code 28 : Muitiple lidars Firmware update.
int sample_firmware_update_multi(std::vector<std::string> lidars_ip, std::string filename)
{
	// start task
	int devs = lidars_ip.size();
	std::vector<std::shared_ptr<std::thread>> thr_vec;
	thr_vec.resize(devs);
	for (int i = 0; i < devs; i++) {
		thr_vec[i].reset(new std::thread(std::bind(&sample_firmware_update, lidars_ip[i], filename)));
	}
	// wait for finish
	for (int i = 0; i < devs; i++) {
		if (thr_vec[i]->joinable())
			thr_vec[i]->join();
	}
	return 0;
}
//Sample code 29 : Config muitiple lidars adhesion enable.
int sample_config_lidar_adhesion_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_adhesion(ip, en);

	return 0;
}
//Sample code 30 : Config muitiple lidars delete_points enable.
int sample_config_lidar_delete_points_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_delete_points(ip, en);

	return 0;
}
//Sample code 31 : Config muitiple lidars retro enable.
int sample_config_lidar_retro_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip: lidars_ip)
		sample_config_lidar_retro_enable(ip, en);

	return 0;
}
//Sample code 32 : Config muitiple lidars phase offset enable.
int sample_config_lidar_phase_offset_multi(std::vector<std::string> lidars_ip, bool en) {

	// start task
	for (auto ip : lidars_ip)
		sample_config_lidar_phase_offset_enable(ip, en);

	return 0;
}
//Sample code 33 : Config muitiple lidars phase offset enable.
int sample_query_lidar_configuration_multi(std::vector<std::string> lidars_ip) {

	// start task
	for (auto ip: lidars_ip)
		sample_query_lidar_configuration(ip);

	return 0;
}
//Sample code 34 : Reboot lidars by tcp connection.
int sample_reboot_lidar_multi(std::vector<std::string> lidars_ip) {

	// start task
	for (auto ip: lidars_ip)
		sample_reboot_lidar(ip);

	return 0;
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
			<< "Format: -config_retro_multi lidar1_ip lidar2_ip (MaxLidarCount:4) mode\n"
            << "Demo:   -config_retro 192.168.10.108 0\n"
			<< "Demo:   -config_retro_multi 192.168.10.108 192.168.10.109 0\n\n"

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
			<< "Format: -query_cfg_multi lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
            << "Demo:   -query_cfg 192.168.10.108\n"
			<< "Demo:   -query_cfg_multi 192.168.10.108 192.168.10.109\n\n"

            << "Sample 11 : get calibration data to file\n"
            << "Format: -get_cal lidar_ip savefilename\n"
            << "Demo:   -get_cal 192.168.10.108 device.cal\n\n"

            << "Sample 12 : firmware update\n"
            << "Format: -firmware_update lidar_ip filename\n"
			<< "Format: -firmware_update_multi lidar1_ip lidar2_ip(MaxLidarCount:4) filename\n"
            << "Demo:   -firmware_update 192.168.10.108 firmware_name.pack\n"
			<< "Demo:   -firmware_update_multi 192.168.10.108 192.168.10.109 firmware_name.pack\n\n"

            << "Sample 13 : reboot\n"
            << "Format: -reboot lidar_ip\n"
			<< "Format: -reboot_multi lidar1_ip lidar2_ip(MaxLidarCount:4) \n"
            << "Demo:   -reboot 192.168.10.108\n"
			<< "Demo:   -reboot_multi 192.168.10.108 192.168.10.109\n\n"

            << "Sample 14 : scan device\n"
            << "Format: -scan_device scan_time(s)\n"
            << "Demo:   -scan_device 5\n\n"

            << "Sample 15 : retro param 1(min ref[0,100])\n"
            << "Format: -retro_p1 value\n"
            << "Demo:   -retro_p1 5\n\n"

            << "Sample 16 : retro param 2(point percentage[0,100])\n"
            << "Format: -retro_p2 value\n"
            << "Demo:   -retro_p2 5\n\n"

            << "Sample 17 : phase offset enable(0 for disable, 1 for enable)\n"
            << "Format: -phase_offset_enable lidar_ip mode\n"
			<< "Format: -phase_offset_enable_multi lidar1_ip lidar2_ip(MaxLidarCount:4)\n"
            << "Demo:   -phase_offset_enable 192.168.10.108 0\n"
			<< "Demo:   -phase_offset_enable_multi 192.168.10.108 192.168.10.109 0\n\n"

            << "Sample 18 : phase offset value\n"
            << "Format: -phase_offset_value value(x5ns)\n"
            << "Demo:   -phase_offset_value 0\n\n"

            << "Sample 19 : config ptp configuration\n"
            << "Format: -set_ptp_cfg filename\n"
            << "Demo:   -set_ptp_cfg test.txt\n\n"

            << "Sample 20 : get ptp configuration to file\n"
            << "Format: -get_ptp_cfg filename\n"
            << "Demo:   -get_ptp_cfg test.txt\n\n"

			<< "Sample 21 : calibration file broadcast enabale\n"
			<< "Format: -cali_file_broadcast_enable lidar_ip enable(0 for disable, 1 for enable) \n"
			<< "Demo:   -cali_file_broadcast_enable 192.168.10.108 0\n\n"

			<< "Sample 22 : config downsample mode\n"
			<< "Format: -downsample_mode lidar_ip mode( none: no downsample, 1/2: 50% downsample, 1/4: 25% downsample)\n"
			<< "Demo:   -downsample_mode 192.168.10.108 none\n\n"

			<< "Sample 23 : set Retro parameters\n"
			<< "1. min gray num:              High gray value num threshold  (int8)\n"
			<< "2. dis thres:                 Lock critical point distance   (uint32)\n"
			<< "3. low range thres:           Delete point range low threshold distance  (uint16)\n"
			<< "4. high range thres:          Delete point range high threshold distance (uint16)\n"
			<< "5. del gray thres::           Delete point grayscale threshold   (uint8)\n"
			<< "6. del ratio gray low thres:  The gray value low threshold of the deleted point ratio  (uint8)\n"
			<< "7. del ratio gray high thres: The gray value high threshold of the deleted point ratio (uint8)\n"
			<< "8. min gray:                  Minimum gray value  (uint8)\n"
			<< "Format: -set_retro_param lidar_ip patameter_id value\n"
			<< "Demo:   -set_retro_param 192.168.10.108 4 3\n\n"

			<< "Sample 24 : set Adhesion parameters\n"
			<< "1. angle hor min:  Minimum horizontal angle range (int32)\n"
			<< "2. angle hor max:  Maximum horizontal angle range (int32)\n"
			<< "3. angle ver min:  Minimum vertical angle range (int32)\n"
			<< "4. angle ver max:  Maximum vertical angle range (int32)\n"
			<< "5. angle hor res:  Horizontal angle resolution (float)\n"
			<< "6. angle ver res:  Vertical angle resolution (float)\n"
			<< "7. diff thres:     Delete point threshold (float)\n"
			<< "8. dis limit:      Maximum processing distance  (float)\n"
			<< "9. min diff:       Distance difference between nearest and farthest points (float)\n"
			<< "Format: -set_adhesion_param lidar_ip patameter_id value\n"
			<< "Demo:   -set_adhesion_param 192.168.10.108 3 -40\n\n"

			<< "Sample 25 : get algorithm parameters\n"
			<< "Format: -get_algo_param lidar_ip\n"
			<< "Demo:   -get_algo_param 192.168.10.108\n\n"

			<< "Sample 26 : config delete close points mode\n"
			<< "Format: -config_delete_points lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_delete_points_multi lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Demo:   -config_delete_points 192.168.10.108 0\n"
			<< "Demo:   -config_delete_points_multi 192.168.10.108 192.168.10.109 0\n\n"

			<< "Sample 27 : config adhesion mode\n"
			<< "Format: -config_adhesion lidar_ip mode(0 for disable, 1 for enable)\n"
			<< "Format: -config_adhesion_multi lidar1_ip lidar2_ip(MaxLidarCount:4) mode\n"
			<< "Demo:   -config_adhesion 192.168.10.108 0\n"
			<< "Demo:   -config_adhesion_multi 192.168.10.108 192.168.10.109 0\n\n"

			<< "############################# END  GUIDE ################################\n\n"
            ;
        getchar();
        return 0;
    }
    std::string lidar_ip = std::string(argv[2]);

	// Get lidar list
	std::vector<std::string> lidars_ip;
	{
		std::set<std::string> temp;
		for (int i = 0; i < argc; i++) {
			std::string ip(argv[i]);
			// filter ip
			if (AssembleIpString(ip)) {
				if (temp.insert(ip).second)
					lidars_ip.push_back(ip);
			}
		}

		// check
		if (lidars_ip.size() > 4)
			lidars_ip = std::vector<std::string>{ std::begin(lidars_ip),std::begin(lidars_ip) + 3 };
	}

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

	else if (0 == std::string(argv[1]).compare("-retro_p1") && argc == 4)
		//Sample code 15 : Config lidar retro parameter 1(min ref, [0,100])
		sample_config_lidar_retro_param_min_ref(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-retro_p2") && argc == 4)
		//Sample code 16 : Config lidar retro parameter 2(point percentage, [0,100])
		sample_config_lidar_retro_param_point_percentage(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-phase_offset_enable") && argc == 4)
		//Sample code 17 : Config lidar phaseoffset enable
		sample_config_lidar_phase_offset_enable(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-phase_offset_value") && argc == 4)
		//Sample code 18 : Config lidar phaseoffset value
		sample_config_lidar_phase_offset_value(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-set_ptp_cfg") && argc == 4)
		//Sample code 19 : Config lidar ptp configuration file
		sample_config_lidar_ptp_configuration_file(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-get_ptp_cfg") && argc == 4)
		//Sample code 20 : Get lidar ptp configuration file
		sample_get_lidar_ptp_configuration_to_file(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-cali_file_broadcast_enable") && argc == 4)
		//Sample code 21 : onfig lidar calibration file broadcast enable
		sample_config_lidar_cali_file_broadcast_mode(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-downsample_mode") && argc == 4)
		//Sample code 22 : Config lidar downsample mode
		sample_config_lidar_downsample_mode(lidar_ip, std::string(argv[3]));

	else if (0 == std::string(argv[1]).compare("-set_retro_param") && argc == 5)
		//Sample code 23 : Set lidar retro parameter
		sample_set_lidar_retro_parameters(lidar_ip, std::atoi(argv[3]), std::string(argv[4]));

	else if (0 == std::string(argv[1]).compare("-set_adhesion_param") && argc == 5)
		//Sample code 24 : Set lidar adhesion parameter
		sample_set_lidar_adhesion_parameters(lidar_ip, std::atoi(argv[3]), std::string(argv[4]));

	else if (0 == std::string(argv[1]).compare("-get_algo_param") && argc == 3)
		//Sample code 25 : Get lidar algorithm parameter(retro and adhesion)
		sample_get_lidar_algorithm_parameters(lidar_ip);

	else if (0 == std::string(argv[1]).compare("-config_delete_points") && argc == 4)
		//Sample code 26 : Config lidar delete close points enable
		sample_config_lidar_delete_points(lidar_ip, std::atoi(argv[3]));

	else if (0 == std::string(argv[1]).compare("-config_adhesion") && argc == 4)
		//Sample code 27 : Config lidar adhesion mode
		sample_config_lidar_adhesion(lidar_ip, std::atoi(argv[3]));

	// Handling multiple lidars
	else if (0 == std::string(argv[1]).compare("-config_retro_multi"))
		//Sample code 28 : Set lidars` retro function
		sample_config_lidar_retro_multi(lidars_ip, std::atoi(argv[argc-1]));

	else if (0 == std::string(argv[1]).compare("-query_cfg_multi"))
		//Sample code 29 : Query lidars` configurature
		sample_query_lidar_configuration_multi(lidars_ip);

	else if (0 == std::string(argv[1]).compare("-firmware_update_multi"))
		//Sample code 30 : Firmwares update
		sample_firmware_update_multi(lidars_ip, std::string(argv[argc-1]));

	else if (0 == std::string(argv[1]).compare("-reboot_multi"))
		//Sample code 31 : Reboot lidars by tcp connection.
		sample_reboot_lidar_multi(lidars_ip);

	else if (0 == std::string(argv[1]).compare("-phase_offset_enable_multi"))
		//Sample code 32 : Config lidars` phaseoffset enable
		sample_config_lidar_phase_offset_multi(lidars_ip, std::atoi(argv[argc-1]));

	else if (0 == std::string(argv[1]).compare("-config_delete_points_multi"))
		//Sample code 33 : Config lidars` delete close points enable
		sample_config_lidar_delete_points_multi(lidars_ip, std::atoi(argv[argc-1]));

	else if (0 == std::string(argv[1]).compare("-config_adhesion_multi"))
		//Sample code 34 : Config lidars` adhesion mode
		sample_config_lidar_adhesion_multi(lidars_ip, std::atoi(argv[argc-1]));

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
