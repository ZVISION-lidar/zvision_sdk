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
#include "tcp_nz1_a2.h"
#include "tcp_ez6_b2.h"
#include "tcp_tools.h"
#include "define.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <thread>
#include <memory>
#include "convert.hpp"
#include "loguru.hpp"
/* Progress bar defination */
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 50
/* Progress information */
static std::map<std::string, int> g_lidars_percent;

//Callback function for progress notify
void print_current_progress(std::string ip, int percent)
{
    LOG_F(INFO, "#%s:Current progress %3d.", ip.c_str(), percent);
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
	uint64_t idx = 0;
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

std::unique_ptr<zvision::tcp_tools> create_client(const std::string& model, std::string lidar_ip) {
    if (model == "ez6_b2") {
        return std::make_unique<zvision::tcp_ez6_b2>(lidar_ip,5000,5000,5000);
    } else if (model == "nz1_a2") {
        return std::make_unique<zvision::tcp_nz1_a2>(lidar_ip,5000,5000,5000);
    } else {
        throw std::invalid_argument("unknown model: " + model);
    }
}

//Sample code 1 : Config lidar ptp configuration file
int sample_config_lidar_ptp_configuration_file(zvision::tcp_tools *tcp_client, std::string filename)
{
	int	ret = tcp_client->set_ptpcfg(filename);
    if (ret)
        LOG_F(ERROR, "Set ptp configuration file to [%s] failed, ret = %d.",filename.c_str(), ret);
    else
        LOG_F(INFO, "Set ptp configuration file to [%s] ok.", filename.c_str());
    return ret;
}

//Sample code 2 : Get lidar ptp configuration file
int sample_get_lidar_ptp_configuration_to_file(zvision::tcp_tools *tcp_client, std::string filename)
{
	int	ret = tcp_client->get_ptpcfg_to_file(filename);
    if (ret)
        LOG_F(ERROR, "Get device ptp configuration file to [%s] failed, ret = %d.", filename.c_str(), ret);
    else
        LOG_F(INFO, "Get device ptp configuration file to [%s] ok.", filename.c_str());
    return ret;
}

//Sample code 3 : Get lidar basic info
int sample_get_lidar_basic_info(zvision::tcp_tools *tcp_client)
{
	zvision::DeviceConfigurationInfo basic_info;
	int	ret = tcp_client->get_basic_info(basic_info);
    if (ret)
	{
		LOG_F(ERROR, "Get basic info error");
	}
    else
	{
		std::ostringstream oss;
		oss << "sn: " << basic_info.serial_number << "\n"
			<< "factory_mac: " << basic_info.factory_mac << "\n"
			<< "fpga_version: " << basic_info.version.boot_version << "\n"
			<< "embedded_version: " << basic_info.version.kernel_version << "\n"
			<< "backup_fpga_version: " << basic_info.backup_version.boot_version << "\n"
			<< "backup_embedded_version: " << basic_info.backup_version.kernel_version << "\n"
			<< "ip: " << basic_info.device_ip << "\n"
			<< "netmask: " << basic_info.subnet_mask << "\n"
			<< "gateway: " << basic_info.gateway_addr << "\n"
			<< "mac: " << basic_info.device_mac << "\n"
			<< "dest_ip: " << basic_info.destination_ip << "\n"
			<< "dest_port: " << basic_info.destination_port << "\n"
			<< "frame_switch: " << basic_info.frame_switch << "\n"
			<< "frame_offset: " << basic_info.frame_offset << "\n"
			<< "work_mode: " << basic_info.work_mode << "\n";

		std::string result = oss.str();

		LOG_F(INFO, "Lidar basic info: ");
		LOG_F(INFO, "%s", result.c_str());
	}

    return ret;
}

//Sample code 4 : Firmware update.
int sample_firmware_update(zvision::tcp_tools *tcp_client, std::string filename)
{
	int	ret = tcp_client->update_firmware(filename, print_current_progress_multi, false);
    if (ret)
        LOG_F(ERROR, "Update device firmware %s failed, ret = %d.", filename.c_str(), ret);
    else
    {
        LOG_F(INFO, "Update device fireware %s ok.", filename.c_str());
    }
    return ret;
}



void init_log(int argc, char* argv[])
{
	loguru::init();
	loguru::add_file("Log/log.txt", loguru::Truncate, loguru::Verbosity_INFO);
	loguru::g_stderr_verbosity = 4;
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
    if (argc <= 2)
    {
        std::cout << "############################# USER GUIDE ################################\n\n"

			<< "Note: Only For EZ6 B2 and NZ1 A2\n\n"

			<< "Sample 1 : config ptp configuration\n"
            << "Format: nz1_a2 -set_ptp_cfg lidar_ip filename\n"
			<< "Demo:   nz1_a2 -set_ptp_cfg 192.168.10.108 test.txt\n"

            << "Sample 2 : get ptp configuration to file\n"
            << "Format: nz1_a2 -get_ptp_cfg lidar_ip filename\n"
			<< "Demo:   nz1_a2 -get_ptp_cfg 192.168.10.108 test.txt\n"

			<< "Sample 3 : get basic info\n"
            << "Format: nz1_a2 -get_basic_info lidar_ip\n"
			<< "Demo:   nz1_a2 -get_basic_info 192.168.10.108\n"

			<< "############################# END  GUIDE ################################\n\n"
            ;
        getchar();
        return 0;
    }

	init_log(argc, argv);

	std::string opt = std::string(argv[1]);
	std::string appname = "";
	std::string lidar_ip = "";

	std::map<std::string, std::string> paras;
	ParamResolver::GetParameters(argc, argv, paras, appname);
	lidar_ip = std::string(argv[3]);

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
			lidars_ip = std::vector<std::string>{ std::begin(lidars_ip),std::begin(lidars_ip) + 4 };
	}

	std::string lidar_model = std::string(argv[1]);
	std::unique_ptr<zvision::tcp_tools> lidar_client = create_client(lidar_model, lidar_ip);

    if (0 == std::string(argv[2]).compare("-set_ptp_cfg") && argc == 5)
		//Sample code 1 : Config lidar ptp configuration file
		sample_config_lidar_ptp_configuration_file(lidar_client.get(), std::string(argv[4]));

	else if (0 == std::string(argv[2]).compare("-get_ptp_cfg") && argc == 5)
		//Sample code 2 : Get lidar ptp configuration file
		sample_get_lidar_ptp_configuration_to_file(lidar_client.get(), std::string(argv[4]));

    else if (0 == std::string(argv[2]).compare("-get_basic_info") && argc == 4)
		//Sample code 3 : get_basic_info
		sample_get_lidar_basic_info(lidar_client.get());

    else
    {
        LOG_F(ERROR, "Invalid parameters.");
        return zvision::InvalidParameter;
    }

    return 0;
}
