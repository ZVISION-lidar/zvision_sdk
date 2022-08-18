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
#include "packet.h"
#include "lidar_tools.h"
#include "print.h"
#include "json_ml30s_plus.h"

#include <rapidjson/document.h>

#include <set>
#include <cmath>
#include <math.h>
#include <cstring>
#include <iostream>
#include <functional>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>
#include <type_traits>

namespace zvision
{
    LidarTools::LidarTools(std::string lidar_ip, int con_timeout, int send_timeout, int recv_timeout):
        client_(new TcpClient(con_timeout, send_timeout, recv_timeout)),
        device_ip_(lidar_ip),
        conn_ok_(false)                                                                                                                                             
    {

    }

    LidarTools::~LidarTools()
    {
        this->DisConnect();
    }

    int LidarTools::ScanDevice(std::vector<DeviceConfigurationInfo>& device_list, int scan_time)
    {
        const auto before = std::chrono::system_clock::now();

        const int heart_beat_port = 55000;
        UdpReceiver recv(heart_beat_port, 100);
        std::string data;
        const int heart_beat_len = 48;

        int len = 0;
        uint32_t ip = 0;
        int ret = 0;
        std::string packet_header = "ZVSHEARTBEAT";
		std::string packet_header_30splus = "zvision";
        std::vector<std::string> lidars;
        device_list.clear();

        LOG_DEBUG("\nScan device on the heart beat port %d for %d second(s).\n", heart_beat_port, scan_time);
        while (1)
        {
            if (0 != (ret = recv.SyncRecv(data, len, ip)))
            {
                LOG_ERROR("Scan device error, receive failed ret = %d.\n", ret);
                break;
            }

			if ((len >= heart_beat_len) \
				&& ((0 == packet_header.compare(0, packet_header.size(), data.substr(0, packet_header.size()))) || \
				(0 == packet_header_30splus.compare(0, packet_header_30splus.size(), data.substr(0, packet_header_30splus.size())))\
					)\
				)
			{
                std::string ip_string = IpToString(ip);
                lidars.push_back(ip_string);
            }

            // time elapse
            std::chrono::duration<double> duration = std::chrono::system_clock::now() - before;
            if (duration.count() >= scan_time)
                break;
        }

        // remove the same value
        // https://stackoverflow.com/questions/1041620/whats-the-most-efficient-way-to-erase-duplicates-and-sort-a-vector
        std::set<std::string> set;
        unsigned size = lidars.size();
        for (unsigned i = 0; i < size; ++i) set.insert(lidars[i]);
        lidars.assign(set.begin(), set.end());

        for (unsigned int i = 0; i < lidars.size(); ++i)
        {
            LidarTools config(lidars[0], 1000);
            DeviceConfigurationInfo info;
            if (0 != (ret = config.QueryDeviceConfigurationInfo(info)))
            {
                LOG_ERROR("Scan device error, query device info failed, ret = %d.\n", ret);
                break;
            }
            device_list.push_back(info);
        }
        return ret;
    }


    int LidarTools::ReadCalibrationData(std::string filename, CalibrationData& cal)
    {
        const int file_min_lines = 4;
        std::ifstream file;
        file.open(filename, std::ios::in);
        std::string line;
        if(!file.is_open())
            return OpenFileError;

        // read all lines to vector
        std::vector<std::vector<std::string>> lines;
        while (std::getline(file, line))
        {
            if (line.size() > 0)
            {
                std::istringstream iss(line);
                std::vector<std::string> datas;
                std::string data;
                while (iss >> data)
                {
                    datas.push_back(data);
                }
                lines.push_back(datas);
            }
        }
        file.close();

        if (lines.size() < file_min_lines)
            return InvalidContent;

        // filter #
        int curr = 0;
        for (auto& au : lines)
        {
            if ('#' != au[0][0])
                break;
            curr++;
        }

        // read version info
        std::string version_str;
        if ((lines[curr].size() >= 2) && (lines[curr][0] == "VERSION"))
        {
            version_str = lines[curr][lines[curr].size() - 1];
            curr++;
        }

        // read scan mode
        std::string mode_str;
        if ((lines[curr].size() >= 2) && (lines[curr][0] == "Mode"))
        {
            mode_str = lines[curr][lines[curr].size() - 1];
            curr++;
        }
        int ret = 0;
        if (mode_str.size())
        {
            if ("MLX_A1_190" == mode_str)
            {
                if ((lines.size() - curr) < 38000)//check the file lines
                    return InvalidContent;

                cal.data.resize(38000 * 3 * 2);
                for (int i = 0; i < 38000; i++)
                {
                    const int column = 7;

                    std::vector<std::string>& datas = lines[i + curr];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarMLX;
                cal.scan_mode = ScanMode::ScanMLX_190;
                cal.description = "";
                for (int i = 0; i < curr; i++)
                {
                    for (int j = 0; j < lines[i].size(); j++)
                        cal.description += lines[i][j];
                    cal.description += "\n";
                }
            }
            else if (("ML30S_160_1_2" == mode_str) || ("ML30S_160_1_4" == mode_str))
            {
                int group = 3200;
                if ("ML30S_160_1_2" == mode_str)
                {
                    group = 3200;
                    cal.scan_mode = ScanMode::ScanML30SA1_160_1_2;
                }
                else
                {
                    group = 1600;
                    cal.scan_mode = ScanMode::ScanML30SA1_160_1_4;
                }

                if ((lines.size() - curr) < group)//check the file lines
                    return InvalidContent;

                cal.data.resize(group * 8 * 2);
                for (int i = 0; i < group; i++)
                {
                    const int column = 17;

                    std::vector<std::string>& datas = lines[i + curr];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarML30SA1;
                cal.description = "";
                for (int i = 0; i < curr; i++)
                {
                    for (int j = 0; j < lines[i].size(); j++)
                        cal.description += lines[i][j];
                    cal.description += "\n";
                }
            }
            else if ("MLXs_180" == mode_str)
            {
                if ((lines.size() - curr) < 36000)//check the file lines
                    return InvalidContent;

                cal.data.resize(36000 * 3 * 2);
                for (int i = 0; i < 36000; i++)
                {
                    const int column = 7;

                    std::vector<std::string>& datas = lines[i + curr];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarMLX;
                cal.scan_mode = ScanMode::ScanMLXS_180;
                cal.description = "";
                for (int i = 0; i < curr; i++)
                {
                    for (int j = 0; j < lines[i].size(); j++)
                        cal.description += lines[i][j];
                    cal.description += "\n";
                }
            }
			else if ("ML30SPlus_160" == mode_str || "ML30SPlus_160_1_2" == mode_str || "ML30SPlus_160_1_4" == mode_str ) {
				// for ml30sa1Plus
				int cali_len = 6400;
				cal.device_type = DeviceType::LidarMl30SA1Plus;
				cal.scan_mode = ScanMode::ScanML30SA1Plus_160;
				cal.description = "";
				if ("ML30SPlus_160_1_2" == mode_str) {
					cali_len = 3200;
					cal.scan_mode = ScanMode::ScanML30SA1Plus_160_1_2;
				}
				else if ("ML30SPlus_160_1_4" == mode_str) {
					cali_len = 1600;
					cal.scan_mode = ScanMode::ScanML30SA1Plus_160_1_4;
				}

				if ((lines.size() - curr) < cali_len)
					return InvalidContent;

				cal.data.resize(cali_len * 8 * 2);
				const int column = 17;
				const int lpos = cal.data.size() / 2;
				for (int i = 0; i < cali_len; i++)
				{
					std::vector<std::string>& datas = lines[i+curr];
					if (datas.size() != column)
					{
						ret = InvalidContent;
						break;
					}

					// for line with fov 0,1,2,3,4,5,6,7
					for (int j = 1; j < column; j++)
					{
						if (j <= column / 2) // for H view
							cal.data[i * 8 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
						else // for L view
							cal.data[i * 8 + j - 1 - column / 2 + lpos] = static_cast<float>(std::atof(datas[j].c_str()));
					}
				}

				for (int i = 0; i < curr; i++)
				{
					for (int j = 0; j < lines[i].size(); j++)
						cal.description += lines[i][j];
					cal.description += "\n";
				}
			}
            else
            {
                return InvalidContent;
            }
        }
        else
        {
            if (10000 == lines.size())
            {
                cal.data.resize(60000);
                for (int i = 0; i < 10000; i++)
                {
                    const int column = 7;

                    std::vector<std::string>& datas = lines[i];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        int fov = (j - 1) % 3;
                        if (0 == ((j - 1) / 3))//azimuth
                        {
                            cal.data[i * 6 + fov * 2] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                        else//elevation
                        {
                            cal.data[i * 6 + fov * 2 + 1] = static_cast<float>(std::atof(datas[j].c_str()));
                        }
                    }
                }
                cal.device_type = DeviceType::LidarML30B1;
                cal.scan_mode = ScanMode::ScanML30B1_100;
            }
            else if (6400 == lines.size())
            {
                cal.data.resize(6400 * 8 * 2);
                for (int i = 0; i < 6400; i++)
                {
                    const int column = 17;

                    std::vector<std::string>& datas = lines[i];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarML30SA1;
                cal.scan_mode = ScanMode::ScanML30SA1_160;
            }
            else if (7600 == lines.size())
            {
                cal.data.resize(7200 * 8 * 2);
                for (int i = 0; i < 7200; i++)
                {
                    const int column = 17;

                    std::vector<std::string>& datas = lines[i];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 16 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarML30SA1;
                cal.scan_mode = ScanMode::ScanML30SA1_190;
            }
            else if (32000 == lines.size())
            {
                cal.data.resize(32000 * 3 * 2);
                for (int i = 0; i < 32000; i++)
                {
                    const int column = 7;

                    std::vector<std::string>& datas = lines[i];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarMLX;
                cal.scan_mode = ScanMode::ScanMLX_160;
            }
            else if (4800 == lines.size())
            {
                cal.data.resize(4800 * 3 * 2);
                for (int i = 0; i < 4800; i++)
                {
                    const int column = 7;

                    std::vector<std::string>& datas = lines[i];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarMLYB;
                cal.scan_mode = ScanMode::ScanMLYB_190;
            }
            else if (38000 == lines.size())
            {
                cal.data.resize(38000 * 3 * 2);
                for (int i = 0; i < 38000; i++)
                {
                    const int column = 7;

                    std::vector<std::string>& datas = lines[i];
                    if (datas.size() != column)
                    {
                        ret = InvalidContent;
                        break;
                    }
                    for (int j = 1; j < column; j++)
                    {
                        cal.data[i * 6 + j - 1] = static_cast<float>(std::atof(datas[j].c_str()));
                    }
                }
                cal.device_type = DeviceType::LidarMLYA;
                cal.scan_mode = ScanMode::ScanMLYA_190;
            }
            else
            {
                cal.device_type = DeviceType::LidarUnknown;
                cal.scan_mode = ScanMode::ScanUnknown;
                ret = NotSupport;
            }
        }

        return ret;
    }

    int LidarTools::ReadCalibrationData(std::string filename, CalibrationDataSinCosTable& cal_cos_sin_lut)
    {
        CalibrationData cal;
        int ret = 0;
        if (0 != (ret = LidarTools::ReadCalibrationData(filename, cal)))
        {
            return ret;
        }
        else
        {
            LidarTools::ComputeCalibrationSinCos(cal, cal_cos_sin_lut);
            return 0;
        }
    }

    int LidarTools::ExportCalibrationData(CalibrationData& cal, std::string filename)
    {
        if ((ScanMode::ScanML30SA1_160 == cal.scan_mode) || (ScanMode::ScanML30SA1_190 == cal.scan_mode))
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            const int data_in_line = 16;
            if (outfile.is_open())
            {
                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                for (unsigned int i = 0; i < cal.data.size(); i++)
                {
                    if (0 == (i % data_in_line))
                    {
                        if (i > 0)
                            outfile << "\n";
                        outfile << i / data_in_line + 1;
                    }
                    outfile << " " << cal.data[i];
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else if ((ScanMode::ScanML30SA1_160_1_2 == cal.scan_mode) || (ScanMode::ScanML30SA1_160_1_4 == cal.scan_mode))
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            const int data_in_line = 16;
            if (outfile.is_open())
            {
                outfile << "# file version : ML30S.cal_v0.4\n";
                outfile << "VERSION 0.1\n";
                if (ScanMode::ScanML30SA1_160_1_2 == cal.scan_mode)
                    outfile << "Mode ML30S_160_1_2\n";
                else
                    outfile << "Mode ML30S_160_1_4\n";

                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                for (unsigned int i = 0; i < cal.data.size(); i++)
                {
                    if (0 == (i % data_in_line))
                    {
                        if (i > 0)
                            outfile << "\n";
                        outfile << i / data_in_line + 1;
                    }
                    outfile << " " << cal.data[i];
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else if (ScanML30SA1Plus_160 == cal.scan_mode || ScanML30SA1Plus_160_1_2 == cal.scan_mode || ScanML30SA1Plus_160_1_4 == cal.scan_mode) {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            const int data_in_line = 16;
            if (outfile.is_open())
            {
                outfile << "# file version:ML30SPlus.cal_v0.1";
                outfile << "VERSION 0.1\n";
				if (ScanML30SA1Plus_160 == cal.scan_mode)
					outfile << "Mode ML30SPlus_160\n";
				else if(ScanML30SA1Plus_160_1_2 == cal.scan_mode)
					outfile << "Mode ML30SPlus_160_1_2\n";
				else
					outfile << "Mode ML30SPlus_160_1_4\n";

                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                int lpos = cal.data.size() / 2;
                for (unsigned int i = 0; i < cal.data.size(); i++)
                {
                    if (0 == (i % data_in_line))
                    {
                        if (i > 0)
                            outfile << "\n";
                        outfile << i / data_in_line + 1;
                    }
                    int idx = i % 16;
                    int grp = i / 16;
                    if (idx < 8) {
                        outfile << " " << cal.data[idx + grp * 8];
                    }
                    else {
                        outfile << " " << cal.data[(idx - 8) + grp * 8 + lpos];
                    }
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else if (ScanMode::ScanML30B1_100 == cal.scan_mode)
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            if (outfile.is_open())
            {
                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                int rows = 10000;
                for (int i = 0; i < rows; i++)
                {
                    outfile << i + 1 << " ";
                    outfile << cal.data[i * 6 + 0] << " ";
                    outfile << cal.data[i * 6 + 2] << " ";
                    outfile << cal.data[i * 6 + 4] << " ";
                    outfile << cal.data[i * 6 + 1] << " ";
                    outfile << cal.data[i * 6 + 3] << " ";
                    outfile << cal.data[i * 6 + 5];
                    if (i < (rows - 1))
                        outfile << "\n";
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else if (ScanMode::ScanMLX_160 == cal.scan_mode)
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            if (outfile.is_open())
            {
                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                int rows = 32000;
                for (int i = 0; i < rows; i++)
                {
                    outfile << i + 1 << " ";
                    outfile << cal.data[i * 6 + 0] << " ";
                    outfile << cal.data[i * 6 + 1] << " ";
                    outfile << cal.data[i * 6 + 2] << " ";
                    outfile << cal.data[i * 6 + 3] << " ";
                    outfile << cal.data[i * 6 + 4] << " ";
                    outfile << cal.data[i * 6 + 5];
                    if (i < (rows - 1))
                        outfile << "\n";
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else if (ScanMode::ScanMLX_190 == cal.scan_mode)
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            if (outfile.is_open())
            {
                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                outfile << cal.description;
                int rows = 38000;
                for (int i = 0; i < rows; i++)
                {
                    outfile << i + 1 << " ";
                    outfile << cal.data[i * 6 + 0] << " ";
                    outfile << cal.data[i * 6 + 1] << " ";
                    outfile << cal.data[i * 6 + 2] << " ";
                    outfile << cal.data[i * 6 + 3] << " ";
                    outfile << cal.data[i * 6 + 4] << " ";
                    outfile << cal.data[i * 6 + 5];
                    if (i < (rows - 1))
                        outfile << "\n";
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else if (ScanMode::ScanMLXS_180 == cal.scan_mode)
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            if (outfile.is_open())
            {
                outfile << "# file version:MLXs.cal_v0.0\n";
                outfile << "VERSION 0.1\n";
                outfile << "Mode MLXs_180\n";

                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                outfile << cal.description;
                int rows = 36000;
                for (int i = 0; i < rows; i++)
                {
                    outfile << i + 1 << " ";
                    outfile << cal.data[i * 6 + 0] << " ";
                    outfile << cal.data[i * 6 + 1] << " ";
                    outfile << cal.data[i * 6 + 2] << " ";
                    outfile << cal.data[i * 6 + 3] << " ";
                    outfile << cal.data[i * 6 + 4] << " ";
                    outfile << cal.data[i * 6 + 5];
                    if (i < (rows - 1))
                        outfile << "\n";
                }
                outfile.close();
                return 0;
            }
            else
            {
                return OpenFileError;
            }
        }
        else
        {
            return NotSupport;
        }
    }

    void LidarTools::ComputeCalibrationSinCos(CalibrationData& cal, CalibrationDataSinCosTable& cal_cos_sin_lut)
    {
        cal_cos_sin_lut.device_type = cal.device_type;
        cal_cos_sin_lut.scan_mode = cal.scan_mode;
        cal_cos_sin_lut.description = cal.description;
        cal_cos_sin_lut.data.resize(cal.data.size() / 2);

        if (ScanMode::ScanML30B1_100 == cal.scan_mode)
        {
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                float azi = static_cast<float>(cal.data[i * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[i * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
                point_cal.azi = azi;
                point_cal.ele = ele;
            }
        }
        else if ((ScanMode::ScanML30SA1_160 == cal.scan_mode) || (ScanMode::ScanML30SA1_190 == cal.scan_mode) || (ScanMode::ScanML30SA1_160_1_2 == cal.scan_mode) || (ScanMode::ScanML30SA1_160_1_4 == cal.scan_mode))
        {
            const int start = 8;
            int fov_index[start] = { 0, 6, 1, 7, 2, 4, 3, 5 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
                point_cal.azi = azi;
                point_cal.ele = ele;
            }
        }
		else if(ScanMode::ScanML30SA1Plus_160 == cal.scan_mode) {
			const int start = 4;
			int fov_index[start] = { 0, 1, 2, 3 };
			for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
			{
				int start_number = i % start;
				int group_number = i / start;
				int point_numer = group_number * start + fov_index[start_number];
				float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
				float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

				CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
				point_cal.cos_ele = std::cos(ele);
				point_cal.sin_ele = std::sin(ele);
				point_cal.cos_azi = std::cos(azi);
				point_cal.sin_azi = std::sin(azi);
				point_cal.azi = azi;
				point_cal.ele = ele;
			}

		}
        else if ((ScanMode::ScanMLX_160 == cal.scan_mode) || (ScanMode::ScanMLX_190 == cal.scan_mode) || (ScanMode::ScanMLXS_180 == cal.scan_mode))
        {
            const int start = 3;
            int fov_index[start] = { 0, 1, 2 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
                point_cal.azi = azi;
                point_cal.ele = ele;
            }
        }
        else if (ScanMode::ScanMLYA_190 == cal.scan_mode)
        {
            const int start = 3;
            int fov_index[start] = { 2, 1, 0 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
                point_cal.azi = azi;
                point_cal.ele = ele;
            }
        }
        else if (ScanMode::ScanMLYB_190 == cal.scan_mode)
        {
            const int start = 3;
            int fov_index[start] = { 2, 1, 0 };
            for (unsigned int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float azi = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cos(ele);
                point_cal.sin_ele = std::sin(ele);
                point_cal.cos_azi = std::cos(azi);
                point_cal.sin_azi = std::sin(azi);
                point_cal.azi = azi;
                point_cal.ele = ele;
            }
        }
        else
        {

        }
    }

	int LidarTools::QueryDeviceHdcpNetAddr(std::string& addrs) {
		if (!CheckConnection())
			return -1;

		/*Read device mac info*/
		const int send_len = 4;
		char sn_read_cmd[send_len] = { (char)0xBA, (char)0x0B, (char)0x03, (char)0x00 };
		std::string cmd(sn_read_cmd, 4);

		if (client_->SyncSend(cmd, send_len))
		{
			DisConnect();
			return -1;
		}

		const int recv_len = 20;
		std::string recv(recv_len, 'x');
		if (client_->SyncRecv(recv, 4))
		{
			DisConnect();
			return -1;
		}

		// check
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}

		// Assuming read failure while len bigger than 16.
		int len = client_->GetAvailableBytesLen();
		if (len > 16) {
			// clear receive buffer
			std::string temp(len, 'x');
			client_->SyncRecv(temp, len);
			return -1;
		}

		if (client_->SyncRecv(recv, 16))
		{
			DisConnect();
			return -1;
		}

		addrs = recv.substr(0,16);

		return 0;
	}

	int LidarTools::QueryDeviceFactoryMac(std::string& mac) {

		if (!CheckConnection())
			return -1;

		/*Read device mac info*/
		const int send_len = 4;
		char sn_read_cmd[send_len] = { (char)0xBA, (char)0x0B, (char)0x02, (char)0x00 };
		std::string cmd(sn_read_cmd, 4);

		if (client_->SyncSend(cmd, send_len))
		{
			DisConnect();
			return -1;
		}

		const int recv_len = 8;
		std::string recv(recv_len, 'x');
		if (client_->SyncRecv(recv, 4))
		{
			DisConnect();
			return -1;
		}

		// check
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}

		// Assuming read failure while length is not 8.
		int len = client_->GetAvailableBytesLen();
		if (len != 8) {
			// clear receive buffer
			std::string temp(len, 'x');
			client_->SyncRecv(temp, len);
			return -1;
		}

		if (client_->SyncRecv(recv, 8))
		{
			DisConnect();
			return -1;
		}

		std::string flag = recv.substr(0,2);
		if (flag.compare("MA") != 0) {
			return -1;
		}

		std::string addr = recv.substr(2,6);
		if (addr[0]!= (char)0xF8 || addr[1] != (char)0xA9 || addr[2] != (char)0x1F) {
			return -1;
		}
		char cmac[256] = "";
		sprintf_s(cmac, "%02X%02X%02X%02X%02X%02X", (uint8_t)addr[0], (uint8_t)addr[1], (uint8_t)addr[2], (uint8_t)addr[3], (uint8_t)addr[4], (uint8_t)addr[5]);
		mac = std::string(cmac);
		return 0;
	}

    int LidarTools::QueryDeviceSnCode(std::string& sn)
    {
        if (!CheckConnection())
            return -1;

        /*Read version info*/
        const int send_len = 4;
        char sn_read_cmd[send_len] = { (char)0xBA, (char)0x08, (char)0x00, (char)0x00 };
        std::string cmd(sn_read_cmd, 4);

        if (client_->SyncSend(cmd, send_len))
        {
            DisConnect();
            return -1;
        }

		const int recv_len = 21;
		std::string recv(recv_len, 'x');
        if (client_->SyncRecv(recv, 4))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

		if (client_->SyncRecv(recv, 17))
		{
			DisConnect();
			return -1;
		}

		{
			int len = client_->GetAvailableBytesLen();
			if (len > 0 ) {
				std::string recvLeft(len, 'x');
				if (client_->SyncRecv(recvLeft, len))
				{
					DisConnect();
				}
				else {
					sn = recv.substr(0, 17) + recvLeft;
					return 0;
				}
			}
		}

        sn = recv.substr(0, 17);
        return 0;
    }

    int LidarTools::QueryDeviceFirmwareVersion(FirmwareVersion& version)
    {
        if (!CheckConnection())
            return -1;

        /*Read version info*/
        const int send_len = 4;
        char bv_read_cmd[send_len] = { (char)0xBA, (char)0x02, (char)0x01, (char)0x00 };
        std::string bv_cmd(bv_read_cmd, 4);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(bv_cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv version data
        {
            DisConnect();
            return -1;
        }

        memcpy(&(version.boot_version), recv.c_str(), 4);

        char kv_read_cmd[send_len] = { (char)0xBA, (char)0x02, (char)0x02, (char)0x00 };
        std::string kv_cmd(kv_read_cmd, 4);

        if (client_->SyncSend(kv_cmd, send_len))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv version data
        {
            DisConnect();
            return -1;
        }

        memcpy(&(version.kernel_version), recv.c_str(), 4);

        return 0;

    }

    int LidarTools::QueryDeviceTemperature(float& PS, float& PL)
    {
        if (!CheckConnection())
            return -1;

        /*Read temperature info*/
        const int send_len = 4;
        char te_read_cmd[send_len] = { (char)0xAB, (char)0x06, (char)0x00, (char)0x00 };
        std::string cmd(te_read_cmd, 4);

        const int recv_len = 12;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        NetworkToHost((const unsigned char*)recv.c_str() + 4, (char*)&PS);
        NetworkToHost((const unsigned char*)recv.c_str() + 8, (char*)&PL);

        return 0;

    }

    int LidarTools::QueryDeviceConfigurationInfo(DeviceConfigurationInfo& info)
    {
        if (!CheckConnection())
            return -1;

        /*Read configuration info*/
        const int send_len = 4;
        char cfg_read_cmd[send_len] = { (char)0xBA, (char)0x0B, (char)0x00, (char)0x00 };
        std::string cmd(cfg_read_cmd, 4);

        const int recv_len = 110;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, 4))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, 106))//recv configuration data
        {
            DisConnect();
            return -1;
        }

        unsigned char* header = (unsigned char*)recv.c_str();
        memcpy(info.version.boot_version,   header + 0,  4);
        memcpy(info.version.kernel_version, header + 16, 4);
        ResolveIpString(header + 32, info.device_ip);
        ResolvePort(header + 36, info.destination_port);

        unsigned char time_sync = (*(header + 40));
        info.time_sync = TimestampType::TimestampUnknown;
        if ((0xFF == time_sync) || (0x01 == time_sync))
        {
            info.time_sync = TimestampType::TimestampPtp;
        }
        else if (0x02 == time_sync)
        {
            info.time_sync = TimestampType::TimestampPpsGps;
        }

        ResolveIpString(header + 41, info.destination_ip);

        unsigned char retro = (*(header + 45));
        info.retro_enable = RetroMode::RetroUnknown;
        if ((0xFF == retro) || (0x00 == retro))
        {
            info.retro_enable = RetroMode::RetroDisable;
        }
        else if (0x01 == retro)
        {
            info.retro_enable = RetroMode::RetroEnable;
        }

        ResolveIpString(header + 46, info.subnet_mask);
        ResolveMacAddress(header + 100, info.device_mac);

		// config mac
		ResolveMacAddress(header + 50, info.config_mac);

        //phase offset
        NetworkToHost(header + 56, (char*)&info.phase_offset);

        //echo mode
        unsigned char echo_mode = (*(header + 60));
        info.echo_mode = EchoMode::EchoUnknown;
        if (0x01 == echo_mode)
            info.echo_mode = EchoSingleFirst;
        else if (0x02 == echo_mode)
            info.echo_mode = EchoSingleStrongest;
        else if (0x04 == echo_mode)
            info.echo_mode = EchoSingleLast;
        else if (0x03 == echo_mode)
            info.echo_mode = EchoDoubleFirstStrongest;
        else if (0x05 == echo_mode)
            info.echo_mode = EchoDoubleFirstLast;
        else if (0x06 == echo_mode)
            info.echo_mode = EchoDoubleStrongestLast;

        // phase offset enable
        unsigned char phase_offset_enable = (*(header + 61));
        if (0x00 == phase_offset_enable)
            info.phase_offset_mode = PhaseOffsetDisable;
        else if(0x01 == phase_offset_enable)
            info.phase_offset_mode = PhaseOffsetEnable;
        else
            info.phase_offset_mode = PhaseOffsetUnknown;

        // retro param
        unsigned char retro_param_1 = (*(header + 62));
        unsigned char retro_param_2 = (*(header + 63));
        info.retro_param_1_ref_min = retro_param_1;
        info.retro_param_2_point_percent = retro_param_2;

        // calibration data send status, 0 disable, 1 enable
        unsigned char cal_send_enable = (*(header + 64));
        if (0x00 == cal_send_enable)
            info.cal_send_mode = CalSendDisable;
        else if (0x01 == cal_send_enable)
            info.cal_send_mode = CalSendEnable;
        else
            info.cal_send_mode = CalSendUnknown;

        // downsample mode
        unsigned char downsample_flag = (*(header + 65));
        if (0x00 == downsample_flag)
            info.downsample_mode = DownsampleNone;
        else if (0x01 == downsample_flag)
            info.downsample_mode = Downsample_1_2;
        else if(0x02 == downsample_flag)
            info.downsample_mode = Downsample_1_4;
        else
            info.downsample_mode = DownsampleUnknown;

		// dhcp enable
		unsigned char value = (*(header + 86));
		if (value == 0x00)
			info.dhcp_enable = StateMode::StateDisable;
		else if (value == 0x01)
			info.dhcp_enable = StateMode::StateEnable;
		else
			info.dhcp_enable = StateMode::StateUnknown;

		// lidar gateway addr
		ResolveIpString(header + 87, info.gateway_addr);

		// delete point switch
		value = (*(header + 91));
		if (value == 0x00)
			info.delete_point_enable = StateMode::StateDisable;
		else if (value == 0x01)
			info.delete_point_enable = StateMode::StateEnable;
		else
			info.delete_point_enable = StateMode::StateUnknown;

		// adhesion switch
		value = (*(header + 92));
		if (value == 0x00)
			info.adhesion_enable = StateMode::StateDisable;
		else if (value == 0x01)
			info.adhesion_enable = StateMode::StateEnable;
		else
			info.adhesion_enable = StateMode::StateUnknown;

		// retro gray low/ threshold
		info.retro_gray_low_threshold = (*(header + 93));
		info.retro_gray_high_threshold = (*(header + 94));

		// clear recv buffer
		int len = client_->GetAvailableBytesLen();
		if (len > 0) {
			std::string temp(len, 'x');
			client_->SyncRecv(temp, len);
		}

        // sn code and device type
        std::string sn = "Unknown";
        info.serial_number = "Unknown";
        info.device = DeviceType::LidarUnknown;
        if (QueryDeviceSnCode(sn))
        {
            info.serial_number = "Unknown";
            info.device = DeviceType::LidarUnknown;
        }
        else
        {
            std::string ml30b1_sn_prefix = "1000";
            std::string ml30sa1_sn_prefix = "1001";
            std::string mlx_sn_prefix = "1002";
            const int hrd_version_pos = 5;
            const int hrd_version_len = 1;

            if (0 == sn.compare(0, ml30b1_sn_prefix.size(), ml30b1_sn_prefix)) // ML30B1
            {
                info.device = DeviceType::LidarML30B1;
            }
            else if (0 == sn.compare(0, ml30sa1_sn_prefix.size(), ml30sa1_sn_prefix)) // ML30SA1
            {
                if (0 == sn.compare(hrd_version_pos, hrd_version_len, "1"))
                    info.device = DeviceType::LidarML30SA1;
                else if (0 == sn.compare(hrd_version_pos, hrd_version_len, "2"))
                    info.device = DeviceType::LidarML30SB1;
                else if (0 == sn.compare(hrd_version_pos, hrd_version_len, "3"))
                    info.device = DeviceType::LidarML30SB2;
                else
                    info.device = DeviceType::LidarUnknown;
            }
            else if (0 == sn.compare(0, mlx_sn_prefix.size(), mlx_sn_prefix)) // MLX
            {
                info.device = DeviceType::LidarMLX;
			}

            info.serial_number = sn;
        }

        // backup firmware version
        memset(info.backup_version.boot_version, 0, 4);
        memset(info.backup_version.kernel_version, 0, 4);
        if (0 != QueryDeviceBackupFirmwareVersion(info.backup_version))
        {
            LOG_ERROR("Query backup firmware version error.\n");
        }

		// get device factory mac
		std::string fcmac = "Unknown";
		if (QueryDeviceFactoryMac(fcmac)) {
			info.factory_mac = "Unknown";
		}
		else {
			info.factory_mac = fcmac;
		}

		// update for dhcp [ip|mask|gateway|dstIp] ,trick ->
		std::string dhcpNetAddr;
		if (QueryDeviceHdcpNetAddr(dhcpNetAddr)) {
			info.dhcp_enable = StateMode::StateUnknown;
		}
		else {
			// update
			unsigned char* pdata = (unsigned char*)dhcpNetAddr.c_str();
			ResolveIpString(pdata, info.device_ip);
			ResolveIpString(pdata + 4, info.subnet_mask);
			ResolveIpString(pdata + 8, info.gateway_addr);
			ResolveIpString(pdata + 12, info.destination_ip);
		}

		// get device algo param
		QueryDeviceAlgoParam(info.algo_param);
		return 0;
    }

	int LidarTools::QueryDeviceAlgoParam(DeviceAlgoParam& param) {
		if (!CheckConnection())
			return -1;

		/*Read configuration info*/
		const int send_len = 4;
		char cfg_read_cmd[send_len] = { (char)0xBA, (char)0x0B, (char)0x01, (char)0x00 };
		std::string cmd(cfg_read_cmd, 4);

		const int recv_len = 104;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, 4))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		// check whether algo param is valid
		int len = client_->GetAvailableBytesLen();
		if (len != 100) {
			std::string temp(len, 'x');
			client_->SyncRecv(temp, len);
			return -1;
		}

		if (client_->SyncRecv(recv, 100))//recv configuration data
		{
			DisConnect();
			return -1;
		}

		unsigned char* header = (unsigned char*)recv.c_str();
		param.isValid = true;

		NetworkToHost(header +0 , (char*)&param.retro_dis_thres);
		NetworkToHostShort(header + 4, (char*)&param.retro_low_range_thres);
		NetworkToHostShort(header + 6, (char*)&param.retro_high_range_thres);

		NetworkToHost(header + 8, (char*)&param.adhesion_angle_hor_min);
		NetworkToHost(header + 12, (char*)&param.adhesion_angle_hor_max);
		NetworkToHost(header + 16, (char*)&param.adhesion_angle_ver_min);
		NetworkToHost(header + 20, (char*)&param.adhesion_angle_ver_max);
		NetworkToHost(header + 24, (char*)&param.adhesion_angle_hor_res);
		NetworkToHost(header + 28, (char*)&param.adhesion_angle_ver_res);
		NetworkToHost(header + 32, (char*)&param.adhesion_diff_thres);
		NetworkToHost(header + 36, (char*)&param.adhesion_dis_limit);

		param.retro_min_gray_num = header[40];
		param.retro_del_gray_thres = header[41];
		param.retro_del_ratio_gray_low_thres = header[42];
		param.retro_del_ratio_gray_high_thres = header[43];
		param.retro_min_gray = header[44];

		NetworkToHost(header + 46, (char*)&param.adhesion_min_diff);

		return 0;
	}

    int LidarTools::GetDeviceCalibrationData(CalibrationData& cal)
    {
        int ret = 0;
        CalibrationPackets pkts;
	ret = LidarTools::GetDeviceCalibrationPackets(pkts);
        if (ret)
            return ret;
        ret = LidarTools::GetDeviceCalibrationData(pkts, cal);
        if (ret)
            return ret;
        
        return 0;
    }

    int LidarTools::GetDeviceCalibrationPackets(CalibrationPackets& pkts, std::string cmd)
    {
        const int ppf = 256000; // points per frame, 256000 reserved
        const int ppk = 128; // points per cal udp packet
        std::unique_ptr<float> angle_data(new float[ppf * 2]); // points( azimuh, elevation);
        int packet_buffer_size = 1040 * (ppf / ppk) + 4; // 128 points in one packet, buffer reserved for ppf points.
        std::unique_ptr<unsigned char> packet_data(new unsigned char[packet_buffer_size]);
        const int send_len = 4;
        char cal_cmd[send_len] = { (char)0xBA, (char)0x07, (char)0x00, (char)0x00 };
        std::string str_cmd(cal_cmd, send_len);

		// use user input tcp cmd
		if (cmd.size() == 4)
			str_cmd = cmd;

        pkts.clear();

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (!CheckConnection())
            return TcpConnTimeout;

        if (client_->SyncSend(str_cmd, send_len))
        {
            DisConnect();
            return TcpSendTimeout;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return TcpRecvTimeout;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return DevAckError;
        }

        const int cal_pkt_len = 1040;
        std::string cal_recv(cal_pkt_len, 'x');
        //CalibrationPacket* pkt = reinterpret_cast<CalibrationPacket*>((char *)cal_recv.c_str());

        //receive first packet to identify the device type
        if (client_->SyncRecv(cal_recv, cal_pkt_len))
        {
            DisConnect();
            return TcpRecvTimeout;
        }

        int total_packet = 0;
        ScanMode sm = CalibrationPacket::GetScanMode(cal_recv);

        if (ScanMode::ScanML30B1_100 == sm)
        {
            total_packet = 235;// 10000 * 3 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanML30SA1_160 == sm || ScanMode::ScanML30SA1Plus_160 == sm)
        {
            total_packet = 400;// 6400 * 8 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanML30SA1_160_1_2 == sm || ScanMode::ScanML30SA1Plus_160_1_2 == sm)
        {
            total_packet = 200;// 6400 * 8 * 2 * 4 / 1024 / 2
        }
        else if (ScanMode::ScanML30SA1_160_1_4 == sm || ScanMode::ScanML30SA1Plus_160_1_4 == sm)
        {
            total_packet = 100;// 6400 * 8 * 2 * 4 / 1024 / 4
        }
        else if (ScanMode::ScanML30SA1_190 == sm)
        {
            total_packet = 450;// 7200 * 8 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanMLX_160 == sm)
        {
            total_packet = 750;// 32000 *3 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanMLXS_180 == sm)
        {
            total_packet = 844;// 36000 *3 * 2 * 4 / 1024
        }
        else
        {
            DisConnect();
            return -1;// not support
        }

        pkts.push_back(cal_recv);
        for (int i = 0; i < total_packet - 1; i++)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(110));
            int ret = client_->SyncRecv(cal_recv, cal_pkt_len);
            if (ret)
            {
                LOG_ERROR("Receive calibration data error, ret = %d.\n", ret);
                DisConnect();
                return TcpRecvTimeout;
            }
            pkts.push_back(cal_recv);
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return TcpRecvTimeout;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return DevAckError;
        }

        if (total_packet != pkts.size())
            return NotEnoughData;

        return 0;
    }

    int LidarTools::GetDeviceCalibrationData(const CalibrationPackets& pkts, CalibrationData& cal)
    {
        if (!pkts.size())
            return InvalidContent;

        CalibrationPackets& packets = const_cast<CalibrationPackets&>(pkts);
        ScanMode sm = CalibrationPacket::GetScanMode(packets[0]);

        // v0.0.7 compatible
        int data_offset = 16;
        if (ScanMode::ScanUnknown == sm)
        {
            data_offset = 5;
            sm = ScanMode::ScanML30SA1_160;
        }

        for (int i = 0; i < pkts.size(); i++)
        {
            char* cal_data_ = (char*)packets[i].c_str();
            int network_data = 0;
            int host_data = 0;
            float* pfloat_data = reinterpret_cast<float *>(&host_data);
            for (int j = 0; j < 128 * 2; j++)
            {
                memcpy(&network_data, cal_data_ + j * 4 + data_offset, 4); // 4 bytes per data, azimuth, elevation, 16 bytes header
                host_data = ntohl(network_data);
                cal.data.push_back(*pfloat_data);
            }
        }

        if (ScanMode::ScanML30B1_100 == sm)
        {
            cal.data.resize(10000 * 3 * 2); //10000 for per sub fov, 3 fovs, 2(azimuth and elevation)
        }
        else if (ScanMode::ScanML30SA1_160 == sm || ScanML30SA1Plus_160 == sm)
        {
            cal.data.resize(6400 * 8 * 2);
        }
        else if (ScanMode::ScanML30SA1_160_1_2 == sm || ScanML30SA1Plus_160_1_2 == sm)
        {
            cal.data.resize(6400 * 8 * 2 / 2);
        }
        else if (ScanMode::ScanML30SA1_160_1_4 == sm || ScanML30SA1Plus_160_1_4 == sm)
        {
            cal.data.resize(6400 * 8 * 2 / 4);
        }
        else if (ScanMode::ScanML30SA1_190 == sm)
        {
            cal.data.resize(6400 * 8 * 2);
        }
        else if (ScanMode::ScanMLX_160 == sm)
        {
            cal.data.resize(32000 * 3 * 2);
        }
        else if (ScanMode::ScanMLXS_180 == sm)
        {
            cal.data.resize(36000 * 3 * 2);
        }
        else
        {

        }

		// now we reorder the cali data   0~15... -> 0~7... + 8~15...
		if (ScanML30SA1Plus_160 == sm || ScanML30SA1Plus_160_1_2 == sm || ScanML30SA1Plus_160_1_4 == sm) {
			std::vector<float> dst;
			dst.resize(cal.data.size(),.0f);
			int id_h = 0;
			int id_l = cal.data.size() / 2;
			for (int i = 0; i < cal.data.size(); i++) {
				if (i / 8 % 2 == 0) {
					dst.at(id_h) = cal.data[i];
					id_h++;
				}
				else {
					dst.at(id_l) = cal.data[i];
					id_l++;
				}
			}
			cal.data = dst;
		}


        cal.scan_mode = sm;
        return 0;
    }

    int LidarTools::GetDeviceCalibrationDataToFile(std::string filename)
    {
        CalibrationData cal;
        int ret = GetDeviceCalibrationData(cal);

        if (ret)
            return ret;
        else
            return LidarTools::ExportCalibrationData(cal, filename);

    }

	int LidarTools::SetDeviceCalibrationData(std::string filename) {

		// 1. read cali data from file
		CalibrationData cal;
		int ret = 0;
		if (0 != (ret = LidarTools::ReadCalibrationData(filename, cal)))
			return ret;

		// only support for ml30sa1
		if (cal.scan_mode != ScanML30SA1_160 && cal.scan_mode != ScanML30SA1Plus_160)
			return -1;

		const int cali_pkt_len = 1024;
		size_t cali_data_len = cal.data.size();
		// 2. generate calibration buffer
		std::vector<std::string> cali_pkts;
		std::string pkt(cali_pkt_len, '0');
		for (int i = 0; i < cali_data_len; i++) {
			// float to str
			char* pdata = (char*)(&cal.data[i]);
			int* paddr = (int*)pdata;
			*paddr = ntohl(*paddr);
			// update packet
			for (int j = 0; j < 4; j++)
				pkt.at(i * 4 % cali_pkt_len + j) = *(pdata + j);

			// generate packet
			if ((i + 1) % 256 == 0) {
				cali_pkts.push_back(pkt);
				pkt = std::string(cali_pkt_len, '0');
			}
		}

        if (!CheckConnection())
            return TcpConnTimeout;

		// 3. send cmd
		const int cmd_len = 4;
		char cal_cmd[cmd_len] = { (char)0xAB, (char)0x04, (char)0x00, (char)0x00 };
		std::string cmd(cal_cmd, cmd_len);
		if (client_->SyncSend(cmd, cmd_len))
		{
			DisConnect();
			return -1;
		}
		// recv ret
		std::string recv(cmd_len, 'x');
		if (client_->SyncRecv(recv, cmd_len))
		{
			DisConnect();
			return -1;
		}
		// check ret
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}

		// 4. send cali data to lidar
		for (int i = 0; i < cali_pkts.size();i++) {
			ret = client_->SyncSend(cali_pkts[i], cali_pkts[i].size());
			if (ret)
			{
				LOG_ERROR("Send calibration data error, ret = %d.\n", ret);
				DisConnect();
				return TcpSendTimeout;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(110));
		}

		// 5. check
		recv = std::string(cmd_len, 'x');
		if (client_->SyncRecv(recv, cmd_len))
		{
			DisConnect();
			return -1;
		}
		// check ret
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

    int LidarTools::SetDeviceStaticIpAddress(std::string ip)
    {
        if (!CheckConnection())
            return -1;

        /*Set device static ip address*/
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xBA, (char)0x03, (char)0x00, (char)0x00 , (char)0x00, (char)0x00};

        if (!AssembleIpString(ip, set_cmd + 2))
            return InvalidParameter;

        std::string bv_cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(bv_cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

	int LidarTools::ResetDeviceMacAddress() {
		if (!CheckConnection())
			return -1;

		/*Reset device mac address */
		const int send_len = 9;
		char set_cmd[send_len] = { (char)0xBA, (char)0x0A, (char)0xFF, (char)0xFF , (char)0xFF, (char)0xFF , (char)0xFF, (char)0xFF };

		std::string bv_cmd(set_cmd, send_len);

		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(bv_cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

    int LidarTools::SetDeviceSubnetMask(std::string mask)
    {
        if (!CheckConnection())
            return -1;

        /*Set device subnet mask, default 255.255.255.0*/
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xBA, (char)0x09, (char)0xFF, (char)0xFF , (char)0xFF, (char)0x00 };

        if (!AssembleIpString(mask, set_cmd + 2))
            return InvalidParameter;

        std::string bv_cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(bv_cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDeviceMacAddress(std::string mac)
    {
        if (!CheckConnection())
            return -1;

        /*Set device subnet mask, default FF:FF:FF:FF:FF:FF*/
        const int send_len = 8;
        char set_cmd[send_len] = { (char)0xBA, (char)0x0A, (char)0xFF, (char)0xFF , (char)0xFF, (char)0xFF , (char)0xFF, (char)0xFF };

        if (!AssembleMacAddress(mac, set_cmd + 2))
            return InvalidParameter;

        std::string bv_cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(bv_cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDeviceUdpDestinationIpAddress(std::string ip)
    {
        if (!CheckConnection())
            return -1;

        /*Set device udp destination ip address*/
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xBA, (char)0x06, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };


        if (!AssembleIpString(ip, set_cmd + 2))
            return InvalidParameter;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDeviceUdpDestinationPort(int port)
    {
        if (!CheckConnection())
            return -1;

        /*Set device udp destination port*/
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xBA, (char)0x04, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };

        AssemblePort(port, set_cmd + 2);

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;

    }

    int LidarTools::SetDeviceTimestampType(TimestampType tp)
    {
        if (!CheckConnection())
            return -1;

        /*Set device timestamp type*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x05, (char)0x00, (char)0x00};

        if (TimestampPtp == tp)
            set_cmd[2] = 0x01;
        else if (TimestampPpsGps == tp)
            set_cmd[2] = 0x02;
        else
            return -1;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;

    }

	int LidarTools::SetDeviceAlgorithmEnable(AlgoType tp, bool en) {

		if (!CheckConnection())
			return -1;

		/*Set device Algorithm enable*/
		const int send_len = 4;
		char set_cmd[send_len] = { (char)0xBA, (char)0x1F, (char)0x00, (char)0x00 };

		if (AlgoDeleteClosePoints == tp)
			set_cmd[2] = 0x01;
		else if (AlgoAdhesion == tp)
			set_cmd[2] = 0x02;
		else if (AlgoRetro == tp)
			set_cmd[2] = 0x03;
		else
			return -1;

		if (en)
			set_cmd[3] = 0x01;
		else
			set_cmd[3] = 0x00;

		std::string cmd(set_cmd, send_len);

		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}


    int LidarTools::SetDeviceRetroEnable(bool en)
    {
        if (!CheckConnection())
            return -1;

        /*Set device timestamp type*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xAB, (char)0x03, (char)0x00, (char)0x00 };

        if (en)
            set_cmd[2] = 0x01;
        else
            set_cmd[2] = 0x00;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;

    }

    int LidarTools::SetDeviceRetroParam1MinRef(int ref_min)
    {
        if ((ref_min < 0) || (ref_min > 250))
            return InvalidParameter;

        if (!CheckConnection())
            return -1;

        /*Set device retro param 1*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x15, (char)0x01, (char)0x00 };

        set_cmd[3] = ref_min & 0xFF;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDeviceRetroParam2PointPercentage(int percentage)
    {
        if ((percentage < 0) || (percentage > 100))
            return InvalidParameter;

        if (!CheckConnection())
            return -1;

        /*Set device retro param 2*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x15, (char)0x02, (char)0x00 };

        set_cmd[3] = percentage & 0xFF;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

	int LidarTools::SetDeviceAdhesionParam(AdhesionParam tp, int val) {

		if (!CheckConnection())
			return -1;

		const int send_len = 7;
		char set_cmd[send_len] = { (char)0xBA, (char)0x20, (char)0x00,(char)0x00, (char)0x00, (char)0x00, (char)0x00 };

		// check param
		if (MinimumHorizontalAngleRange == tp)
			set_cmd[2] = 0x01;
		else if (MaximumHorizontalAngleRange == tp)
			set_cmd[2] = 0x02;
		else if (MinimumVerticalAngleRange == tp)
			set_cmd[2] = 0x03;
		else if (MaximumVerticalAngleRange == tp)
			set_cmd[2] = 0x04;
		else
			return -1;

		HostToNetwork((const unsigned char*)&val, set_cmd + 3);

		std::string cmd(set_cmd, send_len);
		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

	int LidarTools::SetDeviceAdhesionParam(AdhesionParam tp, float val) {

		if (!CheckConnection())
			return -1;

		const int send_len = 7;
		char set_cmd[send_len] = { (char)0xBA, (char)0x20, (char)0x00,(char)0x00, (char)0x00, (char)0x00, (char)0x00 };

		// check param
		if (HorizontalAngleResolution == tp)
			set_cmd[2] = 0x05;
		else if (VerticalAngleResolution == tp)
			set_cmd[2] = 0x06;
		else if (DeletePointThreshold == tp)
			set_cmd[2] = 0x07;
		else if (MaximumProcessingRange == tp)
			set_cmd[2] = 0x08;
		else if(NearFarPointDiff == tp)
			set_cmd[2] = 0x09;
		else
			return -1;

		HostToNetwork((const unsigned char*)&val, set_cmd + 3);

		std::string cmd(set_cmd, send_len);
		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

	int LidarTools::SetDeviceRetroParam(RetroParam tp, int val) {

		if (!CheckConnection())
			return -1;

		if (tp != RetroParam::RetroDisThres)
			return -1;

		const int send_len = 7;
		char set_cmd[send_len] = { (char)0xBA, (char)0x15, (char)0x02,(char)0x00, (char)0x00, (char)0x00, (char)0x00 };

		HostToNetwork((const unsigned char*)&val, set_cmd + 3);
		std::string cmd(set_cmd, send_len);

		// send
		if (client_->SyncSend(cmd, send_len))
		{
			DisConnect();
			return -1;
		}

		// recv
		const int recv_len = 4;
		std::string recv(recv_len, 'x');
		if (client_->SyncRecv(recv, recv_len))
		{
			DisConnect();
			return -1;
		}

		// check
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

	int LidarTools::SetDeviceRetroParam(RetroParam tp, unsigned short val) {

		if (!CheckConnection())
			return -1;

		const int send_len = 5;
		char set_cmd[send_len] = { (char)0xBA, (char)0x15, (char)0x00,(char)0x00, (char)0x00 };

		if (tp == RetroParam::RetroLowRangeThres)
			set_cmd[2] = 0x03;
		else if (tp == RetroParam::RetroHighRangeThres)
			set_cmd[2] = 0x04;
		else
			return -1;

		set_cmd[3] = val >> 8 & 0xFF;
		set_cmd[4] = val & 0xFF;

		std::string cmd(set_cmd, send_len);

		// send
		if (client_->SyncSend(cmd, send_len))
		{
			DisConnect();
			return -1;
		}

		// recv
		const int recv_len = 4;
		std::string recv(recv_len, 'x');
		if (client_->SyncRecv(recv, recv_len))
		{
			DisConnect();
			return -1;
		}

		// check
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}
		return 0;
	}

	int LidarTools::SetDeviceRetroParam(RetroParam tp, unsigned char val) {

		if (!CheckConnection())
			return -1;

		const int send_len = 4;
		char set_cmd[send_len] = { (char)0xBA, (char)0x15, (char)0x00,(char)0x00 };

		if (tp == RetroParam::RetroMinGrayNum)
			set_cmd[2] = 0x01;
		else if (tp == RetroParam::RetroDelGrayThres)
			set_cmd[2] = 0x05;
		else if (tp == RetroParam::RetroDelRatioGrayLowThres)
			set_cmd[2] = 0x06;
		else if (tp == RetroParam::RetroDelRatioGrayHighThres)
			set_cmd[2] = 0x07;
		else if (tp == RetroParam::RetroMinGray)
			set_cmd[2] = 0x08;
		else
			return -1;

		set_cmd[3] = val;
		std::string cmd(set_cmd, send_len);

		// send
		if (client_->SyncSend(cmd, send_len))
		{
			DisConnect();
			return -1;
		}

		// recv
		const int recv_len = 4;
		std::string recv(recv_len, 'x');
		if (client_->SyncRecv(recv, recv_len))
		{
			DisConnect();
			return -1;
		}

		// check
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

    int LidarTools::FirmwareUpdate(std::string& filename, ProgressCallback cb)
    {
        if (!CheckConnection())
            return -1;

        std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
        if (!in.is_open())
        {
            LOG_ERROR("Open file error.\n");
            return -1;
        }

        std::streampos end = in.tellg();
        int size = static_cast<int>(end);
        in.close();

        const int send_len = 6;
        char upgrade_cmd[send_len] = { (char)0xBA, (char)0x01, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };
        HostToNetwork((const unsigned char *)&size, upgrade_cmd + 2);
        std::string cmd(upgrade_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');


        if (client_->SyncSend(cmd, send_len))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        // transfer, erase flash, write
        const int pkt_len = 256;
        int start_percent = 10;
        int block_total = size / pkt_len;
        if (0 != (size % pkt_len))
            block_total += 1;
        int step_per_percent = block_total / 30;//old is 90

        std::ifstream idata(filename, std::ios::in | std::ios::binary);
        char fw_data[pkt_len] = { 0x00 };
        int readed = 0;

        // data transfer
        if (idata.is_open())
        {
            for (readed = 0; readed < block_total; readed++)
            {
                int read_len = pkt_len;
                if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
                    read_len = size % pkt_len;
                idata.read(fw_data, read_len);
                std::string fw(fw_data, read_len);
                if (client_->SyncSend(fw, read_len))
                {
                    break;
                }
                if (0 == (readed % step_per_percent))
                {
                    cb(start_percent++, this->device_ip_.c_str());
                }
            }
            idata.close();
        }
        if (readed != block_total)
        {
            client_->Close();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Data transfer ok...\n");

        // waitting for step 2
        const int recv_step_len = 5;
        std::string recv_step(recv_step_len, 'x');
        bool ok = false;
        while (1)
        {
            if (client_->SyncRecv(recv_step, recv_step_len))
            {
                ok = false;
                break;
            }
            unsigned char step = (unsigned char)recv_step[4];
            cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 == step)
            {
                ok = true;
                break;
            }
        }

        if(!ok)
        {
            LOG_ERROR("Waiting for erase flash failed...\n");
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Waiting for erase flash ok...\n");

        // waitting for step 3
        while (1)
        {
            if (client_->SyncRecv(recv_step, recv_step_len))
            {
                ok = false;
                break;
            }
            unsigned char step = (unsigned char)recv_step[4];
            cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 == step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            LOG_ERROR("Waiting for write flash failed...\n");
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Waiting for write flash ok...\n");

        //device check data
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        return 0;

    }

    int LidarTools::RebootDevice()
    {
        if (!CheckConnection())
            return -1;

        /*Set device timestamp type*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x0C, (char)0x00, (char)0x00 };

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::GetDeviceLog(std::string& log)
    {
        if (!CheckConnection())
            return -1;

        /*Get device log info*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x0D, (char)0x00, (char)0x00 };

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        // get log buffer length
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }
        uint32_t log_buffer_len = 0;
        NetworkToHost((const unsigned char*)recv.c_str(), (char *)&log_buffer_len);

        // get log buffer
        std::string log_buffer;
        log_buffer.resize(log_buffer_len);
        if (client_->SyncRecv(log_buffer, log_buffer_len))
        {
            DisConnect();
            return -1;
        }

        // assemble the buffer to string
        unsigned char file_count = reinterpret_cast<unsigned char&>(log_buffer[0]);

        std::vector<std::string> file_contents;
        uint32_t position = 1;
        for (int i = 0; i < file_count; ++i)
        {
            uint32_t file_len = 0;
            NetworkToHost((const unsigned char*)(&log_buffer[position]), (char *)&file_len);
            LOG_INFO("FILE %d len %u.\n", i, file_len);
            std::string content(log_buffer, position + 4, file_len); // copy to file list
            file_contents.push_back(content);
            position += (file_len + 4);
        }
        
        // reorder the log file
        log = "";
		for (int i = file_count - 1; i >= 0; i--)
		{
			log += file_contents[i];
		}

        //final ret
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDevicePhaseOffset(uint32_t offset)
    {
        if (!CheckConnection())
            return -1;

        /*Phase offset*/
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xBA, (char)0x0E, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };

        HostToNetwork((const unsigned char*)&offset, set_cmd + 2);

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDevicePhaseOffsetEnable(bool en)
    {
        if (!CheckConnection())
            return -1;

        /*Set device phase offset enable*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x14, (char)0x00, (char)0x00};
        if (en)
            set_cmd[2] = 0x01;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDevicePtpConfiguration(std::string ptp_cfg_filename)
    {
        //// --- read ptp configuration file content
		// read file
		std::string data;
		char c;
		std::ifstream inFile(ptp_cfg_filename, std::ios::in | std::ios::binary);
		if (!inFile)
			return ReturnCode::OpenFileError;

		while ((c = inFile.get()) && c != EOF)
			data.push_back(c);
		inFile.close();

		int length = data.size();
		std::string content = data;

        if (!CheckConnection())
            return -1;

        // --- send data to device
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xBA, (char)0x0F, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };

        uint32_t file_len = length;
        HostToNetwork((const unsigned char*)&file_len, set_cmd + 2);

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        // send file to device
        if (client_->SyncSend(content, file_len))
        {
            DisConnect();
            return -1;
        }

        // final ack
        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return ReturnCode::Success;

    }

	int LidarTools::SetDeviceConfigFile(std::string cfg_filename) {

		// read file data
		std::string data;
		std::ifstream inFile(cfg_filename, std::ios::in | std::ios::binary);
		if (!inFile)
			return ReturnCode::OpenFileError;

		std::stringstream ss;
		ss << inFile.rdbuf();
		data = ss.str();
		inFile.close();

		int length = data.size();
		std::string content = data;
		if (!CheckConnection())
			return -1;

		// --- send data to device
		const int send_len = 7;
		char set_cmd[send_len] = { (char)0xBA, (char)0x21, (char)0x01, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };

		uint32_t file_len = length;
		HostToNetwork((const unsigned char*)&file_len, set_cmd + 3);

		std::string cmd(set_cmd, send_len);

		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		// send file to device
		if (client_->SyncSend(content, file_len))
		{
			DisConnect();
			return -1;
		}

		// final ack
		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return ReturnCode::Success;
		return 0;
	}

    int LidarTools::QueryDeviceConfigFileVersion(std::string& ver) {
        if (!CheckConnection())
            return -1;

        /*Read version info*/
        const int send_len = 4;
        char bv_read_cmd[send_len] = { (char)0xBA, (char)0x02, (char)0x03, (char)0x00 };
        std::string bv_cmd(bv_read_cmd, 4);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(bv_cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        const int recv_ver_len = 3;
        std::string recv_ver(recv_ver_len, 'x');
        if (client_->SyncRecv(recv_ver, recv_ver_len))//recv version data
        {
            DisConnect();
            return -1;
        }

        char cver[128] = "";
        const uint8_t* pver = (uint8_t*)recv_ver.data();
        sprintf_s(cver, "%u.%u.%u",*(pver + 0) , *(pver + 1), *(pver + 2));
        ver = std::string(cver);

        return 0;
    }

    int LidarTools::GetDevicePtpConfiguration(std::string& ptp_cfg)
    {
        if (!CheckConnection())
            return -1;

        /*Get device log info*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x10, (char)0x00, (char)0x00 };

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        // get ptp configuration data
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }
        uint32_t ptp_buffer_len = 0;
        NetworkToHost((const unsigned char*)recv.c_str(), (char *)&ptp_buffer_len);

        // get ptp configuration buffer
        std::string ptp_cfg_buffer(ptp_buffer_len, 'x');
        if (client_->SyncRecv(ptp_cfg_buffer, ptp_buffer_len))
        {
            DisConnect();
            return -1;
        }

        ptp_cfg = ptp_cfg_buffer;

        return 0;

    }

    int LidarTools::GetDevicePtpConfigurationToFile(std::string& save_file_name)
    {
        std::string content = "";
        
        int ret = GetDevicePtpConfiguration(content);
        if (ret)
            return ret;
        else
        {
            std::ofstream out(save_file_name, std::ios::out | std::ios::binary);
            if (out.is_open())
            {
                out.write(content.c_str(), content.size());
                out.close();
                return 0;
            }
            else
                return OpenFileError;
        }
    }

    int LidarTools::SetDevicePointFireEnConfiguration(std::string fire_en_filename,DeviceType devType)
    {
        // --- read point fire enbale configuration file content
        std::ifstream infile(fire_en_filename);
        if (!infile.is_open())
        {
            return ReturnCode::OpenFileError;
        }

        std::string line;
        std::vector<int> ids;
        const int column = 1;
        while (std::getline(infile, line))
        {
            int value = 0xFFFFFFFF;
            if (line.size() > 0)
            {
                int match = sscanf_s(line.c_str(), "%x", (unsigned int*)&value);

                //printf("%08X\n", value);
                if (column != match)
                    break;
                ids.push_back(value);
            }
        }
        infile.close();

        if (ids.size() != 1600)
            return InvalidContent;

        if (!CheckConnection())
            return -1;

        // --- send data to device
        const int send_len = 6;
        char set_cmd[send_len] = { (char)0xAB, (char)0x01, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };

        uint32_t size_len = 2;
        uint32_t data_len = 1600 * 4 + size_len;
        HostToNetwork((const unsigned char*)&data_len, set_cmd + 2);
        std::string cmd(set_cmd, send_len);

        std::string content;
        content.resize(data_len);
		if (devType == DeviceType::LidarML30B1 || devType == DeviceType::LidarML30SB1 ) {
			content[0] = 0x02; // 02->00
			content[1] = 0x00; // 00->40
		}
		else {
			content[0] = 0x00; // 02->00
			content[1] = 0x40; // 00->40
		}
        //content[0] = 0x00; // 02->00
        //content[1] = 0x40; // 00->40
        std::unique_ptr<int> point_fire_en_data(new int[ids.size()]);
        int* point_fire_en_data_ptr = point_fire_en_data.get();

        for (int i = 0; i < ids.size(); i++)
        {
            point_fire_en_data_ptr[i] = ids[i];
        }
        char *send = (char*)content.c_str() + 2;
        HostToNetwork((const unsigned char*)point_fire_en_data_ptr, send, 1600 * 4);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        // test
        if(0)
        {
            std::ofstream out("test.hex", std::ios::binary);
            if (out.is_open())
            {
                out.write(content.c_str(), content.size());
                out.close();
            }
            std::cout << "data size is " << content.size() << std::endl;
        }
        //return 0;

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        // send file to device
        if (client_->SyncSend(content, data_len))
        {
            DisConnect();
            return -1;
        }

        // final ack
        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return ReturnCode::Success;

    }

    int LidarTools::BackupFirmwareUpdate(std::string& filename, ProgressCallback cb)
    {
        if (!CheckConnection())
            return -1;

        std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
        if (!in.is_open())
        {
            LOG_ERROR("Open file error.\n");
            return -1;
        }

        std::streampos end = in.tellg();
        int size = static_cast<int>(end);
        in.close();

        const int send_len = 6;
        char upgrade_cmd[send_len] = { (char)0xBA, (char)0x11, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };
        HostToNetwork((const unsigned char *)&size, upgrade_cmd + 2);
        std::string cmd(upgrade_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');


        if (client_->SyncSend(cmd, send_len))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        // transfer, erase flash, write
        int start_percent = 10;
        const int pkt_len = 256;
        int block_total = size / pkt_len;
        if (0 != (size % pkt_len))
            block_total += 1;

        int step_per_percent = block_total / 30;//old is 90

        std::ifstream idata(filename, std::ios::in | std::ios::binary);
        char fw_data[pkt_len] = { 0x00 };
        int readed = 0;

        // data transfer
        if (idata.is_open())
        {
            for (readed = 0; readed < block_total; readed++)
            {
                int read_len = pkt_len;
                if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
                    read_len = size % pkt_len;
                idata.read(fw_data, read_len);
                std::string fw(fw_data, read_len);
                if (client_->SyncSend(fw, read_len))
                {
                    break;
                }
                if (0 == (readed % step_per_percent))
                {
                    cb(start_percent++, this->device_ip_.c_str());
                }
            }
            idata.close();
        }
        if (readed != block_total)
        {
            client_->Close();
            return -1;
        }
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Data transfer ok...\n");

        // waitting for step 2
        const int recv_step_len = 5;
        std::string recv_step(recv_step_len, 'x');
        bool ok = false;
        while (1)
        {
            if (client_->SyncRecv(recv_step, recv_step_len))
            {
                ok = false;
                LOG_ERROR("Waiting for erase flash failed...\n");
                break;
            }
            unsigned char step = (unsigned char)recv_step[4];
            cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 == step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Waiting for erase flash ok...\n");

        // waitting for step 3
        while (1)
        {
            if (client_->SyncRecv(recv_step, recv_step_len))
            {
                ok = false;
                break;
            }
            unsigned char step = (unsigned char)recv_step[4];
            cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 == step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            DisConnect();
            return -1;
        }

        //device check data
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        return 0;

    }

    int LidarTools::QueryDeviceBackupFirmwareVersion(FirmwareVersion& version)
    {
        if (!CheckConnection())
            return -1;

        /*Read version info*/
        const int send_len = 4;
        char bv_read_cmd[send_len] = { (char)0xBA, (char)0x12, (char)0x01, (char)0x00 };
        std::string bv_cmd(bv_read_cmd, 4);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(bv_cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv version data
        {
            DisConnect();
            return -1;
        }

        memcpy(&(version.boot_version), recv.c_str(), 4);

        char kv_read_cmd[send_len] = { (char)0xBA, (char)0x12, (char)0x02, (char)0x00 };
        std::string kv_cmd(kv_read_cmd, 4);

        if (client_->SyncSend(kv_cmd, send_len))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv version data
        {
            DisConnect();
            return -1;
        }

        memcpy(&(version.kernel_version), recv.c_str(), 4);

        return 0;

    }

    int LidarTools::SetDeviceEchoMode(EchoMode mode)
    {
        if (!CheckConnection())
            return -1;

        /*Set device echo mode*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x13, (char)0x00, (char)0x00 };

        if (EchoSingleFirst == mode)
            set_cmd[2] = 0x01;
        else if (EchoSingleStrongest == mode)
            set_cmd[2] = 0x02;
        else if (EchoSingleLast == mode)
            set_cmd[2] = 0x04;
        else if (EchoDoubleFirstStrongest == mode)
            set_cmd[2] = 0x03;
        else if (EchoDoubleFirstLast == mode)
            set_cmd[2] = 0x05;
        else if (EchoDoubleStrongestLast == mode)
            set_cmd[2] = 0x06;
        else
            return ReturnCode::InvalidParameter;


        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDeviceCalSendMode(CalSendMode mode)
    {
        if (!CheckConnection())
            return -1;

        /*Set device cal send mode*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x16, (char)0x00, (char)0x00 };

        if (CalSendDisable == mode)
            set_cmd[2] = 0x00;
        else if (CalSendEnable == mode)
            set_cmd[2] = 0x01;
        else
            return ReturnCode::InvalidParameter;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

    int LidarTools::SetDeviceDownsampleMode(DownsampleMode mode)
    {
        if (!CheckConnection())
            return -1;

        /*Set device downsample*/
        const int send_len = 4;
        char set_cmd[send_len] = { (char)0xBA, (char)0x17, (char)0x00, (char)0x00 };

        if (DownsampleNone == mode)
            set_cmd[2] = 0x00;
        else if (Downsample_1_2 == mode)
            set_cmd[2] = 0x01;
        else if (Downsample_1_4 == mode)
            set_cmd[2] = 0x02;
        else
            return ReturnCode::InvalidParameter;

        std::string cmd(set_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))//send cmd
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))//recv ret
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))//check ret
        {
            DisConnect();
            return -1;
        }

        return 0;
    }

	int LidarTools::SetDeviceDHCPMode(StateMode mode) {

		if (!CheckConnection())
			return -1;

		/*Set device dhcp mode*/
		const int send_len = 4;
		char set_cmd[send_len] = { (char)0xBA, (char)0x1D, (char)0x00, (char)0x00 };

		if (StateDisable == mode)
			set_cmd[2] = 0x00;
		else if (StateEnable == mode)
			set_cmd[2] = 0x01;
		else
			return ReturnCode::InvalidParameter;

		std::string cmd(set_cmd, send_len);

		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

	int LidarTools::SetDeviceGatewayAddr(std::string addr) {

		if (!CheckConnection())
			return -1;

		/*Set device gateway address*/
		const int send_len = 6;
		char set_cmd[send_len] = { (char)0xBA, (char)0x1E, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };

		if (!AssembleIpString(addr, set_cmd + 2))
			return InvalidParameter;

		std::string cmd(set_cmd, send_len);

		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

	int LidarTools::SetDeviceFactoryMac(std::string& mac) {

		if (!CheckConnection())
			return -1;

		/*Set device factory mac*/
		const int send_len = 8;
		char set_cmd[send_len] = { (char)0xAB, (char)0x07, (char)0x00, (char)0x00, (char)0x00,(char)0x00, (char)0x00, (char)0x00 };

		if (!AssembleFactoryMacAddress(mac, set_cmd + 2))
			return InvalidParameter;

        if (set_cmd[2] != (char)0xF8 || set_cmd[3] != (char)0xA9 || set_cmd[4] != (char)0x1F)
            return InvalidParameter;

		std::string cmd(set_cmd, send_len);

		const int recv_len = 4;
		std::string recv(recv_len, 'x');

		if (client_->SyncSend(cmd, send_len))//send cmd
		{
			DisConnect();
			return -1;
		}

		if (client_->SyncRecv(recv, recv_len))//recv ret
		{
			DisConnect();
			return -1;
		}

		if (!CheckDeviceRet(recv))//check ret
		{
			DisConnect();
			return -1;
		}

		return 0;
	}

    bool LidarTools::CheckConnection()
    {
        if (!conn_ok_)
        {
            int ret = client_->Connect(this->device_ip_);
            if (ret)
            {
                client_->GetSysErrorCode();
                return false;
            }
            conn_ok_ = true;
        }

        return true;
    }

    void LidarTools::DisConnect()
    {
        if (conn_ok_)
        {
            this->client_->Close();
            conn_ok_ = false;
        }
    }

    bool LidarTools::CheckDeviceRet(std::string ret)
    {
        return (0x00 == ret[2]) && (0x00 == ret[3]);
    }

	/* for ml30s plus only */
	int LidarTools::GetML30sPlusLogFile(std::string& log, bool isFull) {

		const int recv_len = 4;
		std::string recv(recv_len,'x');
		// get log buffer length
		if (client_->SyncRecv(recv, recv_len))
		{
			DisConnect();
			return -1;
		}
		uint32_t log_buffer_len = 0;
		NetworkToHost((const unsigned char*)recv.c_str(), (char *)&log_buffer_len);

		// get log buffer
		std::string log_buffer(log_buffer_len,'x');
		if (client_->SyncRecv(log_buffer, log_buffer_len))
		{
			DisConnect();
			return -1;
		}

		if (isFull) {
			log = log_buffer;
			return 0;
		}

		// assemble the buffer to string
		unsigned char file_count = reinterpret_cast<unsigned char&>(log_buffer[0]);

		std::vector<std::string> file_contents;
		uint32_t position = 1;
		for (int i = 0; i < file_count; ++i)
		{
			uint32_t file_len = 0;
			NetworkToHost((const unsigned char*)(&log_buffer[position]), (char *)&file_len);
			LOG_INFO("FILE %d len %u.\n", i, file_len);
			// copy to file list
			std::string content(log_buffer, position + 4, file_len);
			file_contents.push_back(content);
			position += (file_len + 4);
		}

		// reorder the log file
		log = "";
		for (int i = file_count - 1; i >= 0; i--)
			log += file_contents[i];

		recv.resize(4, 'x');
		if (client_->SyncRecv(recv, recv_len))
		{
			DisConnect();
			return -1;
		}
		//check ret
		if (!CheckDeviceRet(recv))
		{
			DisConnect();
			return -1;
		}
		return 0;
	}

	int LidarTools::GenerateML30SPlusDeviceCmdString(EML30SPlusCmd type, std::string& buf, zvision::JsonConfigFileParam* param) {

		if (!param) return -1;
		param->temp_recv_data_len = 0;

		std::string str_cmd;
		std::string str_value;
		bool valid = true;
		switch (type)
		{
		// set lidar config
		case zvision::set_dhcp_switch: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x07 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->dhcp_switch, str_value);
		}break;
		case zvision::set_ip: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x01 };
			str_cmd.append(cmd, 3);
            char ip[4] = {0};
            if (!AssembleIpString(param->ip, ip))
                return -1;
            str_value = std::string(ip, 6);
		}break;
		case zvision::set_gateway: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x02 };
			str_cmd.append(cmd, 3);
            char ip[4] = { 0 };
            if (!AssembleIpString(param->gateway, ip))
                return -1;
            str_value = std::string(ip, 6);
		}break;
		case zvision::set_netmask: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x03 };
			str_cmd.append(cmd, 3);
            char ip[4] = { 0 };
            if (!AssembleIpString(param->netmask, ip))
                return -1;
            str_value = std::string(ip, 6);
		}break;
		case zvision::set_mac: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x04 };
			str_cmd.append(cmd, 3);
            char mac[6] = { 0 };
            if (!AssembleMacAddress(param->mac, mac))
               return -1;
            str_value = std::string(mac, 6);
		}break;
		case zvision::set_udp_dest_ip: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x05 };
			str_cmd.append(cmd, 3);
            char ip[4] = { 0 };
            if (!AssembleIpString(param->udp_dest_ip, ip))
                return -1;
            str_value = std::string(ip, 6);
		}break;
		case zvision::set_udp_dest_port: {
			char cmd[3] = { (char)0xBA,(char)0x02,(char)0x06 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->udp_dest_port, str_value);
		}break;
		case zvision::set_near_point_delete_switch: {
			char cmd[3] = { (char)0xBA,(char)0x03,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->switch_near_point_delete, str_value);
		}break;
		case zvision::set_retro_switch: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->switch_retro, str_value);
		}break;
		case zvision::set_retro_target_gray_thre: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x03 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->target_gray_thre_retro, str_value);
		}break;
		case zvision::set_retro_target_point_num_thre: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x02 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->target_point_num_thre_retro, str_value);
		}break;
		case zvision::set_retro_critical_point_dis_thre: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x04 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->critical_point_dis_thre_retro, str_value);
		}break;
		case zvision::set_retro_del_point_dis_low_thre: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x05 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->del_point_dis_low_thre_retro, str_value);
		}break;
		case zvision::set_retro_del_point_dis_high_thre: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x06 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->del_point_dis_high_thre_retro, str_value);
		}break;
		case zvision::set_retro_del_point_gray_thre: {
			char cmd[3] = { (char)0xBA,(char)0x04,(char)0x07 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->del_point_gray_thre_retro, str_value);
		}break;
		case zvision::set_adhesion_switch: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->switch_adhesion, str_value);
		}break;
		case zvision::set_adhesion_angle_hor_min: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x02 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_hor_min_adhesion, str_value);
		}break;
		case zvision::set_adhesion_angle_hor_max: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x03 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_hor_max_adhesion, str_value);
		}break;
		case zvision::set_adhesion_angle_ver_min: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x04 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_ver_min_adhesion, str_value);
		}break;
		case zvision::set_adhesion_angle_ver_max: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x05 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_ver_max_adhesion, str_value);
		}break;
		case zvision::set_adhesion_angle_hor_res: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x06 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_hor_res_adhesion, str_value);
		}break;
		case zvision::set_adhesion_angle_ver_res: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x07 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_ver_res_adhesion, str_value);
		}break;
		case zvision::set_adhesion_diff_thre: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x08 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->diff_thre_adhesion, str_value);
		}break;
		case zvision::set_adhesion_dist_limit: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x09 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->dist_limit_adhesion, str_value);
		}break;
		case zvision::set_adhesion_min_diff: {
			char cmd[3] = { (char)0xBA,(char)0x05,(char)0x0A };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->min_diff_adhesion, str_value);
		}break;
		case zvision::set_down_sample_mode: {
			char cmd[3] = { (char)0xBA,(char)0x06,(char)0x01 };
			str_cmd.append(cmd, 3);
			 GenerateStringFromValue(param->down_sample_mode, str_value);
		}break;
		case zvision::set_dirty_detect_switch: {
			char cmd[3] = { (char)0xBA,(char)0x07,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->switch_dirty_detect, str_value);
		}break;
		case zvision::set_dirty_detect_refresh: {

		}break;
		case zvision::set_dirty_detect_cycle: {

		}break;
		case zvision::set_dirty_detect_set_thre: {

		}break;
		case zvision::set_dirty_detect_reset_thre: {

		}break;
		case zvision::set_dirty_detect_inner_thre: {

		}break;
		case zvision::set_dirty_detect_outer_thre: {

		}break;
		case zvision::set_echo_mode: {
			char cmd[3] = { (char)0xBA,(char)0x08,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->echo_mode, str_value);
		}break;
		case zvision::set_ptp_sync: {
			char cmd[3] = { (char)0xBA,(char)0x09,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->ptp_sync, str_value);
		}break;
		case zvision::set_ptp_file: {
			char cmd[3] = { (char)0xBA,(char)0x09,(char)0x02 };
			str_cmd.append(cmd, 3);
			// read file data
			std::ifstream infile(param->temp_filepath);
			if (!infile.is_open())
				return -1;
			std::stringstream ss;
			ss << infile.rdbuf();
			param->temp_send_data = ss.str();
			param->temp_send_data_len = param->temp_send_data.size();
			GenerateStringFromValue(param->temp_send_data_len, str_value);
		}break;
		case zvision::set_frame_sync: {
			char cmd[3] = { (char)0xBA,(char)0x0A,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->frame_sync, str_value);
		}break;
		case zvision::set_frame_offset: {
			char cmd[3] = { (char)0xBA,(char)0x0A,(char)0x02 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->frame_offset, str_value);
		}break;
		case zvision::set_angle_send: {
			char cmd[3] = { (char)0xBA,(char)0x0B,(char)0x01 };
			str_cmd.append(cmd, 3);
			GenerateStringFromValue(param->angle_send, str_value);
		}break;
		case zvision::set_json_config_file: {
			char cmd[3] = { (char)0xBA,(char)0x0E,(char)0x01 };
			str_cmd.append(cmd, 3);
			// read file data
			std::ifstream infile(param->temp_filepath);
			if (!infile.is_open())
				return -1;
			std::stringstream ss;
			ss << infile.rdbuf();
			std::string str = ss.str();
			// check json format
			rapidjson::Document doc;
			if (doc.Parse(str.c_str()).HasParseError())
				return -1;

			param->temp_send_data = str;
			param->temp_send_data_len = str.size();
			GenerateStringFromValue(param->temp_send_data_len, str_value);
		}break;
		// lidar control
		case zvision::reboot: {
			char cmd[4] = { (char)0xBA,(char)0x0c,(char)0x01,(char)0x01 };
			str_cmd.append(cmd, 4);
		}break;

		// get lidar config
		case zvision::read_serial_number: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x01,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = 0;
		}break;
		case zvision::read_factory_mac: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x02,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = 8;
		}break;
		case zvision::read_network_param: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x03,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = 27;
		}break;
		case zvision::read_embeded_fpga_version: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x04,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = 16;
		}break;
		case zvision::read_config_param_in_use: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x05,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = -2; //50;
		}break;
		case zvision::read_config_param_saved: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x06,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = -2; //50;
		}break;
		case zvision::read_algo_param_in_use: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x07,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = -2; //84;
		}break;
		case zvision::read_algo_param_saved: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x08,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = -2; //84;
		}break;
		case zvision::read_json_config_file: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x09,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = -4;
		}break;
		case zvision::read_cali_packets: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x0A,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = 0;
		}break;
		case zvision::read_temp_log: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x0B,(char)0x00 };
			str_cmd.append(cmd, 4);
			// special deal
			param->temp_recv_data_len = 0;
		}break;
		case zvision::read_full_log: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x0C,(char)0x00 };
			str_cmd.append(cmd, 4);
			// special deal
			param->temp_recv_data_len = 0;
		}break;
		case read_ptp_file: {
			char cmd[4] = { (char)0xBA,(char)0x0D,(char)0x0D,(char)0x00 };
			str_cmd.append(cmd, 4);
			param->temp_recv_data_len = -4;
		}break;

		default:
			return -1;
		}

		std::string str = str_cmd + str_value;
		if (!valid || str.empty())
			return -1;

		buf = str;
		return 0;
	}

	int LidarTools::RunPost(EML30SPlusCmd type, zvision::JsonConfigFileParam* param) {

		if (!param)
			return -1;

		const unsigned char* pdata = (unsigned char*)param->temp_recv_data.c_str();
		const int data_len = param->temp_recv_data.size();
		switch (type)
		{
		case zvision::read_serial_number:
			param->serial_number = param->temp_recv_data;
			break;
		case zvision::read_factory_mac: {
			char fmac[128] = "";
			sprintf_s(fmac, "%02X%02X%02X%02X%02X%02X", pdata[2], pdata[3], pdata[4], pdata[5], pdata[6], pdata[7]);
			param->factory_mac = std::string(fmac);
		}break;
		case zvision::read_network_param: {
			param->dhcp_switch = *(pdata + 0);
			ResolveIpString(pdata + 1, param->ip);
			ResolveIpString(pdata + 5, param->netmask);
			ResolveIpString(pdata + 9, param->gateway);
			ResolveIpString(pdata + 13, param->mac);
			ResolveIpString(pdata + 19, param->udp_dest_ip);
			int port = 0;
			ResolvePort(pdata + 23, port);
			param->udp_dest_port = port;
		}break;
		case zvision::read_embeded_fpga_version: {
			memcpy(param->embedded_ver, pdata, 4);
			memcpy(param->fpga_ver, pdata + 4 , 4);
			ResolveIpString(pdata, param->embedded_version);
			ResolveIpString(pdata + 4, param->fpga_version);

			memcpy(param->embedded_ver_bak, pdata + 8, 4);
			memcpy(param->fpga_ver_bak, pdata + 12, 4);
			ResolveIpString(pdata + 8, param->embedded_version_bak);
			ResolveIpString(pdata + 12, param->fpga_version_bak);
		}break;
		case zvision::read_config_param_in_use:
		case zvision::read_config_param_saved: {
			//NetworkToHostShort(pdata,(char*)&param->algo_param_len);
			if (data_len < 48)
				return -1;
			memcpy(param->embedded_ver, pdata, 4);
			ResolveIpString(pdata, param->embedded_version);
			memcpy(param->fpga_ver, pdata + 4, 4);
			ResolveIpString(pdata + 4, param->fpga_version);
			ResolveIpString(pdata + 8, param->ip);
			ResolveIpString(pdata + 12, param->gateway);
			ResolveIpString(pdata + 16, param->netmask);
			ResolveMacAddress(pdata + 20, param->mac);
			ResolveIpString(pdata + 26, param->udp_dest_ip);
			NetworkToHost(pdata + 30, (char*)&param->udp_dest_port);
			param->dhcp_switch = *(pdata + 34);
			param->switch_retro = *(pdata + 35);
			param->switch_near_point_delete = *(pdata + 36);
			param->switch_adhesion = *(pdata + 37);
			param->down_sample_mode = *(pdata + 38);
			param->echo_mode = *(pdata + 39);
			param->ptp_sync = *(pdata + 40);
			param->switch_dirty_detect = *(pdata + 41);
			param->angle_send = *(pdata + 42);
			param->frame_sync = *(pdata + 43);
			NetworkToHost(pdata + 44, (char*)&param->frame_offset);
		}break;
		case zvision::read_algo_param_in_use:
		case zvision::read_algo_param_saved: {

			// parameter data
			const unsigned char* param_data = pdata + 20;
			uint16_t param_len = data_len - 20;
			// before 20 bytes is header data
			// near point delete
            uint16_t offset = 0;
            uint16_t len = 0;
			{
				NetworkToHostShort(pdata , (char*)&offset);
				NetworkToHostShort(pdata + 2, (char*)&len);
                if (len >= 1 && ((offset + len) <= param_len)) {
                    const unsigned char* param_del = param_data + offset;
                    param->switch_near_point_delete = *(param_del + 0);
                }
			}
			// retro
			{
                NetworkToHostShort(pdata + 4, (char*)&offset);
                NetworkToHostShort(pdata + 6, (char*)&len);
                if (len >= 12 && ((offset + len) <= param_len)) {
                    const unsigned char* param_retro = param_data + offset;
                    param->switch_retro = *(param_retro + 0);
                    param->target_gray_thre_retro = *(param_retro + 1);
                    param->target_point_num_thre_retro = *(param_retro + 2);
                    NetworkToHost(param_retro + 3, (char*)&param->critical_point_dis_thre_retro);
                    NetworkToHostShort(param_retro + 7, (char*)&param->del_point_dis_low_thre_retro);
                    NetworkToHostShort(param_retro + 9, (char*)&param->del_point_dis_high_thre_retro);
                    param->del_point_gray_thre_retro = *(param_retro + 11);
                }
			}

			// adhesion
            {
                NetworkToHostShort(pdata + 8, (char*)&offset);
                NetworkToHostShort(pdata + 10, (char*)&len);
                if (len >= 37 && ((offset + len) <= param_len)) {
                    const unsigned char* param_adhesion = param_data + offset;
                    param->switch_adhesion = *(param_adhesion + 0);
                    NetworkToHost(param_adhesion + 1, (char*)&param->angle_hor_min_adhesion);
                    NetworkToHost(param_adhesion + 5, (char*)&param->angle_hor_max_adhesion);
                    NetworkToHost(param_adhesion + 9, (char*)&param->angle_ver_min_adhesion);
                    NetworkToHost(param_adhesion + 13, (char*)&param->angle_ver_max_adhesion);
                    NetworkToHost(param_adhesion + 17, (char*)&param->angle_hor_res_adhesion);
                    NetworkToHost(param_adhesion + 21, (char*)&param->angle_ver_res_adhesion);
                    NetworkToHost(param_adhesion + 25, (char*)&param->diff_thre_adhesion);
                    NetworkToHost(param_adhesion + 29, (char*)&param->dist_limit_adhesion);
                    NetworkToHost(param_adhesion + 33, (char*)&param->min_diff_adhesion);
                }
            }

			// downsample
            {
                NetworkToHostShort(pdata + 12, (char*)&offset);
                NetworkToHostShort(pdata + 14, (char*)&len);
                if (len >= 1 && ((offset + len) <= param_len)) {
                    const unsigned char* param_downsample = param_data + offset;
                    param->down_sample_mode = *(param_downsample + 0);
                }
            }

			// dirty detect
            {
                NetworkToHostShort(pdata + 16, (char*)&offset);
                NetworkToHostShort(pdata + 18, (char*)&len);
                if (len >= 12 && ((offset + len) <= param_len)) {
                    const unsigned char* param_dirty = param_data + offset;
                    param->switch_dirty_detect = *(param_dirty + 0);
                    param->switch_dirty_refresh = *(param_dirty + 1);
                    NetworkToHostShort(param_dirty + 2, (char*)&param->dirty_refresh_cycle);
                    NetworkToHostShort(param_dirty + 4, (char*)&param->dirty_detect_set_thre);
                    NetworkToHostShort(param_dirty + 6, (char*)&param->dirty_detect_reset_thre);
                    NetworkToHostShort(param_dirty + 8, (char*)&param->dirty_detect_inner_thre);
                    NetworkToHostShort(param_dirty + 10, (char*)&param->dirty_detect_outer_thre);
                }
            }
		}break;
		case zvision::read_json_config_file:
        {

        }break;
		case zvision::read_temp_log:
        {

        }break;
		case zvision::read_full_log:
        {

        }break;
		default:
			break;
		}

		return 0;
	}

	int LidarTools::RunML30sPlusDeviceManager(EML30SPlusCmd type, zvision::JsonConfigFileParam* param){

		// pre check
		if (!param)
			return -1;

		/* for special command */
		// read cali packets
		if (type == read_cali_packets) {
			char cmd[4] = { (char)0xBA, (char)0x0D, (char)0x0A, (char)0x00 };
			return GetDeviceCalibrationPackets(param->temp_recv_packets, std::string(cmd, 4));
		}

		/*  1. We generate tcp command , send data if in need. */
		std::string cmd_str;
		int ret = GenerateML30SPlusDeviceCmdString(type, cmd_str, param);
		if (ret != 0)
			return -1;

		/* 2. Send cmd to device. */
		// Check tcp connection.
		if (!CheckConnection())
			return -1;
		// send cmd
		if (client_->SyncSend(cmd_str, cmd_str.size()))
		{
			DisConnect();
			return -1;
		}

		// need ack
		if (type != set_json_config_file) {
			const int recv_len = 4;
			std::string recv(4,'x');
			if (client_->SyncRecv(recv, recv_len))
			{
				DisConnect();
				return -1;
			}

			if (!CheckDeviceRet(recv))
			{
				DisConnect();
				return -1;
			}
		}

		// If don`t send or receive data, return.
		if (type < cmd_section_control)
			return 0;

		/* 3. for special command */
		// read log ...
		if (type == read_temp_log || type == read_full_log) {
			return GetML30sPlusLogFile(param->temp_recv_data, type == read_full_log);
		}

		// We send/receive 1024 bytes once.
		int pkt_len = 1024;
		/* 4.  Send data to device. */
		if (type > cmd_section_control && type < cmd_section_send_data ) {
			int send_len = param->temp_send_data.size();
			if (!send_len)
				return -1;

			std::string send_data = param->temp_send_data;
			int pkt_cnt = send_len / pkt_len;
			if (send_len % pkt_len) pkt_cnt++;
			for (int i = 0; i < pkt_cnt; i++) {

				int len = pkt_len;
				if (i == send_len / pkt_len)
					len = send_len % pkt_len;

                // send
				std::string str_send(send_data.c_str() + i*pkt_len, len);
				if (client_->SyncSend(str_send, len))
				{
					DisConnect();
					return -1;
				}

				// update
				if(i > 5)
					std::this_thread::sleep_for(std::chrono::microseconds(110));
			}

            // 4.1 check ret.
            {
                const int recv_len = 4;
                std::string recv(4, 'x');
                if (client_->SyncRecv(recv, recv_len))
                {
                    DisConnect();
                    return -1;
                }
                if (!CheckDeviceRet(recv))
                {
                    DisConnect();
                    return -1;
                }
            }
		}
		/* 5.  Receive data from device. */
		else if(type > cmd_section_send_data && type < cmd_section_receive_data ) {

			std::string recv_data;
			uint32_t recv_len = 0;
			// Get recv data length.
			if (param->temp_recv_data_len < 0) {
				int bytes = std::abs(param->temp_recv_data_len);
				std::string str(bytes,'x');
				if (client_->SyncRecv(str, bytes))
				{
					DisConnect();
					return -1;
				}

				// Get length.
				if (bytes == 1)
					recv_len = (uint8_t)str[0];
				else if (bytes == 2) {
					uint16_t val = 0;
					NetworkToHostShort((uint8_t*)str.c_str(), (char*)&val);
					recv_len = val;
				}
				else if (bytes == 4)
					NetworkToHost((uint8_t*)str.c_str(), (char*)&recv_len);
				else
					return -1;
			}
			else if (param->temp_recv_data_len == 0)
				recv_len = client_->GetAvailableBytesLen();
			else
				recv_len = param->temp_recv_data_len;

			if(recv_len == 0) return -1;


			// for special cmd
			if (type == read_algo_param_in_use || type == read_algo_param_saved) {
				// 20 bytes header
				recv_len += 20;
			}

			// Read data.
			int pkt_cnt = recv_len / pkt_len;
			if (recv_len % pkt_len) pkt_cnt++;
			for (int i = 0; i < pkt_cnt; i++) {

				int len = pkt_len;
				if (i == recv_len / pkt_len)
					len = recv_len % pkt_len;

				// recv ret
				std::string pkt(len, 'x');
				if (client_->SyncRecv(pkt, len))
				{
					DisConnect();
					return -1;
				}
				// update
                const char* pdata = pkt.data();
                recv_data += std::string(pdata,len);
				if(i > 10)
					std::this_thread::sleep_for(std::chrono::microseconds(110));
			}

			param->temp_recv_data = recv_data;
		}

		// 6. post deal
		if (RunPost(type, param) != 0)
			return -1;

		// read all recv buf if necessar
		{
			int len = client_->GetAvailableBytesLen();
			if (len > 0) {
				std::string tmp(len,'x');
				client_->SyncRecv(tmp, len);
			}
		}

		return 0;
	}

	int LidarTools::QueryML30sPlusDeviceConfigurationInfo(DeviceConfigurationInfo& info) {

		int ret = 0;
		JsonConfigFileParam param;
		ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_config_param_in_use, &param);
		if (ret != 0)
			return ret;

		ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_embeded_fpga_version, &param);
		if (ret != 0)
			return ret;

        info.device = DeviceType::LidarMl30SA1Plus;
        info.serial_number = param.serial_number;
        memcpy(info.version.boot_version, param.fpga_ver, 4);
        memcpy(info.version.kernel_version, param.embedded_ver, 4);
		memcpy(info.backup_version.boot_version, param.fpga_ver_bak, 4);
		memcpy(info.backup_version.kernel_version, param.embedded_ver_bak, 4);
        info.config_mac = param.mac;
        info.device_mac = param.mac;
        info.factory_mac = param.factory_mac;
        info.device_ip = param.ip;
        info.subnet_mask = param.netmask;
        info.destination_ip = param.udp_dest_ip;
        info.destination_port = param.udp_dest_port;
        info.time_sync = TimestampType::TimestampPtp;
        info.phase_offset = param.frame_offset;
        info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetUnknown;
        if (param.frame_sync == 0)  info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetDisable;
        else if (param.frame_sync == 1)  info.phase_offset_mode = PhaseOffsetMode::PhaseOffsetEnable;

        info.echo_mode = EchoMode::EchoUnknown;
        if (param.echo_mode == 1)  info.echo_mode = EchoMode::EchoSingleFirst;
        else if (param.echo_mode == 2)  info.echo_mode = EchoMode::EchoSingleStrongest;
		else if (param.echo_mode == 3)  info.echo_mode = EchoMode::EchoSingleLast;
		else if (param.echo_mode == 4)  info.echo_mode = EchoMode::EchoDoubleFirstStrongest;
		else if (param.echo_mode == 5)  info.echo_mode = EchoMode::EchoDoubleFirstLast;
		else if (param.echo_mode == 6)  info.echo_mode = EchoMode::EchoDoubleStrongestLast;

        info.cal_send_mode = CalSendMode::CalSendUnknown;
        if (param.angle_send == 0)  info.cal_send_mode = CalSendMode::CalSendDisable;
        else if (param.angle_send == 1)  info.cal_send_mode = CalSendMode::CalSendEnable;

        info.downsample_mode = DownsampleMode::DownsampleUnknown;
        if (param.down_sample_mode == 0)  info.downsample_mode = DownsampleMode::DownsampleNone;
        else if (param.switch_near_point_delete == 1)  info.downsample_mode = DownsampleMode::Downsample_1_2;
        else if (param.switch_near_point_delete == 2)  info.downsample_mode = DownsampleMode::Downsample_1_4;

        info.dhcp_enable = StateMode::StateUnknown;
        if (param.dhcp_switch == 0)  info.dhcp_enable = StateMode::StateDisable;
        else if (param.dhcp_switch == 1)  info.dhcp_enable = StateMode::StateEnable;

        info.gateway_addr = param.gateway;

		//return 0;

		// serial number
		ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_serial_number, &param);
		if (ret != 0)
			return ret;

		info.serial_number = param.serial_number;

        // algo param section
        info.algo_param.isValid = false;
        info.adhesion_enable = StateMode::StateUnknown;
        info.retro_enable = RetroMode::RetroUnknown;
        info.delete_point_enable = StateMode::StateUnknown;
        ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_algo_param_in_use, &param);
        if (ret != 0)
           return ret;

        info.algo_param.isValid = true;
        if(param.switch_adhesion == 0)  info.adhesion_enable = StateMode::StateDisable;
        else if(param.switch_adhesion == 1)  info.adhesion_enable = StateMode::StateEnable;

        if (param.switch_retro == 0)  info.retro_enable = RetroMode::RetroDisable;
        else if (param.switch_retro == 1)  info.retro_enable = RetroMode::RetroEnable;

        if (param.switch_near_point_delete == 0)  info.delete_point_enable = StateMode::StateDisable;
        else if (param.switch_near_point_delete == 1)  info.delete_point_enable = StateMode::StateEnable;

        info.algo_param.isValid = true;
        info.algo_param.adhesion_angle_hor_max = param.angle_hor_max_adhesion;
        info.algo_param.adhesion_angle_hor_min = param.angle_hor_min_adhesion;
        info.algo_param.adhesion_angle_hor_res = param.angle_hor_res_adhesion;
        info.algo_param.adhesion_angle_ver_max = param.angle_ver_max_adhesion;
        info.algo_param.adhesion_angle_ver_min = param.angle_ver_min_adhesion;
        info.algo_param.adhesion_angle_ver_res = param.angle_ver_res_adhesion;
        info.algo_param.adhesion_diff_thres = param.diff_thre_adhesion;
        info.algo_param.adhesion_dis_limit = param.dist_limit_adhesion;
        info.algo_param.adhesion_min_diff = param.min_diff_adhesion;

        info.algo_param.retro_del_gray_thres = param.del_point_gray_thre_retro;
        info.algo_param.retro_del_ratio_gray_high_thres = 0;
        info.algo_param.retro_del_ratio_gray_low_thres = 0;
        info.algo_param.retro_dis_thres = param.critical_point_dis_thre_retro;
        info.algo_param.retro_high_range_thres = param.del_point_dis_high_thre_retro;
        info.algo_param.retro_low_range_thres = param.del_point_dis_low_thre_retro;
        info.algo_param.retro_min_gray = param.target_gray_thre_retro;
        info.algo_param.retro_min_gray_num = param.target_point_num_thre_retro;

		// read factory mac
		ret = RunML30sPlusDeviceManager(EML30SPlusCmd::read_factory_mac, &param);
		if (ret != 0)
			return ret;
		info.factory_mac = param.factory_mac;
		return 0;
	}

    int LidarTools::ML30sPlusFirmwareUpdate(std::string& filename, ProgressCallback cb,bool isBak)
    {
        if (!CheckConnection())
            return -1;

        std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
        if (!in.is_open())
        {
            LOG_ERROR("Open file error.\n");
            return -1;
        }

        std::streampos end = in.tellg();
        int size = static_cast<int>(end);
        in.close();

        const int send_len = 7;
        char upgrade_cmd[send_len] = { (char)0xBA, (char)0x01, (char)0x01, (char)0x00, (char)0x00, (char)0x00 ,(char)0x00 };
        if (isBak)
            upgrade_cmd[2] = 0x02;

        HostToNetwork((const unsigned char*)&size, upgrade_cmd + 3);
        std::string cmd(upgrade_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');


        if (client_->SyncSend(cmd, send_len))
        {
            DisConnect();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        // transfer, erase flash, write
        const int pkt_len = 256;
        int start_percent = 10;
        int block_total = size / pkt_len;
        if (0 != (size % pkt_len))
            block_total += 1;
        int step_per_percent = block_total / 30;

        std::ifstream idata(filename, std::ios::in | std::ios::binary);
        char fw_data[pkt_len] = { 0x00 };
        int readed = 0;

        // data transfer
        if (idata.is_open())
        {
            for (readed = 0; readed < block_total; readed++)
            {
                int read_len = pkt_len;
                if ((readed == (block_total - 1)) && (0 != (size % pkt_len)))
                    read_len = size % pkt_len;
                idata.read(fw_data, read_len);
                std::string fw(fw_data, read_len);
                if (client_->SyncSend(fw, read_len))
                {
                    break;
                }
                if (0 == (readed % step_per_percent))
                {
                    cb(start_percent++, this->device_ip_.c_str());
                }
            }
            idata.close();
        }
        if (readed != block_total)
        {
            client_->Close();
            return -1;
        }

        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Data transfer ok...\n");

        // waitting for step 2
        const int recv_step_len = 5;
        std::string recv_step(recv_step_len, 'x');
        bool ok = false;
        while (1)
        {
            if (client_->SyncRecv(recv_step, recv_step_len))
            {
                ok = false;
                break;
            }
            unsigned char step = (unsigned char)recv_step[4];
            cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 == step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            LOG_ERROR("Waiting for erase flash failed...\n");
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Waiting for erase flash ok...\n");

        // waitting for step 3
        while (1)
        {
            if (client_->SyncRecv(recv_step, recv_step_len))
            {
                ok = false;
                break;
            }
            unsigned char step = (unsigned char)recv_step[4];
            cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 == step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            LOG_ERROR("Waiting for write flash failed...\n");
            DisConnect();
            return -1;
        }
        LOG_DEBUG("Waiting for write flash ok...\n");

        //device check data
        if (client_->SyncRecv(recv, recv_len))
        {
            DisConnect();
            return -1;
        }

        if (!CheckDeviceRet(recv))
        {
            DisConnect();
            return -1;
        }

        return 0;

    }

}
