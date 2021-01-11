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
        std::string packet_header = "ZVS";
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

            if ((heart_beat_len == len) && (0 == packet_header.compare(0, packet_header.size(), data.substr(0, packet_header.size())))) // heart beat packet
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
            InvalidContent;

        // filter #
        int curr = 0;
        for (auto& line : lines)
        {
            if ('#' != line[0][0])
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
                if ((lines.size() - curr - 1) < 38000)//check the file lines
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

    int LidarTools::ExportCalibrationData(CalibrationData& cal, std::string filename)
    {
        int data_in_line = 0;
        if ((ScanMode::ScanML30SA1_160 == cal.scan_mode) || (ScanMode::ScanML30SA1_190 == cal.scan_mode))
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            data_in_line = 16;
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
                        outfile << i / data_in_line + 1 << " ";
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
        else if (ScanMode::ScanML30B1_100 == cal.scan_mode)
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            data_in_line = 6;
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
            data_in_line = 6;
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
            data_in_line = 6;
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
        cal_cos_sin_lut.data.resize(cal.data.size());

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
        else if ((ScanMode::ScanML30SA1_160 == cal.scan_mode) || (ScanMode::ScanML30SA1_190 == cal.scan_mode))
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
        else if ((ScanMode::ScanMLX_160 == cal.scan_mode) || (ScanMode::ScanMLX_190 == cal.scan_mode))
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

    int LidarTools::QueryDeviceSnCode(std::string& sn)
    {
        if (!CheckConnection())
            return -1;

        /*Read version info*/
        const int send_len = 4;
        char sn_read_cmd[send_len] = { (char)0xBA, (char)0x08, (char)0x00, (char)0x00 };
        std::string cmd(sn_read_cmd, 4);

        const int recv_len = 21;
        std::string recv(recv_len, 'x');

        if (client_->SyncSend(cmd, send_len))
        {
            DisConnect();
            return -1;
        }

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
        else if (0x01 == time_sync)
        {
            info.retro_enable = RetroMode::RetroEnable;
        }

        ResolveIpString(header + 46, info.subnet_mask);
        ResolveMacAddress(header + 100, info.device_mac);

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

            if (0 == sn.compare(0, ml30b1_sn_prefix.size(), ml30b1_sn_prefix)) // ML30B1
            {
                info.device = DeviceType::LidarML30B1;
            }
            else if (0 == sn.compare(0, ml30sa1_sn_prefix.size(), ml30sa1_sn_prefix)) // ML30SA1
            {
                info.device = DeviceType::LidarML30SA1;
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
        return 0;

    }

    int LidarTools::GetDeviceCalibrationData(CalibrationData& cal)
    {
        std::string ec;
        const int ppf = 256000; // points per frame, 256000 reserved
        const int ppk = 128; // points per cal udp packet
        std::unique_ptr<float> angle_data(new float[ppf * 2]); // points( azimuh, elevation);
        int packet_buffer_size = 1040 * (ppf / ppk) + 4; // 128 points in one packet, buffer reserved for ppf points.
        std::unique_ptr<unsigned char> packet_data(new unsigned char[packet_buffer_size]);
        const int send_len = 4;
        char cal_cmd[send_len] = { (char)0xBA, (char)0x07, (char)0x00, (char)0x00 };
        std::string cmd(cal_cmd, send_len);

        const int recv_len = 4;
        std::string recv(recv_len, 'x');

        if (!CheckConnection())
            return -1;

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

        const int cal_pkt_len = 1040;
        std::string cal_recv(cal_pkt_len, 'x');
        CalibrationPacket* pkt = reinterpret_cast<CalibrationPacket*>((char *)cal_recv.c_str());

        //receive first packet to identify the device type
        if (client_->SyncRecv(cal_recv, cal_pkt_len))
        {
            DisConnect();
            return -1;
        }

        int total_packet = 0;
        DeviceType tp = pkt->GetDeviceType();
        ScanMode sm = pkt->GetScanMode();

        if (ScanMode::ScanML30B1_100 == sm)
        {
            total_packet = 235;// 10000 * 3 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanML30SA1_160 == sm)
        {
            total_packet = 400;// 6400 * 8 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanML30SA1_190 == sm)
        {
            total_packet = 450;// 7200 * 8 * 2 * 4 / 1024
        }
        else if (ScanMode::ScanMLX_160 == sm)
        {
            total_packet = 750;// 32000 *3 * 2 * 4 / 1024
        }
        else
        {
            DisConnect();
            return -1;// not support
        }

        //check the data
        unsigned char* check_data = (unsigned char *)cal_recv.c_str();
        unsigned char check_all_00 = 0x00;
        unsigned char check_all_ff = 0xFF;
        for (int i = 0; i < 1040 - 16; i++)
        {
            check_all_00 |= check_data[i];
            check_all_ff &= check_data[i];
        }
        if (0x00 == check_all_00)
        {
            LOG_ERROR("Check calibration data error, data is all 0x00.\n");
            DisConnect();
            return -1;
        }
        if (0xFF == check_all_ff)
        {
            LOG_ERROR("Check calibration data error, data is all 0xFF.\n");
            DisConnect();
            return -1;
        }

        pkt->ExtractData(cal.data);

        for (int i = 0; i < total_packet - 1; i++)
        {
            //LOG_DEBUG("Get calibration data %d.\n", i);
            std::this_thread::sleep_for(std::chrono::microseconds(110));
            int ret = client_->SyncRecv(cal_recv, cal_pkt_len);
            if (ret)
            {
                LOG_ERROR("Receive calibration data error, ret = %d.\n", ret);
                DisConnect();
                return -1;
            }
            pkt->ExtractData(cal.data);
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

        if (ScanMode::ScanML30B1_100 == sm)
        {
            cal.data.resize(10000 * 3 * 2); //10000 for per sub fov, 3 fovs, 2(azimuth and elevation)
        }
        else if (ScanMode::ScanML30SA1_160 == sm)
        {
            cal.data.resize(6400 * 8 * 2);
        }
        else if (ScanMode::ScanML30SA1_190 == sm)
        {
            cal.data.resize(6400 * 8 * 2);
        }
        else if (ScanMode::ScanMLX_160 == sm)
        {
            cal.data.resize(32000 * 3 * 2);
        }
        else
        {

        }

        LOG_DEBUG("Get calibration data ok.\n");
        cal.device_type = tp;
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
                if (readed == (block_total - 1))
                    read_len = size % pkt_len;
                idata.read(fw_data, read_len);
                std::string fw(fw_data, read_len);
                if (client_->SyncSend(fw, read_len))
                {
                    break;
                }
                if (0 == (readed % step_per_percent))
                {
                    cb(start_percent++);
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
            cb(40 + int((double)step / 3.3));
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
            cb(70 + int((double)step / 3.3));
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
        for (int i = file_count - 1; i > 0; i--)
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
        // --- read ptp configuration file content
        std::ifstream infile(ptp_cfg_filename);

        //get length of file
        infile.seekg(0, std::ios::end);
        size_t length = infile.tellg();
        infile.seekg(0, std::ios::beg);

        if (!infile.is_open())
        {
            return ReturnCode::OpenFileError;
        }

        std::unique_ptr<char> data(new char[length]);
        infile.read(data.get(), length);
        infile.close();

        std::string content(data.get(), length);

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
        std::string ptp_cfg_buffer;
        ptp_cfg_buffer.resize(ptp_buffer_len);
        if (client_->SyncRecv(ptp_cfg_buffer, ptp_buffer_len))
        {
            DisConnect();
            return -1;
        }

        ptp_cfg = ptp_cfg_buffer;

        return 0;

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
                if (readed == (block_total - 1))
                    read_len = size % pkt_len;
                idata.read(fw_data, read_len);
                std::string fw(fw_data, read_len);
                if (client_->SyncSend(fw, read_len))
                {
                    break;
                }
                if (0 == (readed % step_per_percent))
                {
                    cb(start_percent++);
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
            cb(40 + int((double)step / 3.3));
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
            cb(70 + int((double)step / 3.3));
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

        /*Get device log info*/
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

   

}
