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


#include "tcp_client.h"
#include "packet.h"
#include "lidar_tools.h"
#include "print.h"
#include <iostream>
#include <functional>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>

namespace zvision
{
    LidarTools::LidarTools(std::string lidar_ip, int con_timeout, int send_timeout, int recv_timeout):
        device_ip_(lidar_ip),
        conn_ok_(false),
        client_(new TcpClient(con_timeout, send_timeout, recv_timeout))
    {

    }

    LidarTools::~LidarTools()
    {
        this->DisConnect();
    }

    int LidarTools::ScanDevice(std::vector<DeviceConfigurationInfo>& device_list, int scan_time)
    {
        const auto before = std::chrono::system_clock::now();

        UdpReceiver recv(55000, 100);
        std::string data;
        int len = 48;
        int ip = 0;
        int ret = 0;
        std::string packet_header = "ZVI";
        std::vector<std::string> lidars;
        device_list.clear();

        while (1)
        {
            if (ret = recv.SyncRecv(data, len, ip))
            {
                LOG_ERROR("Scan device error, receive failed ret = %d.\n", ret);
                break;
            }

            if (0 == packet_header.compare(0, packet_header.size(), data)) // heart beat packet
            {
                std::string ip_string = IpToString(ip);
                lidars.push_back(ip_string);
            }

            // time elapse
            std::chrono::duration<double> duration = std::chrono::system_clock::now() - before;
            if (duration.count() >= scan_time)
                break;
        }

        for (int i = 0; i < lidars.size(); ++i)
        {
            LidarTools config(lidars[0], 1000);
            DeviceConfigurationInfo info;
            if (ret = config.QueryDeviceConfigurationInfo(info))
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
        DeviceType tp = DeviceType::LidarUnknown;
        int file_lines = 0;
        int file_cols = 0;
        std::ifstream file;
        file.open(filename, std::ios::in);
        std::string line;
        if (file.is_open())
        {
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
            int ret = 0;
            if (10000 == lines.size())
            {
                cal.data.resize(60000);
                for (int i = 0; i < 10000; i++)
                {
                    const int column = 7;
                    const int fovs = 3;

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
            }
            else
            {
                cal.device_type = DeviceType::LidarUnknown;
                ret = NotSupport;
            }

            return ret;
        }
        else
        {
            return OpenFileError;
        }
    }

    int LidarTools::ExportCalibrationData(CalibrationData& cal, std::string filename)
    {
        int data_in_line = 0;
        if (DeviceType::LidarML30SA1 == cal.device_type)
        {
            std::fstream outfile;
            outfile.open(filename, std::ios::out);
            data_in_line = 16;
            if (outfile.is_open())
            {
                outfile.setf(std::ios::fixed, std::ios::floatfield);
                outfile.precision(3);
                for (int i = 0; i < cal.data.size(); i++)
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
        else if (DeviceType::LidarML30B1 == cal.device_type)
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
        else
        {
            return NotSupport;
        }
    }

    void LidarTools::ComputeCalibrationSinCos(CalibrationData& cal, CalibrationDataSinCosTable& cal_cos_sin_lut)
    {
        cal_cos_sin_lut.device_type = cal.device_type;
        cal_cos_sin_lut.data.resize(cal.data.size());

        if (LidarML30B1 == cal.device_type)
        {
            for (int i = 0; i < cal.data.size() / 2; ++i)
            {
                float azi = static_cast<float>(cal.data[i * 2] / 180.0 * 3.1416);
                float ele = static_cast<float>(cal.data[i * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cosf(ele);
                point_cal.sin_ele = std::sinf(ele);
                point_cal.cos_azi = std::cosf(azi);
                point_cal.sin_azi = std::sinf(azi);
            }
        }
        else if (LidarML30SA1 == cal.device_type)
        {
            const int start = 8;
            int fov_index[start] = { 0, 6, 1, 7, 2, 4, 3, 5 };
            for (int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float ele = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float azi = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cosf(ele);
                point_cal.sin_ele = std::sinf(ele);
                point_cal.cos_azi = std::cosf(azi);
                point_cal.sin_azi = std::sinf(azi);

            }
        }
        else if (LidarMLX == cal.device_type)
        {
            const int start = 3;
            int fov_index[start] = { 2, 1, 0 };
            for (int i = 0; i < cal.data.size() / 2; ++i)
            {
                int start_number = i % start;
                int group_number = i / start;
                int point_numer = group_number * start + fov_index[start_number];
                float ele = static_cast<float>(cal.data[point_numer * 2] / 180.0 * 3.1416);
                float azi = static_cast<float>(cal.data[point_numer * 2 + 1] / 180.0 * 3.1416);

                CalibrationDataSinCos& point_cal = cal_cos_sin_lut.data[i];
                point_cal.cos_ele = std::cosf(ele);
                point_cal.sin_ele = std::sinf(ele);
                point_cal.cos_azi = std::cosf(azi);
                point_cal.sin_azi = std::sinf(azi);
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
            std::string ml30b1_sn_prefix = "10X0";
            std::string ml30sa1_sn_prefix = "10X0";

            if (0 == sn.compare(0, ml30sa1_sn_prefix.size(), ml30sa1_sn_prefix)) // ML30SA1
            {
                info.device = DeviceType::LidarML30SA1;
            }
            info.serial_number = sn;
        }
        return 0;

    }

    int LidarTools::GetDeviceCalibrationData(CalibrationData& cal)
    {
        std::string ec;
        int packets = 0;
        const int ppf = 256000; // points per frame, 256000 reserved
        const int ppk = 128; // points per cal udp packet
        std::unique_ptr<float> angle_data(new float[ppf * 2]); // points( azimuh, elevation)
        float *p = angle_data.get();
        int packet_buffer_size = 1040 * (ppf / ppk) + 4; // 128 points in one packet, buffer reserved for ppf points.
        int data_per_pkt = ppk * 4 * 2; //128 points per pkt, 4 bytes per angle, 2 angle(azimuth and elevation)
        std::unique_ptr<unsigned char> packet_data(new unsigned char[packet_buffer_size]);
        unsigned char *pcal = packet_data.get();
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

        if (DeviceType::LidarML30B1 == tp)
        {
            total_packet = 235;
        }
        else if (DeviceType::LidarML30SA1 == tp)
        {
            total_packet = 400;
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
            std::cerr << "Check calibration data error, data is all 0x00" << std::endl;
            DisConnect();
            return -1;
        }
        if (0xFF == check_all_ff)
        {
            std::cerr << "Check calibration data error, data is all 0xFF" << std::endl;
            DisConnect();
            return -1;
        }

        pkt->ExtractData(cal.data);

        for (int i = 0; i < total_packet - 1; i++)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(110));
            int ret = client_->SyncRecv(cal_recv, cal_pkt_len);
            if (ret)
            {
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

        if (DeviceType::LidarML30B1 == tp)
        {
            cal.data.resize(10000 * 3 * 2); //10000 for per sub fov, 3 fovs, 2(azimuth and elevation)
        }
        else if (DeviceType::LidarML30SA1 == tp)
        {
            cal.data.resize(6400 * 8 * 2);
        }
        else
        {

        }

        cal.device_type = tp;
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

    int LidarTools::FirmwareUpdate(std::string& filename, ProgressCallback& cb)
    {
        std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
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
        int start_percent = 10;
        int block_total = static_cast<int>(ceil(size / 256.0));
        int step_per_percent = block_total / 30;//old is 90
        int block_writed = 0;
        int arm_step = 0;

        std::ifstream idata(filename, std::ios::in | std::ios::binary);
        const int pkt_len = 256;
        char fw_data[pkt_len] = { 0x00 };
        int readed = 0;

        // data transfer
        if (idata.is_open())
        {
            for (readed = 0; readed < block_total; readed++)
            {
                idata.read(fw_data, pkt_len);
                std::string fw(fw_data, pkt_len);
                if (client_->SyncSend(fw, pkt_len))
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
            DisConnect();
            return -1;
        }

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