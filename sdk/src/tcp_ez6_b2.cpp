#include "tcp_ez6_b2.h"
#include "client.h"
#include "loguru.hpp"
#include "packet.h"
#include "print.h"
#include "tcp_tools.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <math.h>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>

static const int EZ6_ANGLE_COMP_LEN = 1536;

static const uint32_t CRC32Table_[] = {
        0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
        0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
        0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
        0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
        0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
        0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
        0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
        0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
        0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
        0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
        0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
        0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
        0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
        0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
        0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
        0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
        0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
        0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
        0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
        0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
        0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
        0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
        0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
        0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
        0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
        0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
        0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
        0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
        0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
        0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
        0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
        0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
        0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
        0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
        0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
        0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
        0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
        0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
        0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
        0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
        0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
        0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
        0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
        0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
        0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
        0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
        0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
        0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
        0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
        0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
        0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
        0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
        0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
        0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
        0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
        0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
        0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
        0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
        0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
        0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
        0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
        0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
        0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
        0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
    };

static uint32_t CRC_32(const uint8_t* data, size_t length)
{
    uint32_t i, crc;
    crc = 0xFFFFFFFF;

    for (i = 0; i < length; i++) {
        crc = CRC32Table_[(crc ^ data[i]) & 0xff] ^ (crc >> 8);
    }

    return crc ^ 0xFFFFFFFF;
} 

namespace zvision 
{
    /**
    *@ brief constructor, initialize TCP client
    *@ param lidar IP address of LiDAR device
    *@ param con_timeout connection timeout (in milliseconds)
    *@ paramsend_timeout timeout (in milliseconds)
    *@ param recv_timeout receive timeout (in milliseconds)
    */
    tcp_ez6_b2::tcp_ez6_b2(std::string lidar_ip, int con_timeout, int send_timeout, int recv_timeout):   
        client_(new TcpClient(con_timeout, send_timeout, recv_timeout)),
        device_ip_(lidar_ip), conn_ok_(false) {}

    tcp_ez6_b2::~tcp_ez6_b2() { this->DisConnect(); }
    /**
    *@ brief Check and establish TCP connection
    *@ return true indicates successful connection, false indicates failure
    */
    bool tcp_ez6_b2::CheckConnection() 
    {
        if (!conn_ok_) 
        {
            int ret = client_->Connect(this->device_ip_);
            if (ret) {
            client_->GetSysErrorCode();
            return false;
            }
            conn_ok_ = true;
        }

        return true;
    }
    /**
    *@ brief Disconnect TCP connection
    */
    void tcp_ez6_b2::DisConnect() 
    {
        if (conn_ok_) 
        {
            this->client_->Close();
            conn_ok_ = false;
        }
    }
    /**
    *@ brief receives TCP response
    *Response data output by  paramout
    *@ return response code, or error code (such as TcpRecvTimeout)
    */
    int tcp_ez6_b2::recv_tcp_resp(std::string& out)
    {
        int recv_header_len = 7;
        std::string recv_header(recv_header_len, 'x');
        if (client_->SyncRecv(recv_header, recv_header_len))
        {
            LOG_F(ERROR, "Receive recv header error:  %d.", client_->GetSysErrorCode());
            DisConnect();
            return TcpRecvTimeout;
        }
        uint16_t ack_len = (((uint8_t)recv_header.at(3) << 8) | (uint8_t)recv_header.at(4));
        if (ack_len == 0)
        {
            std::string recv_crc(4, 'x');
            if (client_->SyncRecv(recv_crc, 4))
            {
                LOG_F(ERROR, "Receive recv header error:  %d.", client_->GetSysErrorCode());
                DisConnect();
                return TcpRecvTimeout;
            }

            return (int)recv_header.at(6);
        }

        std::string recv_data(ack_len + 4, 'x');
        if (client_->SyncRecv(recv_data, ack_len + 4))
        {
            LOG_F(ERROR, "Receive recv header error:  %d.", client_->GetSysErrorCode());
            DisConnect();
            return TcpRecvTimeout;
        }
        out = recv_data.substr(0, ack_len);
        return (int)recv_header.at(6);
    }
    /**
    *Send TCP request @ brief
    *@ param cmd request command string
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::send_tcp_request(const std::string &cmd)
    {
        const int crc_len = 4;
        if (cmd.size() < crc_len)
        {
            DisConnect();
            return InvalidParameter;
        }

        int data_len = cmd.size() - 11;
        uint32_t chk_sum = CRC_32((uint8_t *)cmd.c_str() + 7, data_len);
        std::string cmd_str = cmd;
        cmd_str.at(cmd_str.size() - crc_len + 0) = char((chk_sum >> 24) & 0xFF);
        cmd_str.at(cmd_str.size() - crc_len + 1) = char((chk_sum >> 16) & 0xFF);
        cmd_str.at(cmd_str.size() - crc_len + 2) = char((chk_sum >> 8) & 0xFF);
        cmd_str.at(cmd_str.size() - crc_len + 3) = char((chk_sum >> 0) & 0xFF);

        if (client_->SyncSend(cmd_str, cmd_str.size()))
        {
            DisConnect();
            return TcpSendTimeout;
        }

        return 0;
    }
    /**
    *@ brief generates request package
    *@ param id Request ID
    *@ param data requests data
    *@ param cmd_type command type
    *@ param paramotype parameter type
    *@ paramout output the generated request packet
    *@ return 0 indicates success
    */
    int tcp_ez6_b2::generate_request_pkg(uint16_t id, const std::string &data, uint8_t cmd_type, uint8_t param_type, std::string &out) 
    {
        const int header_len = 7;

        // generate pkt header
        uint8_t cmd[header_len] = {0xBA, 0x00, 0x00, 0x00, 0x00, cmd_type, param_type};
        cmd[1] = ((id >> 8) & 0xFF);
        cmd[2] = (id & 0xFF);
        uint16_t len = data.size();
        cmd[3] = ((len >> 8) & 0xFF);
        cmd[4] = (len & 0xFF);
        std::string pkt(reinterpret_cast<char*>(cmd), header_len);

        // get pkt check sum
        int crc_res = CRC_32(reinterpret_cast<const uint8_t *>(data.c_str()),data.size());
        char crc_buffer[4];
        AssemblePort(crc_res,crc_buffer);
        out = pkt+data+crc_buffer;
        
        return 0;
    }
    /**
    *@ brief receives data of a specified length
    *The number of bytes that param recv-num needs to receive
    *@ param data output the received data
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::recv_specific_number_data(int recv_num,std::string &data)
    {   
        int recved_len = 0;
        while(recved_len < recv_num)
        {
            int recv_header_len = 7;
            std::string recv_header(recv_header_len, 'x');
            if (client_->SyncRecv(recv_header, recv_header_len))
            {
                LOG_F(ERROR, "Receive recv header error:  %d.", client_->GetSysErrorCode());
                DisConnect();
                return TcpRecvTimeout;
            }

            int data_len = (recv_header.at(3)<<8)|recv_header.at(4);
            std::string recv_buffer(data_len+4,'x');
            if (client_->SyncRecv(recv_buffer, data_len+4))
            {
                LOG_F(ERROR, "Receive data error:  %d.", client_->GetSysErrorCode());
                DisConnect();
                return TcpRecvTimeout;
            }
            recv_buffer.resize(data_len); 
            data += recv_buffer;
            recved_len += data_len;
        }

        return 0;
    }
    /**
    *@ brief Get PTP configuration
    *PTP configuration data output from @ paramptp_cfg
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::get_ptpcfg(std::string& ptp_cfg)
    {
        if (!CheckConnection())
        {
            return TcpConnTimeout;
        }

        const int send_len = 11;
        char cmd[send_len] = {(char)0xBA, 
                                (char)0x00, (char)0x00,
                                (char)0x00, (char)0x00, 
                                (char)0x0D, (char)0x0B, 
                                (char)0x00, (char)0x00, (char)0x00, (char)0x00};
        std::string cmd_str(cmd, send_len);
        std::string out;

        int ret = send_tcp_request(cmd_str);
        if(ret != 0)
            return ret;

        ret = recv_tcp_resp(out);
        if(ret != 0)
            return ret;

        // get ptp file data length
        if (out.size() != 4)
            return InvalidContent;

        int total_ptp_len = ntohl(*(uint32_t *)out.data());
        ret = recv_specific_number_data(total_ptp_len,ptp_cfg);

        return ret; 
    }
    /**
    *@Retrieve PTP configuration and save it to a file
    *@Aram save_filename Save file path
    *@Returning 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::get_ptpcfg_to_file(std::string &save_file_name) 
    {
        std::string content = "";
        int ret = get_ptpcfg(content);
        if (ret)
            return ret;
        else {
            std::ofstream out(save_file_name, std::ios::out | std::ios::binary);
            if (out.is_open()) {
            out.write(content.c_str(), content.size());
            out.close();
            return 0;
            } else
            return OpenFileError;
        }
    }
    /**
    *@ brief Set PTP configuration
    *@ param filename PTP configuration file path
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::set_ptpcfg(std::string filename)
    {
        if (!CheckConnection())
        {
            return TcpConnTimeout;
        }
        
        // read file
        std::string content;
        char c;
        std::ifstream inFile(filename, std::ios::in | std::ios::binary);
        if (!inFile)
        {
            LOG_F(ERROR,"open file error");
            return ReturnCode::OpenFileError;
        }
            
        while ((c = inFile.get()) && c != EOF)
        {
            content.push_back(c);
        }
        inFile.close();

        //generate cmd
        int length = content.size();
        const int send_len = 15;
        char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x00, 
                                (char)0x00, (char)0x04, 
                                (char)0x09, (char)0x02,
                                (char)0x00, (char)0x00, (char)0x00, (char)0x00,
                                (char)0x00, (char)0x00, (char)0x00, (char)0x00};
        AssemblePort(length, cmd + 7);

        // send cmd and check ret
        std::string cmd_str(cmd, send_len);
        std::string out;

        int ret = send_tcp_request(cmd_str);
        if (ret)
            return ret;
        ret = recv_tcp_resp(out);
        if (ret)
            return ret;

        // generate packet
        std::string packet;
        generate_request_pkg(0x01, content, 0, 0, packet);
        
        // send file and check ret
        ret = send_tcp_request(packet);
        if (ret)
            return ret;
        ret = recv_tcp_resp(out);

        return ret;
    }
    /**
    *@ brief Update firmware
    *@ param filename firmware file path
    *@ paramcb progress callback function
    *Does  paramisBak update the backup firmware
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::update_firmware(std::string& filename, ProgressCallback cb, bool isBak)
    {
        if (!CheckConnection())
        {
            return TcpConnTimeout;
        }

        // get file size
        std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
        if (!in.is_open())
        {
            LOG_F(ERROR, "Open file error.");
            return OpenFileError;
        }
        std::streampos end = in.tellg();
        uint32_t size = static_cast<int>(end);
        in.close();

        // generate cmd
        const int send_len = 15;
        char cmd[send_len] = { (char)0xBA, (char)0x00, (char)0x00, (char)0x00, (char)0x04, (char)0x01, (char)0x01, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };
        if (isBak)
            cmd[6] = (char)0x02;

        AssemblePort(size,cmd+7);
        std::string cmd_str(cmd, send_len);

        // send cmd and get ret, ignore out(empty)
        std::string out;

        int ret = send_tcp_request(cmd_str);
        if(ret != 0)
            return ret;

        ret = recv_tcp_resp(out);
        if(ret != 0)
            return ret;

        // transfer, erase flash, write
        const int pkt_len = 1024;
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

                // generate packet
                char header[10];
                header[0] = 0xba;
                AssemblePort_2byte(readed+1,header+1);
                AssemblePort_2byte(read_len,header+3);
                header[5] = 0x01;
                header[6] = 0x01;
                std::string packet(header,7);
                packet += fw;
                // LOG_F(INFO,"packet size  = %d",packet.size());
                
                if (client_->SyncSend(packet, packet.size()))
                {
                    break;
                }
                if (0 == (readed % step_per_percent))
                {
                    cb(start_percent++, this->device_ip_.c_str());
                }
                
                // sleep
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            idata.close();
        }
        if (readed != block_total)
        {
            client_->Close();
            return TcpSendTimeout;
        }

        // get ret
        std::string str_ret;
        ret = recv_tcp_resp(str_ret); //5
        if ( ret != 0)
        {
            DisConnect();
            return ret;
        }
        LOG_F(2, "Data transfer ok...");

        ret = recv_tcp_resp(str_ret); //5
        if (ret != 0)
        {
            DisConnect();
            return ret;
        }
        LOG_F(2, "Data transfer ok...");

        // waitting for step 2
        bool ok = false;
        while (1)
        {

            if (recv_tcp_resp(str_ret) != 0 || str_ret.size() != 1)
            {
                ok = false;
                break;
            }
            unsigned char step = *((uint8_t*)str_ret.data());
            
            cb(40 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 <= step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            LOG_F(ERROR, "Waiting for erase flash failed...");
            DisConnect();
            return TcpRecvTimeout;
        }
        LOG_F(2, "Waiting for erase flash ok...");

        // waitting for step 3
        while (1)
        {
            if (recv_tcp_resp(str_ret) != 0 || str_ret.size() != 1)
            {
                ok = false;
                break;
            }
            unsigned char step = *((uint8_t*)str_ret.data());
            cb(70 + int((double)step / 3.3), this->device_ip_.c_str());
            if (100 <= step)
            {
                ok = true;
                break;
            }
        }

        if (!ok)
        {
            LOG_F(ERROR, "Waiting for write flash failed...");
            DisConnect();
            return TcpRecvTimeout;
        }
        LOG_F(2, "Waiting for write flash ok...");

        // recv ret
        ret = recv_tcp_resp(str_ret);
        if (ret != 0)
        {
            DisConnect();
            return ret;
        }

        return 0;
    }
    /**
    *@ brief Get basic device information
    *The device configuration information output by  param info
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::get_basic_info(DeviceConfigurationInfo &info) 
    {
        if (!CheckConnection())
        {
            return TcpConnTimeout;
        }

        {
            /* get the sn */
            const int send_len = 11;
            char cmd[send_len] = { (char)0xBA, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x0D, (char)0x01, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };
            std::string cmd_str(cmd, send_len);
            int ret = send_tcp_request(cmd_str);
            if(ret != 0)
                return ret;

            std::string out;
            ret = recv_tcp_resp(out);
            if(ret != 0)
                return ret;

            info.serial_number = out.c_str();
        }
        
        {
            /* get the network info */
            const int send_len = 11;
            char cmd[send_len] = { (char)0xBA, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x0D, (char)0x03, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };
            std::string cmd_str(cmd, send_len);

            int ret = send_tcp_request(cmd_str);
            if(ret != 0)
                return ret;

            std::string out;
            ret = recv_tcp_resp(out);
            if(ret != 0)
                return ret;

            uint8_t* pdata = (uint8_t*)out.data();

            ResolveIpString(pdata + 1, info.device_ip);
            ResolveIpString(pdata + 5, info.subnet_mask);
            ResolveIpString(pdata + 9, info.gateway_addr);
            ResolveMacAddress(pdata + 13, info.device_mac);
            ResolveIpString(pdata + 19, info.destination_ip);
            info.destination_port = ntohs( * ((uint16_t*)(pdata + 23)));
        }

        {
            /* get the version */
            const int send_len = 11;
            char cmd[send_len] = { (char)0xBA, (char)0x00, (char)0x00, (char)0x00, (char)0x00, (char)0x0D, (char)0x04, (char)0x00, (char)0x00, (char)0x00, (char)0x00 };
            std::string cmd_str(cmd, send_len);

            int ret = send_tcp_request(cmd_str);
            if(ret != 0)
                return ret;

            std::string out;
            ret = recv_tcp_resp(out);
            if(ret != 0)
                return ret;

            uint8_t* pdata = (uint8_t*)out.data();

            ResolveIpString(pdata + 0, info.version.kernel_version);
            ResolveIpString(pdata + 4, info.version.boot_version);
            ResolveIpString(pdata + 8, info.backup_version.kernel_version);
            ResolveIpString(pdata + 12, info.backup_version.boot_version);
        }

        {
            info.factory_mac = "-1";
            info.frame_switch = 0xffffffff;
            info.frame_offset = 0xffffffff;
            info.work_mode = 0xffffffff;
        }

        DisConnect();
        
        return 0;
    }
    /**
    *@ brief reads angle compensation data from CSV file
    *@ param filename CSV file path
    *Angle compensation data output from paramangle_comp
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::read_comp_from_csv(const std::string &filename, angle_comp_t &angle_comp)
    {
        std::ifstream file(filename);
        if (!file.is_open()) {
            LOG_F(ERROR,"open angle comp file error");
            return OpenFileError;
        }

        std::string line;

        while (std::getline(file, line)) 
        {
            std::istringstream ss(line);
            std::string token;

            if (std::getline(ss, token, ',')) {
                angle_comp.azi.push_back(std::stof(token));
            } else {
                LOG_F(ERROR,"parse azi error");
                continue;
            }

            if (std::getline(ss, token, ',')) {
                angle_comp.ele.push_back(std::stof(token));
            } else {
                LOG_F(ERROR,"parse ele error");
                continue;
            }
        }

        return 0;
    }
    /**
    *@ brief Get angle compensation data
    *Angle compensation data output from  paramangle_comp
    *@ return 0 indicates success, non-zero indicates error code
    */
    int tcp_ez6_b2::get_angle_comp(angle_comp_t &angle_comp) 
    {
        if (!CheckConnection())
        {
            int ret = read_comp_from_csv("EZ6_B2_192_angle_comp.csv",angle_comp);
            if(ret != 0)
            {
                return TcpConnTimeout;
            }
            else
            {
                LOG_F(INFO,"using the default comp files");

                return 0;
            }
        }

        const int send_len = 11;

        char cmd[send_len] = {(char)0xBA, (char)0x00, (char)0x00, (char)0x00,
                                (char)0x00, (char)0x0B, (char)0x01, (char)0x00,
                                (char)0x00, (char)0x00, (char)0x00};
        std::string cmd_str(cmd, send_len);

        client_->SyncSend(cmd_str, cmd_str.size());

        uint32_t totalRecvLen = 0;
        char charRecv[7] = {0};
        client_->SyncRecv(charRecv, 7);

        if ((((int8_t)charRecv[5]) == (int8_t)0xAC) && (((int8_t)charRecv[6]) == (int8_t)0x00)) 
        {
            uint16_t recvLen = ntohs(*((uint16_t *)(charRecv + 3)) & 0xFFFF);

            char charRecvLen[8] = {0};
            client_->SyncRecv(charRecvLen, 8);
            if (recvLen == 4) 
            {
                totalRecvLen = (charRecvLen[0] << 24) | (charRecvLen[1] << 16) | (charRecvLen[2] << 8) | (charRecvLen[3]);
            }
        }

        std::string totalRecv;

        while (totalRecv.size() < totalRecvLen) 
        {
            char charRecv[7] = {0};
            client_->SyncRecv(charRecv, 7);

            if ((((int8_t)charRecv[5]) == (int8_t)0xAC) && (((int8_t)charRecv[6]) == (int8_t)0x00)) 
            {
                uint16_t recvLen = ntohs(*((uint16_t *)(charRecv + 3)) & 0xFFFF);
                char charRecvAll[1040] = {0};
                client_->SyncRecv(charRecvAll, recvLen + 4);

                std::string strallData(charRecvAll, recvLen);

                totalRecv += strallData;
            }
        }
        
        client_->Close();

        if (totalRecv.size() ==  EZ6_ANGLE_COMP_LEN)
        {
            uint64_t pos = 0;
            float angle;

            uint8_t *data_ = (uint8_t *)totalRecv.c_str();
            while (pos < totalRecv.size()) 
            {
                angle = get_float(data_+pos);
                angle_comp.azi.push_back(angle);
                pos += 4;

                angle = get_float(data_+pos);
                angle_comp.ele.push_back(angle);
                pos += 4;
            } 

            LOG_F(INFO, "Get angle comp ok.");
            return 0;
        }

        return Unknown;
    }
}




