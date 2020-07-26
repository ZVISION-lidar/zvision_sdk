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


#include "packet.h"
#include "packet_source.h"
#include "tcp_client.h"
#include "lidar_tools.h"
#include <limits>
#include <iostream>

#ifdef WIN32

#else

#endif
namespace zvision
{
    Reader::Reader() {}

    Reader::~Reader() {}

    int Reader::Open()
    {
        return 0;
    }

    int Reader::Close()
    {
        return 0;
    }

    int Reader::Read(std::string& data, int& len)
    {
        return 0;
    }

    PcapReader::PcapReader(std::string filename):
        filename_(filename),
        init_ok_(false)
    {

    }

    PcapReader::~PcapReader()
    {
        this->Close();
    }

    int PcapReader::Close()
    {
        if (file_.is_open())
            file_.close();

        this->init_ok_ = false;
        return 0;
    }

    int PcapReader::Open()
    {
        const int PCAP_HEADER_LEN = 24; /*pcap fileheaser 24 bytes*/
        char buffer[PCAP_HEADER_LEN];
        int ret = 0;

        if (!init_ok_)
        {
            file_.open(this->filename_, std::ios::in | std::ios::binary);
            if (file_.is_open())
            {
                file_.read(buffer, PCAP_HEADER_LEN); /*20 bytes pcap file header*/
                if (ret = file_.fail())
                {
                    init_ok_ = false;
                    return ret;
                }
                else
                {
                    init_ok_ = true;
                    return 0;
                }
            }
            else
            {
                return OpenFileError;
            }
        }
        else
        {
            return 0;
        }
    }

    int PcapReader::Read(std::string& data, int& len)
    {
        char buffer[256];
        unsigned int cap_len = 0;

        data.resize(2048);
        char* data_out = const_cast<char*>(data.c_str());

        const int PCAP_PKT_HEADER_LEN = 16; /*pcap header 16 bytes for every IP packet*/
        if (!init_ok_)
        {
            return InitFailure;
        }

        file_.read(buffer, PCAP_PKT_HEADER_LEN); /*16 bytes pcap packet header*/
        if (file_.fail())
        {
            if (file_.eof())
                return EndOfFile;
            else
                return ReadFileError;
        }

        unsigned int cap_len_network_byte_order = 0;
        SwapByteOrder(buffer + 8, (char*)&cap_len_network_byte_order);
        NetworkToHost((const unsigned char*)&cap_len_network_byte_order, (char*)&cap_len);

        file_.read(data_out, cap_len); /*16 bytes pcap packet header*/

        if (file_.fail())
        {
            return ReadFileError;
        }

        len = cap_len;
        return 0;
    }

    int PcapReader::SetFilePosition(std::streampos pos)
    {
        if (init_ok_)
        {
            file_.seekg(pos, file_.beg);

            if (file_.fail())
                return file_.exceptions();
            else
                return 0;
        }
        else
        {
            return NotInit;
        }
    }

    int PcapReader::GetFilePosition(std::streampos& pos)
    {
        if (init_ok_)
        {
            pos = file_.tellg();
            return 0;
        }
        else
        {
            return NotInit;
        }
    }


    PcapUdpSource::PcapUdpSource(std::string ip, int port, std::string filename):
        sender_ip_(ip),
        destination_port_(port),
        filename_(filename),
        init_ok_(false)
    {

    }

    PcapUdpSource::~PcapUdpSource()
    {
        this->Close();
    }

    int PcapUdpSource::Open()
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (!StringToIp(this->sender_ip_, filter_ip_))
            {
                return InvalidParameter;
            }

            reader_.reset(new PcapReader(filename_));
            if (ret = reader_->Open())
                return ret;
            else
                init_ok_ = true;
        }

        return ret;
    }

    int PcapUdpSource::GetPointCloudPackets(int frame_number, std::vector<PointCloudPacket>& packets)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (ret = this->Open())
                return ret;
        }

        if (frame_number >= this->position_.size())
            return InvalidParameter;

        if (ret = this->reader_->SetFilePosition(this->position_[frame_number]))
            return ret;

        std::string data;
        int len = 0;
        int last_seq = (std::numeric_limits<int>::min)();

        while (1)
        {
            if (ret = ReadOne(data, len))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                }
                break;
            }

            std::string packet = data.substr(42, len - 42);
            int seq = PointCloudPacket::GetPacketSeq(packet);

            if (seq <= last_seq)// new frame
            {
                //std::cout << "header seq:" << seq << std::endl;
                break;
            }
            else
            {
                PointCloudPacket pkt;
                memcpy(pkt.data_, packet.c_str(), packet.size());
                packets.push_back(pkt);
                last_seq = seq;
                //std::cout << "not header seq:, " << seq  << " size is " << packet.size() << std::endl;
            }
        }

        return ret;

    }

    int PcapUdpSource::Close()
    {
        if (reader_)
        {
            reader_.reset();
        }
        init_ok_ = false;

        return 0;
    }

    int PcapUdpSource::ReadOne(std::string& data, int& len)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (ret = this->Open())
                return ret;
        }

        while (1)
        {
            int ret = reader_->Read(data, len);
            if (ret)
                return ret;

            if (1346 != len)
                continue;

            const unsigned char* pc = (unsigned char*)data.c_str();
            uint16_t flag = 0x0000;
            flag = *(uint16_t *)(pc + 42);

            /*0xAAAA 0xBBBB 0xCCCC we identify the lidar udp pkt by frame flag*/
            if (!((flag == 0xAAAA) || (flag == 0xBBBB) || (flag == 0xCCCC)))
            {
                continue;
            }

            unsigned int ip_host_order = 0;
            unsigned int ip_network_order = 0;
            u_short port = 0;

            SwapByteOrder((char*)pc + 14 + 12, (char*)&ip_network_order);//big endian
            NetworkToHost((const unsigned char*)&ip_network_order, (char*)&ip_host_order);
            NetworkToHostShort(pc + 36 +  0, (char*)&port);
            
            //std::cout << "read: " << std::hex << ip << std::dec << " port :" << port << std::endl;
            //std::cout << "wanted: " << std::hex << this->filter_ip_ << std::dec << " port :" << this->destination_port_ << std::endl;
            if ((ip_host_order == this->filter_ip_) && (port == this->destination_port_))
                return 0;
            else
                continue;
        }
    }

    int PcapUdpSource::ReadFrameInfo(int& size, DeviceType& type)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (ret = this->Open())
                return ret;
        }
        
        // https://stackoverflow.com/questions/1394132/macro-and-member-function-conflict
        int last_seq = (std::numeric_limits<int>::max)();
        bool need_find_type = true;
        size = 0;
        type = LidarUnknown;
        while (1)
        {
            std::string data;
            int len = 0;
            std::streampos pos = 0;

            if (ret = reader_->GetFilePosition(pos))
                break;

            if (ret = ReadOne(data, len))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                    break;
                }
                else
                    break;
            }

            std::string packet = data.substr(42, len - 42);
            int seq = PointCloudPacket::GetPacketSeq(packet);
            if (seq <= last_seq)// new frame
            {
                if (need_find_type)
                {
                    type = PointCloudPacket::GetDeviceType(data);
                    need_find_type = false;
                }
                this->position_.push_back(pos);
                last_seq = seq;
                //std::cout << "header seq:" << seq << std::endl;
            }
            else
            {
                last_seq = seq;
                //std::cout << "not header seq:" << seq << std::endl;
            }
        }
        size = static_cast<int>(this->position_.size());
        this->reader_->Close();
        init_ok_ = false;
        return ret;

    }

}