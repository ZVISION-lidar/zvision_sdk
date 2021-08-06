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
#include "client.h"
#include "lidar_tools.h"
#include <cstring>
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
        init_ok_(false),
        filename_(filename)
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
                if (0 != (ret = file_.fail()))
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

    int PcapReader::ClearEofBit()
    {
        if (init_ok_)
        {
            if (file_.eof())
                file_.clear();
            return 0;
        }
        else
        {
            return NotInit;
        }
    }

    PcapAnalyzer::PcapAnalyzer(std::string filename):
        filename_(filename)
    {

    }

    PcapAnalyzer::~PcapAnalyzer()
    {

    }

    int PcapAnalyzer::Analyze()
    {
        std::shared_ptr<PcapReader> reader(new PcapReader(filename_));
        int ret = 0;
        if (0 != (ret = reader->Open()))
            return ret;

        std::string data;
        int len = 0;

        // iterate all packet
        while (1)
        {
            std::streampos pos;
            if (0 != (ret = reader->GetFilePosition(pos)))
                break;

            //int ret = reader->Read(data, len);
            if (0 != (ret = reader->Read(data, len)))
            {
                if (EndOfFile == ret)
                {
                    ret = 0;
                    break;
                }
                else
                    return ret;
            }


            std::string packet = data.substr(42, len - 42);
            unsigned int ip_host_order = 0;
            u_short port = 0;

            const unsigned char* pc = (unsigned char*)data.c_str();
            NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);
            NetworkToHostShort(pc + 36 + 0, (char*)&port);
            std::string ip = IpToString(*(int*)&ip_host_order);

            if (PointCloudPacket::IsValidPacket(packet))
            {
                if (0 == PointCloudPacket::GetPacketSeq(packet))
                {
                    this->info_map_[ip].sweep_headers_.push_back(pos);
                    this->info_map_[ip].dev_cfg_.device_ip = ip;
                    this->info_map_[ip].dev_cfg_.destination_port = port;
                }
            }
            else if (CalibrationPacket::IsValidPacket(packet))
            {
                if (0 == CalibrationPacket::GetPacketSeq(packet))
                {
                    this->info_map_[ip].cal_headers_.push_back(pos);
                }
            }
            else
            {
                continue;
            }
        }

        // seek error https://stackoverflow.com/questions/16364301/whats-wrong-with-the-ifstream-seekg
        reader->ClearEofBit();
        // get calibration data packet
        for (auto& info : this->info_map_)
        {
            const std::string& name = info.first;
            DeviceDataInfo& inf = info.second;
            ScanMode tp_in_pointcloud_packet = ScanMode::ScanUnknown;
            ScanMode tp_in_calibration_packet = ScanMode::ScanUnknown;
            int packets = 0;
            unsigned int uint_ip = 0;
            if (!StringToIp(name, uint_ip))
                continue;

            // identify the pointcloud packet scan mode
            if (inf.sweep_headers_.size())
            {
                if (0 != (ret = reader->SetFilePosition(inf.sweep_headers_[0])))
                    continue;
                if (0 != (ret = reader->Read(data, len)))
                    continue;
                std::string packet = data.substr(42, len - 42);
                tp_in_pointcloud_packet = PointCloudPacket::GetScanMode(packet);
            }
            // identify the calibration packet scan mode
            if (inf.cal_headers_.size())
            {
                if (0 != (ret = reader->SetFilePosition(inf.cal_headers_[0])))
                    continue;
                if (0 != (ret = reader->Read(data, len)))
                    continue;
                std::string packet = data.substr(42, len - 42);
                //tp_in_calibration_packet = CalibrationPacket::GetScanMode(packet);
                //packets = CalibrationPacket::GetMaxSeq(tp_in_calibration_packet);
                //----------------------need a new version protocal
                tp_in_calibration_packet = tp_in_pointcloud_packet;
                packets = CalibrationPacket::GetMaxSeq(tp_in_calibration_packet) + 1;
            }
            // scan mode matched
            if ((ScanMode::ScanUnknown != tp_in_pointcloud_packet) && (tp_in_pointcloud_packet == tp_in_calibration_packet))
            {
                if (0 != (ret = reader->SetFilePosition(inf.cal_headers_[0])))
                    continue;

                std::unique_ptr<bool> s_bitmap(new bool[packets]);
                bool* bitmap = s_bitmap.get();
                std::fill(bitmap, bitmap + packets, false);
                int packet_found = 0;
                CalibrationPacketPos& cal_pos = inf.cal_;
                CalibrationPackets& cal_pkts = inf.cal_pkts_;
                cal_pos.resize(packets);
                cal_pkts.resize(packets);

                while (1)
                {
                    std::streampos pos;
                    if (0 != (ret = reader->GetFilePosition(pos)))
                        break;

                    if (0 != (ret = reader->Read(data, len)))
                        break;

                    if (packet_found == packets)
                        break;

                    std::string packet = data.substr(42, len - 42);
                    unsigned int ip_host_order = 0;
                    const unsigned char* pc = (unsigned char*)data.c_str();
                    NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);

                    if ((uint_ip == ip_host_order) && CalibrationPacket::IsValidPacket(packet))
                    {
                        int seq = CalibrationPacket::GetPacketSeq(packet);
                        if (0 <= seq)
                        {
                            if (!bitmap[seq])
                            {
                                cal_pkts[seq] = packet;
                                cal_pos[seq] = pos;
                                bitmap[seq] = true;
                                packet_found++;
                            }
                        }
                    }
                    else
                    {
                        continue;
                    }
                }

                if (packet_found != packets)
                {
                    cal_pos.clear();
                    cal_pkts.clear();
                }
            }
        }
        return 0;
    }

    const PcapAnalyzer::DeviceDataInfoMap& PcapAnalyzer::GetDetailInfo()
    {
        return this->info_map_;
    }

    SocketUdpReader::SocketUdpReader(std::string ip, int port, int time_out_ms):
        server_ip_(ip),
        local_port_(port),
        read_timeout_ms_(time_out_ms),
        receiver_(new UdpReceiver(port, time_out_ms))
    {

    }

    SocketUdpReader::~SocketUdpReader() {}

    int SocketUdpReader::Read(std::string& data, int& len, int& ep)
    {
        uint32_t u_ep = 0;
        int ret = receiver_->SyncRecv(data, len, u_ep);
        ep = *(int*)(&u_ep);
        return ret;
    }

    int SocketUdpReader::Close()
    {
        return receiver_->Close();
    }

    int SocketUdpReader::Open()
    {
        return 0;
    }

    PcapUdpSource::PcapUdpSource(std::string ip, int port, std::string filename):
        init_ok_(false),
        sender_ip_(ip),
        destination_port_(port),
        filename_(filename)
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
            if (0 != (ret = reader_->Open()))
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
            if (0 != (ret = this->Open()))
                return ret;
        }

        if ((unsigned int)frame_number >= this->position_.size())
            return InvalidParameter;

        if (0 != (ret = this->reader_->SetFilePosition(this->position_[frame_number])))
            return ret;

        std::string data;
        int len = 0;
        int last_seq = (std::numeric_limits<int>::min)();

        while (1)
        {
            if (0 != (ret = ReadOne(data, len)))
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
            if (0 != (ret = this->Open()))
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
            u_short port = 0;

            //SwapByteOrder((char*)pc + 14 + 12, (char*)&ip_network_order);//big endian
            //NetworkToHost((const unsigned char*)&ip_network_order, (char*)&ip_host_order);
            NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);
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
            if (0 != (ret = this->Open()))
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

            if (0 != (ret = reader_->GetFilePosition(pos)))
                break;

            if (0 != (ret = ReadOne(data, len)))
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
                    type = PointCloudPacket::GetDeviceType(packet);
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

    CalibrationDataSource::CalibrationDataSource(std::string ip, int port, std::string filename):
        init_ok_(false),
        sender_ip_(ip),
        destination_port_(port),
        filename_(filename)
    {

    }

    CalibrationDataSource::~CalibrationDataSource()
    {

    }

    int CalibrationDataSource::Open()
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (!StringToIp(this->sender_ip_, filter_ip_))
            {
                return InvalidParameter;
            }

            reader_.reset(new PcapReader(filename_));
            if (0 != (ret = reader_->Open()))
                return ret;
            else
                init_ok_ = true;
        }

        return ret;
    }

    int CalibrationDataSource::Close()
    {
        if (reader_)
        {
            reader_.reset();
        }
        init_ok_ = false;

        return 0;
    }

    int CalibrationDataSource::ReadOne(std::string& data, int& len)
    {
        int ret = 0;
        if(this)
        if (!init_ok_)
        {
            if (0 != (ret = this->Open()))
                return ret;
        }

        while (1)
        {
            int ret = reader_->Read(data, len);
            if (ret)
                return ret;

            if (1071 != len)// 1029 + 42 = 1071
                continue;

            const unsigned char* pc = (unsigned char*)data.c_str();
            const char* flag = data.c_str();

            /* CAL we identify the lidar calibration udp pkt by frame flag*/
            if ((flag[0] != 'C') || (flag[1] != 'A') || (flag[2] != 'L'))
            {
                continue;
            }

            unsigned int ip_host_order = 0;
            u_short port = 0;

            NetworkToHost(pc + 14 + 12, (char*)&ip_host_order);
            NetworkToHostShort(pc + 36 + 0, (char*)&port);

            //std::cout << "read: " << std::hex << ip << std::dec << " port :" << port << std::endl;
            //std::cout << "wanted: " << std::hex << this->filter_ip_ << std::dec << " port :" << this->destination_port_ << std::endl;
            if ((ip_host_order == this->filter_ip_) && (port == this->destination_port_))
                return 0;
            else
                continue;
        }
    }

    int CalibrationDataSource::GetCalibrationPackets(std::vector<CalibrationPacket>& packets)
    {
        int ret = 0;
        if (!init_ok_)
        {
            if (0 != (ret = this->Open()))
                return ret;
        }

        // https://stackoverflow.com/questions/1394132/macro-and-member-function-conflict
        int last_seq = (std::numeric_limits<int>::max)();
        bool need_find_type = true;

        const int cal_pkt_cnt = 400;
        std::vector<bool> bit_map(cal_pkt_cnt, false);
        packets.resize(cal_pkt_cnt);
        int cal_pkt_valid = 0;
        DeviceType tp = LidarUnknown;
        std::string data;
        int len = 0;
        while (cal_pkt_valid < cal_pkt_cnt)
        {
            if (0 != (ret = ReadOne(data, len)))
            {
                break;
            }

            //std::string packet = data.substr(42, len - 42);
            CalibrationPacket pkt;
            memcpy((void *)pkt.cal_data_, (void *)data.c_str(), len);
            int seq = pkt.GetPacketSeq();

            if (!bit_map[seq])
            {
                packets[seq] = pkt;
                bit_map[seq] = true;
                cal_pkt_valid++;
            }
        }
        this->reader_->Close();
        init_ok_ = false;

        if (cal_pkt_valid != cal_pkt_cnt)
            if (EndOfFile == ret)
                return NotEnoughData;
            else
                return ret;

        for (auto& p : packets)
        {
            if (DeviceType::LidarUnknown == tp)
                tp = p.GetDeviceType();
            else if (tp != p.GetDeviceType())
                return NotMatched;
            else
                ;
        }

        return ret;
    }


}
