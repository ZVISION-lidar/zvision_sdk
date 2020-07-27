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


#ifndef PACKET_SOURCE_H_
#define PACKET_SROUCE_H_
#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>

#include "define.h"

namespace zvision
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    class Reader
    {
    public:

        /** \brief zvision Reader constructor.
        */
        Reader();

        /** \brief Empty destructor */
        virtual ~Reader();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class PcapReader : public Reader
    {
    public:

        /** \brief zvision PcapReader constructor.
        * \param[in] filename        pcap filename
        */
        PcapReader(std::string filename);

        /** \brief Empty destructor */
        virtual ~PcapReader();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();

        /** \brief Calls the SetFilePosition method to move the file ptr.
        * \param[in] pos  file position to set.
        * \return 0 for success, others for failure.
        */
        int SetFilePosition(std::streampos pos);

        /** \brief Calls the GetFilePosition method to get the file position.
        * \param[in] pos  file position to set.
        * \return 0 for success, others for failure.
        */
        int GetFilePosition(std::streampos& pos);


    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool init_ok_;

        /** \brief Server ip address. */
        std::ifstream file_;

        /** \brief pcap filename. */
        std::string filename_;

    private:

        /** \brief socket which represents the socket resource. */
        int socket_;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class SocketReader : public Reader
    {
    public:

        /** \brief zvision SocketReader constructor.
        * \param[in] ip              lidar's ip address
        * \param[in] port            local port to receive data
        * \param[in] time_out_ms     timeout in ms for SyncRecv function
        */
        SocketReader(std::string ip, int port, int time_out_ms);

        /** \brief Empty destructor */
        virtual ~SocketReader();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \param[in] ep   the sender's ip address.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len, int& ep);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool is_open_;

        /** \brief Server ip address. */
        std::string server_ip_;

        /** \brief Server listening port. */
        int local_port_;

        /** \brief timeout(ms) for read.
        int read_timeout_ms_;*/

        /** \brief timeout(ms) for read. */
        int read_timeout_ms_;

    private:

        /** \brief socket which represents the socket resource. */
        int socket_;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class PcapUdpSource
    {
    public:

        /** \brief zvision PcapUdpSource constructor.
        * \param[in] port            local port to receive data
        * \param[in] recv_timeout    timeout in ms for SyncRecv function
        * \param[in] filename        pcap filename
        */
        explicit PcapUdpSource(std::string ip, int port, std::string filename);

        PcapUdpSource() = delete;

        /** \brief Empty destructor */
        virtual ~PcapUdpSource();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \return 0 for success, others for failure.
        */
        virtual int ReadOne(std::string& data, int& len);

        /** \brief Calls the GetPointCloudPackets method to get one frame pointcloud's packets from pcap file.
        * \param[in]  data     the buffer to store the data received.
        * \param[out] packets  the packets which is a full pointcloud.
        * \return 0 for success, others for failure.
        */
        int GetPointCloudPackets(int frame_number, std::vector<PointCloudPacket>& packets);

        /** \brief Calls the ReadFrameInfo method to get the frame information.
        * \param[out] size      return the frame counter in the pcap file.
        * \param[out] type      output the device type.
        * \return 0 for success, others for failure.
        */
        int ReadFrameInfo(int& size, DeviceType& type);

    protected:

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool init_ok_;

        /** \brief Server ip address. */
        std::string sender_ip_;

        /** \brief filter ip address. */
        unsigned int filter_ip_;

        /** \brief Server listening port. */
        int destination_port_;

        /** \brief timeout(ms) for read.
        int read_timeout_ms_;*/

        /** \brief Server listening port. */
        std::vector<std::streampos> position_;

        /** \brief Pcap filename. */
        std::string filename_;

    private:

        /** \brief socket which represents the udp resource. */
        std::shared_ptr<PcapReader> reader_;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////
    class SocketUdpSource
    {
    public:

        /** \brief zvision SocketUdpSource constructor.
        * \param[in] port            local port to receive data
        * \param[in] recv_timeout    timeout in ms for SyncRecv function
        */
        SocketUdpSource(int ip, int port);

        /** \brief Empty destructor */
        virtual ~SocketUdpSource();

        /** \brief Calls the Read method to reveive data from local udp port.
        * \param[in] data the buffer to store the data received.
        * \param[in] len  the length received.
        * \param[in] ep   the sender's ip address.
        * \return 0 for success, others for failure.
        */
        virtual int Read(std::string& data, int& len, int& ep);

        /** \brief Calls the Close method to close the resource.
        * \return 0 for success, others for failure.
        */
        virtual int Close();

        /** \brief Calls the Open method to Open the source.
        * \return 0 for success, others for failure.
        */
        virtual int Open();


    protected:

        /** \brief socket is initialised ok.
        *  false init error
        *  true  init ok
        */
        bool is_open_;

        /** \brief Server ip address. */
        std::string server_ip_;

        /** \brief Server listening port. */
        int local_port_;

        /** \brief timeout(ms) for read.
        int read_timeout_ms_;*/


    private:

        /** \brief socket which represents the socket resource. */
        int socket_;

    };


}

#endif // end PACKET_SROUCE_H_
