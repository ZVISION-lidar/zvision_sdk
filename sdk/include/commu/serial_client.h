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


#ifndef SERIAL_CLIENT_H_
#define SERIAL_CLIENT_H_

#include <string>

#include "define.h"

namespace zvision
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief SerialClient is a serial transport wrapper for Linux / Windows.
      * \author zvision
      *
      * The transport is split into two roles opened by Connect:
      *   - send role (commands, usually 9600 baud) -> used to send "$LDCMD" etc.
      *   - recv role (point cloud, custom baud such as 3125000) -> point cloud frames
      *     and "$LDACK" replies.
      *
      * A single full-duplex UART fits the same model: pass the same device path to
      * Connect twice and commands/responses go over the shared line. The class
      * intentionally mirrors the TcpClient usage style (Connect/SyncSend/SyncRecv/
      * Close) so it can be dropped into the PointCloudProducer the same way the
      * UDP receiver is used.
      */
    class SerialClient
    {
    public:
        /** \brief zvision SerialClient constructor.
          * \param[in] send_timeout timeout in ms for SyncSend function
          * \param[in] recv_timeout timeout in ms for SyncRecv function
          */
        SerialClient(int send_timeout = 1000, int recv_timeout = 100);

        /** \brief Empty destructor */
        virtual ~SerialClient();

        /** \brief Opens both serial roles.
          * \param[in] port_send the command serial device/COM name
          * \param[in] port_recv the point cloud serial device/COM name
          * \param[in] baud_send baud rate of the command port
          * \param[in] baud_recv baud rate of the point cloud port
          * \return 0 for success, others for failure.
          */
        int Connect(std::string port_send, std::string port_recv, int baud_send = 9600, int baud_recv = 3125000);

        /** \brief Calls the SyncSend method to send data on the command port.
        * \param[in] data the data to send
        * \param[in] len  the length of data
        * \return 0 for success, others for failure.
        */
        int SyncSend(std::string& data, int len);

        /** \brief Calls the SyncRecv method to receive data from the point cloud port.
          * \param[out] data buffer that stores the received bytes
          * \param[out] len  number of bytes actually received (0 when idle timeout)
          * \return 0 for success (including idle timeout), others for failure.
          */
        int SyncRecv(std::string& data, int& len);

        /** \brief Discard pending input bytes on the point cloud port.
          * \return 0 for success, others for failure.
          */
        int FlushRecvInput();

        /** \brief Calls the Close method to close both serial ports.
          * \return 0 for success, others for failure.
          */
        int Close();

        /** \brief Connection is established or not.
        * \return true connection is ok, false connection error.
        */
        bool isOpen() const { return conn_ok_; }

    private:

        /** \brief timeout(ms) for send. */
        int send_timeout_ms_;

        /** \brief timeout(ms) for recv. */
        int recv_timeout_ms_;

        /** \brief command port name. */
        std::string port_send_;

        /** \brief point cloud port name. */
        std::string port_recv_;

        /** \brief both ports opened ok.
        * false error
        * true  ok
        */
        bool conn_ok_;

#ifdef _WIN32
        /** \brief serial handles on Windows. */
        void* h_send_;
        void* h_recv_;
#else
        /** \brief serial file descriptors on Linux. */
        int fd_send_;
        int fd_recv_;
#endif

    };
}

#endif // end SERIAL_CLIENT_H_
