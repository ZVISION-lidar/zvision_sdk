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


//reference: https://gist.github.com/FedericoPonzi/2a37799b6c601cce6c1b
//cross platform https://stackoverflow.com/questions/28027937/cross-platform-sockets

#ifdef WIN32
/* See http://stackoverflow.com/questions/12765743/getaddrinfo-on-win32 */
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#ifndef WIN32_WINNT
#define WIN32_WINNT 0x0501  /* Windows XP. */
#endif
#include <winsock2.h>
#include <Ws2tcpip.h>
#else
/* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */
#endif

#if defined WIN32
#include <windows.h>
#include <winsock.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library

#else
#define closesocket close
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#endif
#include "print.h"
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#include <cstdarg>

#include "client.h"

#define BUFFERSIZE 2048
#define PROTOPORT 5193 // Default port number
#define LOG_BUFFER_SIZE 512

#define IN
#define OUT
#define INOUT

#ifdef WIN32

typedef int	            socklen_t;

#else

#define INVALID_SOCKET	-1
#define SOCKET_ERROR	-1
typedef int				SOCKET;

#endif

namespace zvision
{
    bool AssembleIpString(std::string ip, char* addr)
    {
        try
        {
            int value = inet_addr(ip.c_str());
            memcpy(addr, &value, sizeof(value));
        }
        catch (std::exception e)
        {
            return false;
        }
        return true;
    }

    void AssemblePort(int port, char* addr)
    {
        int value = htonl(port);
        memcpy(addr, &value, sizeof(value));
    }

    bool AssembleMacAddress(std::string mac, char* addr)
    {
        return (6 == sscanf_s(mac.c_str(), "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx", &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]));
    }

    void ResolveIpString(const unsigned char* addr, std::string& ip)
    {
        char cip[128] = "";
        sprintf_s(cip, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
        ip = std::string(cip);
    }

    void ResolvePort(const unsigned char* addr, int& port)
    {
        int* old = (int*)(addr);
        port = ntohl(*old);
    }

    void ResolveMacAddress(const unsigned char* addr, std::string& mac)
    {
        char cmac[128] = "";
        sprintf_s(cmac, "%02X-%02X-%02X-%02X-%02X-%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        mac = std::string(cmac);
    }

    bool StringToIp(std::string ip, unsigned int& iip)
    {
        try
        {
            unsigned int net_byte_order = inet_addr(ip.c_str());
            NetworkToHost((const unsigned char*)&net_byte_order, (char*)&iip);
        }
        catch (std::exception e)
        {
            return false;
        }
        return true;
    }

    std::string IpToString(int ip)
    {
        char cip[128] = "";
        uint32_t network_order = htonl(ip);
        unsigned char* addr = (unsigned char*)(&network_order);
        sprintf_s(cip, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
        return std::string(cip);
    }

    void NetworkToHost(const unsigned char* net, char* host, int len)
    {
        for (int i = 0; i < len / 4; i++)
        {
            int ori = *(int*)(net + 4 * i);
            int* now = (int*)(host + 4 * i);
            *now = ntohl(ori);
        }
    }

    void NetworkToHostShort(const unsigned char* net, char* host, int len)
    {
        for (int i = 0; i < len / 2; i++)
        {
            u_short ori = *(u_short*)(net + 2 * i);
            u_short* now = (u_short*)(host + 2 * i);
            *now = ntohs(ori);
        }
    }

    void HostToNetwork(const unsigned char* host, char* net, int len)
    {
        for (int i = 0; i < len / 4; i++)
        {
            int ori = *(int*)(host + 4 * i);
            int* now = (int*)(net + 4 * i);
            *now = htonl(ori);
        }
    }

    void SwapByteOrder(char* src, char* dst, int len)
    {
        for (int i = 0; i < len / 4; i++)
        {
            for(int j = 0; j < 4; j++)
                dst[i * 4 + j] = src[i * 4 + (3 - j)];
        }
    }

    int GetSysErrorCode()
    {
        int err = 0;
#ifdef WIN32
        err = WSAGetLastError();
#else
        err = errno;
#endif

        return err;
    }

    class Log
    {

    public:
        static void Info(const std::string& format, ...)
        {
            ;
        }

        static void Warning(const std::string& format, ...)
        {
            ;
        }

        static void Error(const std::string& format, ...)
        {
            ;
        }

    private:
        void Error(const std::string& format, va_list args)
        {
            printf(format.c_str(), args);
        }
    };

    #if 0
    static std::string ErrorString(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        char buffer[LOG_BUFFER_SIZE];
        va_list argscopy;
        va_copy(argscopy, args);

        size_t newlen = vsnprintf(&buffer[0], LOG_BUFFER_SIZE - 1, format, args);
        if (newlen > (LOG_BUFFER_SIZE - 1))
        {
            buffer[LOG_BUFFER_SIZE - 1] = '\0';
        }
        else
        {
            buffer[newlen] = '\0';
        }
        va_end(args);

        return std::string(buffer);
    }
    #endif
#if 0
    static void PrintLog(const char* format, ...)
    {

#if 1
        va_list args;
        va_start(args, format);
        char buffer[LOG_BUFFER_SIZE];
        va_list argscopy;
        va_copy(argscopy, args);

        size_t newlen = vsnprintf(&buffer[0], LOG_BUFFER_SIZE - 1, format, args);
        if (newlen > (LOG_BUFFER_SIZE - 1))
        {
            buffer[LOG_BUFFER_SIZE - 1] = '\0';
        }
        else
        {
            buffer[newlen] = '\0';
        }
        va_end(args);
        printf("%s", buffer);
#endif
    }
#endif

    static void PrintError(const char *msg)
    {
#ifdef WIN32
        printf("%s: %d\n", msg, WSAGetLastError());
#else
        perror(msg);
#endif
    }



    //////////////////////////////////////////////////////////////////////////////////////////////
    Env::Env():
        socket_env_status_((int)ReturnCode::InitFailure)
    {

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    bool Env::Ok()
    {
        static Env env;
        if ((int)ReturnCode::InitSuccess != env.socket_env_status_)
        {
#if defined WIN32
            // Windows socket service init, necessary for windows platform
            WSADATA wsaData;
            int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
            if (iResult != 0)
            {
                LOG_ERROR("Error at WSASturtup: ret = %d\n", iResult);
                env.socket_env_status_ = (int)ReturnCode::InitFailure;
            }
            else
            {
                env.socket_env_status_ = (int)ReturnCode::InitSuccess;
            }
#else
            env.socket_env_status_ = (int)ReturnCode::InitSuccess;
#endif
        }
        return ((int)ReturnCode::InitSuccess == env.socket_env_status_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    TcpClient::TcpClient(int connect_timeout, int send_timeout, int recv_timeout) :
        conn_timeout_ms_(connect_timeout),
        send_timeout_ms_(send_timeout),
        recv_timeout_ms_(recv_timeout),
        socket_(INVALID_SOCKET),
        conn_ok_(false),
        error_code_(0),
        error_str_("")
    {
        
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    TcpClient::~TcpClient()
    {
        if (conn_ok_)
            Close();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int TcpClient::Connect(std::string dst_ip, int dst_port)
    {
        if (!Env::Ok())
            return -1;

        LOG_DEBUG("Connect to %s:%d.\n", dst_ip.c_str(), dst_port);
        // Socket creation
        this->socket_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (this->socket_ == INVALID_SOCKET)
        {
            LOG_ERROR("Socket creation failed.\n");
            return -1;
        }

        // Server address construction
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET; //internet
        addr.sin_addr.s_addr = inet_addr(dst_ip.c_str());// server IP
        addr.sin_port = htons(dst_port); // Server port
#ifdef WIN32

        // set socket non-blocking mode...
        u_long block = 1;
        if (ioctlsocket(this->socket_, FIONBIO, &block) == SOCKET_ERROR)
        {
            LOG_ERROR("Set nonblock error, error code = %d.\n", GetSysErrorCode());
            Close();
            return -1;
        }

        struct timeval time_out = { 0 };
        time_out.tv_sec = this->conn_timeout_ms_ / 1000;
        time_out.tv_usec = (this->conn_timeout_ms_ % 1000) * 1000;

        if (connect(this->socket_, (struct sockaddr *) &addr, sizeof(addr)) < 0)
        {
            int ret = GetSysErrorCode();
            if (ret != WSAEWOULDBLOCK)
            {
                LOG_ERROR("Connect error, error code = %d.\n", ret);
                Close();
                return -1;
            }

            // connection pending
            fd_set setW, setE;

            FD_ZERO(&setW);
            FD_SET(this->socket_, &setW);
            FD_ZERO(&setE);
            FD_SET(this->socket_, &setE);

            LOG_DEBUG("Set connect timeout to %d seconds %d usec.\n", time_out.tv_sec, time_out.tv_usec);
            ret = select(0, NULL, &setW, &setE, &time_out);
            if (ret <= 0)
            {
                // https://docs.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-select
                if(0 == ret)
                    LOG_ERROR("Connect timeout.\n");
                else
                    LOG_ERROR("Select error, ret = %d error code = %d.\n", ret, this->GetSysErrorCode());
                Close();
                return -1;
            }

            ret = FD_ISSET(this->socket_, &setE);
            if (ret)
            {
                // connection failed
                LOG_ERROR("FD_ISSET error: ret = %d, error code = %d.\n", ret, this->GetSysErrorCode());
                Close();
                return -1;
            }
        }

        // put socked in blocking mode...
        block = 0;
        // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-ioctlsocket
        if (ioctlsocket(this->socket_, FIONBIO, &block) == SOCKET_ERROR)
        {
            LOG_ERROR("Set block mode error, error code = %d.\n",this->GetSysErrorCode());
            Close();
            return -1;
        }

        // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-setsockopt
        int timeout = this->send_timeout_ms_;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout)))
        {
            LOG_ERROR("Set send timeout error, error code = %d.\n", this->GetSysErrorCode());
            Close();
            return -1;
        }
        else
        {
            LOG_DEBUG("Set send timeout ok, %d ms\n", timeout);
        }

        timeout = this->recv_timeout_ms_;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)))
        {
            LOG_ERROR("Set receive timeout error, error code = %d.\n ", timeout);
            Close();
            return -1;
        }
        else
        {
            LOG_DEBUG("Set receive timeout ok, %d ms\n", timeout);
        }

        return 0;

#else
        int ret = 0;
        addr.sin_addr.s_addr = inet_addr(dst_ip.c_str());// lidar IP
        int timeout = this->send_timeout_ms_;
        struct timeval time_out = { 0 };
        time_out.tv_sec = this->conn_timeout_ms_ / 1000;
        time_out.tv_usec = (this->conn_timeout_ms_ % 1000) * 1000;

        if(0 > (ret = fcntl(this->socket_, F_SETFL, O_NONBLOCK)))
        {
            LOG_ERROR("Set non-block mode error, error code = %d.\n", GetSysErrorCode());
            Close();
            return -1;
        }

        if(0 > (ret = connect(this->socket_, (struct sockaddr *)&addr, sizeof(addr)))) // connect
        {
            int err = GetSysErrorCode();
            if((EINPROGRESS == err) || (EWOULDBLOCK == err))
            {
                fd_set fdset;
                FD_ZERO(&fdset);
                FD_SET(this->socket_, &fdset);

                if (select(this->socket_ + 1, NULL, &fdset, NULL, &time_out) == 1)
                {
                    int so_error;
                    socklen_t len = sizeof so_error;

                    getsockopt(this->socket_, SOL_SOCKET, SO_ERROR, &so_error, &len);

                    if (so_error == 0)
                    {
                        ;
                    }
                    else
                    {
                        LOG_ERROR("Connect error, error code = %d\n", so_error);
                        Close();
                        return -1;
                    }
                }
                else
                {
                    LOG_ERROR("Connect error, error code = %d.\n", this->GetSysErrorCode());
                    Close();
                    return -1;
                }
            }
            else
            {
                LOG_ERROR("Connect error, error code = %d.\n", err);
                Close();
                return -1;
            }


        }

        LOG_DEBUG("Connect to %s ok.\n", dst_ip.c_str());;

        int flag = fcntl(this->socket_, F_GETFL, 0);
        if(0 > (ret = fcntl(this->socket_, F_SETFL, flag & ~O_NONBLOCK)))
        {
            LOG_ERROR("Set block mode error, error code = %d.\n", GetSysErrorCode());
            Close();
            return -1;
        }

        time_out.tv_sec = this->send_timeout_ms_ / 1000;
        time_out.tv_usec = (this->send_timeout_ms_ % 1000) * 1000;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&time_out, sizeof(time_out)))
        {
            LOG_ERROR("Set send timeout error, error code = %d.\n", this->GetSysErrorCode());
            Close();
            return -1;
        }
        else
        {
            LOG_DEBUG("Set send timeout ok, %d ms\n", timeout);
        }

        time_out.tv_sec = this->recv_timeout_ms_ / 1000;
        time_out.tv_usec = (this->recv_timeout_ms_ % 1000) * 1000;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&time_out, sizeof(time_out)))
        {
            LOG_ERROR("Set receive timeout error, error code = %d.\n ", timeout);
            Close();
            return -1;
        }
        else
        {
            LOG_DEBUG("Set receive timeout ok, %d ms\n", timeout);
        }
        return 0;
#endif


    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int TcpClient::SyncSend(std::string& data, int len)
    {
        int ret;
        int count = 0;
        int flags = 0;

        const char *buf = reinterpret_cast<const char*>(data.c_str());
        do
        {
            //windows: https://docs.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-send
            ret = send(this->socket_, buf + count, len - count, flags);
            if (SOCKET_ERROR == ret)
            {
                LOG_ERROR("send error, return value is %d", GetSysErrorCode());
                return -1;
            }
            else if (0 == ret)
            {
                LOG_ERROR("The connection has been gracefully closed, send return");
                return -1;
            }

            count += ret;
        } while (count < len);

        return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int TcpClient::SyncRecv(std::string& data, int len)
    {
        int flags = 0;
        char *buf = const_cast<char*>(data.c_str());
        char *move = buf;
        unsigned int total_read = 0;
        int need_read = len;
        while(1)
        {
            //Windows: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-recv
            move = buf + total_read;

            int ret = recv(this->socket_, move, need_read, flags);
            if (SOCKET_ERROR == ret)
            {
                LOG_ERROR("Recv error, value is %d", GetSysErrorCode());
                return -1;
            }
            else if (0 == ret)
            {
                LOG_ERROR("The connection has been gracefully closed, recv return");
                return -2;
            }
            else
            {
                total_read += ret;
                if ((len >= 0) && ((unsigned int)len <= total_read))
                    return 0;
                else
                {
                    need_read = len - total_read;
                    continue;
                }
            }
        }

    }

    int TcpClient::SyncRecv(char* data, int len)
    {
        int flags = 0;
        char *buf = data;
        unsigned int total_read = 0;
        int need_read = len;
        while (1)
        {
            //Windows: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-recv
            buf = data + total_read;

            int ret = recv(this->socket_, buf, need_read, flags);
            if (SOCKET_ERROR == ret)
            {
                LOG_ERROR("Recv error, value is %d", GetSysErrorCode());
                return -1;
            }
            else if (0 == ret)
            {
                LOG_ERROR("The connection has been gracefully closed, recv return");
                return -2;
            }
            else
            {
                total_read += ret;
                if ((len >= 0) && ((unsigned int)len <= total_read))
                    return 0;
                else
                {
                    need_read = len - total_read;
                    continue;
                }
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int TcpClient::Close()
    {
        int status = 0;
        if (INVALID_SOCKET == this->socket_)
        {
            return 0;
        }
#ifdef WIN32
        status = shutdown(this->socket_, SD_BOTH);
        if (status == 0) { status = closesocket(this->socket_); }
        else
           status = closesocket(this->socket_);
#else
        status = shutdown(this->socket_, SHUT_RDWR);
        if (status == 0) { status = close(this->socket_); }
        else
            status = closesocket(this->socket_);
#endif
        this->socket_ = INVALID_SOCKET;
        return status;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int TcpClient::GetSysErrorCode()
    {
        int err = 0;
#ifdef WIN32
        err = WSAGetLastError();
#else
        err = errno;
#endif

        return err;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    std::string TcpClient::RecordErrorInfo()
    {
        int err = 0;
#ifdef WIN32
        err = WSAGetLastError();
#else
        err = errno;
#endif
        err+=1;
        return std::string("");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    UdpReceiver::UdpReceiver(int port, int recv_timeout):
        local_port_(port),
        recv_timeout_ms_(recv_timeout),
        socket_(INVALID_SOCKET),
        init_ok_(false)
    {

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    UdpReceiver::~UdpReceiver()
    {
        Close();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int UdpReceiver::JoinMulticastGroup(std::string& mtip)
    {
        multicast_ip_ = mtip;
        return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int UdpReceiver::Bind()
    {
        if (!Env::Ok())
            return InitFailure;

        if (!init_ok_)
        {
            // Socket creation
            this->socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (INVALID_SOCKET == this->socket_)
            {
                PrintError("Socket creation failed.\n");
                return -1;
            }

            // Server address construction
            struct sockaddr_in addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET; //internet
            addr.sin_port = htons(local_port_); // local port
            addr.sin_addr.s_addr = htonl(INADDR_ANY);// local ip

            // set socket blocking mode...
            #ifdef WIN32
            u_long block = 0;
            if (ioctlsocket(this->socket_, FIONBIO, &block) == SOCKET_ERROR)
            {
                PrintError("Set nonblock error ");
                Close();
                return -1;
            }
            
            #else
            #if 1
            int flags = fcntl(this->socket_, F_GETFL);
            if(0 > fcntl(this->socket_, flags & ~O_NONBLOCK))
            {
                LOG_ERROR("Set block error, error code  %d.\n", GetSysErrorCode());
                Close();
                return -1;
            }
            #endif
            #endif

            int reuseaddr = 1;
            if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuseaddr, sizeof(reuseaddr)))
            {
                LOG_ERROR("Set reuse addr error, error code = %d.\n", GetSysErrorCode());
                Close();
                return -1;
            }
            else
            {
                ;
            }

            //set timeout
            #ifdef WIN32
            int recv_timeout = this->recv_timeout_ms_;
            #else
            struct timeval recv_timeout = { 0 };
            recv_timeout.tv_sec = this->recv_timeout_ms_ / 1000;
            recv_timeout.tv_usec = (this->recv_timeout_ms_ % 1000) * 1000;
            #endif
            // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-setsockopt
            if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&recv_timeout, sizeof(recv_timeout)))
            {
                LOG_ERROR("Set receive timeout %d ms error, error code = %d.\n", this->recv_timeout_ms_, GetSysErrorCode());
                Close();
                return -1;
            }
            else
            {
                LOG_DEBUG("Set receive timeout ok, %d ms.\n", this->recv_timeout_ms_);
            }

            int recv_buffer_size = 1024 * 1000;
            if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, (const char *)&recv_buffer_size, sizeof(recv_buffer_size)))
            {
                LOG_ERROR("Set receive buffer size %d byte(s) error, error code = %d.\n", recv_buffer_size, GetSysErrorCode());
                Close();
                return -1;
            }
            else
            {
                LOG_DEBUG("Set receive buffer size ok, %d byte(s).\n", recv_buffer_size);
            }

            // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-bind
            if (bind(this->socket_, (struct sockaddr*)&addr, sizeof(addr)))
            {
                LOG_ERROR("Bind error, error code = %d.\n", GetSysErrorCode());
                Close();
                return -1;
            }

            // multicast group
            if (multicast_ip_.size())
            {
                struct ip_mreq mreq;
                mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_.c_str());
                mreq.imr_interface.s_addr = htonl(INADDR_ANY);
                if (0 != setsockopt(this->socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)))
                {
                    // "error 19: No such device.", https://stackoverflow.com/questions/3187919/error-no-such-device-in-call-setsockopt-when-joining-multicast-group
                    LOG_ERROR("Join multicast group %s error, error code = %d.\n", multicast_ip_.c_str(), GetSysErrorCode());
                    Close();
                    return -1;
                }
            }

            init_ok_ = true;
        }
        return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int UdpReceiver::SyncRecv(std::string& data, int& len, uint32_t& ip)
    {
        len = 0;
        if (!Env::Ok())
            return InitFailure;

        if (!init_ok_)
        {
            int ret = 0;
            if(0 != (ret = this->Bind()))
                LOG_ERROR("Bind error.\n");
        }

        if(init_ok_)
        {
            int flags = 0;
            const int buffer_len = 2048;
            data = std::string(buffer_len, '0');
            char *buf = const_cast<char*>(data.c_str());

            struct sockaddr_in sender_addr;
            int sender_addr_size = sizeof(sender_addr);

            //Windows: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-recv
            #ifdef WIN32
            int ret = recvfrom(this->socket_, buf, buffer_len, flags, (sockaddr *)&sender_addr, &sender_addr_size);
            #else
            int ret = recvfrom(this->socket_, buf, buffer_len, flags, (sockaddr *)&sender_addr, (socklen_t*)&sender_addr_size);
            #endif
            if (SOCKET_ERROR == ret)
            {
                int er = GetSysErrorCode();
                #ifdef WIN32
                if (WSAETIMEDOUT == er)
                #else
                if ((EAGAIN == er) || (EWOULDBLOCK == er))
                #endif
                {
                    len = 0;
                    return 0;
                }
                LOG_ERROR(" Recvfrom error, error code = %d.\n", er);
                return -1;
            }
            else if (0 == ret)
            {
                LOG_ERROR("The connection has been gracefully closed, recv return.\n");
                return -2;
            }
            else
            {
                ip = ntohl(sender_addr.sin_addr.s_addr);
                len = ret;
                return 0;
            }
        }
        else
            return -1;

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    int UdpReceiver::Close()
    {
        int status = 0;
        if (!init_ok_)
            return 0;

        // drop multicast group
        if (multicast_ip_.size())
        {
            struct ip_mreq mreq;
            mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip_.c_str());
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
            if (0 != setsockopt(this->socket_, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char*)&mreq, sizeof(mreq)))
            {
                // "error 19: No such device.", https://stackoverflow.com/questions/3187919/error-no-such-device-in-call-setsockopt-when-joining-multicast-group
                LOG_ERROR("Drop multicast group %s error, error code = %d.\n", multicast_ip_.c_str(), GetSysErrorCode());
                //Close();
            }
        }

#ifdef WIN32
        status = shutdown(this->socket_, SD_BOTH);
        if (status == 0) { status = closesocket(this->socket_); }
#else
        status = shutdown(this->socket_, SHUT_RDWR);
        if (status == 0) { status = close(this->socket_); }
#endif

        return status;

    }
}
