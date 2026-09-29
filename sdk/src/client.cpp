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
#include "loguru.hpp"

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
    /**
    *@Short converts an IP string (such as "192.168.1.1") into a 4-byte binary representation.
    *
    *@Input the IP address string for 'param ip'.
    *@The output buffer of paramaddr (at least 4 bytes) is used to store binary IP addresses.
    *@If the conversion is successful, return true; otherwise, return false.
    */
    bool AssembleIpString(std::string ip, char* addr)
    {
        try
        {
            int value = inet_addr(ip.c_str());
            memcpy(addr, &value, sizeof(value));
        }
        catch (const std::exception& e)
        {
            return false;
        }
        return true;
    }
    /**
    * @brief Assemble a 32-bit port number into network byte order (4 bytes).
    * @param port Port number in host byte order.
    * @param addr Output buffer (4 bytes).
    */
    void AssemblePort(int port, char* addr)
    {
        int value = htonl(port);
        memcpy(addr, &value, sizeof(value));
    }
    /**
     * @brief Assemble a 16-bit port number into network byte order (2 bytes).
     * @param port Port number in host byte order.
     * @param addr Output buffer (2 bytes).
     */
    void AssemblePort_2byte(int port, char* addr)
    {
        uint16_t value = htons(port);
        memcpy(addr, &value, 2);
    }
    /**
     * @brief Parse a MAC address string ("AA-BB-CC-DD-EE-FF") into 6 bytes.
     * @param mac  Input MAC string.
     * @param addr Output buffer (6 bytes).
     * @return true if parsing succeeds, false otherwise.
     */
    bool AssembleMacAddress(std::string mac, char* addr)
    {
        return (6 == sscanf_s(mac.c_str(), "%hhx-%hhx-%hhx-%hhx-%hhx-%hhx", (unsigned char*)&addr[0], (unsigned char*)&addr[1], (unsigned char*)&addr[2], (unsigned char*)&addr[3], (unsigned char*)&addr[4], (unsigned char*)&addr[5]));
    }
    /**
     * @brief Parse a factory MAC string ("AABBCCDDEEFF") into 6 bytes.
     * @param mac  Input MAC string.
     * @param addr Output buffer (6 bytes).
     * @return true if parsing succeeds, false otherwise.
     */
	bool AssembleFactoryMacAddress(std::string mac, char* addr)
	{
		return (6 == sscanf_s(mac.c_str(), "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx", (unsigned char*)&addr[0], (unsigned char*)&addr[1], (unsigned char*)&addr[2], (unsigned char*)&addr[3], (unsigned char*)&addr[4], (unsigned char*)&addr[5]));
	}
    /**
     * @brief Convert a 4-byte binary IP into a dotted string.
     * @param addr Input binary IP (4 bytes).
     * @param ip   Output string.
     */
    void ResolveIpString(const unsigned char* addr, std::string& ip)
    {
        char cip[128] = "";
        sprintf_s(cip, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
        ip = std::string(cip);
    }
    /**
     * @brief Convert a 4-byte binary port into host byte order.
     * @param addr Input buffer (4 bytes).
     * @param port Output port number.
     */
    void ResolvePort(const unsigned char* addr, int& port)
    {
        int* old = (int*)(addr);
        port = ntohl(*old);
    }
    /**
     * @brief Convert a 6-byte MAC into a string ("AA-BB-CC-DD-EE-FF").
     * @param addr Input buffer (6 bytes).
     * @param mac  Output string.
     */
    void ResolveMacAddress(const unsigned char* addr, std::string& mac)
    {
        char cmac[128] = "";
        sprintf_s(cmac, "%02X-%02X-%02X-%02X-%02X-%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        mac = std::string(cmac);
    }
    /**
     * @brief Convert IP string to integer (host byte order).
     * @param ip  Input IP string.
     * @param iip Output integer IP.
     * @return true if conversion succeeds, false otherwise.
     */
    bool StringToIp(std::string ip, unsigned int& iip)
    {
        try
        {
            unsigned int net_byte_order = inet_addr(ip.c_str());
            NetworkToHost((const unsigned char*)&net_byte_order, (char*)&iip);
        }
        catch (const std::exception& e)
        {
            return false;
        }
        return true;
    }
    /**
     * @brief Convert integer IP (host byte order) to dotted string.
     * @param ip Input integer IP.
     * @return IP string.
     */
    std::string IpToString(int ip)
    {
        char cip[128] = "";
        uint32_t network_order = htonl(ip);
        unsigned char* addr = (unsigned char*)(&network_order);
        sprintf_s(cip, "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
        return std::string(cip);
    }
    /**
     * @brief Convert 32-bit integers from network byte order to host byte order.
     * @param net  Input buffer.
     * @param host Output buffer.
     * @param len  Length in bytes (must be multiple of 4).
     */
    void NetworkToHost(const unsigned char* net, char* host, int len)
    {
        for (int i = 0; i < len / 4; i++)
        {
            int ori = *(int*)(net + 4 * i);
            int* now = (int*)(host + 4 * i);
            *now = ntohl(ori);
        }
    }
    /**
     * @brief Convert 16-bit integers from network byte order to host byte order.
     * @param net  Input buffer.
     * @param host Output buffer.
     * @param len  Length in bytes (must be multiple of 2).
     */
    void NetworkToHostShort(const unsigned char* net, char* host, int len)
    {
        for (int i = 0; i < len / 2; i++)
        {
            u_short ori = *(u_short*)(net + 2 * i);
            u_short* now = (u_short*)(host + 2 * i);
            *now = ntohs(ori);
        }
    }
    /**
     * @brief Convert 32-bit integers from host byte order to network byte order.
     * @param host Input buffer.
     * @param net  Output buffer.
     * @param len  Length in bytes (must be multiple of 4).
     */
    void HostToNetwork(const unsigned char* host, char* net, int len)
    {
        for (int i = 0; i < len / 4; i++)
        {
            int ori = *(int*)(host + 4 * i);
            int* now = (int*)(net + 4 * i);
            *now = htonl(ori);
        }
    }
    /**
     * @brief Swap byte order of 32-bit integers.
     * @param src Input buffer.
     * @param dst Output buffer.
     * @param len Length in bytes (must be multiple of 4).
     */
    void SwapByteOrder(char* src, char* dst, int len)
    {
        for (int i = 0; i < len / 4; i++)
        {
            for(int j = 0; j < 4; j++)
                dst[i * 4 + j] = src[i * 4 + (3 - j)];
        }
    }
    /**
    * @brief Get last system error code (cross-platform).
    * @return Error code.
    */
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
#if 0
        void Error(const std::string& format, va_list args)
        {
            printf(format.c_str(), args);
        }
#endif
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
        LOG_F(ERROR, "%s: %d", msg, WSAGetLastError());
#else
        perror(msg);
#endif
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * @brief Construct Env object, initialize socket environment status as failure.
    */
    Env::Env():
        socket_env_status_((int)ReturnCode::InitFailure)
    {

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /**
    * @brief Initialize socket environment if not already done.
    * 
    * On Windows, calls WSAStartup. On Linux/Unix, always returns success.
    * 
    * @return true if socket environment is ready, false otherwise.
    */
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
                LOG_F(ERROR, "Error at WSASturtup: ret = %d", iResult);
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
    /**
     * @brief Construct a TcpClient with timeout settings.
     * 
     * @param connect_timeout Connection timeout in milliseconds.
     * @param send_timeout    Send timeout in milliseconds.
     * @param recv_timeout    Receive timeout in milliseconds.
     */
    TcpClient::TcpClient(int connect_timeout, int send_timeout, int recv_timeout) :
        conn_timeout_ms_(connect_timeout),
        send_timeout_ms_(send_timeout),
        recv_timeout_ms_(recv_timeout),
        socket_(INVALID_SOCKET),
        conn_ok_(false),
        ///error_code_(0),
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
    /**
     * @brief Connect to a remote server.
     * 
     * @param dst_ip   Destination IP address string.
     * @param dst_port Destination port number.
     * @return 0 on success, -1 on failure.
     */
    int TcpClient::Connect(std::string dst_ip, int dst_port)
    {
        if (!Env::Ok())
            return -1;

        LOG_F(2, "Connect to %s:%d.", dst_ip.c_str(), dst_port);
        // Socket creation
        this->socket_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (this->socket_ == INVALID_SOCKET)
        {
            LOG_F(ERROR, "Socket creation failed.");
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
            LOG_F(ERROR, "Set nonblock error, error code = %d.", GetSysErrorCode());
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
                LOG_F(ERROR, "Connect error, error code = %d.", ret);
                Close();
                return -1;
            }

            // connection pending
            fd_set setW, setE;

            FD_ZERO(&setW);
            FD_SET(this->socket_, &setW);
            FD_ZERO(&setE);
            FD_SET(this->socket_, &setE);

            LOG_F(2, "Set connect timeout to %d seconds %d usec.", time_out.tv_sec, time_out.tv_usec);
            ret = select(0, NULL, &setW, &setE, &time_out);
            if (ret <= 0)
            {
                // https://docs.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-select
                if(0 == ret)
                    LOG_F(ERROR, "Connect timeout.");
                else
                    LOG_F(ERROR, "Select error, ret = %d error code = %d.", ret, this->GetSysErrorCode());
                Close();
                return -1;
            }

            ret = FD_ISSET(this->socket_, &setE);
            if (ret)
            {
                // connection failed
                LOG_F(ERROR, "FD_ISSET error: ret = %d, error code = %d.", ret, this->GetSysErrorCode());
                Close();
                return -1;
            }
        }

        // put socked in blocking mode...
        block = 0;
        // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-ioctlsocket
        if (ioctlsocket(this->socket_, FIONBIO, &block) == SOCKET_ERROR)
        {
            LOG_F(ERROR, "Set block mode error, error code = %d.",this->GetSysErrorCode());
            Close();
            return -1;
        }

        // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-setsockopt
        int timeout = this->send_timeout_ms_;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout)))
        {
            LOG_F(ERROR, "Set send timeout error, error code = %d.", this->GetSysErrorCode());
            Close();
            return -1;
        }
        else
        {
            LOG_F(2, "Set send timeout ok, %d ms", timeout);
        }

        timeout = this->recv_timeout_ms_;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout)))
        {
            LOG_F(ERROR, "Set receive timeout error, error code = %d. ", timeout);
            Close();
            return -1;
        }
        else
        {
            LOG_F(2, "Set receive timeout ok, %d ms", timeout);
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
            LOG_F(ERROR, "Set non-block mode error, error code = %d.", GetSysErrorCode());
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
                        LOG_F(ERROR, "Connect error, error code = %d", so_error);
                        Close();
                        return -1;
                    }
                }
                else
                {
                    LOG_F(ERROR, "Connect error, error code = %d.", this->GetSysErrorCode());
                    Close();
                    return -1;
                }
            }
            else
            {
                LOG_F(ERROR, "Connect error, error code = %d.", err);
                Close();
                return -1;
            }


        }

        LOG_F(2, "Connect to %s ok.", dst_ip.c_str());;

        int flag = fcntl(this->socket_, F_GETFL, 0);
        if(0 > (ret = fcntl(this->socket_, F_SETFL, flag & ~O_NONBLOCK)))
        {
            LOG_F(ERROR, "Set block mode error, error code = %d.", GetSysErrorCode());
            Close();
            return -1;
        }

        time_out.tv_sec = this->send_timeout_ms_ / 1000;
        time_out.tv_usec = (this->send_timeout_ms_ % 1000) * 1000;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&time_out, sizeof(time_out)))
        {
            LOG_F(ERROR, "Set send timeout error, error code = %d.", this->GetSysErrorCode());
            Close();
            return -1;
        }
        else
        {
            LOG_F(2, "Set send timeout ok, %d ms", timeout);
        }

        time_out.tv_sec = this->recv_timeout_ms_ / 1000;
        time_out.tv_usec = (this->recv_timeout_ms_ % 1000) * 1000;
        if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&time_out, sizeof(time_out)))
        {
            LOG_F(ERROR, "Set receive timeout error, error code = %d. ", timeout);
            Close();
            return -1;
        }
        else
        {
            LOG_F(2, "Set receive timeout ok, %d ms", timeout);
        }
        return 0;
#endif


    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Send data synchronously.
     * 
     * @param data Data buffer (string).
     * @param len  Number of bytes to send.
     * @return 0 on success, -1 on error.
     */
    int TcpClient::SyncSend(std::string& data, int len)
    {
        int ret = 0;
        int count = 0;
        int flags = 0;

        const char *buf = reinterpret_cast<const char*>(data.c_str());
        do
        {
            //windows: https://docs.microsoft.com/en-us/windows/win32/api/winsock2/nf-winsock2-send
            ret = send(this->socket_, buf + count, len - count, flags);
            if (SOCKET_ERROR == ret)
            {
                LOG_F(ERROR, "send error, return value is %d", GetSysErrorCode());
                return -1;
            }
            else if (0 == ret)
            {
                LOG_F(ERROR, "The connection has been gracefully closed, send return");
                return -1;
            }

            count += ret;
        } while (count < len);

        return 0;
    }
    /**
     * @brief Get number of bytes available to read without blocking.
     * 
     * @return Number of bytes available, or -1 on error.
     */
	int TcpClient::GetAvailableBytesLen() {
		int bytesRdy = 0;

		char tempbuf[BUFFERSIZE];
		int timeout = 1;
		setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
#ifdef WIN32
		bytesRdy = recv(this->socket_, tempbuf, sizeof(tempbuf), MSG_PEEK | MSG_PUSH_IMMEDIATE);
#else
		bytesRdy = recv(this->socket_, tempbuf, sizeof(tempbuf), MSG_PEEK | MSG_DONTWAIT);
#endif

		setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *)&this->recv_timeout_ms_, sizeof(this->recv_timeout_ms_));

		return bytesRdy;
	}

    //////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Receive data synchronously into string buffer.
     * 
     * @param data Output buffer (pre-allocated string).
     * @param len  Expected number of bytes to read.
     * @return 0 on success, -1 on error, -2 if connection closed.
     */
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
                LOG_F(ERROR, "Recv error, value is %d", GetSysErrorCode());
                return -1;
            }
            else if (0 == ret)
            {
                LOG_F(ERROR, "The connection has been gracefully closed, recv return");
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
    /**
     * @brief Receive data synchronously into raw buffer.
     * 
     * @param data Output buffer (char*).
     * @param len  Expected number of bytes to read.
     * @return 0 on success, -1 on error, -2 if connection closed.
     */
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
                LOG_F(ERROR, "Recv error, value is %d", GetSysErrorCode());
                return -1;
            }
            else if (0 == ret)
            {
                LOG_F(ERROR, "The connection has been gracefully closed, recv return");
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
    /**
     * @brief Close the TCP connection and release socket.
     * 
     * @return 0 on success, error code otherwise.
     */
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
    /**
     * @brief Get last system error code (cross-platform).
     * 
     * @return Error code.
     */
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
    /**
     * @brief Record error info (currently placeholder).
     * 
     * @return Empty string.
     */
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
    /**
     * @brief Construct a UDP receiver.
     * 
     * @param port            Local port to bind.
     * @param recv_timeout    Receive timeout in milliseconds.
     * @param recv_buffer_len Receive buffer length in bytes.
     */
    UdpReceiver::UdpReceiver(int port, int recv_timeout, int recv_buffer_len):
        local_port_(port),
        recv_timeout_ms_(recv_timeout),
        recv_buffer_len_(recv_buffer_len),
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
    /**
     * @brief Join a multicast group.
     * 
     * @param mtip Multicast IP address string.
     * @return 0 on success.
     */
    int UdpReceiver::JoinMulticastGroup(std::string& mtip)
    {
        multicast_ip_ = mtip;
        return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Bind UDP socket to local port and configure options.
     * 
     * @return 0 on success, -1 on failure.
     */
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
                LOG_F(ERROR, "Socket creation failed.");
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
                LOG_F(ERROR, "Set nonblock error");
                Close();
                return -1;
            }
            
            #else
            #if 1
            int flags = fcntl(this->socket_, F_GETFL);
            if(0 > fcntl(this->socket_, flags & ~O_NONBLOCK))
            {
                LOG_F(ERROR, "Set block error, error code  %d.", GetSysErrorCode());
                Close();
                return -1;
            }
            #endif
            #endif

            int reuseaddr = 1;
            if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuseaddr, sizeof(reuseaddr)))
            {
                LOG_F(ERROR, "Set reuse addr error, error code = %d.", GetSysErrorCode());
                Close();
                return -1;
            }

            // SO_REUSEPORT allows multiple sockets to bind to the same port
            // This is needed on macOS for port reuse when reconnecting
            #ifndef WIN32
            int reuseport = 1;
            if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_REUSEPORT, (const char *)&reuseport, sizeof(reuseport)))
            {
                LOG_F(WARNING, "Set reuse port error, error code = %d.", GetSysErrorCode());
                // Continue even if SO_REUSEPORT fails (not supported on all systems)
            }
            #endif

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
                LOG_F(ERROR, "Set receive timeout %d ms error, error code = %d.", this->recv_timeout_ms_, GetSysErrorCode());
                Close();
                return -1;
            }
            else
            {
                LOG_F(2, "Set receive timeout ok, %d ms.", this->recv_timeout_ms_);
            }

            int recv_buffer_size = 1024 * 1000;
            if (0 != setsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, (const char *)&recv_buffer_size, sizeof(recv_buffer_size)))
            {
                LOG_F(ERROR, "Set receive buffer size %d byte(s) error, error code = %d.", recv_buffer_size, GetSysErrorCode());
                Close();
                return -1;
            }
            else
            {
                LOG_F(2, "Set receive buffer size ok, %d byte(s).", recv_buffer_size);
            }

            // return value reference: https://docs.microsoft.com/en-us/windows/win32/api/winsock/nf-winsock-bind
            if (bind(this->socket_, (struct sockaddr*)&addr, sizeof(addr)))
            {
                LOG_F(ERROR, "Bind error, error code = %d.", GetSysErrorCode());
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
                    LOG_F(ERROR, "Join multicast group %s error, error code = %d.", multicast_ip_.c_str(), GetSysErrorCode());
                    Close();
                    return -1;
                }
            }

            init_ok_ = true;
        }
        return 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief Receive data synchronously from UDP socket.
     * 
     * @param data Output buffer (string).
     * @param len  Output length of received data.
     * @param ip   Output sender IP (host byte order).
     * @return 0 on success, -1 on error, -2 if connection closed.
     */
    int UdpReceiver::SyncRecv(std::string& data, int& len, uint32_t& ip)
    {
        len = 0;
        if (!Env::Ok())
            return InitFailure;

        if (!init_ok_)
        {
            int ret = 0;
            if(0 != (ret = this->Bind()))
                LOG_F(ERROR, "Bind error.");
        }

        if(init_ok_)
        {
            int flags = 0;
            //const int buffer_len = 1024*32;//2048
            int buffer_len = 2048;
            if (recv_buffer_len_ >= 2048 && recv_buffer_len_ <= (1024*32))
            {
                //if (recv_buffer_len_ % 1024 == 0)
                buffer_len = recv_buffer_len_;
            }

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
                    //LOG_F(WARNING, "Recvfrom timeout, error code = %d.", er);
                    len = 0;
                    return 0;
                }
                LOG_F(ERROR, " Recvfrom error, error code = %d.", er);
                return -1;
            }
            else if (0 == ret)
            {
                LOG_F(ERROR, "The connection has been gracefully closed, recv return.");
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
    /**
     * @brief Close UDP socket and drop multicast group if joined.
     * 
     * @return 0 on success, error code otherwise.
     */
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
                LOG_F(ERROR, "Drop multicast group %s error, error code = %d.", multicast_ip_.c_str(), GetSysErrorCode());
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
