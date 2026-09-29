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


#include "serial_client.h"

#include <cerrno>
#include <cstring>
#include <vector>

#ifndef _WIN32

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
// Custom baud (e.g. 3125000) requires termios2 + BOTHER. Do not mix with
// <termios.h> here; <asm/termbits.h> alone is used.
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#ifndef TCIFLUSH
#define TCIFLUSH 0
#endif

#ifndef CIBAUD
#define CIBAUD (CBAUD << 16)
#endif

#else  // _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#endif  // _WIN32

namespace zvision
{
    namespace
    {
#ifndef _WIN32
        void close_fd(int *fd)
        {
            if (fd && *fd >= 0)
            {
                ::close(*fd);
                *fd = -1;
            }
        }

        // Map common rates to termios baud bits; return 0 if custom (BOTHER).
        unsigned int standard_baud_bit(int baud)
        {
            switch (baud)
            {
                case 9600: return B9600;
                case 19200: return B19200;
                case 38400: return B38400;
                case 57600: return B57600;
                case 115200: return B115200;
                case 230400: return B230400;
                case 460800: return B460800;
                case 500000: return B500000;
                case 921600: return B921600;
                case 1000000: return B1000000;
                case 1500000: return B1500000;
                case 2000000: return B2000000;
                default: return 0;  // e.g. 3125000 -> BOTHER
            }
        }

        int open_port(const std::string &path, int baud)
        {
            const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd < 0)
            {
                return -1;
            }

            struct termios2 tio {};
            if (ioctl(fd, TCGETS2, &tio) != 0)
            {
                ::close(fd);
                return -1;
            }

            // Raw 8N1
            tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                             IXON | IXOFF | IXANY);
            tio.c_oflag &= ~OPOST;
            tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

            tio.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
            tio.c_cflag |= (CS8 | CLOCAL | CREAD);

            tio.c_cc[VMIN] = 0;
            tio.c_cc[VTIME] = 0;

            const unsigned int std_bit = standard_baud_bit(baud);
            tio.c_cflag &= ~(CBAUD | CIBAUD);
            if (std_bit != 0)
            {
                tio.c_cflag |= std_bit;
                tio.c_ispeed = static_cast<unsigned int>(baud);
                tio.c_ospeed = static_cast<unsigned int>(baud);
            }
            else
            {
                tio.c_cflag |= BOTHER;
                tio.c_ispeed = static_cast<unsigned int>(baud);
                tio.c_ospeed = static_cast<unsigned int>(baud);
            }

            if (ioctl(fd, TCSETS2, &tio) != 0)
            {
                ::close(fd);
                return -1;
            }

            const int flags = fcntl(fd, F_GETFL, 0);
            if (flags >= 0)
            {
                fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
            }
            return fd;
        }
#else  // _WIN32
        std::string normalize_port(const std::string &p)
        {
            if (p.rfind("\\\\.\\", 0) == 0)
            {
                return p;
            }
            return "\\\\.\\" + p;
        }

        bool apply_timeouts(HANDLE h, DWORD total_timeout_ms)
        {
            COMMTIMEOUTS ct {};
            ct.ReadIntervalTimeout = 0;
            ct.ReadTotalTimeoutMultiplier = 0;
            ct.ReadTotalTimeoutConstant = total_timeout_ms;
            ct.WriteTotalTimeoutMultiplier = 0;
            ct.WriteTotalTimeoutConstant = 1000;
            return SetCommTimeouts(h, &ct) != 0;
        }

        HANDLE open_one(const std::string &path, int baud)
        {
            const std::string name = normalize_port(path);
            HANDLE h = CreateFileA(name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                   OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
            if (h == INVALID_HANDLE_VALUE)
            {
                return INVALID_HANDLE_VALUE;
            }

            DCB dcb {};
            dcb.DCBlength = sizeof(dcb);
            if (!GetCommState(h, &dcb))
            {
                CloseHandle(h);
                return INVALID_HANDLE_VALUE;
            }

            // Raw 8N1, no flow control
            dcb.BaudRate = static_cast<DWORD>(baud);
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;
            dcb.fBinary = TRUE;
            dcb.fParity = FALSE;
            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutxDsrFlow = FALSE;
            dcb.fDtrControl = DTR_CONTROL_DISABLE;
            dcb.fRtsControl = RTS_CONTROL_DISABLE;
            dcb.fOutX = FALSE;
            dcb.fInX = FALSE;
            dcb.fErrorChar = FALSE;
            dcb.fNull = FALSE;
            dcb.fAbortOnError = FALSE;
            if (!SetCommState(h, &dcb))
            {
                CloseHandle(h);
                return INVALID_HANDLE_VALUE;
            }

            SetupComm(h, 4096, 4096);
            apply_timeouts(h, 0);
            return h;
        }
#endif  // _WIN32
    }

    SerialClient::SerialClient(int send_timeout, int recv_timeout) :
        send_timeout_ms_(send_timeout),
        recv_timeout_ms_(recv_timeout),
        port_send_(),
        port_recv_(),
        conn_ok_(false)
#ifdef _WIN32
        , h_send_(NULL),
        h_recv_(NULL)
#else
        , fd_send_(-1),
        fd_recv_(-1)
#endif
    {
    }

    SerialClient::~SerialClient()
    {
        Close();
    }

    int SerialClient::Connect(std::string port_send, std::string port_recv, int baud_send, int baud_recv)
    {
        if (conn_ok_)
        {
            return 0;
        }

        port_send_ = port_send;
        port_recv_ = port_recv;

#ifdef _WIN32
        HANDLE h_send = open_one(port_send, baud_send);
        if (h_send == INVALID_HANDLE_VALUE)
        {
            return -1;
        }
        HANDLE h_recv = open_one(port_recv, baud_recv);
        if (h_recv == INVALID_HANDLE_VALUE)
        {
            CloseHandle(h_send);
            return -1;
        }
        h_send_ = h_send;
        h_recv_ = h_recv;
#else
        fd_send_ = open_port(port_send, baud_send);
        if (fd_send_ < 0)
        {
            return -1;
        }
        fd_recv_ = open_port(port_recv, baud_recv);
        if (fd_recv_ < 0)
        {
            close_fd(&fd_send_);
            return -1;
        }
#endif

        conn_ok_ = true;
        return 0;
    }

    int SerialClient::SyncSend(std::string& data, int len)
    {
        if (!conn_ok_ || len <= 0 || len > static_cast<int>(data.size()))
        {
            return -1;
        }

#ifdef _WIN32
        HANDLE h = static_cast<HANDLE>(h_send_);
        if (h == INVALID_HANDLE_VALUE)
        {
            return -1;
        }
        COMMTIMEOUTS ct {};
        if (!GetCommTimeouts(h, &ct))
        {
            return -1;
        }
        ct.WriteTotalTimeoutMultiplier = 0;
        ct.WriteTotalTimeoutConstant = static_cast<DWORD>(send_timeout_ms_);
        if (!SetCommTimeouts(h, &ct))
        {
            return -1;
        }
        size_t written = 0;
        while (written < static_cast<size_t>(len))
        {
            DWORD n = 0;
            const DWORD chunk = static_cast<DWORD>(len - written);
            if (!WriteFile(h, data.data() + written, chunk, &n, NULL))
            {
                return -1;
            }
            if (n == 0)
            {
                return -1;
            }
            written += static_cast<size_t>(n);
        }
#else
        if (fd_send_ < 0)
        {
            return -1;
        }
        size_t written = 0;
        while (written < static_cast<size_t>(len))
        {
            const ssize_t n = ::write(fd_send_, data.data() + written, static_cast<size_t>(len) - written);
            if (n < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                return -1;
            }
            if (n == 0)
            {
                return -1;
            }
            written += static_cast<size_t>(n);
        }
#endif
        return 0;
    }

    int SerialClient::SyncRecv(std::string& data, int& len)
    {
        len = 0;
        if (!conn_ok_)
        {
            return -1;
        }

#ifdef _WIN32
        HANDLE h = static_cast<HANDLE>(h_recv_);
        if (h == INVALID_HANDLE_VALUE)
        {
            return -1;
        }
        if (!apply_timeouts(h, static_cast<DWORD>(recv_timeout_ms_)))
        {
            return -1;
        }
        std::vector<unsigned char> buf(8192);
        DWORD n = 0;
        if (!ReadFile(h, buf.data(), static_cast<DWORD>(buf.size()), &n, NULL))
        {
            if (GetLastError() == ERROR_SEM_TIMEOUT)
            {
                return 0;  // idle
            }
            return -1;
        }
        if (n > 0)
        {
            data.assign(reinterpret_cast<const char*>(buf.data()), static_cast<size_t>(n));
            len = static_cast<int>(n);
        }
#else
        if (fd_recv_ < 0)
        {
            return -1;
        }
        struct pollfd pfd {};
        pfd.fd = fd_recv_;
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, recv_timeout_ms_);
        if (pr < 0)
        {
            if (errno == EINTR)
            {
                return 0;
            }
            return -1;
        }
        if (pr == 0)
        {
            return 0;  // idle timeout
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            return -1;
        }
        std::vector<unsigned char> buf(8192);
        const ssize_t n = ::read(fd_recv_, buf.data(), buf.size());
        if (n < 0)
        {
            if (errno == EINTR || errno == EAGAIN)
            {
                return 0;
            }
            return -1;
        }
        if (n > 0)
        {
            data.assign(reinterpret_cast<const char*>(buf.data()), static_cast<size_t>(n));
            len = static_cast<int>(n);
        }
#endif
        return 0;
    }

    int SerialClient::FlushRecvInput()
    {
        if (!conn_ok_)
        {
            return -1;
        }
#ifdef _WIN32
        HANDLE h = static_cast<HANDLE>(h_recv_);
        if (h == INVALID_HANDLE_VALUE)
        {
            return -1;
        }
        return PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR) ? 0 : -1;
#else
        if (fd_recv_ < 0)
        {
            return -1;
        }
        return ioctl(fd_recv_, TCFLSH, TCIFLUSH) == 0 ? 0 : -1;
#endif
    }

    int SerialClient::Close()
    {
        if (!conn_ok_)
        {
            conn_ok_ = false;
            return 0;
        }
#ifdef _WIN32
        HANDLE h_send = static_cast<HANDLE>(h_send_);
        HANDLE h_recv = static_cast<HANDLE>(h_recv_);
        if (h_send != INVALID_HANDLE_VALUE)
        {
            CloseHandle(h_send);
        }
        if (h_recv != INVALID_HANDLE_VALUE)
        {
            CloseHandle(h_recv);
        }
        h_send_ = NULL;
        h_recv_ = NULL;
#else
        close_fd(&fd_send_);
        close_fd(&fd_recv_);
#endif
        conn_ok_ = false;
        return 0;
    }

} // end namespace zvision
