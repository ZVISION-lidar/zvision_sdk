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


#include "print.h"
#include "define.h"

namespace zvision
{
    std::string get_device_type_string(DeviceType tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case DeviceType::LidarML30B1:
            str = "ML30B1";
            break;
        case DeviceType::LidarML30SA1:
            str = "ML30SA1";
            break;
        case DeviceType::LidarMLX:
            str = "MLX";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_time_sync_type_string(TimestampType tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case TimestampType::TimestampPtp:
            str = "PTP";
            break;
        case TimestampType::TimestampPpsGps:
            str = "PpsGPS";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_retro_mode_string(RetroMode tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case RetroMode::RetroDisable:
            str = "Disable";
            break;
        case RetroMode::RetroEnable:
            str = "Enable";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_return_code_string(ReturnCode tp)
    {
        std::string str = "Unknown";
        switch (tp)
        {
        case ReturnCode::Success:
            str = "Success";
            break;
        case ReturnCode::Failure:
            str = "Failure";
            break;
        case ReturnCode::Timeout:
            str = "Timeout";
            break;
        case ReturnCode::InvalidParameter:
            str = "Invalid parameter";
            break;
        case ReturnCode::NotSupport:
            str = "Not support";
            break;
        case ReturnCode::InitSuccess:
            str = "Init success";
            break;
        case ReturnCode::InitFailure:
            str = "Init failure";
            break;
        case ReturnCode::NotInit:
            str = "Not init";
            break;
        case ReturnCode::OpenFileError:
            str = "Open file error";
            break;
        case ReturnCode::ReadFileError:
            str = "Read file error";
            break;
        case ReturnCode::InvalidContent:
            str = "Invalid content";
            break;
        case ReturnCode::EndOfFile:
            str = "End of file";
            break;
        case ReturnCode::NotMatched:
            str = "Not matched";
            break;
        case ReturnCode::BufferOverflow:
            str = "Buffer overflow";
            break;
        case ReturnCode::Unknown:
            str = "Unknown";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_echo_mode_string(EchoMode mode)
    {
        std::string str = "Unknown";
        switch (mode)
        {
        case EchoMode::EchoSingleFirst:
            str = "Single first echo";
            break;
        case EchoMode::EchoSingleStrongest:
            str = "Singe strongest echo";
            break;
        case EchoMode::EchoSingleLast:
            str = "Singe last echo";
            break;
        case EchoMode::EchoDoubleFirstStrongest:
            str = "Double(first and strongest echo)";
            break;
        case EchoMode::EchoDoubleFirstLast:
            str = "Double(first and last echo)";
            break;
        case EchoMode::EchoDoubleStrongestLast:
            str = "Double(strongest and last echo)";
            break;
        default:
            break;
        }
        return str;
    }

    std::string get_cfg_info_string(DeviceConfigurationInfo& info)
    {
        const int buffer_len = 4096;
        std::shared_ptr<char> buffer(new char[buffer_len]);
        char* ptr = buffer.get();
        int pos = 0;
        pos += snprintf(ptr + pos, buffer_len - pos, "Serial number: %s\n", info.serial_number.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Device ip: %s\n", info.device_ip.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Device subnet mask: %s\n", info.subnet_mask.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Device mac: %s\n", info.device_mac.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Destination ip: %s\n", info.destination_ip.c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Destination port: %d\n", info.destination_port);
        pos += snprintf(ptr + pos, buffer_len - pos, "Timestamp syn mode: %s\n", zvision::get_time_sync_type_string(info.time_sync).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Retro enbale: %s\n", zvision::get_retro_mode_string(info.retro_enable).c_str());
        pos += snprintf(ptr + pos, buffer_len - pos, "Sync offset: %u\n", info.phase_offset);
        pos += snprintf(ptr + pos, buffer_len - pos, "Echo mode: %s\n", zvision::get_echo_mode_string(info.echo_mode));
        pos += snprintf(ptr + pos, buffer_len - pos, "Boot   version: %u.%u.%u\n", info.version.boot_version[0], info.version.boot_version[1], info.version.boot_version[2], info.version.boot_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "Kernel version: %u.%u.%u\n", info.version.kernel_version[0], info.version.kernel_version[1], info.version.kernel_version[2], info.version.kernel_version[3]);
        pos += snprintf(ptr + pos, buffer_len - pos, "Device type: %s\n", zvision::get_device_type_string(info.device).c_str());
        return std::string(ptr);
    }

} // end namespace zvision

