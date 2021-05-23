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

#ifndef LIDAR_TOOLS_H_
#define LIDAR_TOOLS_H_
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include "define.h"


namespace zvision
{
    class TcpClient;


    //////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief LidarTools for lidar's configuration.
    * \author zvision
    */
	class LidarTools
	{
	public:

        typedef std::function<void(int)> ProgressCallback;

        /** \brief zvision LidarTools constructor.
          * \param[in] lidar_ip        lidar's ip address
          * \param[in] con_timeout     timeout in ms for Connect function
          * \param[in] send_timeout    timeout in ms for SyncSend function
          * \param[in] recv_timeout    timeout in ms for SyncRecv function
          */
        LidarTools(std::string lidar_ip, int con_timeout = 100, int send_timeout = 100, int recv_timeout = 10000);

        LidarTools() = delete;


        /** \brief zvision LidarTools destructor.
          */
		~LidarTools();


        /** \brief Scan lidar device on the heartbeat port.
        * \param[out] device_list      the device list
        * \param[in]  scan_time        how long for this operation in second
        * \return 0 for ok, others for failure.
        */
        static int ScanDevice(std::vector<DeviceConfigurationInfo>& device_list, int scan_time);


        /** \brief Get calibration data from calibration file.
          * \param[in]  filename      the file to get the calibration data
          * \param[out] cal           the calibration data
          */
        static int ReadCalibrationData(std::string filename, CalibrationData& cal);

        /** \brief Get calibration data from calibration file.
        * \param[in]  filename         the file to get the calibration data
        * \param[out] cal_cos_sin_lut  the calibration sin cos table
        */
        static int ReadCalibrationData(std::string filename, CalibrationDataSinCosTable& cal_cos_sin_lut);

        /** \brief Export calibration data to file.
          * \param[in]  filename      the file to save the calibration data
          * \param[out] cal           the calibration data
          */
        static int ExportCalibrationData(CalibrationData& cal, std::string filename);

        /** \brief Export calibration data to file.
        * \param[in]   cal                  the calibration data
        * \param[out]  cal_cos_sin_lut      the sin cos data
        */
        static void ComputeCalibrationSinCos(CalibrationData& cal, CalibrationDataSinCosTable& cal_cos_sin_lut);

        /** \brief Get lidar's sn code.
          * \param[out] sn            return the device's sn code
          */
        int QueryDeviceSnCode(std::string& sn);

        /** \brief Get lidar firmware version.
          * \param[out] version       return the firmware version info
          */
		int QueryDeviceFirmwareVersion(FirmwareVersion& version);

        /** \brief Get calibration data from lidar.
          * \param[out] PS            return device's PS's temperature
          * \param[out] PL            return device's PL's temperature
          */
        int QueryDeviceTemperature(float& PS, float& PL);

        /** \brief Get device configuration data from lidar.
          * \param[out] info          return configuration info
          */
        int QueryDeviceConfigurationInfo(DeviceConfigurationInfo& info);

        /** \brief Get calibration data from lidar.
          * \param[out] cal           return device's calibration data
          */
		int GetDeviceCalibrationData(CalibrationData& cal);

        /** \brief Get calibration data from lidar.
        * \param[out] pkts          return device's calibration packets
        */
        int GetDeviceCalibrationPackets(CalibrationPackets& pkts);

        /** \brief Get calibration data from calibration packet.
        * \param[in]  pkts          the whole calibration packets.
        * \param[out] cal           return device's calibration data.
        * \return 0 for ok, others for failure.
        */
        static int GetDeviceCalibrationData(const CalibrationPackets& pkts, CalibrationData& cal);

        /** \brief Get calibration data from lidar and save to file.
          * \param[in] filename       the file to store the calibration data
          */
        int GetDeviceCalibrationDataToFile(std::string filename);

        /** \brief Set lidar static ip address.
          * \param[in] ip             ip to configure
          */
		int SetDeviceStaticIpAddress(std::string ip);

        /** \brief Set lidar subnet mask.
          * \param[in] mask             mask to configure
          */
        int SetDeviceSubnetMask(std::string mask);

        /** \brief Set lidar mac address.
          * \param[in] mac              mac to configure
          */
        int SetDeviceMacAddress(std::string mac);

        /** \brief Set lidar udp destination ip address.
          * \param[in] ip             ip to configure
          */
		int SetDeviceUdpDestinationIpAddress(std::string ip);

        /** \brief Set lidar udp destination port.
          * \param[in] port            port to configure
          */
		int SetDeviceUdpDestinationPort(int port);

        /** \brief Set lidar timestamp type.
          * \param[in] tp              timestamp type to configure
          */
		int SetDeviceTimestampType(TimestampType tp);

        /** \brief Set lidar retro mode.
          * \param[in] en              true for enable, false for disable
          */
        int SetDeviceRetroEnable(bool en);

        /** \brief Set lidar retro param 1 [0,250].
        * \param[in] ref_min           retro parameter 1 (min reflectivity)
        */
        int SetDeviceRetroParam1MinRef(int ref_min);

        /** \brief Set lidar retro param 2 [0,100].
        * \param[in] en                point percentage
        */
        int SetDeviceRetroParam2PointPercentage(int percentage);

        /** \brief Firmware update.
          * \param[in] filename        firmware filename
          */
        int FirmwareUpdate(std::string& filename, ProgressCallback cb);

        /** \brief Reboot device.
        * \return 0 for ok, others for failure.
        */
        int RebootDevice();

        /** \brief Get device's log info.
        * \param[in] log             log string
        * \return 0 for ok, others for failure.
        */
        int GetDeviceLog(std::string& log);

        /** \brief Set device's phase offset.
        * \param[in] offset          offset, unit is 5ns 
        * \return 0 for ok, others for failure.
        */
        int SetDevicePhaseOffset(uint32_t offset);

        /** \brief  Enable/disable device's phase offset.
        * \param[in] en          true for enable, false for disable
        * \return 0 for ok, others for failure.
        */
        int SetDevicePhaseOffsetEnable(bool en);

        /** \brief Set device's PTP configuration.
        * \param[in] ptp_cfg_filename  ptp config filename
        * \return 0 for ok, others for failure.
        */
        int SetDevicePtpConfiguration(std::string ptp_cfg_filename);

        /** \brief Get device's PTP configuration.
        * \param[out] ptp_cfg          ptp config string
        * \return 0 for ok, others for failure.
        */
        int GetDevicePtpConfiguration(std::string& ptp_cfg);

        /** \brief Save device's PTP configuration to file.
        * \param[out] ptp_cfg          ptp config string
        * \return 0 for ok, others for failure.
        */
        int GetDevicePtpConfigurationToFile(std::string& save_file_name);

        /** \brief Set device's point fire enable configuration.
        * \param[in] fire_en_filename  point fire enable config filename
        * \return 0 for ok, others for failure.
        */
        int SetDevicePointFireEnConfiguration(std::string fire_en_filename);

        /** \brief Backup firmware update.
        * \param[in] filename        backup firmware filename
        */
        int BackupFirmwareUpdate(std::string& filename, ProgressCallback cb);

        /** \brief Get lidar backup-firmware version.
        * \param[out] version       return the firmware version info
        */
        int QueryDeviceBackupFirmwareVersion(FirmwareVersion& version);

        /** \brief Set device's echo mode.
        * \param[in] mode            echo mode
        * \return 0 for ok, others for failure.
        */
        int SetDeviceEchoMode(EchoMode mode);
        
        /** \brief Set device's cal send mode.
        * \param[in] mode            CalSendMode
        * \return 0 for ok, others for failure.
        */
        int SetDeviceCalSendMode(CalSendMode mode);

        /** \brief Set device's downsample mode.
        * \param[in] mode            downsample mode
        * \return 0 for ok, others for failure.
        */
        int SetDeviceDownsampleMode(DownsampleMode mode);

	protected:
        
        /** \brief Check the connection to device, if connection is not established, try to connect.
          * \return true for ok, false for failure.
          */
        bool CheckConnection();

        /** \brief Close the connection to device.
          */
        void DisConnect();

        /** \brief Check the return status.
        * \param[in]  ret      return code, 4 bytes
        * \return true for ok, false for failure.
        */
		bool CheckDeviceRet(std::string ret);

	//private:

        /** \brief Tcp client used for lidar configure.
        */
        std::shared_ptr<TcpClient> client_;

        /** \brief device ip to connect.
        */
        std::string device_ip_;

        /** \brief connection status flag. true for ok, false for not.
        */
        bool conn_ok_;

	};
}

#endif //end LIDAR_TOOLS_H_