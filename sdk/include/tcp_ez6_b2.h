#ifndef TCP_EZ6_B2_H_
#define TCP_EZ6_B2_H_
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <type_traits>
#include "define.h"
#include "tcp_tools.h"

namespace zvision 
{
    class TcpClient;

    class tcp_ez6_b2 : public tcp_tools 
    {
        public:
            tcp_ez6_b2(std::string lidar_ip, int con_timeout = 1000, int send_timeout = 1000, int recv_timeout = 10000);
            tcp_ez6_b2() = delete;

            ~tcp_ez6_b2();

            int get_ptpcfg(std::string& ptp_cfg) override;
            int get_ptpcfg_to_file(std::string &save_file_name) override;
            int set_ptpcfg(std::string filename) override;
            int update_firmware(std::string& filename, ProgressCallback cb, bool isBak) override;
            int read_comp_from_csv(const std::string &filename, angle_comp_t &angle_comp) override;
            int get_angle_comp(angle_comp_t &angle_comp) override;
            int get_basic_info(DeviceConfigurationInfo & info) override;

        protected:
            bool CheckConnection();
            void DisConnect();
            int recv_tcp_resp(std::string& out);
            int send_tcp_request(const std::string &cmd);
            int generate_request_pkg(uint16_t id, const std::string &data, uint8_t cmd_type, uint8_t param_type, std::string &out);
            int recv_specific_number_data(int recv_num,std::string &data);

        private:
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

#endif