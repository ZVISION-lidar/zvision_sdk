#ifndef TCP_TOOLS_H_
#define TCP_TOOLS_H_
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <type_traits>
#include "define.h"

namespace zvision 
{
    class tcp_tools
    {
        public:
            typedef std::function<void(int,std::string)> ProgressCallback;

            virtual ~tcp_tools() = default;

            virtual int get_ptpcfg(std::string& ptp_cfg) = 0;
            virtual int get_ptpcfg_to_file(std::string &save_file_name) = 0;
            virtual int set_ptpcfg(std::string filename) = 0;
            virtual int update_firmware(std::string& filename, ProgressCallback cb, bool isBak) = 0;
            virtual int read_comp_from_csv(const std::string &filename, angle_comp_t &angle_comp) = 0;
            virtual int get_angle_comp(angle_comp_t &angle_comp) = 0;
            virtual int get_basic_info(DeviceConfigurationInfo & info) = 0;
    };

    float get_float(uint8_t * buffer);
}

#endif
