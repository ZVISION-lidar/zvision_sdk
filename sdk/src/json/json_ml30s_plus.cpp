
#include <iostream>
#include <fstream>
#include <string>
#include <map>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>

#include <crypto/md5.h>
#include <json_ml30s_plus.h>

/** \brief read file data.
* \param[in]  path         file path.
* \param[out] context      return file context.
* \return 0 for ok, others for failure.
*/
int readFile(const std::string& path, std::string& context) {

	std::ifstream in;
	in.open(path, std::ifstream::in);
	if (!in.is_open())
		return -1;

	std::string line;
	while (std::getline(in, line)) {
		context.append(line);
		context.append("\n");
	}

	in.close();
	return 0;
}

/** \brief save file data.
* \param[in]  path         file path.
* \param[in] context       file context.
* \return 0 for ok, others for failure.
*/
int saveFile(const std::string& path, const std::string& context) {

	std::ofstream fileout(path, std::ios::out | std::ios::trunc);
	if (!fileout.good())
		return -1;

	fileout << context;
	fileout.close();
	return 0;
}


int GetJsonConfigFileFromString(const std::string& context, zvision::JsonConfigFileParam& param) {

	// prasing json
	rapidjson::Document doc;
	if (doc.Parse(context.data()).HasParseError())
		return -1;

	if (!doc.IsObject())
		return -1;

	// check data type:JsonConfigFileParam
	std::map<std::string, std::vector<std::string>> mbrs{
		std::pair<std::string,std::vector<std::string>>("soft version",{ "embeded version","fpga version" }),
		std::pair<std::string,std::vector<std::string>>("device net info",{ "dhcp switch","ip","gateway","netmask","mac","udp dest ip" }),
		std::pair<std::string,std::vector<std::string>>("retro para",{ "switch","retro num","retro gray","distance thres",\
														"low range thres","high range thres","del gray thres" }),
		std::pair<std::string,std::vector<std::string>>("adhesion para",{ "switch","min horizon angle","max horizon angle",\
														"min vertical angle","max vertical angle","horizon angle resolution",\
														"vertical angle resolution","diff thres","max dist","min diff" }),
		std::pair<std::string,std::vector<std::string>>("dirty detect",{ "detect switch","refresh switch","refresh cycle",\
														"detect set thres","detect reset thres","detect inner thres","detect outer thres" }),
		std::pair<std::string,std::vector<std::string>>("delete near point",{ "switch" }),
		std::pair<std::string,std::vector<std::string>>("downsample mode",{ "mode" }),
		std::pair<std::string,std::vector<std::string>>("echo mode",{ "mode" }),
		std::pair<std::string,std::vector<std::string>>("ptp sync",{ "switch" }),
		std::pair<std::string,std::vector<std::string>>("frame sync",{ "switch", "offset" }),
		std::pair<std::string,std::vector<std::string>>("angle send",{ "switch" })
	};

	// check all keys
	for (auto& mbr : mbrs) {
		if (!doc.HasMember(mbr.first.c_str()))
			return -1;
		rapidjson::Value &vals = doc[mbr.first.c_str()];
		for (auto& sub : mbr.second) {
			if (!vals.HasMember(sub.c_str()))
				return -1;
		}
	}

	// soft versio
	{
		rapidjson::Value &val = doc["soft version"];
		param.embedded_version = val["embeded version"].GetString();
		param.fpga_version = val["fpga version"].GetString();
	}
	// device net info
	{
		rapidjson::Value &val = doc["device net info"];
		param.dhcp_switch = val["dhcp switch"].GetInt();
		param.ip = val["ip"].GetString();
		param.gateway = val["gateway"].GetString();
		param.netmask = val["netmask"].GetString();
		param.mac = val["mac"].GetString();
		param.udp_dest_ip = val["udp dest ip"].GetString();
		param.udp_dest_port = val["udp dest port"].GetInt();
	}
	// retro para
	{
		rapidjson::Value &val = doc["retro para"];
		param.switch_retro = val["switch"].GetInt();
		param.target_point_num_thre_retro = val["retro num"].GetInt();
		param.target_gray_thre_retro = val["retro gray"].GetInt();
		param.critical_point_dis_thre_retro = val["distance thres"].GetInt();
		param.del_point_dis_low_thre_retro = val["low range thres"].GetInt();
		param.del_point_dis_high_thre_retro = val["high range thres"].GetInt();
		param.del_point_gray_thre_retro = val["del gray thres"].GetInt();
	}
	// adhesion para
	{
		rapidjson::Value &val = doc["adhesion para"];
		param.switch_adhesion = val["switch"].GetInt();
		param.angle_hor_min_adhesion = val["min horizon angle"].GetInt();
		param.angle_hor_max_adhesion = val["max horizon angle"].GetInt();
		param.angle_ver_min_adhesion = val["min vertical angle"].GetFloat();
		param.angle_ver_max_adhesion = val["max vertical angle"].GetFloat();
		param.angle_hor_res_adhesion = val["horizon angle resolution"].GetFloat();
		param.angle_ver_res_adhesion = val["vertical angle resolution"].GetFloat();
		param.diff_thre_adhesion = val["diff thres"].GetFloat();
		param.dist_limit_adhesion = val["max dist"].GetFloat();
		param.min_diff_adhesion = val["min diff"].GetFloat();
	}
	// dirty detect
	{
		rapidjson::Value &val = doc["dirty detect"];
		param.switch_dirty_detect = val["detect switch"].GetInt();
		param.switch_dirty_refresh = val["refresh switch"].GetInt();
		param.dirty_refresh_cycle = val["refresh cycle"].GetInt();
		param.dirty_detect_set_thre = val["detect set thres"].GetInt();
		param.dirty_detect_reset_thre = val["detect reset thres"].GetInt();
		param.dirty_detect_inner_thre = val["detect inner thres"].GetInt();
		param.dirty_detect_outer_thre = val["detect outer thres"].GetInt();
	}
	// delete near point
	{
		rapidjson::Value &val = doc["delete near point"];
		param.switch_near_point_delete = val["switch"].GetInt();
	}
	// downsample mode
	{
		rapidjson::Value &val = doc["downsample mode"];
		param.down_sample_mode = val["mode"].GetInt();
	}
	// echo mode
	{
		rapidjson::Value &val = doc["echo mode"];
		param.echo_mode = val["mode"].GetInt();
	}
	// ptp sync
	{
		rapidjson::Value &val = doc["ptp sync"];
		param.ptp_sync = val["switch"].GetInt();
	}
	// frame sync
	{
		rapidjson::Value &val = doc["frame sync"];
		param.frame_sync = val["switch"].GetInt();
		param.frame_offset = val["offset"].GetInt();
	}
	// angle send
	{
		rapidjson::Value &val = doc["angle send"];
		param.angle_send = val["switch"].GetInt();
	}

	return 0;
}


int GenerateJsonConfigFile2String(const zvision::JsonConfigFileParam& param, std::string& context) {

	// json file pre
	rapidjson::StringBuffer buffer;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
	rapidjson::Document document(rapidjson::kObjectType);
	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

	// add param to doc
	rapidjson::Value soft_version(rapidjson::kObjectType);
	soft_version.AddMember("embeded version", rapidjson::Value(param.embedded_version.c_str(), allocator), allocator);
	soft_version.AddMember("fpga version", rapidjson::Value(param.fpga_version.c_str(), allocator), allocator);
	document.AddMember("soft version", soft_version, allocator);

	rapidjson::Value device_net_info(rapidjson::kObjectType);
	device_net_info.AddMember("dhcp switch", rapidjson::Value(param.dhcp_switch), allocator);
	device_net_info.AddMember("ip", rapidjson::Value(param.ip.c_str(), allocator), allocator);
	device_net_info.AddMember("gateway", rapidjson::Value(param.gateway.c_str(), allocator), allocator);
	device_net_info.AddMember("netmask", rapidjson::Value(param.netmask.c_str(), allocator), allocator);
	device_net_info.AddMember("mac", rapidjson::Value(param.mac.c_str(), allocator), allocator);
	device_net_info.AddMember("udp dest ip", rapidjson::Value(param.udp_dest_ip.c_str(), allocator), allocator);
	device_net_info.AddMember("udp dest port", rapidjson::Value(param.udp_dest_port), allocator);
	document.AddMember("device net info", device_net_info, allocator);

	rapidjson::Value retro_para(rapidjson::kObjectType);
	retro_para.AddMember("switch", rapidjson::Value(param.switch_retro), allocator);
	retro_para.AddMember("retro gray", rapidjson::Value(param.target_gray_thre_retro), allocator);
	retro_para.AddMember("retro num", rapidjson::Value(param.target_point_num_thre_retro), allocator);
	retro_para.AddMember("distance thres", rapidjson::Value(param.critical_point_dis_thre_retro), allocator);
	retro_para.AddMember("low range thres", rapidjson::Value(param.del_point_dis_low_thre_retro), allocator);
	retro_para.AddMember("high range thres", rapidjson::Value(param.del_point_dis_high_thre_retro), allocator);
	retro_para.AddMember("del gray threso", rapidjson::Value(param.del_point_gray_thre_retro), allocator);
	document.AddMember("retro para", retro_para, allocator);

	rapidjson::Value adhesion_para(rapidjson::kObjectType);
	adhesion_para.AddMember("switch", rapidjson::Value(param.switch_adhesion), allocator);
	adhesion_para.AddMember("min horizon angle", rapidjson::Value(param.angle_hor_min_adhesion), allocator);
	adhesion_para.AddMember("max horizon angle", rapidjson::Value(param.angle_hor_max_adhesion), allocator);
	adhesion_para.AddMember("min vertical angle", rapidjson::Value(param.angle_ver_min_adhesion), allocator);
	adhesion_para.AddMember("max vertical angle", rapidjson::Value(param.angle_ver_max_adhesion), allocator);
	adhesion_para.AddMember("horizon angle resolution", rapidjson::Value(param.angle_hor_res_adhesion), allocator);
	adhesion_para.AddMember("vertical angle resolution", rapidjson::Value(param.angle_ver_res_adhesion), allocator);
	adhesion_para.AddMember("diff thres", rapidjson::Value(param.diff_thre_adhesion), allocator);
	adhesion_para.AddMember("max dist", rapidjson::Value(param.dist_limit_adhesion), allocator);
	adhesion_para.AddMember("min diff", rapidjson::Value(param.min_diff_adhesion), allocator);
	document.AddMember("adhesion para", adhesion_para, allocator);

	rapidjson::Value dirty_detect(rapidjson::kObjectType);
	dirty_detect.AddMember("detect switch", rapidjson::Value(param.switch_dirty_detect), allocator);
	dirty_detect.AddMember("refresh switch", rapidjson::Value(param.switch_dirty_refresh), allocator);
	dirty_detect.AddMember("refresh cycle", rapidjson::Value(param.dirty_refresh_cycle), allocator);
	dirty_detect.AddMember("detect set thres", rapidjson::Value(param.dirty_detect_set_thre), allocator);
	dirty_detect.AddMember("detect reset thres", rapidjson::Value(param.dirty_detect_reset_thre), allocator);
	dirty_detect.AddMember("detect inner thres", rapidjson::Value(param.dirty_detect_inner_thre), allocator);
	dirty_detect.AddMember("detect outer thres", rapidjson::Value(param.dirty_detect_outer_thre), allocator);
	document.AddMember("dirty detect", dirty_detect, allocator);

	rapidjson::Value delete_near_point(rapidjson::kObjectType);
	delete_near_point.AddMember("switch", rapidjson::Value(param.switch_near_point_delete), allocator);
	document.AddMember("delete near point", delete_near_point, allocator);

	rapidjson::Value downsample_mode(rapidjson::kObjectType);
	downsample_mode.AddMember("mode", rapidjson::Value(param.down_sample_mode), allocator);
	document.AddMember("downsample mode", downsample_mode, allocator);

	rapidjson::Value echo_mode(rapidjson::kObjectType);
	echo_mode.AddMember("mode", rapidjson::Value(param.echo_mode), allocator);
	document.AddMember("echo mode", echo_mode, allocator);

	rapidjson::Value ptp_sync(rapidjson::kObjectType);
	ptp_sync.AddMember("switch", rapidjson::Value(param.ptp_sync), allocator);
	document.AddMember("ptp sync", ptp_sync, allocator);

	rapidjson::Value frame_sync(rapidjson::kObjectType);
	frame_sync.AddMember("switch", rapidjson::Value(param.frame_sync), allocator);
	frame_sync.AddMember("offset", rapidjson::Value(param.frame_offset), allocator);
	document.AddMember("frame sync", frame_sync, allocator);

	rapidjson::Value angle_send(rapidjson::kObjectType);
	angle_send.AddMember("switch", rapidjson::Value(param.angle_send), allocator);
	document.AddMember("angle send", angle_send, allocator);

	// get string buffer
	document.Accept(writer);
	context = buffer.GetString();

	return 0;
}
