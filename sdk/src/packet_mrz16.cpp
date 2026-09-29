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


#include "packet_mrz16.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include "loguru.hpp"
#include "point_cloud.h"
#include "serial_client.h"

namespace zvision
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        int64_t steady_now_ms()
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
        }

        uint64_t wall_now_us()
        {
            return static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count());
        }

        // Decode the 6-byte UTC time + 4-byte us-of-second carried by every frame
        // (year stored as year-1900). Returns epoch-microseconds or 0 when invalid.
        uint64_t data_header_utc_us(const mrz16_data_header& h)
        {
            const int year = 1900 + static_cast<int>(h.utc_time.year);
            const unsigned month = h.utc_time.month;
            const unsigned day = h.utc_time.day;
            const unsigned hour = h.utc_time.hour;
            const unsigned minute = h.utc_time.minute;
            const unsigned second = h.utc_time.second;
            if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 ||
                hour > 23 || minute > 59 || second > 59)
            {
                return 0;
            }

            // days_from_civil (Hinnant)
            const int y = month <= 2 ? year - 1 : year;
            const unsigned era = (y >= 0 ? y : y - 399) / 400;
            const unsigned yoe = static_cast<unsigned>(y - era * 400);
            const long mm = static_cast<long>(month) + (month > 2 ? -3 : 9);
            const unsigned doy = static_cast<unsigned>((153 * mm + 2) / 5) + day - 1u;
            const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
            const uint64_t days = static_cast<uint64_t>(era) * 146097u + doe - 719468u;

            const uint64_t seconds = days * 86400u + hour * 3600u + minute * 60u + second;
            return seconds * 1000000ull + h.timestamp_us;
        }

        uint64_t data_header_utc_us(const uint8_t* p)
        {
            mrz16_data_header h{};
            std::memcpy(&h, p, sizeof(h));
            return data_header_utc_us(h);
        }

        ////////////////////////////////////////////////////////////////////////
        // $LDCMD building / $LDACK extraction (ported from LidarCalibStudio)
        ////////////////////////////////////////////////////////////////////////

        // CRC32 MPEG-2 table
        const uint32_t kCrc32Mpeg2Table[0x100] = {
            0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
            0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
            0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
            0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
            0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
            0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
            0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
            0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
            0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
            0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
            0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
            0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
            0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
            0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
            0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
            0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
            0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
            0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
            0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
            0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
            0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
            0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
            0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
            0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
            0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
            0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
            0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
            0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
            0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
            0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
            0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
            0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4,
        };

        uint32_t compute_crc32_mpeg2(const uint8_t* data, size_t len)
        {
            uint32_t checksum = 0xFFFFFFFFu;
            size_t i = 0;
            for (; i < len; ++i)
            {
                const uint8_t top = static_cast<uint8_t>((checksum >> 24) ^ data[i]);
                checksum = (checksum << 8) ^ kCrc32Mpeg2Table[top];
            }
            while (i % 4 > 0)
            {
                const uint8_t top = static_cast<uint8_t>(checksum >> 24);
                checksum = (checksum << 8) ^ kCrc32Mpeg2Table[top];
                ++i;
            }
            return checksum;
        }

        std::vector<uint8_t> build_read_channel_angles()
        {
            // LidarCmdSender::cmdChannelParameters_Read -> type 0x05, id 0x0E, sub 0x00
            const int data_payload_size = 3 + 4;

            std::vector<uint8_t> body;
            body.reserve(static_cast<size_t>(data_payload_size) + 1);
            body.push_back(static_cast<uint8_t>(data_payload_size - 4));  // = 3
            body.push_back(0x05);   // cmd_type
            body.push_back(0x0E);   // cmd_id
            body.push_back(0x00);   // sub_id
            body.push_back(0x78);   // check id
            body.push_back(0x56);
            body.push_back(0x43);
            body.push_back(0x21);

            const uint32_t crc = compute_crc32_mpeg2(body.data(), body.size());

            std::vector<uint8_t> packet;
            packet.reserve(6 + body.size() + 4 + 2);
            const char* hdr = "$LDCMD,";
            packet.insert(packet.end(), hdr, hdr + 7);
            packet.insert(packet.end(), body.begin(), body.end());
            packet.push_back(static_cast<uint8_t>(crc & 0xFF));
            packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
            packet.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
            packet.push_back(static_cast<uint8_t>((crc >> 24) & 0xFF));
            packet.push_back(0xEE);
            packet.push_back(0xFF);
            return packet;
        }

        constexpr size_t kLdackHdrLen = 7;        // "$LDACK,"
        constexpr size_t kLdackCheckIdLen = 4;
        constexpr size_t kLdackCrcLen = 4;
        constexpr size_t kLdackTermLen = 2;       // trailing 0xEE 0xFF
        constexpr size_t kLdackTailLen = kLdackCheckIdLen + kLdackCrcLen + kLdackTermLen;
        constexpr uint8_t kLdackTerm0 = 0xEE;
        constexpr uint8_t kLdackTerm1 = 0xFF;
        // CheckID observed on the serial command port (COM31@9600). A different device
        // variant would need this constant updated.
        constexpr uint32_t kLdackCheckId = 0x21435678u;
        // type + id + sub + count (4) plus the single trailing reserved byte the device
        // appends after the last angle pair.
        constexpr size_t kChannelAnglesAckFixedPayload = 5;
        const char kLdackHdr[] = "$LDACK,";

        // Defined below, needed by the resync logic in extract_ldack_frame().
        bool verify_ldack_frame(const uint8_t* d, size_t len, std::string* why);

        bool extract_ldack_frame(std::vector<uint8_t>* buf, std::vector<uint8_t>* frame)
        {
            if (!buf || !frame)
            {
                return false;
            }
            // Rejection warnings are throttled to one per call: a healthy link never
            // rejects a frame, while a broken one would otherwise flood the log.
            bool warned = false;
            for (;;)
            {
                // Need the header plus the length byte before anything can be decided.
                if (buf->size() < kLdackHdrLen + 1)
                {
                    return false;
                }
                size_t start = 0;
                bool found = false;
                for (; start + kLdackHdrLen <= buf->size(); ++start)
                {
                    if (std::memcmp(buf->data() + start, kLdackHdr, kLdackHdrLen) == 0)
                    {
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    // Discard point-cloud noise, keep a partial header suffix
                    if (buf->size() > kLdackHdrLen - 1)
                    {
                        buf->erase(buf->begin(), buf->end() - static_cast<std::ptrdiff_t>(kLdackHdrLen - 1));
                    }
                    return false;
                }
                if (start > 0)
                {
                    buf->erase(buf->begin(), buf->begin() + static_cast<std::ptrdiff_t>(start));
                }
                if (buf->size() < kLdackHdrLen + 1)
                {
                    return false;
                }
                const uint8_t data_len = (*buf)[kLdackHdrLen];
                const size_t frame_len = kLdackHdrLen + 1 + static_cast<size_t>(data_len) + kLdackTailLen;
                if (buf->size() < frame_len)
                {
                    return false;
                }
                std::string why;
                if (verify_ldack_frame(buf->data(), frame_len, &why))
                {
                    frame->assign(buf->begin(), buf->begin() + static_cast<std::ptrdiff_t>(frame_len));
                    buf->erase(buf->begin(), buf->begin() + static_cast<std::ptrdiff_t>(frame_len));
                    return true;
                }

                // Reject rather than trust a bad slice. Drop exactly one byte and resync,
                // so a genuine frame that starts later -- including one buried inside this
                // rejected window -- is still recovered instead of being skipped.
                if (!warned)
                {
                    LOG_F(WARNING, "[packet_mrz16] rejecting %zub $LDACK slice (%s), resyncing",
                          frame_len, why.c_str());
                    warned = true;
                }
                buf->erase(buf->begin());
            }
        }

        bool is_ldack_with_cmd_id(const uint8_t* data, size_t len, uint8_t cmd_id)
        {
            if (!data || len < 10)
            {
                return false;
            }
            if (std::memcmp(data, kLdackHdr, kLdackHdrLen) != 0)
            {
                return false;
            }
            return data[9] == cmd_id;
        }

        // pcap record-0 angle-metadata header for offline MRZ16 replay.
        constexpr char kMrz16MetaCompany[3] = {'Z', 'V', 'S'};
        constexpr char kMrz16MetaModel[9]   = {'M', 'R', 'Z', '1', '6', ' ', ' ', ' ', ' '};
        constexpr char kMrz16MetaPacket[5]  = {'A', 'N', 'G', 'L', 'E'};
        constexpr uint8_t kMrz16MetaVerMajor = 1;
        constexpr uint8_t kMrz16MetaVerMinor = 0;
        constexpr int kMrz16MetaHeaderLen = 3 + 9 + 5 + 1 + 1; // 19

        // Decode the device's native angle payload: int32 count followed by
        // count * (int32 horizontal_deg*100, int32 vertical_deg*100), little-endian.
        // Shared by the live $LDACK path and the offline pcap record-0 path.
        bool decode_channel_angles(const uint8_t* ptr, int count, mrz16_channel_angles* out)
        {
            if (!ptr || !out || count < 1 || count > mrz16_channel_count)
            {
                return false;
            }
            out->count = count;
            for (int i = 0; i < count; ++i)
            {
                int32_t h = 0;
                int32_t v = 0;
                std::memcpy(&h, ptr + i * 8,     sizeof(h));
                std::memcpy(&v, ptr + i * 8 + 4, sizeof(v));
                out->horizontal_deg[i] = h / 100.0;
                out->vertical_deg[i]   = v / 100.0;
            }
            return true;
        }

        // Mirror a decoded mrz16_channel_angles table into the angle_comp_t owned by
        // the caller (PointCloudProducer::angle_comp_). Shared by the CSV loader and
        // the live $LDACK path so both write through the same single source of truth.
        bool fill_angle_comp(const mrz16_channel_angles& src, angle_comp_t* out)
        {
            if (!out || src.count < 1)
            {
                return false;
            }
            out->azi.assign(static_cast<size_t>(src.count), 0.0f);
            out->ele.assign(static_cast<size_t>(src.count), 0.0f);
            for (int i = 0; i < src.count; ++i)
            {
                out->azi[i] = static_cast<float>(src.horizontal_deg[i]);
                out->ele[i] = static_cast<float>(src.vertical_deg[i]);
            }
            return true;
        }

        // ---------------------------------------------------------------------
        // Strict acceptance gate for a candidate $LDACK slice. Every check below must
        // pass before a slice is handed to the angle parser, so a corrupted, misaligned
        // or point-cloud-polluted one is rejected instead of silently producing wrong
        // channel angles. Layout is the one confirmed by a live COM31@9600 capture
        // (CRC32 checks out, decoded angles match doc/mrz16_angle.csv row for row):
        //   [0..6]    "$LDACK,"
        //   [7]       len                       -> payload length declared by device
        //   [8]       cmd_type
        //   [9]       cmd_id
        //   [10]      sub_id
        //   [11]      count (u8)                -> angle pairs start at [12]
        //   [12..]    count * (int32 H*100, int32 V*100), little-endian
        //   [..]      1 reserved byte           -> payload is 5 + count*8, not 4 + ...
        //   tail      CheckID u32 LE + CRC32-MPEG2 u32 LE over [7, len-6) + EE FF
        // ---------------------------------------------------------------------

        uint32_t read_u32_le(const uint8_t* d)
        {
            return static_cast<uint32_t>(d[0]) | (static_cast<uint32_t>(d[1]) << 8) |
                   (static_cast<uint32_t>(d[2]) << 16) | (static_cast<uint32_t>(d[3]) << 24);
        }

        void append_hex_u32(std::string* s, uint32_t v)
        {
            static const char kHex[] = "0123456789ABCDEF";
            for (int shift = 28; shift >= 0; shift -= 4)
            {
                s->push_back(kHex[(v >> shift) & 0x0Fu]);
            }
        }

        bool verify_ldack_frame(const uint8_t* d, size_t len, std::string* why)
        {
            if (!d || len < kLdackHdrLen + 1 + kLdackTailLen)
            {
                if (why) { *why = "truncated frame"; }
                return false;
            }
            if (std::memcmp(d, kLdackHdr, kLdackHdrLen) != 0)
            {
                if (why) { *why = "bad header"; }
                return false;
            }
            if (d[len - kLdackTermLen] != kLdackTerm0 || d[len - 1] != kLdackTerm1)
            {
                if (why) { *why = "missing EE FF terminator"; }
                return false;
            }

            // CRC32-MPEG2 covers [len byte .. CheckID] = [kLdackHdrLen, crc_pos).
            const size_t crc_pos = len - kLdackCrcLen - kLdackTermLen;
            const uint32_t crc_calc = compute_crc32_mpeg2(d + kLdackHdrLen, crc_pos - kLdackHdrLen);
            const uint32_t crc_rx = read_u32_le(d + crc_pos);
            if (crc_calc != crc_rx)
            {
                if (why)
                {
                    *why = "crc32 mismatch calc=";
                    append_hex_u32(why, crc_calc);
                    *why += " rx=";
                    append_hex_u32(why, crc_rx);
                }
                return false;
            }

            const uint32_t check_id = read_u32_le(d + len - kLdackTailLen);
            if (check_id != kLdackCheckId)
            {
                if (why)
                {
                    *why = "check_id=";
                    append_hex_u32(why, check_id);
                    *why += " want=";
                    append_hex_u32(why, kLdackCheckId);
                }
                return false;
            }
            return true;
        }

        bool parse_channel_angles_ack(const uint8_t* data, size_t len, mrz16_channel_angles* out)
        {
            if (!data || !out || len < 12)
            {
                return false;
            }
            if (std::memcmp(data, kLdackHdr, kLdackHdrLen) != 0)
            {
                return false;
            }
            if (data[8] != 0x05 || data[9] != 0x0E)
            {
                return false;
            }
            const int count = static_cast<int>(data[11]);
            if (count < 1 || count > mrz16_channel_count)
            {
                return false;
            }
            // Length self-consistency: the device declares its own payload length at [7].
            // For this reply it must be exactly type+id+sub+count + count*8 pairs + one
            // reserved byte, and the frame must be exactly that plus header/len/tail.
            // Anything else means this is not a genuine channel-angle reply, even when
            // the individual bytes happen to look plausible.
            const size_t pairs = static_cast<size_t>(count) * 8;
            const size_t want_payload = pairs + kChannelAnglesAckFixedPayload;
            const size_t want_frame = want_payload + kLdackHdrLen + 1 + kLdackTailLen;
            if (static_cast<size_t>(data[7]) != want_payload || len != want_frame)
            {
                LOG_F(WARNING, "[packet_mrz16] channel-angle ACK length mismatch: "
                               "declared payload=%u, want %zu; frame=%zu, want %zu",
                      data[7], want_payload, len, want_frame);
                return false;
            }
            if (len < 12 + pairs)
            {
                return false;
            }
            return decode_channel_angles(data + 12, count, out);
        }

        ////////////////////////////////////////////////////////////////////////
        // channel-angle file parsing (CSV, one row per channel)
        ////////////////////////////////////////////////////////////////////////

        bool ends_with(const std::string& s, const std::string& suffix)
        {
            return s.size() >= suffix.size() &&
                   s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
        }

        std::string trimmed(const std::string& s)
        {
            const auto b = s.find_first_not_of(" \t\r\n");
            if (b == std::string::npos)
            {
                return "";
            }
            const auto e = s.find_last_not_of(" \t\r\n");
            return s.substr(b, e - b + 1);
        }

        std::vector<std::string> split_csv_fields(const std::string& s)
        {
            std::vector<std::string> fields;
            if (s.find(',') == std::string::npos)
            {
                // Tolerate whitespace-only separated values as well.
                std::istringstream iss(s);
                std::string tok;
                while (iss >> tok)
                {
                    fields.push_back(tok);
                }
                return fields;
            }
            std::string cur;
            for (char c : s)
            {
                if (c == ',')
                {
                    fields.push_back(trimmed(cur));
                    cur.clear();
                }
                else
                {
                    cur.push_back(c);
                }
            }
            fields.push_back(trimmed(cur));
            return fields;
        }

        bool parse_f64(const std::string& s, double* out)
        {
            std::istringstream iss(s);
            double v = 0.0;
            if (!(iss >> v))
            {
                return false;
            }
            std::string rest;
            if (iss >> rest)
            {
                return false;
            }
            *out = v;
            return true;
        }

        // csv: '<horizontal_deg>,<vertical_deg>' one row per channel. There is no
        // channel-number column: the zero-based row index is the channel number.
        bool parse_angles_csv(const std::string& path, mrz16_channel_angles* out, std::string* err)
        {
            std::ifstream in(path);
            if (!in.is_open())
            {
                *err = "cannot open file: " + path;
                return false;
            }
            mrz16_channel_angles angles{};
            std::string line;
            int channel = 0;
            int lineno = 0;
            while (std::getline(in, line))
            {
                ++lineno;
                const std::string t = trimmed(line);
                if (t.empty() || t[0] == '#')
                {
                    continue;
                }
                if (channel >= mrz16_channel_count)
                {
                    *err = "line " + std::to_string(lineno) + ": more rows than channels (" +
                           std::to_string(mrz16_channel_count) + ")";
                    return false;
                }
                const std::vector<std::string> fields = split_csv_fields(t);
                if (fields.size() < 2)
                {
                    *err = "line " + std::to_string(lineno) + ": expected '<H>,<V>'";
                    return false;
                }
                if (fields.size() > 2)
                {
                    *err = "line " + std::to_string(lineno) +
                           ": unexpected extra column; remove the channel-number column";
                    return false;
                }
                double h_deg = 0.0;
                double v_deg = 0.0;
                if (!parse_f64(fields[0], &h_deg) || !parse_f64(fields[1], &v_deg))
                {
                    *err = "line " + std::to_string(lineno) + ": non-numeric angle value";
                    return false;
                }
                angles.horizontal_deg[channel] = h_deg;
                angles.vertical_deg[channel] = v_deg;
                ++channel;
            }
            if (channel < 1)
            {
                *err = "no angle rows found";
                return false;
            }
            angles.count = channel;
            *out = angles;
            return true;
        }
    } // namespace

    ////////////////////////////////////////////////////////////////////////////
    // pkg_parse_mrz16 : PointCloudPacket interface
    ////////////////////////////////////////////////////////////////////////////

    bool pkg_parse_mrz16::IsValidPacket(std::string& packet)
    {
        return packet.size() == static_cast<size_t>(mrz16_point_cloud_frame_len) &&
               static_cast<uint8_t>(packet[0]) == 0xEE &&
               static_cast<uint8_t>(packet[1]) == 0xFF &&
               static_cast<uint8_t>(packet[5]) == 0x00;
    }

    DeviceType pkg_parse_mrz16::GetDeviceType(std::string& packet)
    {
        return LidarMRZ16;
    }

    int pkg_parse_mrz16::GetFrameNum(std::string& packet)
    {
        if (!IsValidPacket(packet))
        {
            return -1;
        }
        // udp_sequence little-endian at offset 74..75
        return static_cast<int>(
            static_cast<uint8_t>(packet[74]) |
            (static_cast<uint8_t>(packet[75]) << 8));
    }

    ScanMode pkg_parse_mrz16::GetScanMode(std::string& packet)
    {
        return ScanMode::ScanUnknown;
    }

    int pkg_parse_mrz16::GetPacketSeq(std::string& packet)
    {
        return GetFrameNum(packet);
    }

    bool pkg_parse_mrz16::GetPkgAngle(std::string& packet, double* az_deg)
    {
        // Caller guarantees a valid point-cloud packet (PcapIndexer::open checks
        // IsValidPacket first). Read body.azimuth directly: it sits at offset 16
        // (header 6 + data_header 10), uint16 LE, raw value * 0.01 deg. No full
        // frame decode and no validity check here.
        uint16_t raw = 0;
        std::memcpy(&raw, packet.data() + 16, sizeof(raw));
        if (az_deg)
        {
            *az_deg = static_cast<double>(raw) * mrz16_azimuth_scale_deg;
        }
        return true;
    }

    uint64_t pkg_parse_mrz16::GetTimestamp(uint8_t* data)
    {
        if (!data)
        {
            return 0;
        }
        // data points to the start of a MRZ16 frame; data header starts at offset 6.
        return data_header_utc_us(data + 6);
    }

    int pkg_parse_mrz16::ProcessPacket(std::string& packet, angle_comp_t* angle_comp, PointCloud& cloud)
    {
        if (packet.size() != static_cast<size_t>(mrz16_point_cloud_frame_len))
        {
            return 0;
        }

        mrz16_point_cloud_packet pkt{};
        std::memcpy(&pkt, packet.data(), mrz16_point_cloud_frame_len);

        const double az_deg = pkt.body.azimuth * mrz16_azimuth_scale_deg;

        // A new revolution begins when the azimuth counter wraps 360 -> 0.
        if (last_azimuth_deg_ >= 300.0 && az_deg < 60.0)
        {
            // A revolution just ended: tell the producer to flush the accumulated
            // cloud. This frame will be re-processed into the next revolution.
            last_azimuth_deg_ = az_deg;
            // this frame is the first column of the new revolution
            col_base_seq_ = pkt.body.udp_sequence;
            return 2;
        }

        if (last_azimuth_deg_ < 0.0)
        {
            // very first frame after start: it defines the origin of the columns
            col_base_seq_ = pkt.body.udp_sequence;
        }
        last_azimuth_deg_ = az_deg;

        // Same entry point as the other lidars: one packet -> points in cloud.
        // MRZ16 renders from the angle_comp_t vectors (azi = horizontal, ele = vertical).
        static std::vector<float> empty_azi_comp;
        static std::vector<float> empty_ele_comp;
        Pkg2Points(packet,
                   (angle_comp != nullptr) ? angle_comp->azi : empty_azi_comp,
                   (angle_comp != nullptr) ? angle_comp->ele : empty_ele_comp,
                   cloud);
        return 0;
    }

    void pkg_parse_mrz16::Pkg2Points(std::string& packet, std::vector<float>& v_azi_comp,
                                     std::vector<float>& v_ele_comp, PointCloud& cloud)
    {
        // MRZ16 geometry is driven by the per-channel angles carried in angle_comp_t:
        // v_azi_comp holds the horizontal angle and v_ele_comp the vertical angle for
        // each channel, taken from the loaded angle table (file / $LDCMD / offline pcap).

        if (packet.size() != static_cast<size_t>(mrz16_point_cloud_frame_len))
        {
            return;
        }

        mrz16_point_cloud_packet pkt{};
        std::memcpy(&pkt, packet.data(), mrz16_point_cloud_frame_len);

        cloud.dev_type = LidarMRZ16;
        cloud.scan_mode = ScanMode::ScanUnknown;

        const uint64_t utc_us = data_header_utc_us(pkt.data_header);
        if (utc_us != 0)
        {
            cloud.stamp_us = utc_us;
        }
        else
        {
            // Device left the UTC header empty on this frame; fall back to the
            // host wall clock right here so timestamps are never 0. This mirrors
            // the IMU path below (ParseImuPkg: utc_us != 0 ? utc_us : wall_now_us()).
            cloud.stamp_us = wall_now_us();
        }

        const double disc_deg = pkt.body.azimuth * mrz16_azimuth_scale_deg;

        // Column index of this frame, taken from the packet sequence carried by the
        // frame itself and measured against the first packet of the current
        // revolution (uint16 wrap safe). A dropped packet therefore leaves a gap in
        // the column indices instead of shifting every following column.
        const int col = (static_cast<int>(pkt.body.udp_sequence) -
                         static_cast<int>(col_base_seq_)) & 0xFFFF;

        // Optical-center offset per the protocol: delta_l = sqrt(7^2 + 13.86^2) mm,
        // delta_theta = atan(7 / 13.86).
        const double horizontal_offset =
            std::sqrt(mrz16_x_offset_m * mrz16_x_offset_m + mrz16_y_offset_m * mrz16_y_offset_m);
        const double horizontal_offset_rad = std::atan2(mrz16_x_offset_m, mrz16_y_offset_m);

        for (int ch = 0; ch < mrz16_channel_count; ++ch)
        {
            const double dist_m = pkt.body.channels[ch].distance * mrz16_distance_scale_m;
            if (dist_m < min_distance_m_ || dist_m > max_distance_m_)
            {
                continue;
            }

            double vertical_deg = 0.0;
            double horizontal_deg = 0.0;
            if (ch < static_cast<int>(v_azi_comp.size()))
            {
                horizontal_deg = static_cast<double>(v_azi_comp[ch]);
                vertical_deg = static_cast<double>(v_ele_comp[ch]);
            }

            const double vertical_rad = vertical_deg * kPi / 180.0;
            const double horizontal_rad = horizontal_deg * kPi / 180.0;
            const double disc_rad = disc_deg * kPi / 180.0;

            // LidarModel_API SingleLineLaserModel::apply
            const double theta_rad = horizontal_rad + disc_rad;
            const double theta_off_rad = theta_rad + horizontal_offset_rad;

            zvision::Point pt;
            pt.rowCnt = static_cast<uint16_t>(mrz16_channel_count);
            pt.colCnt = static_cast<uint16_t>(mrz16_columns_per_revolution);
            pt.x = static_cast<float>(dist_m * std::cos(vertical_rad) * std::sin(theta_rad) +
                                      horizontal_offset * std::sin(theta_off_rad));
            pt.y = static_cast<float>(dist_m * std::cos(vertical_rad) * std::cos(theta_rad) +
                                      horizontal_offset * std::cos(theta_off_rad));
            pt.z = static_cast<float>(dist_m * std::sin(vertical_rad) + mrz16_z_offset_m);
            pt.distance = static_cast<float>(dist_m);
            pt.reflectivity = static_cast<int>(pkt.body.channels[ch].reflectivity);
            pt.valid = 1;
            pt.row = ch;
            pt.col = col;
            pt.pointid = col*16+ch;
            pt.channel = static_cast<uint16_t>(ch);
            pt.line_id = ch;
            pt.azimuth = static_cast<float>(disc_rad);
            pt.elevation = static_cast<float>(vertical_rad);
            pt.timestamp_us = cloud.stamp_us;
            cloud.points.push_back(pt);
        }
    }

    bool pkg_parse_mrz16::IsValidAnglePacket(std::string& packet)
    {
        // MRZ16 does not carry angles over UDP, but an offline pcap can embed a
        // self-contained angle packet (the same role as nz1_a2's UDP angle packets).
        // Detect it by the meta-record header we write at record time.
        const uint8_t* data = reinterpret_cast<const uint8_t*>(packet.data());
        const int len = static_cast<int>(packet.size());
        if (len < kMrz16MetaHeaderLen)
        {
            return false;
        }
        if (std::memcmp(data, kMrz16MetaCompany, 3) != 0)
        {
            return false;
        }
        if (std::memcmp(data + 3, kMrz16MetaModel, 9) != 0)
        {
            return false;
        }
        if (std::memcmp(data + 12, kMrz16MetaPacket, 5) != 0)
        {
            return false;
        }
        if (data[17] != kMrz16MetaVerMajor || data[18] != kMrz16MetaVerMinor)
        {
            return false;
        }
        return true;
    }

    int pkg_parse_mrz16::ParseAnglePkg(std::string& packet, angle_comp_t& angle_comp)
    {
        // The packet has already been validated by IsValidAnglePacket() in
        // PcapIndexer::open(); here we only decode the per-channel angle table and
        // mirror it into angle_comp_t so it rides the generic
        // Offline_update_angle_comp() path to the producer's parser for rendering.
        const uint8_t* data = reinterpret_cast<const uint8_t*>(packet.data());
        const int len = static_cast<int>(packet.size());
        if (len < kMrz16MetaHeaderLen + 4)
        {
            return 0;
        }
        const int payload_len = len - kMrz16MetaHeaderLen;
        int32_t count = 0;
        std::memcpy(&count, data + kMrz16MetaHeaderLen, 4);
        if (count < 1 || count > mrz16_channel_count)
        {
            return 0;
        }
        if (payload_len < 4 + count * 8)
        {
            return 0;
        }
        mrz16_channel_angles tmp{};
        if (!decode_channel_angles(data + kMrz16MetaHeaderLen + 4, count, &tmp))
        {
            return 0;
        }
        angle_comp.azi.assign(static_cast<size_t>(tmp.count), 0.0f);
        angle_comp.ele.assign(static_cast<size_t>(tmp.count), 0.0f);
        for (int i = 0; i < tmp.count; ++i)
        {
            angle_comp.azi[i] = static_cast<float>(tmp.horizontal_deg[i]);
            angle_comp.ele[i] = static_cast<float>(tmp.vertical_deg[i]);
        }
        return 1;
    }

    bool pkg_parse_mrz16::IsValidImuPacket(std::string& packet)
    {
        return packet.size() == static_cast<size_t>(mrz16_imu_frame_len) &&
               static_cast<uint8_t>(packet[0]) == 0xEE &&
               static_cast<uint8_t>(packet[1]) == 0xFF &&
               static_cast<uint8_t>(packet[5]) == 0x01;
    }

    int pkg_parse_mrz16::ParseImuPkg(std::string& packet, imu_data_t& imu_data)
    {
        if (packet.size() != static_cast<size_t>(mrz16_imu_frame_len))
        {
            return -1;
        }
        mrz16_imu_packet pkt{};
        std::memcpy(&pkt, packet.data(), mrz16_imu_frame_len);

        imu_data.acc_x = static_cast<float>(pkt.body.acc_x * mrz16_imu_acc_scale);
        imu_data.acc_y = static_cast<float>(pkt.body.acc_y * mrz16_imu_acc_scale);
        imu_data.acc_z = static_cast<float>(pkt.body.acc_z * mrz16_imu_acc_scale);
        imu_data.gyro_x = static_cast<float>(pkt.body.gyro_x * mrz16_imu_gyro_scale);
        imu_data.gyro_y = static_cast<float>(pkt.body.gyro_y * mrz16_imu_gyro_scale);
        imu_data.gyro_z = static_cast<float>(pkt.body.gyro_z * mrz16_imu_gyro_scale);

        const uint64_t utc_us = data_header_utc_us(pkt.data_header);
        imu_data.timestamp_us = (utc_us != 0) ? utc_us : wall_now_us();
        return 0;
    }

    ////////////////////////////////////////////////////////////////////////////
    // pkg_parse_mrz16 : serial helpers used by PointCloudProducer
    ////////////////////////////////////////////////////////////////////////////

    bool pkg_parse_mrz16::ExtractNextFrame(std::string& stream, std::string* frame)
    {
        if (!frame)
        {
            return false;
        }
        while (stream.size() >= 2)
        {
            const uint8_t a = static_cast<uint8_t>(stream[0]);
            const uint8_t b = static_cast<uint8_t>(stream[1]);
            if (!(a == 0xEE && b == 0xFF))
            {
                stream.erase(0, 1);
                continue;
            }
            if (stream.size() < 6)
            {
                // partial header, wait for more bytes
                return false;
            }
            const uint8_t data_type = static_cast<uint8_t>(stream[5]);
            size_t frame_len = 0;
            if (data_type == 0x00)
            {
                frame_len = static_cast<size_t>(mrz16_point_cloud_frame_len);
            }
            else if (data_type == 0x01)
            {
                frame_len = static_cast<size_t>(mrz16_imu_frame_len);
            }
            else
            {
                // EE FF with an unknown type: drop one byte and resync
                stream.erase(0, 1);
                continue;
            }
            if (stream.size() < frame_len)
            {
                // incomplete frame, wait for more bytes
                return false;
            }
            frame->assign(stream, 0, frame_len);
            stream.erase(0, frame_len);
            return true;
        }
        return false;
    }

    bool pkg_parse_mrz16::LoadChannelAnglesFile(const std::string& path, angle_comp_t* out,
                                                std::string* err)
    {
        if (err)
        {
            err->clear();
        }
        if (path.empty())
        {
            if (err)
            {
                *err = "empty angle file path";
            }
            return false;
        }
        if (!out)
        {
            if (err)
            {
                *err = "null angle output";
            }
            return false;
        }

        if (!ends_with(path, ".csv"))
        {
            if (err)
            {
                *err = "angle file must be a CSV: " + path;
            }
            return false;
        }

        mrz16_channel_angles angles{};
        if (!parse_angles_csv(path, &angles, err))
        {
            return false;
        }

        if (!fill_angle_comp(angles, out))
        {
            if (err)
            {
                *err = "angle file carries no channel";
            }
            return false;
        }

        LOG_F(INFO, "[packet_mrz16] loaded %d channels from %s (csv)",
              static_cast<int>(out->azi.size()), path.c_str());
        for (size_t i = 0; i < out->azi.size(); ++i)
        {
            LOG_F(INFO, "  ch%02d H=%.3f V=%.3f", static_cast<int>(i),
                  out->azi[i], out->ele[i]);
        }
        return true;
    }

    void pkg_parse_mrz16::ResetRevolutionState()
    {
        last_azimuth_deg_ = -1.0;
        col_base_seq_ = 0;
    }

    void pkg_parse_mrz16::ResetParserState()
    {
        ResetRevolutionState();
    }

    bool pkg_parse_mrz16::GetAngleMetaRecord(const angle_comp_t& angle_comp,
                                             std::vector<uint8_t>& out) const
    {
        if (angle_comp.azi.empty())
        {
            return false;
        }
        BuildAngleMetaRecord(angle_comp, out);
        return true;
    }

    void pkg_parse_mrz16::BuildAngleMetaRecord(const angle_comp_t& ac, std::vector<uint8_t>& out)
    {
        const int count = static_cast<int>(ac.azi.size());
        out.clear();
        out.resize(kMrz16MetaHeaderLen + 4 + count * 8);
        std::memcpy(out.data(), kMrz16MetaCompany, 3);
        std::memcpy(out.data() + 3, kMrz16MetaModel, 9);
        std::memcpy(out.data() + 12, kMrz16MetaPacket, 5);
        out[17] = kMrz16MetaVerMajor;
        out[18] = kMrz16MetaVerMinor;
        int32_t c = count;
        std::memcpy(out.data() + kMrz16MetaHeaderLen, &c, 4);
        for (int i = 0; i < count; ++i)
        {
            int32_t h = static_cast<int32_t>(std::lround(ac.azi[i] * 100.0));
            int32_t v = static_cast<int32_t>(std::lround(ac.ele[i] * 100.0));
            std::memcpy(out.data() + kMrz16MetaHeaderLen + 4 + i * 8,     &h, 4);
            std::memcpy(out.data() + kMrz16MetaHeaderLen + 4 + i * 8 + 4, &v, 4);
        }
    }

    bool pkg_parse_mrz16::FetchChannelAnglesOverSerial(zvision::SerialClient* serial,
                                                       angle_comp_t* out, std::string* err)
    {
        if (err)
        {
            err->clear();
        }
        if (!serial)
        {
            if (err)
            {
                *err = "serial not initialized";
            }
            return false;
        }
        if (!out)
        {
            if (err)
            {
                *err = "null angle output";
            }
            return false;
        }

        const std::vector<uint8_t> cmd = build_read_channel_angles();
        std::string cmd_str(reinterpret_cast<const char*>(cmd.data()), cmd.size());

        constexpr int kRetries = 3;
        constexpr int kTimeoutMs = 3000;
        constexpr uint8_t kChannelAnglesCmdId = 0x0E;

        std::vector<uint8_t> ack_buf;
        for (int attempt = 1; attempt <= kRetries; ++attempt)
        {
            serial->FlushRecvInput();
            ack_buf.clear();

            if (serial->SyncSend(cmd_str, static_cast<int>(cmd_str.size())) != 0)
            {
                if (err)
                {
                    *err = "failed to write $LDCMD on the command port";
                }
                return false;
            }

            const int64_t deadline_ms = steady_now_ms() + kTimeoutMs;
            while (steady_now_ms() < deadline_ms)
            {
                std::vector<uint8_t> frame;
                if (extract_ldack_frame(&ack_buf, &frame))
                {
                    if (is_ldack_with_cmd_id(frame.data(), frame.size(), kChannelAnglesCmdId))
                    {
                        mrz16_channel_angles angles{};
                        if (parse_channel_angles_ack(frame.data(), frame.size(), &angles) &&
                            fill_angle_comp(angles, out))
                        {
                            LOG_F(INFO, "[packet_mrz16] got %d channels from $LDACK",
                                  static_cast<int>(out->azi.size()));
                            for (size_t i = 0; i < out->azi.size(); ++i)
                            {
                                LOG_F(INFO, "  ch%02d H=%.3f V=%.3f", static_cast<int>(i),
                                      out->azi[i],
                                      out->ele[i]);
                            }
                            return true;
                        }
                    }
                    continue;
                }

                if (steady_now_ms() >= deadline_ms)
                {
                    break;
                }
                std::string rx;
                int len = 0;
                if (serial->SyncRecv(rx, len) != 0)
                {
                    break;  // serial link error
                }
                if (len > 0)
                {
                    ack_buf.insert(ack_buf.end(), rx.begin(), rx.end());
                }
            }
            LOG_F(WARNING, "[packet_mrz16] $LDACK attempt %d/%d timed out",
                  attempt, kRetries);
        }

        if (err)
        {
            *err = "$LDCMD/$LDACK channel angle exchange failed";
        }
        return false;
    }

} // end namespace zvision
