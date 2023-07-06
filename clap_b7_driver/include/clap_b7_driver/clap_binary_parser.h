//
// Created by elaydin on 06.07.2023.
//

#ifndef CLAP_BINARY_PARSER_H
#define CLAP_BINARY_PARSER_H
#include <cstdint>
#include "clap_b7_driver/clap_structs.h"
#include <vector>
#include <experimental/optional>

namespace clap_b7 {
    // Callback function type
    using receiveCallback = void (*)(const uint8_t* buffer, uint16_t msg_id);
    class  BinaryParser {
    private:
        enum class ParseStatus
        {
            kSynch1Control = 0,
            kSynch2Control,
            kSynch3Control,
            kHeaderLength,
            kHeaderAdd,
            kDataAdd,
        };
        ParseStatus status_{BinaryParser::ParseStatus::kSynch1Control};
        std::vector<uint8_t> data_buffer_{};
        clap_b7::Header header_{};
        bool header_detected_{false};
        void clap_parser();
        receiveCallback callback_{nullptr};
    public:
        BinaryParser();
        int64_t gnss_unixtime_ns_{0};
        InsPvax ins_pvax_{};
        RawImu raw_imu_{};
        BestGnssPos best_gnss_pos_{};
        BestGnssVel best_gnss_vel_{};
        UniHeading heading_{};
        bool ins_active_{false};
        void received_new_data(const uint8_t* buffer, uint16_t size);
        void set_receive_callback(receiveCallback callback) {
            callback_ = callback;
        }

    };





}//clap namespace
#endif CLAP_BINARY_PARSER_H
