//
// Created by elaydin on 06.07.2023.
//

#ifndef CLAP_BINARY_PARSER_H
#define CLAP_BINARY_PARSER_H
#include <cstdint>
#include "clap_b7_driver/clap_structs.h"
#include <utility>
#include <vector>
#include <functional>

namespace clap_b7 {
    class  BinaryParser {
    private:
        std::vector<uint8_t> data_buffer_{};
        clap_b7::Header header_{};
        bool header_detected_{false};
        void clap_parser();
        std::function<void(const uint8_t *, uint16_t)> callback_{nullptr};
    public:
        BinaryParser() = default;
        enum class MessageId : std::uint16_t{
            kRAWIMU = 268,
            kHeading = 971,
            kBestGnssPos = 1429,
            kBestGnssVel = 1430,
            kINSPVAX = 1465,
            kECEF = 241,
        };
        int64_t gnss_unix_time_ns_{0};
        void received_new_data(const uint8_t* buffer, uint16_t size);
        void set_receive_callback(std::function<void(const uint8_t* buffer, uint16_t msg_id)> callback) {
            callback_ = std::move(callback);
        }
    };





} // namespace clap_b7
#endif //CLAP_BINARY_PARSER_H
