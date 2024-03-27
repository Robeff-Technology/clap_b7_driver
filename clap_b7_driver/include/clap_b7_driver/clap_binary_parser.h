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
        clap_b7::Header header_{};
        std::function<void(const uint8_t *, uint16_t)> callback_{nullptr};

        enum ClapB7ParseStatus: uint8_t
        {
            SYNCH1_CONTROL,
            SYNCH2_CONTROL,
            SYNCH3_CONTROL,
            CLAP_B7_HEADER_LENGTH,
            CLAP_B7_HEADER_ADD,
            CLAP_B7_HEADER_ADD_AGRIC,
            CLAP_B7_DATA_ADD_AGRIC,
            CLAP_B7_DATA_ADD,
        };
        ClapB7ParseStatus status_{ClapB7ParseStatus::SYNCH1_CONTROL};
        uint16_t data_index_{0};
        uint8_t raw_data_[256];
    public:
        BinaryParser() = default;
        enum class MessageId : std::uint16_t{
            kRAWIMU = 268,
            kHeading = 971,
            kBestGnssPos = 1429,
            kBestGnssVel = 1430,
            kINSPVAX = 1465,
            kECEF = 241,
            kWheelOdom = 622,
        };

        void received_new_data(const uint8_t* buffer, uint16_t size);
        void set_receive_callback(std::function<void(const uint8_t* buffer, uint16_t msg_id)> callback) {
            callback_ = std::move(callback);
        }

        int64_t get_unix_time_ns();
    };





} // namespace clap_b7
#endif //CLAP_BINARY_PARSER_H
