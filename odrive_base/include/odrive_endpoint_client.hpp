#pragma once

#include "socket_can.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

class ODriveEndpointClient {
public:
    static constexpr uint8_t kCmdIdRxSdo = 0x04;
    static constexpr uint8_t kCmdIdAddress = 0x06;
    static constexpr uint8_t kOpcodeRead = 0x00;
    static constexpr uint8_t kOpcodeWrite = 0x01;
    static constexpr uint8_t kOpcodeContinue = 0x03;

    ODriveEndpointClient(SocketCanIntf* can_intf, uint32_t node_id, uint8_t connection_id = 10)
        : can_intf_(can_intf), node_id_(node_id), connection_id_(connection_id) {}

    bool ensure_address_claimed() {
        if (address_claimed_) {
            return true;
        }

        can_frame frame = {};
        frame.can_id = (node_id_ << 5) | kCmdIdAddress;
        frame.can_dlc = 8;
        frame.data[0] = static_cast<uint8_t>(node_id_);
        frame.data[7] = static_cast<uint8_t>(connection_id_ << 2);

        if (!can_intf_ || !can_intf_->send_can_frame(frame)) {
            return false;
        }

        address_claimed_ = true;
        return true;
    }

    void invalidate_address() {
        address_claimed_ = false;
    }

    bool call_function(uint16_t endpoint_id, std::span<const uint8_t> payload) {
        return write_endpoint(endpoint_id, payload);
    }

    bool write_endpoint(uint16_t endpoint_id, std::span<const uint8_t> payload) {
        if (!ensure_address_claimed()) {
            return false;
        }

        const size_t continuation_payload_size = payload.size() > 4 ? payload.size() - 4 : 0;
        const size_t continuation_frames = (continuation_payload_size + 5) / 6;
        const size_t total_segments = 1 + continuation_frames;
        if (total_segments > 16) {
            return false;
        }

        const uint8_t sequence_number = next_sequence_number();
        can_frame frame = {};
        frame.can_id = (node_id_ << 5) | kCmdIdRxSdo;
        frame.can_dlc = 8;
        frame.data[0] = static_cast<uint8_t>((connection_id_ << 2) | kOpcodeWrite);
        frame.data[1] = static_cast<uint8_t>(endpoint_id & 0xff);
        frame.data[2] = static_cast<uint8_t>((endpoint_id >> 8) & 0xff);
        frame.data[3] = static_cast<uint8_t>((sequence_number << 4) | (total_segments - 1));

        const size_t first_payload_size = std::min<size_t>(payload.size(), 4);
        if (first_payload_size > 0) {
            std::memcpy(frame.data + 4, payload.data(), first_payload_size);
        }

        if (!can_intf_->send_can_frame(frame)) {
            return false;
        }

        for (size_t segment = 1; segment < total_segments; ++segment) {
            can_frame continuation = {};
            continuation.can_id = frame.can_id;
            continuation.can_dlc = 8;
            continuation.data[0] = static_cast<uint8_t>((connection_id_ << 2) | kOpcodeContinue);
            continuation.data[1] = static_cast<uint8_t>((sequence_number << 4) | segment);

            const size_t offset = 4 + ((segment - 1) * 6);
            const size_t chunk_size = std::min<size_t>(payload.size() - offset, 6);
            std::memcpy(continuation.data + 2, payload.data() + offset, chunk_size);

            if (!can_intf_->send_can_frame(continuation)) {
                return false;
            }
        }

        return true;
    }

private:
    uint8_t next_sequence_number() {
        sequence_number_ = static_cast<uint8_t>((sequence_number_ + 1) & 0x0f);
        return sequence_number_;
    }

    SocketCanIntf* can_intf_;
    uint32_t node_id_;
    uint8_t connection_id_;
    uint8_t sequence_number_ = 0;
    bool address_claimed_ = false;
};