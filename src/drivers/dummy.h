#pragma once

#include "idriver.h"

namespace pd {

class Sink;

// Dummy driver for testing
class DummyDriver : public IDriver {
public:
    DummyDriver(Sink& sink);

    void start() override {};

    //
    // TCPC
    //
    auto get_state() -> TCPC_STATE& override { return state; };

    void req_cc_both() override {};
    void req_cc_active() override {};
    auto get_cc(TCPC_CC::Type cc) -> TCPC_CC_LEVEL::Type override {
        return TCPC_CC_LEVEL::RP_3_0;
    };
    void set_polarity(TCPC_CC::Type active_cc) override {};

    void set_rx_enable(bool enable) override {};

    bool has_rx_data() override { return true; };
    void fetch_rx_data(PKT_INFO& data) override { data = rx_info; };

    void transmit(const PKT_INFO& tx_info) override {};

    void bist_carrier_enable(bool enable) override {};

    void hard_reset() override {};

    void set_tcpc_event_handler(etl::imessage_router& handler) override {};

    auto get_hw_features() -> TCPC_HW_FEATURES override {
        return tcpc_hw_features; };

    //
    // Timer
    //
    void set_tick_handler(etl::delegate<void()> handler) override {};
    uint64_t get_timestamp() override { return 0; };
    void rearm(uint32_t interval) override {};
    bool is_rearm_supported() override { return false; };

private:
    Sink& sink;
    PKT_INFO rx_info{};
    TCPC_STATE state{};

    static constexpr TCPC_HW_FEATURES tcpc_hw_features = {
        .rx_goodcrc_send = true,
        .tx_goodcrc_receive = true,
        .tx_retransmit = false,
        .cc_update_event = true,
        .unchunked_ext_msg = false
    };
};

} // namespace pd