#pragma once

#include "pd_conf.h"
#include "idriver.h"

namespace pd {

// Interface to abstract hardware use.
class IFusb302RtosHal {
public:
    virtual void set_tick_handler(etl::delegate<void()> handler) = 0;
    virtual uint64_t get_timestamp() = 0;
    virtual bool read(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool write(uint8_t reg, uint8_t *data, uint32_t size) = 0;
};

class Sink;

// This class implements generic FUSB302B logic, and relies on FreeRTOS
// to make i2c calls sync.
class Fusb302Rtos : public IDriver {
public:
    Fusb302Rtos(Sink& sink, IFusb302RtosHal& hal);
    void start() override;

    //
    // TCPC
    //
    auto get_state() -> TCPC_STATE& override { return state; };

    void req_cc_both() override;
    void req_cc_active() override;
    auto get_cc(TCPC_CC::Type cc) -> TCPC_CC_LEVEL::Type override;
    void set_polarity(TCPC_CC::Type active_cc) override;
    void set_rx_enable(bool enable) override;
    bool has_rx_data() override;
    void fetch_rx_data(PKT_INFO& data) override;
    void transmit(const PKT_INFO& tx_info) override;
    void bist_carrier_enable(bool enable) override;
    void hard_reset() override;
    void set_tcpc_event_handler(etl::imessage_router& handler) override;

    auto get_hw_features() -> TCPC_HW_FEATURES override { return tcpc_hw_features; };

    //
    // Timer
    //
    void set_tick_handler(etl::delegate<void()> handler) override;
    uint64_t get_timestamp() override { return hal.get_timestamp(); };
    void rearm(uint32_t interval) override {};
    bool is_rearm_supported() override { return false; };

private:
    Sink& sink;
    IFusb302RtosHal& hal;
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