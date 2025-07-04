#pragma once

#include "pd_conf.h"
#include "idriver.h"

namespace pd {

class Sink;

namespace fusb302 {

// Hal messages to TCPC
class MsgHal_Timer : public etl::message<0> {};
class MsgHal_Interrupt : public etl::message<1> {};

// Interface to abstract hardware use.
class IFusb302RtosHal {
public:
    virtual void start() = 0;
    virtual void set_msg_router(etl::imessage_router& router) = 0;
    virtual uint64_t get_timestamp() = 0;
    virtual bool read(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool write(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool is_interrupt_active() = 0;
};

class Fusb302Rtos;

class HalMsgHandler : public etl::message_router<HalMsgHandler, MsgHal_Timer, MsgHal_Interrupt> {
public:
    HalMsgHandler(Fusb302Rtos& drv);
    void on_receive(const MsgHal_Timer& msg);
    void on_receive(const MsgHal_Interrupt& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    Fusb302Rtos& drv;
};

// This class implements generic FUSB302B logic, and relies on FreeRTOS
// to make i2c calls sync.
class Fusb302Rtos : public IDriver {
public:
    Fusb302Rtos(Sink& sink, IFusb302RtosHal& hal);
    void start() override;
    void set_msg_router(etl::imessage_router& router) override {
        msg_router = &router;
    };
    void notify_task();

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

    auto get_hw_features() -> TCPC_HW_FEATURES override { return tcpc_hw_features; };

    //
    // Timer
    //
    uint64_t get_timestamp() override { return hal.get_timestamp(); };
    void rearm(uint32_t interval) override {};
    bool is_rearm_supported() override { return false; };

private:
    Sink& sink;
    IFusb302RtosHal& hal;
    PKT_INFO rx_info{};
    TCPC_STATE state{};
    etl::imessage_router* msg_router{nullptr};
    HalMsgHandler hal_msg_handler{*this};

    static constexpr TCPC_HW_FEATURES tcpc_hw_features{
        .rx_goodcrc_send = true,
        .tx_goodcrc_receive = true,
        .tx_retransmit = false,
        .cc_update_event = true,
        .unchunked_ext_msg = false
    };
};

} // namespace fusb302

} // namespace pd