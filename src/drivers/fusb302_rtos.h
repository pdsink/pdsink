#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <etl/atomic.h>
#include <etl/queue_spsc_atomic.h>

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

namespace HAL_EVENT_FLAG {
    enum Type {
        TIMER,
        FUSB302_INTERRUPT,
        FLAGS_COUNT
    };
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
    void set_polarity(TCPC_POLARITY::Type active_cc) override;
    void set_rx_enable(bool enable) override;
    bool has_rx_data() override;
    bool fetch_rx_data(PKT_INFO& data) override;
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

    AtomicBits<HAL_EVENT_FLAG::FLAGS_COUNT> hal_events{};

private:
    void task();

    Sink& sink;
    IFusb302RtosHal& hal;
    etl::imessage_router* msg_router{nullptr};
    HalMsgHandler hal_msg_handler{*this};
    bool started{false};
    TaskHandle_t xWaitingTaskHandle{nullptr};

    TCPC_STATE state{};
    etl::queue_spsc_atomic<PKT_INFO, 4> rx_queue{};
    etl::atomic<TCPC_CC_LEVEL::Type> cc1_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_CC_LEVEL::Type> cc2_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_POLARITY::Type> polarity{TCPC_POLARITY::NONE};

    static constexpr TCPC_HW_FEATURES tcpc_hw_features{
        .rx_goodcrc_send = true,
        .tx_goodcrc_receive = true,
        .tx_retransmit = false,
        .cc_update_event = true,
        .unchunked_ext_msg = false
    };

    // TCPC call arguments, used for state transitions to task().
    TCPC_POLARITY::Type call_arg_set_polarity{};
    bool call_arg_set_rx_enable{};
    PKT_INFO call_arg_transmit{};
    bool call_arg_bist_carrier_enable{};
};

} // namespace fusb302

} // namespace pd