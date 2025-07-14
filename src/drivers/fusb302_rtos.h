#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <etl/atomic.h>
#include <etl/cyclic_value.h>

#include "pd_conf.h"
#include "idriver.h"
#include "utils/spsc_overwrite_queue.h"
#include "fusb302_regs.h"

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

    virtual bool read_reg(uint8_t reg, uint8_t& data) = 0;
    virtual bool write_reg(uint8_t reg, uint8_t data) = 0;
    virtual bool read_block(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool write_block(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool set_bits(uint8_t reg, uint8_t mask) = 0;
    virtual bool clear_bits(uint8_t reg, uint8_t mask) = 0;

    virtual bool is_interrupt_active() = 0;
};

namespace DRV_FLAG {
    enum Type {
        FUSB_SETUP_DONE,
        FUSB_SETUP_FAILED,
        VBUS_OK,
        TIMER_EVENT,

        API_CALL,
        ENQUIRED_REQ_CC_ACTIVE,
        ENQUIRED_TRANSMIT,
        ENQUIRED_HR_SEND,

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
    Fusb302Rtos(Sink& sink, IFusb302RtosHal& hal, bool emulate_vbus_ok = false);
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
    bool is_vbus_ok() override;
    void set_polarity(TCPC_POLARITY::Type active_cc) override;
    void set_rx_enable(bool enable) override;
    bool has_rx_data() override;
    bool fetch_rx_data(PD_CHUNK& data) override;
    void transmit(const PD_CHUNK& tx_info) override;
    void bist_carrier_enable(bool enable) override;
    void hr_send() override;

    auto get_hw_features() -> TCPC_HW_FEATURES override { return tcpc_hw_features; };

    //
    // Timer
    //
    uint64_t get_timestamp() override { return hal.get_timestamp(); };
    void rearm(uint32_t interval) override {};
    bool is_rearm_supported() override { return false; };

    AtomicBits<DRV_FLAG::FLAGS_COUNT> flags{};
private:
    void task();
    bool handle_interrupt();
    bool handle_timer();
    bool handle_tcpc_calls();
    bool handle_get_active_cc();

    bool fusb_setup();
    bool fusb_set_polarity(TCPC_POLARITY::Type polarity);
    bool fusb_set_rx_enable(bool enable);
    bool fusb_flush_rx_fifo();
    bool fusb_flush_tx_fifo();
    bool fusb_scan_cc();
    bool fusb_tx_pkt();
    bool fusb_rx_pkt();
    void fusb_complete_tx(TCPC_TRANSMIT_STATUS::Type status);
    bool fusb_hr_send_begin();
    bool fusb_hr_send_end();

    Sink& sink;
    IFusb302RtosHal& hal;
    etl::imessage_router* msg_router{nullptr};
    HalMsgHandler hal_msg_handler{*this};
    bool started{false};
    TaskHandle_t xWaitingTaskHandle{nullptr};

    bool emulate_vbus_ok;
    etl::cyclic_value<uint32_t, 0, 49> emulate_vbus_counter{0};

    TCPC_STATE state{};
    spsc_overwrite_queue<PD_CHUNK, 4> rx_queue{};
    etl::atomic<TCPC_CC_LEVEL::Type> cc1_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_CC_LEVEL::Type> cc2_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_POLARITY::Type> polarity{TCPC_POLARITY::NONE};
    // Marks of signal activity, to measure CC at safe periods
    bool activity_on{false};
    uint64_t last_activity_ts{0};

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
    PD_CHUNK call_arg_transmit{};
    bool call_arg_bist_carrier_enable{};
};

} // namespace fusb302

} // namespace pd