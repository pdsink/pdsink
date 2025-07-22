#pragma once

#include <etl/atomic.h>
#include <etl/cyclic_value.h>
#include <etl/delegate.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "fusb302_regs.h"
#include "idriver.h"
#include "utils/spsc_overwrite_queue.h"

namespace pd {

class Sink;

namespace fusb302 {

// Hal messages to TCPC

enum class HAL_EVENT_TYPE {
    Timer,
    FUSB302_Interrupt
};
using hal_event_handler_t = etl::delegate<void(HAL_EVENT_TYPE, bool)>;

// Interface to abstract hardware use.
class IFusb302RtosHal {
public:
    virtual void start() = 0;
    virtual void set_event_handler(const hal_event_handler_t& handler) = 0;
    virtual uint64_t get_timestamp() = 0;

    virtual bool read_reg(uint8_t reg, uint8_t& data) = 0;
    virtual bool write_reg(uint8_t reg, uint8_t data) = 0;
    virtual bool read_block(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool write_block(uint8_t reg, const uint8_t *data, uint32_t size) = 0;
    virtual bool is_interrupt_active() = 0;
};

namespace DRV_FLAG {
    enum Type {
        FUSB_SETUP_DONE,
        FUSB_SETUP_FAILED,
        TIMER_EVENT,

        API_CALL,
        ENQUIRED_HR_SEND,
        ENQUIRED_TRANSMIT,

        FLAGS_COUNT
    };
};


class Fusb302Rtos;

// This class implements generic FUSB302B logic, and relies on FreeRTOS
// to make i2c calls sync.
class Fusb302Rtos : public IDriver {
public:
    Fusb302Rtos(Sink& sink, IFusb302RtosHal& hal, bool emulate_vbus_ok = false);

    // Prohibit copy/move because class manages FreeRTOS tasks,
    // hardware resources, and contains callback references.
    Fusb302Rtos(const Fusb302Rtos&) = delete;
    Fusb302Rtos& operator=(const Fusb302Rtos&) = delete;
    Fusb302Rtos(Fusb302Rtos&&) = delete;
    Fusb302Rtos& operator=(Fusb302Rtos&&) = delete;

    void start() override;
    void set_msg_router(etl::imessage_router& router) override {
        msg_router = &router;
    };

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

    void on_hal_event(HAL_EVENT_TYPE event, bool from_isr);
    void pass_up(const etl::imessage& msg) {
        if (msg_router) { msg_router->receive(msg); }
    }
    void kick_task(bool from_isr = false);

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
    bool started{false};
    TaskHandle_t xWaitingTaskHandle{nullptr};

    bool emulate_vbus_ok;
    uint64_t vbus_ok_emulator_last_measure_ts{0};

    TCPC_STATE state{};
    spsc_overwrite_queue<PD_CHUNK, 4> rx_queue{};
    etl::atomic<TCPC_CC_LEVEL::Type> cc1_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_CC_LEVEL::Type> cc2_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_POLARITY::Type> polarity{TCPC_POLARITY::NONE};
    etl::atomic<bool> vbus_ok{false};
    etl::atomic<uint32_t> sem_req_cc_active{0};
    etl::atomic<uint32_t> sem_req_cc_scan{0};
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