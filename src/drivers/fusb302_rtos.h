#pragma once

#include <etl/atomic.h>
#include <etl/cyclic_value.h>
#include <etl/delegate.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "fusb302_regs.h"
#include "idriver.h"
#include "utils/leapsync.h"
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
    void req_scan_cc() override {
        sync_scan_cc.enquire();
        kick_task();
    };
    bool is_scan_cc_done() override { return sync_scan_cc.is_ready(); };
    void req_active_cc() override {
        sync_active_cc.enquire();
        kick_task();
    };
    bool is_active_cc_done() override { return sync_active_cc.is_ready(); };
    auto get_cc(TCPC_CC::Type cc) -> TCPC_CC_LEVEL::Type override;

    bool is_vbus_ok() override;

    void req_set_polarity(TCPC_POLARITY::Type active_cc) override {
        sync_set_polarity.enquire(active_cc);
        kick_task();
    };
    bool is_set_polarity_done() override { return sync_set_polarity.is_ready(); };

    void req_rx_enable(bool enable) override {
        sync_rx_enable.enquire(enable);
        kick_task();
    };
    bool is_rx_enable_done() override { return sync_rx_enable.is_ready(); };

    bool fetch_rx_data(PD_CHUNK& data) override;

    void req_transmit(const PD_CHUNK& tx_info) override {
        call_arg_transmit = tx_info;
        sync_transmit.enquire();
        kick_task();
    };
    bool is_transmit_done() override { return sync_transmit.is_ready(); };

    void req_bist_carrier_enable(bool enable) override {
        sync_bist_carrier_enable.enquire(enable);
        kick_task();
    };
    bool is_bist_carrier_enable_done() override { return sync_bist_carrier_enable.is_ready(); };

    void req_hr_send() override {
        sync_hr_send.enquire();
        kick_task();
    };
    bool is_hr_send_done() override { return sync_hr_send.is_ready(); };

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
    bool handle_meter();
    bool meter_tick(bool &retry);

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
    bool fusb_tx_pkt_begin();
    void fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::Type status);
    bool fusb_rx_pkt();
    bool fusb_hr_send_begin();
    bool fusb_hr_send_end();
    bool fusb_bist(bool enable);

    Sink& sink;
    IFusb302RtosHal& hal;
    etl::imessage_router* msg_router{nullptr};
    bool started{false};
    TaskHandle_t xWaitingTaskHandle{nullptr};

    bool emulate_vbus_ok;
    uint64_t vbus_ok_emulator_last_measure_ts{0};

    spsc_overwrite_queue<PD_CHUNK, 4> rx_queue{};
    etl::atomic<TCPC_CC_LEVEL::Type> cc1_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_CC_LEVEL::Type> cc2_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_POLARITY::Type> polarity{TCPC_POLARITY::NONE};
    etl::atomic<bool> vbus_ok{false};

    // Marks of signal activity, to measure CC at safe periods
    // TODO: remove
    bool activity_on{false};
    uint64_t last_activity_ts{0};

    static constexpr TCPC_HW_FEATURES tcpc_hw_features{
        .rx_goodcrc_send = true,
        .tx_goodcrc_receive = true,
        .tx_retransmit = false,
        .cc_update_event = true,
        .unchunked_ext_msg = false
    };

    // Call sync  + param store primitives
    LeapSync<> sync_scan_cc;
    LeapSync<> sync_active_cc;
    LeapSync<TCPC_POLARITY::Type> sync_set_polarity;
    LeapSync<bool> sync_rx_enable;
    LeapSync<> sync_transmit;
    LeapSync<bool> sync_bist_carrier_enable;
    LeapSync<> sync_hr_send;

    // Not atomic, but fixed buffer size - ok for our needs.
    PD_CHUNK call_arg_transmit{};

    enum class MeterState {
        IDLE,
        CC_ACTIVE_BEGIN,
        CC_ACTIVE_MEASURE_WAIT,
        CC_ACTIVE_END,
        SCAN_CC_BEGIN,
        SCAN_CC1_MEASURE_WAIT,
        SCAN_CC2_MEASURE_WAIT,
    };
    MeterState meter_state{MeterState::IDLE};
    uint64_t meter_wait_until_ts{0};
    uint8_t meter_backup_cc1{0};
    uint8_t meter_backup_cc2{0};
};

} // namespace fusb302

} // namespace pd