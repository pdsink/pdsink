#pragma once

#include <etl/atomic.h>
#include <etl/delegate.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "data_objects.h"
#include "fusb302_regs.h"
#include "idriver.h"
#include "utils/atomic_enum_bits.h"
#include "utils/leapsync.h"
#include "utils/spsc_overwrite_queue.h"

namespace pd {

class Port;

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
    virtual void setup() = 0;
    virtual void set_event_handler(const hal_event_handler_t& handler) = 0;
    virtual ITimer::TimeFunc get_time_func() const = 0;

    virtual bool read_reg(uint8_t reg, uint8_t& data) = 0;
    virtual bool write_reg(uint8_t reg, uint8_t data) = 0;
    virtual bool read_block(uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool write_block(uint8_t reg, const uint8_t *data, uint32_t size) = 0;
    virtual bool is_interrupt_active() = 0;
};

enum class DRV_FLAG {
    FUSB_SETUP_DONE,
    FUSB_SETUP_FAILED,
    TIMER_EVENT,
    _Count
};

// This class implements generic FUSB302B logic, and relies on FreeRTOS
// to make i2c calls sync.
class Fusb302Rtos : public IDriver {
public:
    Fusb302Rtos(Port& port, IFusb302RtosHal& hal) : port{port}, hal{hal} {
        get_timestamp = hal.get_time_func();
    };

    // Prohibit copy/move because class manages FreeRTOS tasks,
    // hardware resources, and contains callback references.
    Fusb302Rtos(const Fusb302Rtos&) = delete;
    Fusb302Rtos& operator=(const Fusb302Rtos&) = delete;
    Fusb302Rtos(Fusb302Rtos&&) = delete;
    Fusb302Rtos& operator=(Fusb302Rtos&&) = delete;

    void setup() override;


    //
    // TCPC
    //
    void req_scan_cc() override {
        sync_scan_cc.enquire();
        kick_task();
    };
    bool try_scan_cc_result(TCPC_CC_LEVEL::Type& cc1, TCPC_CC_LEVEL::Type& cc2) override;

    void req_active_cc() override {
        sync_active_cc.enquire();
        kick_task();
    };
    bool try_active_cc_result(TCPC_CC_LEVEL::Type& cc) override;

    bool is_vbus_ok() override;

    void req_set_polarity(TCPC_POLARITY active_cc) override {
        sync_set_polarity.enquire(active_cc);
        kick_task();
    };
    bool is_set_polarity_done() override { return sync_set_polarity.is_idle(); };

    void req_rx_enable(bool enable) override {
        sync_rx_enable.enquire(enable);
        kick_task();
    };
    bool is_rx_enable_done() override { return sync_rx_enable.is_idle(); };

    bool fetch_rx_data() override;

    void req_transmit() override;

    void req_set_bist(TCPC_BIST_MODE mode) override {
        sync_set_bist.enquire(mode);
        kick_task();
    };
    bool is_set_bist_done() override { return sync_set_bist.is_idle(); };

    void req_hr_send() override {
        sync_hr_send.enquire();
        kick_task();
    };
    bool is_hr_send_done() override { return sync_hr_send.is_idle(); };

    auto get_hw_features() -> TCPC_HW_FEATURES override { return tcpc_hw_features; };

    //
    // Timer
    //
    ITimer::TimeFunc get_time_func() const override { return hal.get_time_func(); };
    void rearm(uint32_t interval) override {};
    bool is_rearm_supported() override { return false; };

    AtomicEnumBits<DRV_FLAG> flags{};

protected:
    void task();
    bool handle_interrupt();
    bool handle_timer();
    bool handle_tcpc_calls();
    bool handle_meter();
    bool meter_tick(bool &retry);

    void on_hal_event(HAL_EVENT_TYPE event, bool from_isr);
    void kick_task(bool from_isr = false);

    bool fusb_setup();
    bool fusb_set_rxtx_interrupts(bool enable);
    bool fusb_set_auto_goodcrc(bool enable);
    bool fusb_set_tx_auto_retries(uint8_t count);
    bool fusb_flush_rx_fifo();
    bool fusb_flush_tx_fifo();
    bool fusb_pd_reset();
    bool fusb_set_polarity(TCPC_POLARITY polarity);
    bool fusb_set_rx_enable(bool enable);
    bool fusb_tx_pkt_begin(PD_CHUNK& chunk);
    void fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS status);
    bool fusb_rx_pkt();
    bool fusb_hr_send();
    bool fusb_set_bist(TCPC_BIST_MODE mode);
    // Clear insternal states after hard reset received or been sent.
    bool hr_cleanup();

    Port& port;
    IFusb302RtosHal& hal;
    ITimer::TimeFunc get_timestamp;
    bool started{false};
    TaskHandle_t xWaitingTaskHandle{nullptr};

    spsc_overwrite_queue<PD_CHUNK, 4> rx_queue{};
    etl::atomic<TCPC_CC_LEVEL::Type> cc1_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_CC_LEVEL::Type> cc2_cache{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_POLARITY> polarity{TCPC_POLARITY::NONE};
    etl::atomic<bool> vbus_ok{false};
    bool rx_enabled{false};
    bool has_deferred_wakeup{false};


    static constexpr TCPC_HW_FEATURES tcpc_hw_features{
        .rx_goodcrc_send = true,
        .tx_goodcrc_receive = true,
        .tx_retransmit = false,
        .unchunked_ext_msg = false
    };

    // Call sync  + param store primitives
    LeapSync<> sync_scan_cc;
    LeapSync<> sync_active_cc;
    LeapSync<TCPC_POLARITY> sync_set_polarity;
    LeapSync<bool> sync_rx_enable;
    LeapSync<TCPC_BIST_MODE> sync_set_bist;
    LeapSync<> sync_hr_send;

    PD_CHUNK enquired_tx_chunk{};

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
    uint32_t meter_wait_until_ts{0};
    Switches0 meter_sw0_backup{0};

    // Overwrite in inherited class if needed.
    uint32_t task_stack_size{1024*4}; // 4K
    uint32_t task_priority{10};
};

} // namespace fusb302

} // namespace pd