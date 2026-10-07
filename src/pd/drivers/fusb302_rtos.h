#pragma once

#include "../pd_conf.h"

#include <etl/atomic.h>
#include <etl/delegate.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../data_objects.h"
#include "fusb302_regs.h"
#include "../idriver.h"
#include "../utils/atomic_enum_bits.h"
#include "../utils/leapsync.h"
#include "../utils/spsc_overwrite_queue.h"
#include "../timers.h"

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
    virtual ITimer::TickSource get_tick_source() const = 0;
    virtual void rearm(uint32_t deadline_ticks) = 0;

    virtual bool read_reg(uint8_t i2c_addr, uint8_t reg, uint8_t& data) = 0;
    virtual bool write_reg(uint8_t i2c_addr, uint8_t reg, uint8_t data) = 0;
    virtual bool read_block(uint8_t i2c_addr, uint8_t reg, uint8_t *data, uint32_t size) = 0;
    virtual bool write_block(uint8_t i2c_addr, uint8_t reg, const uint8_t *data, uint32_t size) = 0;
    virtual bool is_interrupt_active() = 0;
};

enum class DRV_FLAG {
    FUSB_SETUP_DONE,
    FUSB_SETUP_FAILED,
    _Count
};

// This class implements generic FUSB302B logic and relies on FreeRTOS
// to make I2C calls synchronous.
class Fusb302Rtos : public IDriver {
    static constexpr uint32_t MSK_PD_INTERRUPT = (1u << 0);
    static constexpr uint32_t MSK_TIMER = (1u << 1);
    static constexpr uint32_t MSK_API_CALL = (1u << 2);
    static constexpr uint32_t MSK_WAKEUP = (1u << 3);

public:
    Fusb302Rtos(Port& port, IFusb302RtosHal& hal)
        : port{port}, hal{hal}, tick_source{hal.get_tick_source()} {
        timers.set_tick_source(tick_source);
    };

    // Prohibit copy/move because class manages FreeRTOS tasks,
    // hardware resources, and contains callback references.
    Fusb302Rtos(const Fusb302Rtos&) = delete;
    Fusb302Rtos& operator=(const Fusb302Rtos&) = delete;
    Fusb302Rtos(Fusb302Rtos&&) = delete;
    Fusb302Rtos& operator=(Fusb302Rtos&&) = delete;

    void setup() override;
    void wakeup() override { kick_task(MSK_WAKEUP); };


    //
    // TCPC
    //
    void req_fetch_cc(TCPC_CC_REQ selector) override;
    bool is_fetch_cc_done() const override { return sync_fetch_cc.is_idle(); };
    auto get_cc(TCPC_CC_GET selector) const -> TCPC_CC_LEVEL::Type override;

    bool check_vbus(TCPC_VBUS_LEVEL level) override;

    void req_set_polarity(TCPC_POLARITY active_cc) override {
        sync_set_polarity.enqueue(active_cc);
        kick_task(MSK_API_CALL);
    };
    bool is_set_polarity_done() override { return sync_set_polarity.is_idle(); };
    auto get_polarity() const -> TCPC_POLARITY override { return polarity.load(); };

    void req_rx_enable(bool enable) override {
        sync_rx_enable.enqueue(enable);
        kick_task(MSK_API_CALL);
    };
    bool is_rx_enable_done() override { return sync_rx_enable.is_idle(); };

    bool fetch_rx_data() override;

    void req_transmit() override;

    void req_set_bist(TCPC_BIST_MODE mode) override {
        sync_set_bist.enqueue(mode);
        kick_task(MSK_API_CALL);
    };
    bool is_set_bist_done() override { return sync_set_bist.is_idle(); };
    auto get_bist_mode() const -> TCPC_BIST_MODE override { return bist_mode.load(); };

    void req_hr_send() override {
        sync_hr_send.enqueue();
        kick_task(MSK_API_CALL);
    };
    bool is_hr_send_done() override { return sync_hr_send.is_idle(); };

    auto get_features() -> TCPC_FEATURES override { return tcpc_features; };

    //
    // Timer
    //
    TickSource get_tick_source() const override { return tick_source; };
    void rearm(uint32_t deadline_ticks) override;

    AtomicEnumBits<DRV_FLAG> flags{};

protected:
    void task();
    void handle_interrupt();
    void handle_tcpc_calls();
    void handle_meter();
    bool meter_tick(bool &retry);

    void on_hal_event(HAL_EVENT_TYPE event, bool from_isr);
    void kick_task(uint32_t event_mask, bool from_isr = false);

    bool fusb_setup();
    bool fusb_set_rxtx_interrupts(bool enable);
    bool fusb_set_auto_goodcrc(bool enable);
    bool fusb_set_tx_auto_retries(uint8_t count);
    bool fusb_flush_rx_fifo();
    bool fusb_flush_tx_fifo();
    bool fusb_pd_reset();
    bool fusb_set_polarity(TCPC_POLARITY polarity);
    bool fusb_start_toggling();
    bool fusb_handle_togdone();
    bool fusb_set_rx_enable(bool enable);
    bool fusb_tx_pkt_begin(PD_CHUNK& chunk);
    void fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS status);
    bool fusb_rx_pkt();
    bool fusb_hr_send();
    bool fusb_set_bist(TCPC_BIST_MODE mode);
    // Clear internal states after a hard reset is received or sent.
    bool hr_cleanup();

    uint8_t i2c_addr{ChipAddress::FUSB302B};
    Port& port;
    IFusb302RtosHal& hal;
    TickSource tick_source;
    bool started{false};
    TaskHandle_t xWaitingTaskHandle{nullptr};

    spsc_overwrite_queue<PD_CHUNK, 4> rx_queue{};
    etl::atomic<TCPC_CC_LEVEL::Type> cc1_value{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_CC_LEVEL::Type> cc2_value{TCPC_CC_LEVEL::NONE};
    etl::atomic<TCPC_POLARITY> polarity{TCPC_POLARITY::NONE};
    etl::atomic<TCPC_BIST_MODE> bist_mode{TCPC_BIST_MODE::Off};
    etl::atomic<bool> vbus_ok{false};
    bool rx_enabled{false};
    bool has_deferred_wakeup{false};
    bool has_deferred_timer{false};


    static constexpr TCPC_FEATURES tcpc_features{
        .cc_toggling = true,
        .rx_goodcrc_send = true,
        .tx_goodcrc_wait = true,
        // Software retries are for testing only. USB PD requires the retry
        // preamble to start within 195 us after CRCReceiveTimer expires.
        // Software retries over I2C cannot reliably meet this deadline.
#if defined(FUSB302_NO_HW_RETRIES)
        .tx_retries = false
#else
        .tx_retries = true
#endif
    };

    // Call sync + param store primitives
    LeapSync<TCPC_CC_REQ> sync_fetch_cc;
    LeapSync<TCPC_POLARITY> sync_set_polarity;
    LeapSync<bool> sync_rx_enable;
    LeapSync<TCPC_BIST_MODE> sync_set_bist;
    LeapSync<> sync_hr_send;

    PD_CHUNK enqueued_tx_chunk{};

    enum class MeterState {
        IDLE,
        CC_ACTIVE_MEASURE_WAIT,
    };
    MeterState meter_state{MeterState::IDLE};

    enum DriverTimer {
        PD_CORE_TIMERS,
        CC,
        COUNT,
    };
#if defined(USE_FUSB302_COARSE_TIMER)
    static constexpr PD_TIMEOUT::Type CC_SETTLE{DriverTimer::CC, 2000};
#else
    static constexpr PD_TIMEOUT::Type CC_SETTLE{DriverTimer::CC, 250};
#endif
    Timers<DriverTimer::COUNT> timers;

    // Override in an inherited class if needed.
    uint32_t task_stack_size_bytes{1024*4}; // 4K
    uint32_t task_priority{10};
};

} // namespace fusb302

} // namespace pd
