#include "driver_mock.h"

#include "pd/pd_log.h"
#include "pd/port.h"

namespace cts {

using namespace pd;

//
// Driver mock
//

uint32_t DriverMock::now_us{0};

DriverMock::DriverMock(Port& port, const TCPC_FEATURES& features)
    : port{port}, features{features} {}

bool DriverMock::start() { return true; }

void DriverMock::stop() {
    cc_pending.reset();
    polarity_pending.reset();
    rx_enabled = false;
    rx_buffer_has_pkt = false;
    tx_pending = false;
    hr_pending = false;
    bist_mode = TCPC_BIST_MODE::Off;
    retry_at_us.reset();
    wakeup_pending = false;
    rearm_at_us.reset();
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
}

void DriverMock::wakeup() { wakeup_pending = true; }

auto DriverMock::get_tick_source() const -> TickSource {
    return TickSource{+[]() -> uint32_t {
        return static_cast<uint32_t>(
            static_cast<uint64_t>(now_us) / PD_TIMER_TICK_US);
    }};
}

void DriverMock::rearm(uint32_t deadline_ticks) {
    rearm_at_us = static_cast<uint32_t>(
        static_cast<uint64_t>(deadline_ticks) * PD_TIMER_TICK_US);
}

void DriverMock::req_fetch_cc(TCPC_CC_REQ selector) {
    switch (selector) {
        case TCPC_CC_REQ::CC1:
        case TCPC_CC_REQ::CC2:
        case TCPC_CC_REQ::CC1CC2:
        case TCPC_CC_REQ::ACTIVE_CC:
            cc_pending = selector;
            wakeup_pending = true;
            return;
    }

    DRV_LOGE("DriverMock: unsupported CC selector: {}", static_cast<int>(selector));
    cc_pending.reset();
    wakeup_pending = true;
}

bool DriverMock::is_fetch_cc_done() const { return !cc_pending.has_value(); }

auto DriverMock::get_cc(TCPC_CC_GET selector) const -> TCPC_CC_LEVEL::Type {
    switch (selector) {
        case TCPC_CC_GET::CC1:
            return cc1_cache;
        case TCPC_CC_GET::CC2:
            return cc2_cache;
        case TCPC_CC_GET::ACTIVE_CC:
            if (polarity == TCPC_POLARITY::CC1) {
                return cc1_cache;
            } else if (polarity == TCPC_POLARITY::CC2) {
                return cc2_cache;
            } else {
                DRV_LOGE("DriverMock: ACTIVE_CC without selected polarity");
            }
            break;
        default:
            DRV_LOGE("DriverMock: unsupported CC cache selector: {}",
                static_cast<int>(selector));
            break;
    }

    return TCPC_CC_LEVEL::NONE;
}

bool DriverMock::check_vbus(TCPC_VBUS_LEVEL level) {
    switch (level) {
        case TCPC_VBUS_LEVEL::PRESENT:
            return vbus;
        case TCPC_VBUS_LEVEL::SINK_DISCONNECTED:
        case TCPC_VBUS_LEVEL::SAFE0V:
            return !vbus;
    }

    DRV_LOGE("DriverMock: unsupported VBUS level: {}", static_cast<int>(level));
    return false;
}

void DriverMock::req_set_polarity(TCPC_POLARITY active_cc) {
    polarity_pending = active_cc;
    wakeup_pending = true;
}
bool DriverMock::is_set_polarity_done() { return !polarity_pending.has_value(); }
auto DriverMock::get_polarity() const -> TCPC_POLARITY { return polarity; }

void DriverMock::req_rx_enable(bool enable) {
    if (!enable || !rx_enabled) { rx_buffer_has_pkt = false; }
    tx_pending = false;
    retry_at_us.reset();
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
    rx_enabled = enable;
    wakeup_pending = true;
}

bool DriverMock::is_rx_enable_done() { return true; }

auto DriverMock::fetch_rx() -> FETCH_RX_STATUS {
    return rx_buffer_has_pkt ? FETCH_RX_STATUS::READY : FETCH_RX_STATUS::EMPTY;
}

void DriverMock::reset_rx_id_tracker() { rx_msg_id_stored = -1; }

void DriverMock::receive_chunk(const PD_CHUNK& chunk) {
    if (chunk.is_ctrl_msg(PD_CTRL_MSGT::GoodCRC)) { return; }

    if (features.tx_retries) {
        if (!chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset) &&
            rx_msg_id_stored == chunk.header.message_id)
        {
            return;
        }
        rx_msg_id_stored = chunk.header.message_id;
        if (chunk.is_ctrl_msg(PD_CTRL_MSGT::Ping)) { return; }

        if (is_tcpc_transmit_in_progress(port.tcpc_tx_status.load())) {
            tx_pending = false;
            retry_at_us.reset();
            port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::FAILED);
        }
    }

    rx_buffer = chunk;
    rx_buffer_has_pkt = true;
    wakeup_pending = true;
}

bool DriverMock::get_rx() {
    if (!rx_buffer_has_pkt) { return false; }

    port.rx_chunk = rx_buffer;
    rx_buffer_has_pkt = false;
    return true;
}

void DriverMock::req_transmit() {
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
    tx_chunk = port.tx_chunk;
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::ENQUEUED);
    tx_pending = true;
    retry_at_us.reset();
    if (features.tx_retries && rx_buffer_has_pkt) {
        tx_pending = false;
        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::FAILED);
        wakeup_pending = true;
    }
}

void DriverMock::req_set_bist(TCPC_BIST_MODE mode) {
    bist_mode = mode;
    bist_log.push_back({now_us, mode});
    wakeup_pending = true;
}
bool DriverMock::is_set_bist_done() { return true; }
auto DriverMock::get_bist_mode() const -> TCPC_BIST_MODE { return bist_mode; }

void DriverMock::req_hr_send() {
    rx_buffer_has_pkt = false;
    tx_pending = false;
    retry_at_us.reset();
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
    hr_pending = true;
}
bool DriverMock::is_hr_send_done() const { return !hr_pending; }

auto DriverMock::get_features() -> TCPC_FEATURES {
    return features;
}

} // namespace cts
