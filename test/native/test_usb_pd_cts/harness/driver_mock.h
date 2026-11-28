#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "pd/data_objects.h"
#include "pd/idriver.h"

namespace pd {
class Port;
}

namespace cts {

//
// Driver mock (TCPC side, used by the PD stack). Emulates the TCPC hardware
// and the wire between UUT and Tester.
//

class Tester;

class DriverMock : public pd::IDriver {
public:
    DriverMock(pd::Port& port, const pd::TCPC_FEATURES& features);

    // IDriver
    bool start() override;
    void stop() override;
    void wakeup() override;

    // ITimer
    TickSource get_tick_source() const override;
    void rearm(uint32_t deadline_ticks) override;

    // ITCPC
    void req_fetch_cc(pd::TCPC_CC_REQ selector) override;
    bool is_fetch_cc_done() const override;
    auto get_cc(pd::TCPC_CC_GET selector) const -> pd::TCPC_CC_LEVEL::Type override;
    bool check_vbus(pd::TCPC_VBUS_LEVEL level) override;
    void req_set_polarity(pd::TCPC_POLARITY active_cc) override;
    bool is_set_polarity_done() override;
    auto get_polarity() const -> pd::TCPC_POLARITY override;
    void req_rx_enable(bool enable) override;
    bool is_rx_enable_done() override;
    auto fetch_rx() -> pd::FETCH_RX_STATUS override;
    bool get_rx() override;
    void reset_rx_id_tracker() override;
    void req_transmit() override;
    void req_set_bist(pd::TCPC_BIST_MODE mode) override;
    bool is_set_bist_done() override;
    auto get_bist_mode() const -> pd::TCPC_BIST_MODE override;
    void req_hr_send() override;
    bool is_hr_send_done() const override;
    auto get_features() -> pd::TCPC_FEATURES override;

    // Virtual time.
    static uint32_t now_us;

private:
    friend class Tester;

    pd::Port& port;
    pd::TCPC_FEATURES features;

    // Line state, controlled by Tester
    pd::TCPC_CC_LEVEL::Type cc1{pd::TCPC_CC_LEVEL::NONE};
    pd::TCPC_CC_LEVEL::Type cc2{pd::TCPC_CC_LEVEL::NONE};
    bool vbus{false};
    pd::TCPC_CC_LEVEL::Type cc1_cache{pd::TCPC_CC_LEVEL::NONE};
    pd::TCPC_CC_LEVEL::Type cc2_cache{pd::TCPC_CC_LEVEL::NONE};
    pd::TCPC_POLARITY polarity{pd::TCPC_POLARITY::NONE};
    std::optional<pd::TCPC_CC_REQ> cc_pending{};
    std::optional<pd::TCPC_POLARITY> polarity_pending{};

    bool rx_enabled{false};
    pd::PD_CHUNK rx_buffer{};
    bool rx_buffer_has_pkt{false};
    int8_t rx_msg_id_stored{-1};
    void receive_chunk(const pd::PD_CHUNK& chunk);

    // Async TCPC operations are completed by Tester::process()
    bool tx_pending{false};
    pd::PD_CHUNK tx_chunk{};
    bool hr_pending{false};
    // TCPC retries of tx_chunk (no GoodCRC from Tester)
    int retries_left{0};
    std::optional<uint32_t> retry_at_us{};
    bool wakeup_pending{false};

    std::optional<uint32_t> rearm_at_us{};

    struct BistEvent {
        uint32_t time;
        pd::TCPC_BIST_MODE mode;
    };
    std::vector<BistEvent> bist_log{};
    pd::TCPC_BIST_MODE bist_mode{pd::TCPC_BIST_MODE::Off};
};

} // namespace cts
