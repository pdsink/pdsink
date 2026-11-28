#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "driver_mock.h"
#include "pd/data_objects.h"
#include "pd/dpm.h"
#include "pd/pe.h"
#include "pd/port.h"
#include "pd/prl.h"
#include "pd/task.h"
#include "pd/tc.h"

namespace cts {

//
// Timings
//

// All values in microseconds
struct Interval {
    uint32_t min;
    uint32_t max;

    // Extend upper bound only: {min, max + other.max}
    constexpr Interval add_max(Interval other) const {
        return {min, max + other.max};
    }
};

constexpr uint32_t us = 1;
constexpr uint32_t ms = 1000 * us;

// [rev3.2 v1.2] Table 7.9 Time Values
constexpr Interval tBISTCarrierMode{0, 300 * ms};
constexpr Interval tBISTContMode{30 * ms, 60 * ms};
constexpr Interval tChunkReceiverRequest{0, 15 * ms};
constexpr Interval tChunkSenderResponse{24 * ms, 30 * ms};
constexpr Interval tChunkingNotSupported{40 * ms, 50 * ms};
constexpr Interval tEnterEPR{450 * ms, 550 * ms};
constexpr Interval tHardReset{0, 5 * ms};
constexpr Interval tProtErrHardReset{0, 15 * ms};
constexpr Interval tProtErrSoftReset{0, 15 * ms};
constexpr Interval tPSHardReset{25 * ms, 35 * ms};
constexpr Interval tPPSRequest{0, 10000 * ms};
constexpr Interval tPPSTimeout{12000 * ms, 15000 * ms};
constexpr Interval tPSTransition{450 * ms, 550 * ms}; // SPR Mode
constexpr Interval tReceive{900 * us, 1100 * us};
constexpr Interval tReceiverResponse{0, 15 * ms};
constexpr Interval tRetry{0, 195 * us};
constexpr Interval tSendHardReset{0, 15 * ms};
constexpr Interval tSinkEPRKeepAlive{250 * ms, 500 * ms};
constexpr Interval tSinkRequest{100 * ms, 100 * ms}; // min only
constexpr Interval tSourceEPRKeepAlive{750 * ms, 1000 * ms};
constexpr Interval tSrcHoldsBus{0, 50 * ms};
constexpr Interval tSoftReset{0, 15 * ms};
constexpr Interval tSrcRecover{660 * ms, 1000 * ms}; // SPR Mode
constexpr Interval tSrcTransReq{0, 325 * ms}; // SPR Mode
constexpr Interval tSrcTransReqEPR{0, 760 * ms};
constexpr Interval tTypeCSendSourceCap{100 * ms, 200 * ms};
constexpr Interval tTypeCSinkWaitCap{310 * ms, 620 * ms};

// [rev3.2 v1.2] Table 7.11 Counter parameters
constexpr int nCapsCount = 50;

// CTS Table 19 Timing Table & Calculations
constexpr Interval tCtsAfterLastPoint{0, 500 * ms};
constexpr Interval tCtsBIST{0, 15 * ms};
constexpr Interval tCtsChunkMargin{0, 5 * ms};
constexpr Interval tCtsHardReset{0, 15 * ms};
constexpr Interval tCtsInfinite{0, 5000 * ms};
constexpr Interval tCtsSenderResponse{0, 50 * ms};
constexpr Interval tCtsSrcAcceptPD2{0, 22760 * us};
constexpr Interval tCtsSrcAcceptPD3{0, 25760 * us};
// PD2 max: the tester uses PD3 max timing on PD3 devices operating in PD2 mode
constexpr Interval tSenderResponsePD2{24 * ms, 50 * ms};
constexpr Interval tSenderResponsePD3{27 * ms, 50 * ms};

//
// Message data
//

namespace pdos {

// Flags for the first Fixed Source PDO. Base PDO constants have none set.
constexpr uint32_t dual_role_power = 1u << 29;
constexpr uint32_t unconstrained_power = 1u << 27;
constexpr uint32_t epr_capable = 1u << 23;

extern const uint32_t fixed_5v_500ma;
extern const uint32_t fixed_5v_3a;
extern const uint32_t fixed_9v_3a;
extern const uint32_t fixed_15v_3a;
extern const uint32_t fixed_20v_3a;
extern const uint32_t fixed_20v_5a;
extern const uint32_t fixed_28v_5a;
extern const uint32_t fixed_36v_5a;
extern const uint32_t fixed_48v_5a;

extern const uint32_t pps_5_21v_3a;
extern const uint32_t pps_5_21v_5a;
extern const uint32_t spr_avs_15v_3a_20v_5a;
extern const uint32_t epr_avs_15_48v_240w;
extern const uint32_t epr_avs_15_36v_240w;

} // namespace pdos

//
// Variants
//

struct Variant {
    pd::TCPC_FEATURES features;
    pd::PD_REVISION::Type revision;
};

// Rp levels presented by Tester
namespace Rp {
    constexpr auto Default = pd::TCPC_CC_LEVEL::RP_0_5;
    constexpr auto SinkTxNG = pd::TCPC_CC_LEVEL::RP_1_5;
    constexpr auto SinkTxOK = pd::TCPC_CC_LEVEL::RP_3_0;
} // namespace Rp

//
// Tester (acts as Source). Owns a fresh PD stack (UUT) on the mocked driver.
// Expect intervals are counted from the last activity on the wire.
//

// Message or Hard Reset seen on the wire (sent by UUT)
struct Frame {
    uint32_t time; // us
    bool hard_reset;
    pd::PD_CHUNK chunk;

    // Message predicates exclude Hard Reset.
    bool is_ctrl(pd::PD_CTRL_MSGT::Type type) const {
        return !hard_reset && chunk.is_ctrl_msg(type);
    }
    bool is_data(pd::PD_DATA_MSGT::Type type) const {
        return !hard_reset && chunk.is_data_msg(type);
    }
    bool is_ext(pd::PD_EXT_MSGT::Type type) const {
        return !hard_reset && chunk.is_ext_msg(type);
    }
    bool is_epr_mode(pd::EPR_MODE_ACTION::Type action) const {
        return is_data(pd::PD_DATA_MSGT::EPR_Mode) &&
            pd::EPRMDO{chunk.read32(0)}.action == action;
    }

    // Compare the full header and payload, ignoring time.
    bool same_msg(const pd::PD_CHUNK& other) const {
        return !hard_reset && chunk.header.raw_value == other.header.raw_value &&
            chunk.get_data() == other.get_data();
    }
    bool same_msg(const Frame& other) const {
        return !other.hard_reset && same_msg(other.chunk);
    }
};

// Complete extended message after chunk reassembly. `msg` contains only the
// data block, without Extended Header or wire padding.
struct ExtendedMessage {
    uint32_t time; // time of the first chunk, us
    pd::PD_MSG msg;
};

// Validate an already received raw frame. These checks do not receive messages
// or advance virtual time.
void check_chunk_request(const Frame& frame, pd::PD_EXT_MSGT::Type expected_type,
    uint8_t chunk_number);
void check_ext_ctrl_msg(const Frame& frame, pd::PD_EXT_CTRL_MSGT::type expected_type);

class Tester {
public:
    explicit Tester(const Variant& variant);

    Tester(const Tester&) = delete;
    Tester& operator=(const Tester&) = delete;

    // CTS procedures
    // Repeat Source_Capabilities up to nCapsCount times until GoodCRC.
    // Wait tTypeCSendSourceCap.min after each unacknowledged attempt.
    bool send_source_caps_until_acked(const std::vector<uint32_t>& source_caps);
    // COMMON.PROC.BU.2. Returns the Request from UUT. The final
    // tCtsAfterLastPoint observation ends early if UUT sends a message.
    auto bring_up_sink() -> std::optional<Frame>;
    auto bring_up_sink(const std::vector<uint32_t>& source_caps) -> std::optional<Frame>;
    // COMMON.PROC.BU.5. Returns the Request for the PPS APDO.
    auto bring_up_sink_pps() -> std::optional<Frame>;
    bool accept_request();  // COMMON.PROC.PD.10
    // Hard Reset, VBUS to vSafe0V, Rp default, VBUS restored.
    // Returns the time VBUS is restored.
    auto hard_reset_cycle_vbus() -> uint32_t;

    // Return true if GoodCRC received
    bool send_ctrl_msg(pd::PD_CTRL_MSGT::Type type);
    bool send_data_msg(pd::PD_DATA_MSGT::Type type, const std::vector<uint32_t>& objects);
    bool send_epr_source_caps(const std::vector<uint32_t>& source_caps);
    bool send_ext_ctrl_msg(pd::PD_EXT_CTRL_MSGT::type type, uint8_t data = 0);
    // Full extended message, serves chunk requests from UUT
    bool send_ext_msg(pd::PD_EXT_MSGT::Type type, const std::vector<uint8_t>& data);
    // Chunk `chunk_number` of a chunked extended message with data block `data`
    bool send_ext_msg_chunk(pd::PD_EXT_MSGT::Type type, const std::vector<uint8_t>& data,
        uint8_t chunk_number);
    // Repeat the last message with the same MessageID
    bool resend_last_msg();
    void send_hard_reset();

    // `from`: explicit start of interval, instead of the last wire activity
    auto expect_ctrl_msg(pd::PD_CTRL_MSGT::Type type, Interval interval,
        std::optional<uint32_t> from = std::nullopt) -> std::optional<Frame>;
    auto expect_data_msg(pd::PD_DATA_MSGT::Type type, Interval interval,
        std::optional<uint32_t> from = std::nullopt) -> std::optional<Frame>;
    // Chunked message is assembled, Tester requests the chunks
    auto expect_ext_msg(pd::PD_EXT_MSGT::Type type, Interval interval,
        std::optional<uint32_t> from = std::nullopt) -> std::optional<ExtendedMessage>;
    // Request for the next chunk of a message sent by the Tester
    auto expect_chunk_request(pd::PD_EXT_MSGT::Type type, uint8_t chunk_number, Interval interval,
        std::optional<uint32_t> from = std::nullopt) -> std::optional<Frame>;
    auto expect_ext_ctrl_msg(pd::PD_EXT_CTRL_MSGT::type type, Interval interval,
        std::optional<uint32_t> from = std::nullopt) -> std::optional<Frame>;
    auto expect_retries(const Frame& frame, int count, Interval interval) -> std::optional<Frame>;
    bool expect_hard_reset(Interval interval, std::optional<uint32_t> from = std::nullopt);
    // BIST mode set by UUT in TCPC. Returns time of change.
    auto expect_bist_mode(pd::TCPC_BIST_MODE mode, Interval interval,
        std::optional<uint32_t> from = std::nullopt) -> std::optional<uint32_t>;

    // Next pending/new frame from UUT, nullopt on timeout (not a failure).
    // Timeout is counted from `from`, or the last activity on the wire.
    auto receive(uint32_t timeout_us, std::optional<uint32_t> from = std::nullopt) -> std::optional<Frame>;

    // COMMON.CHECK.PD.4 is not applicable when UUT ignores messages (BIST)
    void set_response_check(bool enable);
    void set_goodcrc_reply(bool enable);
    // Specification Revision in headers of Tester messages (default: variant revision)
    void set_tx_spec_revision(uint8_t revision);
    void set_rp(pd::TCPC_CC_LEVEL::Type level);
    void set_vbus(bool on);

    // Wait for the full duration/deadline, preserving all frames from UUT
    // for expect_*/receive. A deadline at or before now does not change time.
    void wait(uint32_t duration_us);
    void wait_until(uint32_t time_us);
    // Stop when a pending/new frame from UUT is available or timeout expires.
    // The frame is not consumed and remains available for expect_*/receive.
    void wait_for_frame(uint32_t timeout_us);
    auto now() const -> uint32_t;

    auto revision() const -> pd::PD_REVISION::Type;
    bool is_pd2() const;
    int nRetryCount() const;

    pd::Port port{};
    DriverMock driver;
    pd::Task task;
    // UUT policy (what the VIF declares), e.g. which PDO to request
    pd::DPM dpm;
    pd::PRL prl;
    pd::PE pe;
    pd::TC tc;

private:
    Variant variant;

    uint32_t last_activity_us{0};
    uint8_t tx_msg_id{0};
    bool goodcrc_reply{true};
    std::optional<uint8_t> tx_spec_revision{};
    pd::PD_CHUNK last_tx_chunk{};
    std::deque<Frame> wire{}; // frames sent by UUT, not yet consumed

    // COMMON.CHECK.PD.4: UUT must respond within tReceiverResponse max
    void start_response_check(const std::string& request);
    void cancel_expected_response();
    std::optional<uint32_t> response_due_us{};
    std::string pending_request{};
    bool response_check_enabled{true};

    // State for COMMON.CHECK.*
    std::vector<uint32_t> last_src_caps{};     // last Source Capabilities from Tester
    std::optional<pd::PD_CHUNK> last_uut_msg{}; // to detect retries
    std::optional<uint8_t> last_uut_msg_id{};  // nullopt: next MessageID must be 0
    bool last_uut_msg_acked{false};            // retry is possible only without GoodCRC
    // COMMON.CHECK.* on every message from UUT
    void run_common_checks(const Frame& frame);
    void run_ext_msg_checks(const ExtendedMessage& message);

    bool send_chunk(pd::PD_CHUNK chunk);
    bool send_chunk_request(pd::PD_EXT_MSGT::Type type, uint8_t chunk_number);
    bool put_on_wire(const pd::PD_CHUNK& chunk);
    auto expect_frame(const std::function<bool(const Frame&)>& match,
        const std::string& name, Interval interval, uint32_t anchor) -> std::optional<Frame>;

    // Simulation
    void process();
    void emit_tx_chunk();
    auto next_tick_us() const -> std::optional<uint32_t>;
    // Stop at `time_us` or when wire has more than `frames` frames
    void run_until(uint32_t time_us, size_t frames = SIZE_MAX);
};

//
// GoogleTest integration
//

enum Revisions {
    REV3,    // PD3 only
    REV_ALL  // PD2 + PD3
};

// Number of fatal and nonfatal assertion failures in the current test.
int test_failure_count();

using BodyFunc = void (*)(Tester& t);

bool register_test(const char* suite_name, const char* test_name, Revisions revisions,
    BodyFunc body, const char* file, int line);

} // namespace cts

#define CTS_TEST_CONCAT_INNER_(left, right) left##right
#define CTS_TEST_CONCAT_(left, right) CTS_TEST_CONCAT_INNER_(left, right)

// Runs for each predefined feature set x revision (REV_ALL: PD2 + PD3),
// `t` is the Tester (acts as Source) bound to the mocked driver.
// Expect intervals are counted from the last activity on the wire.
#define CTS_TEST(suite_name, test_name, revisions)                             \
    static void CTS_TEST_CONCAT_(cts_test_body_, __LINE__)(::cts::Tester& t);  \
    [[maybe_unused]] static const bool CTS_TEST_CONCAT_(cts_test_reg_, __LINE__) = \
        ::cts::register_test((suite_name), (test_name), (revisions),           \
            &CTS_TEST_CONCAT_(cts_test_body_, __LINE__), __FILE__, __LINE__);   \
    static void CTS_TEST_CONCAT_(cts_test_body_, __LINE__)([[maybe_unused]] ::cts::Tester& t)
