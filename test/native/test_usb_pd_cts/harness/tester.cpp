#include "tester.h"

#include <algorithm>
#include <string>

namespace cts {

using namespace pd;

//
// Message data
//

namespace {

uint32_t make_fixed_pdo(uint32_t mv, uint32_t ma) {
    PDO_FIXED pdo{};
    pdo.pdo_type = PDO_TYPE::FIXED;
    pdo.voltage = mv / 50;
    pdo.max_current = ma / 10;
    return pdo.raw_value;
}

uint32_t make_pps_pdo(uint32_t min_mv, uint32_t max_mv, uint32_t ma) {
    PDO_SPR_PPS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_PPS;
    pdo.min_voltage = min_mv / 100;
    pdo.max_voltage = max_mv / 100;
    pdo.max_current = ma / 50;
    return pdo.raw_value;
}

uint32_t make_spr_avs_pdo(uint32_t ma_15v, uint32_t ma_20v) {
    PDO_SPR_AVS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::SPR_AVS;
    pdo.max_current_15v = ma_15v / 10;
    pdo.max_current_20v = ma_20v / 10;
    return pdo.raw_value;
}

uint32_t make_epr_avs_pdo(uint32_t min_mv, uint32_t max_mv, uint32_t pdp_w) {
    PDO_EPR_AVS pdo{};
    pdo.pdo_type = PDO_TYPE::AUGMENTED;
    pdo.apdo_subtype = PDO_AUGMENTED_SUBTYPE::EPR_AVS;
    pdo.min_voltage = min_mv / 100;
    pdo.max_voltage = max_mv / 100;
    pdo.pdp = pdp_w;
    return pdo.raw_value;
}

} // namespace

namespace pdos {

const uint32_t fixed_5v_500ma = make_fixed_pdo(5000, 500);
const uint32_t fixed_5v_3a = make_fixed_pdo(5000, 3000);
const uint32_t fixed_9v_3a = make_fixed_pdo(9000, 3000);
const uint32_t fixed_15v_3a = make_fixed_pdo(15000, 3000);
const uint32_t fixed_20v_3a = make_fixed_pdo(20000, 3000);
const uint32_t fixed_20v_5a = make_fixed_pdo(20000, 5000);
const uint32_t fixed_28v_5a = make_fixed_pdo(28000, 5000);
const uint32_t fixed_36v_5a = make_fixed_pdo(36000, 5000);
const uint32_t fixed_48v_5a = make_fixed_pdo(48000, 5000);

const uint32_t pps_5_21v_3a = make_pps_pdo(5000, 21000, 3000);
const uint32_t pps_5_21v_5a = make_pps_pdo(5000, 21000, 5000);
const uint32_t spr_avs_15v_3a_20v_5a = make_spr_avs_pdo(3000, 5000);
const uint32_t epr_avs_15_48v_240w = make_epr_avs_pdo(15000, 48000, 240);
const uint32_t epr_avs_15_36v_240w = make_epr_avs_pdo(15000, 36000, 240);

} // namespace pdos

//
// Tester
//

Tester::Tester(const Variant& variant)
    : driver{port, variant.features},
      task{port},
      dpm{port},
      prl{port},
      pe{port},
      tc{port},
      variant{variant}
{
    const auto& features = variant.features;
    if (!features.rx_goodcrc_send || !features.tx_goodcrc_wait) {
        ADD_FAILURE() << "DriverMock: GoodCRC send and wait must be enabled";
    }

    DriverMock::now_us = 0;
    port.start(driver);
    process();
}

//
// Simulation
//

// Put tx_chunk on the wire and handle Tester GoodCRC.
// Without GoodCRC, the TCPC waits tReceive before retrying or reporting failure.
void Tester::emit_tx_chunk() {
    wire.push_back({DriverMock::now_us, false, driver.tx_chunk});
    last_activity_us = DriverMock::now_us;
    run_common_checks(wire.back());

    if (goodcrc_reply) {
        // Tester PRL layer reset on received Soft_Reset
        if (driver.tx_chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) { tx_msg_id = 0; }

        driver.retry_at_us.reset();
        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::SUCCEEDED);
        driver.wakeup_pending = true;
        return;
    }

    // Nominal tReceive
    driver.retry_at_us = DriverMock::now_us + 1 * ms;
}

// Complete pending async operations at the current time.
void Tester::process() {
    for (int guard = 0; guard < 10000; guard++) {
        if (driver.polarity_pending) {
            driver.polarity = *driver.polarity_pending;
            driver.polarity_pending.reset();

            if (driver.polarity == TCPC_POLARITY::TOGGLING) {
                driver.cc1_cache = TCPC_CC_LEVEL::NONE;
                driver.cc2_cache = TCPC_CC_LEVEL::NONE;
                const bool cc1_rp = driver.cc1 != TCPC_CC_LEVEL::NONE;
                const bool cc2_rp = driver.cc2 != TCPC_CC_LEVEL::NONE;
                if (cc1_rp != cc2_rp) {
                    driver.polarity = cc1_rp ? TCPC_POLARITY::CC1 : TCPC_POLARITY::CC2;
                    driver.cc1_cache = cc1_rp ? driver.cc1 : TCPC_CC_LEVEL::NONE;
                    driver.cc2_cache = cc2_rp ? driver.cc2 : TCPC_CC_LEVEL::NONE;
                }
            }

            driver.wakeup_pending = true;
            continue;
        }

        if (driver.cc_pending) {
            const auto selector = *driver.cc_pending;
            driver.cc_pending.reset();
            switch (selector) {
                case TCPC_CC_REQ::CC1:
                    driver.cc1_cache = driver.cc1;
                    break;
                case TCPC_CC_REQ::CC2:
                    driver.cc2_cache = driver.cc2;
                    break;
                case TCPC_CC_REQ::CC1CC2:
                    driver.cc1_cache = driver.cc1;
                    driver.cc2_cache = driver.cc2;
                    break;
                case TCPC_CC_REQ::ACTIVE_CC:
                    if (driver.polarity == TCPC_POLARITY::CC1) {
                        driver.cc1_cache = driver.cc1;
                    } else if (driver.polarity == TCPC_POLARITY::CC2) {
                        driver.cc2_cache = driver.cc2;
                    } else {
                        ADD_FAILURE() << "DriverMock: ACTIVE_CC without selected polarity";
                    }
                    break;
            }
            driver.wakeup_pending = true;
            continue;
        }

        if (driver.tx_pending) {
            driver.tx_pending = false;
            driver.retries_left = variant.features.tx_retries
                ? port.max_retries() : 0;
            emit_tx_chunk();
            continue;
        }

        if (driver.retry_at_us && *driver.retry_at_us <= DriverMock::now_us) {
            driver.retry_at_us.reset();

            if (driver.retries_left > 0) {
                driver.retries_left--;
                emit_tx_chunk();
            } else {
                port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::FAILED);
                driver.wakeup_pending = true;
            }
            continue;
        }

        if (driver.hr_pending) {
            driver.hr_pending = false;

            wire.push_back({DriverMock::now_us, true, {}});
            last_activity_us = DriverMock::now_us;
            run_common_checks(wire.back());
            tx_msg_id = 0;

            driver.wakeup_pending = true;
            continue;
        }

        if (driver.wakeup_pending) {
            driver.wakeup_pending = false;
            port.notify_task(MsgTask_Wakeup{});
            continue;
        }

        return;
    }
    ADD_FAILURE() << "Tester: event loop does not settle";
}

auto Tester::next_tick_us() const -> std::optional<uint32_t> {
    return driver.rearm_at_us;
}

void Tester::run_until(uint32_t time_us, size_t frames) {
    for (;;) {
        process();

        // COMMON.CHECK.PD.4: response deadline passed with no message from UUT
        if (response_due_us && DriverMock::now_us > *response_due_us) {
            ADD_FAILURE() << "No response to " << pending_request << " within "
                << tReceiverResponse.max << "us";
            response_due_us.reset();
        }

        if (wire.size() > frames) { return; }
        if (DriverMock::now_us >= time_us) { return; }

        auto next = time_us;
        const auto tick = next_tick_us();
        if (tick) { next = std::min(next, *tick); }
        if (driver.retry_at_us) { next = std::min(next, *driver.retry_at_us); }

        DriverMock::now_us = std::max(next, DriverMock::now_us);

        if (tick && *tick <= DriverMock::now_us) {
            driver.rearm_at_us.reset();
            port.notify_task(MsgTask_Timer{});
        }
    }
}

void Tester::wait(uint32_t duration_us) {
    wait_until(DriverMock::now_us + duration_us);
}

void Tester::wait_until(uint32_t time_us) {
    run_until(time_us);
}

void Tester::wait_for_frame(uint32_t timeout_us) {
    run_until(DriverMock::now_us + timeout_us, 0);
}

// COMMON.CHECK.PD.4: messages that start an AMS and require a UUT response.
// Exceptions from the check are not listed here (PS_RDY in Power Negotiation,
// Not_Supported for a chunked request and so on).
void Tester::start_response_check(const std::string& request) {
    if (!response_check_enabled) { return; }

    response_due_us = DriverMock::now_us + tReceiverResponse.max;
    pending_request = request;
}

void Tester::cancel_expected_response() { response_due_us.reset(); }

void Tester::set_response_check(bool enable) {
    response_check_enabled = enable;
    if (!enable) { response_due_us.reset(); }
}

//
// Wire
//

// Fill common header fields and send. Message type, extended flag and
// payload are set by caller.
bool Tester::send_chunk(PD_CHUNK chunk) {
    auto& header = chunk.header;

    header.spec_revision = tx_spec_revision.value_or(variant.revision);
    header.port_power_role = 1; // Source
    header.port_data_role = 1;  // DFP
    // Tester PRL layer reset on sent Soft_Reset
    if (chunk.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) {
        tx_msg_id = 0;
        last_uut_msg_id.reset();
        last_uut_msg.reset();
    }
    header.message_id = tx_msg_id;

    // MessageID is incremented after both success and failure
    tx_msg_id = (tx_msg_id + 1) & 7;

    last_tx_chunk = chunk;
    return put_on_wire(chunk);
}

bool Tester::put_on_wire(const PD_CHUNK& chunk) {
    process();

    last_activity_us = DriverMock::now_us;

    // TCPC sends GoodCRC only when RX is enabled.
    if (!driver.rx_enabled) { return false; }

    // process() must finish delivery before the Tester supplies another packet.
    if (driver.rx_buffer_has_pkt) {
        ADD_FAILURE() << "Tester: previous RX packet was not consumed";
        return false;
    }

    driver.receive_chunk(chunk);
    process();
    return true;
}

bool Tester::resend_last_msg() {
    return put_on_wire(last_tx_chunk);
}

void Tester::send_hard_reset() {
    process();

    tx_msg_id = 0;
    last_uut_msg_id.reset();
    last_uut_msg.reset();
    last_activity_us = DriverMock::now_us;

    // TCPC cleanup on received Hard Reset
    driver.bist_mode = TCPC_BIST_MODE::Off;
    driver.rx_buffer_has_pkt = false;
    driver.tx_pending = false;
    driver.retry_at_us.reset();
    port.notify_prl(MsgToPrl_TcpcHardReset{});
    driver.wakeup_pending = true;
    process();
}

bool Tester::send_ctrl_msg(PD_CTRL_MSGT::Type type) {
    switch (type) {
        case PD_CTRL_MSGT::Get_Sink_Cap:
        case PD_CTRL_MSGT::Get_Sink_Cap_Extended:
        case PD_CTRL_MSGT::Get_Source_Cap:
        case PD_CTRL_MSGT::Get_Status:
        case PD_CTRL_MSGT::Get_Revision:
        case PD_CTRL_MSGT::Soft_Reset:
            start_response_check("control message " + std::to_string(type));
            break;
        default:
            break;
    }

    PD_CHUNK chunk{};
    chunk.header.message_type = type;
    const bool sent = send_chunk(chunk);
    if (!sent) { cancel_expected_response(); }
    return sent;
}

bool Tester::send_data_msg(PD_DATA_MSGT::Type type, const std::vector<uint32_t>& objects) {
    if (type == PD_DATA_MSGT::Source_Capabilities) { last_src_caps = objects; }

    // Before the UUT says anything (bring-up, Hard Reset recovery) the Tester is
    // expected to repeat Source_Capabilities, so there is no AMS to check yet.
    if (type == PD_DATA_MSGT::Source_Capabilities && last_uut_msg_id) {
        start_response_check("Source_Capabilities");
    }

    PD_CHUNK chunk{};
    chunk.header.message_type = type;
    chunk.header.data_obj_count = objects.size();
    for (auto obj : objects) { chunk.append32(obj); }
    const bool sent = send_chunk(chunk);
    if (!sent) { cancel_expected_response(); }
    return sent;
}

bool Tester::send_epr_source_caps(const std::vector<uint32_t>& source_caps) {
    last_src_caps = source_caps;

    std::vector<uint8_t> data;
    for (auto obj : source_caps) {
        for (int i = 0; i < 4; i++) { data.push_back(uint8_t(obj >> (i * 8))); }
    }
    return send_ext_msg(PD_EXT_MSGT::EPR_Source_Capabilities, data);
}

bool Tester::send_ext_ctrl_msg(PD_EXT_CTRL_MSGT::type type, uint8_t data) {
    return send_ext_msg(PD_EXT_MSGT::Extended_Control, { uint8_t(type), data });
}

namespace {

auto ext_msg_name(PD_EXT_MSGT::Type type, const std::vector<uint8_t>& data) -> std::string {
    if (type == PD_EXT_MSGT::Extended_Control && !data.empty()) {
        return "extended control message " + std::to_string(data[0]);
    }
    return "extended message " + std::to_string(type);
}

} // namespace

bool Tester::send_ext_msg(PD_EXT_MSGT::Type type, const std::vector<uint8_t>& data) {
    // The response is expected after the last chunk
    const auto last_chunk = data.empty() ? 0
        : uint8_t((data.size() - 1) / MaxExtendedMsgChunkLen);
    const bool wait_response = type == PD_EXT_MSGT::EPR_Source_Capabilities ||
        (type == PD_EXT_MSGT::Extended_Control &&
            !data.empty() && data[0] == PD_EXT_CTRL_MSGT::EPR_Get_Sink_Cap);

    if (wait_response && last_chunk == 0) { start_response_check(ext_msg_name(type, data)); }
    if (!send_ext_msg_chunk(type, data, 0)) {
        cancel_expected_response();
        return false;
    }

    for (uint8_t n = 1; n * MaxExtendedMsgChunkLen < data.size(); n++) {
        if (!expect_chunk_request(type, n, tChunkReceiverRequest)) { return false; }
        if (wait_response && n == last_chunk) { start_response_check(ext_msg_name(type, data)); }
        if (!send_ext_msg_chunk(type, data, n)) {
            cancel_expected_response();
            return false;
        }
    }
    return true;
}

bool Tester::send_ext_msg_chunk(PD_EXT_MSGT::Type type, const std::vector<uint8_t>& data,
    uint8_t chunk_number)
{
    PD_EXT_HEADER ext_header{};
    ext_header.chunked = 1;
    ext_header.chunk_number = chunk_number;
    ext_header.data_size = data.size();

    PD_CHUNK chunk{};
    chunk.header.message_type = type;
    chunk.header.extended = 1;
    chunk.append16(ext_header.raw_value);

    const size_t start = size_t(chunk_number) * MaxExtendedMsgChunkLen;
    const size_t end = std::min(data.size(), start + MaxExtendedMsgChunkLen);
    for (size_t i = start; i < end; i++) { chunk.get_data().push_back(data[i]); }

    // Pad to data objects
    while (chunk.data_size() % 4) { chunk.get_data().push_back(0); }
    chunk.header.data_obj_count = chunk.data_size() / 4;

    return send_chunk(chunk);
}

bool Tester::send_chunk_request(PD_EXT_MSGT::Type type, uint8_t chunk_number) {
    PD_EXT_HEADER ext_header{};
    ext_header.chunked = 1;
    ext_header.request_chunk = 1;
    ext_header.chunk_number = chunk_number;

    PD_CHUNK chunk{};
    chunk.header.message_type = type;
    chunk.header.extended = 1;
    chunk.append16(ext_header.raw_value);
    chunk.append16(0); // Padding
    chunk.header.data_obj_count = 1;

    return send_chunk(chunk);
}

auto Tester::expect_frame(const std::function<bool(const Frame&)>& match,
    const std::string& name, Interval interval, uint32_t anchor) -> std::optional<Frame>
{
    run_until(anchor + interval.max, 0);

    if (wire.empty()) {
        ADD_FAILURE() << "Expected " << name << " within " << interval.max << "us, got nothing";
        return std::nullopt;
    }

    auto frame = wire.front();
    wire.pop_front();

    const auto elapsed = int64_t(frame.time) - int64_t(anchor);
    const auto& hdr = frame.chunk.header;

    if (!match(frame)) {
        if (frame.hard_reset) {
            ADD_FAILURE() << "Expected " << name << ", got Hard Reset after " << elapsed << "us";
        } else {
            ADD_FAILURE() << "Expected " << name << ", got message type " << hdr.message_type
                << " (extended " << hdr.extended << ", data objects " << hdr.data_obj_count
                << ") after " << elapsed << "us";
        }
        return std::nullopt;
    }

    if (elapsed < int64_t(interval.min) || elapsed > int64_t(interval.max)) {
        ADD_FAILURE() << name << " received after " << elapsed << "us, expected ["
            << interval.min << ", " << interval.max << "]us";
        return std::nullopt;
    }

    return frame;
}

auto Tester::expect_ctrl_msg(PD_CTRL_MSGT::Type type, Interval interval,
    std::optional<uint32_t> from) -> std::optional<Frame>
{
    return expect_frame(
        [type](const Frame& frame) { return frame.is_ctrl(type); },
        "control message " + std::to_string(type), interval, from.value_or(last_activity_us));
}

auto Tester::expect_data_msg(PD_DATA_MSGT::Type type, Interval interval,
    std::optional<uint32_t> from) -> std::optional<Frame>
{
    return expect_frame(
        [type](const Frame& frame) { return frame.is_data(type); },
        "data message " + std::to_string(type), interval, from.value_or(last_activity_us));
}

auto Tester::expect_ext_msg(PD_EXT_MSGT::Type type, Interval interval,
    std::optional<uint32_t> from) -> std::optional<ExtendedMessage>
{
    const auto is_type = [type](const Frame& frame) {
        return frame.is_ext(type);
    };
    const auto name = "extended message " + std::to_string(type);

    auto frame = expect_frame(is_type, name, interval, from.value_or(last_activity_us));
    if (!frame) { return std::nullopt; }

    const PD_EXT_HEADER first{frame->chunk.read16(0)};
    if (first.request_chunk) {
        ADD_FAILURE() << "Expected " << name << ", got a chunk request";
        return std::nullopt;
    }
    if (first.chunked && first.chunk_number != 0) {
        ADD_FAILURE() << "First chunk number is " << first.chunk_number;
        return std::nullopt;
    }

    if (first.data_size > PD_MSG::MAX_SIZE) {
        ADD_FAILURE() << name << " data size " << first.data_size
            << " exceeds message capacity " << PD_MSG::MAX_SIZE;
        return std::nullopt;
    }

    ExtendedMessage message{frame->time, {}};
    message.msg.header = frame->chunk.header;
    auto& data = message.msg.get_data();

    const auto append_data = [&data, &name](const Frame& chunk, size_t size) -> bool {
        const auto& chunk_data = chunk.chunk.get_data();
        if (chunk_data.size() < 2 + size) {
            ADD_FAILURE() << name << " chunk payload has " << chunk_data.size()
                << " bytes, expected at least " << 2 + size;
            return false;
        }
        if (data.available() < size) {
            ADD_FAILURE() << name << " does not fit the reassembly buffer";
            return false;
        }
        data.insert(data.end(), chunk_data.begin() + 2, chunk_data.begin() + 2 + size);
        return true;
    };

    const size_t first_size = std::min<size_t>(MaxExtendedMsgChunkLen, first.data_size);
    if (!append_data(*frame, first_size)) { return std::nullopt; }

    if (!first.chunked && data.size() != first.data_size) {
        ADD_FAILURE() << name << " unchunked payload has " << data.size()
            << " bytes, expected " << first.data_size;
        return std::nullopt;
    }

    for (uint8_t n = 1; data.size() < first.data_size; n++) {
        if (!send_chunk_request(type, n)) { return std::nullopt; }

        auto chunk = expect_frame(is_type, name + " chunk " + std::to_string(n),
            tReceiverResponse, last_activity_us);
        if (!chunk) { return std::nullopt; }

        const PD_EXT_HEADER header{chunk->chunk.read16(0)};
        if (header.request_chunk || header.chunk_number != n) {
            ADD_FAILURE() << "Expected chunk " << int(n) << ", got chunk " << header.chunk_number
                << " (request " << header.request_chunk << ")";
            return std::nullopt;
        }
        if (header.data_size != first.data_size) {
            ADD_FAILURE() << "Chunk " << int(n) << " data size is " << header.data_size
                << ", expected " << first.data_size;
            return std::nullopt;
        }

        const size_t size = std::min<size_t>(MaxExtendedMsgChunkLen,
            first.data_size - data.size());
        if (!append_data(*chunk, size)) { return std::nullopt; }
    }

    run_ext_msg_checks(message);
    return message;
}

auto Tester::expect_chunk_request(PD_EXT_MSGT::Type type, uint8_t chunk_number,
    Interval interval, std::optional<uint32_t> from) -> std::optional<Frame>
{
    auto frame = expect_frame(
        [type](const Frame& frame) { return frame.is_ext(type); },
        "chunk " + std::to_string(chunk_number) + " request", interval,
        from.value_or(last_activity_us));
    if (!frame) { return std::nullopt; }

    check_chunk_request(*frame, type, chunk_number);

    return frame;
}

auto Tester::expect_ext_ctrl_msg(PD_EXT_CTRL_MSGT::type type, Interval interval,
    std::optional<uint32_t> from) -> std::optional<Frame>
{
    auto frame = expect_frame(
        [type](const Frame& frame) {
            return frame.is_ext(PD_EXT_MSGT::Extended_Control) &&
                ECDB{frame.chunk.read16(2)}.type == type;
        },
        "extended control message " + std::to_string(type), interval,
        from.value_or(last_activity_us));
    if (!frame) { return std::nullopt; }

    check_ext_ctrl_msg(*frame, type);

    return frame;
}

auto Tester::expect_retries(const Frame& frame, int count, Interval interval) -> std::optional<Frame> {
    std::optional<Frame> last{frame};

    for (int i = 1; i <= count && last; i++) {
        last = expect_frame(
            [&frame](const Frame& retry) { return retry.same_msg(frame); },
            "retry " + std::to_string(i), interval, last_activity_us);
    }
    return last;
}

auto Tester::expect_bist_mode(TCPC_BIST_MODE mode, Interval interval,
    std::optional<uint32_t> from) -> std::optional<uint32_t>
{
    const auto anchor = from.value_or(last_activity_us);

    run_until(anchor + interval.max);

    for (const auto& event : driver.bist_log) {
        if (event.mode != mode || event.time < anchor) { continue; }

        const auto elapsed = event.time - anchor;
        if (elapsed < interval.min || elapsed > interval.max) {
            ADD_FAILURE() << "BIST mode " << int(mode) << " set after " << elapsed
                << "us, expected [" << interval.min << ", " << interval.max << "]us";
            return std::nullopt;
        }
        return event.time;
    }

    ADD_FAILURE() << "Expected BIST mode " << int(mode) << " within " << interval.max << "us";
    return std::nullopt;
}

auto Tester::receive(uint32_t timeout_us, std::optional<uint32_t> from) -> std::optional<Frame> {
    const auto anchor = from.value_or(last_activity_us);

    run_until(anchor + timeout_us, 0);

    if (wire.empty()) { return std::nullopt; }

    auto frame = wire.front();
    wire.pop_front();
    return frame;
}

bool Tester::expect_hard_reset(Interval interval, std::optional<uint32_t> from) {
    return expect_frame([](const Frame& frame) { return frame.hard_reset; },
        "Hard Reset", interval, from.value_or(last_activity_us)).has_value();
}

void Tester::set_goodcrc_reply(bool enable) { goodcrc_reply = enable; }

void Tester::set_tx_spec_revision(uint8_t revision) { tx_spec_revision = revision; }

void Tester::set_rp(TCPC_CC_LEVEL::Type level) {
    driver.cc1 = level;
    if (driver.polarity == TCPC_POLARITY::TOGGLING &&
        (driver.cc1 != TCPC_CC_LEVEL::NONE) != (driver.cc2 != TCPC_CC_LEVEL::NONE)) {
        driver.polarity = driver.cc1 != TCPC_CC_LEVEL::NONE
            ? TCPC_POLARITY::CC1 : TCPC_POLARITY::CC2;
        driver.cc1_cache = driver.cc1;
        driver.cc2_cache = driver.cc2;
    }
    driver.wakeup_pending = true;
    process();
}

void Tester::set_vbus(bool on) {
    // Detach restarts the UUT MessageID sequence
    if (!on) {
        last_uut_msg_id.reset();
        last_uut_msg.reset();
    }

    driver.vbus = on;
    driver.wakeup_pending = true; // VBUS change interrupt
    process();
}

auto Tester::now() const -> uint32_t { return DriverMock::now_us; }

auto Tester::revision() const -> PD_REVISION::Type { return variant.revision; }

bool Tester::is_pd2() const { return variant.revision == PD_REVISION::REV20; }

int Tester::nRetryCount() const {
    // [rev3.2 v1.2] Table 7.11 for PD3, [rev2.0] for PD2. Kept independent of
    // the stack constants on purpose.
    return is_pd2() ? 3 : 2;
}

//
// CTS procedures
//

bool Tester::send_source_caps_until_acked(const std::vector<uint32_t>& source_caps) {
    for (int i = 0; i < nCapsCount; i++) {
        if (send_data_msg(PD_DATA_MSGT::Source_Capabilities, source_caps)) { return true; }
        wait(tTypeCSendSourceCap.min);
    }
    return false;
}

// COMMON.PROC.BU.2 Bring-up Sink UUT
auto Tester::bring_up_sink() -> std::optional<Frame> {
    // Step 5: single PDO
    return bring_up_sink({ pdos::fixed_5v_500ma |
        pdos::dual_role_power | pdos::unconstrained_power });
}

auto Tester::bring_up_sink(const std::vector<uint32_t>& source_caps) -> std::optional<Frame> {
    const auto failures_before = test_failure_count();

    // Steps 1-3
    set_rp(Rp::Default);
    set_vbus(true);

    // Step 4
    wait(50 * ms);

    // Steps 5-6
    if (!send_source_caps_until_acked(source_caps)) {
        ADD_FAILURE() << "COMMON.PROC.BU.2: no GoodCRC to Source_Capabilities";
        return std::nullopt;
    }

    // Steps 7-10
    auto request = expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [COMMON.PROC.BU.2#1]
    if (!request) { return std::nullopt; }
    if (!accept_request()) { return std::nullopt; }

    // Step 11
    if (!is_pd2()) { set_rp(Rp::SinkTxOK); }
    wait_for_frame(tCtsAfterLastPoint.max);

    if (test_failure_count() != failures_before) { return std::nullopt; }
    return request;
}

// COMMON.PROC.BU.5 Bring-up Sink UUT for PPS Test
auto Tester::bring_up_sink_pps() -> std::optional<Frame> {
    const auto failures_before = test_failure_count();

    // Steps 1-3
    set_rp(Rp::SinkTxOK);
    set_vbus(true);

    // Step 4
    const std::vector<uint32_t> caps{
        pdos::fixed_5v_3a | pdos::dual_role_power | pdos::unconstrained_power,
        pdos::fixed_9v_3a, pdos::fixed_15v_3a, pdos::fixed_20v_3a,
        pdos::pps_5_21v_3a
    };
    constexpr uint32_t pps_position = 5;

    // Step 5
    if (!send_source_caps_until_acked(caps)) {
        ADD_FAILURE() << "COMMON.PROC.BU.5: no GoodCRC to Source_Capabilities";
        return std::nullopt;
    }

    // Steps 6-9
    auto request = expect_data_msg(PD_DATA_MSGT::Request, tReceiverResponse); // [COMMON.PROC.BU.5#1]
    if (!request) { return std::nullopt; }
    if (!accept_request()) { return std::nullopt; }

    // Step 10
    set_rp(Rp::SinkTxOK);

    // Steps 11-12, if the first contract is not on the PPS APDO
    if (RDO_ANY{request->chunk.read32(0)}.obj_position != pps_position) {
        request = receive(5000 * ms);
        if (!request || !request->is_data(PD_DATA_MSGT::Request) ||
            RDO_ANY{request->chunk.read32(0)}.obj_position != pps_position)
        {
            ADD_FAILURE() << "COMMON.PROC.BU.5: no Request for the PPS APDO";
            return std::nullopt;
        }
        if (!accept_request()) { return std::nullopt; }
    }

    // Step 13
    set_rp(Rp::SinkTxOK);
    wait_for_frame(tCtsAfterLastPoint.max);

    if (test_failure_count() != failures_before) { return std::nullopt; }
    return request;
}

// COMMON.PROC.PD.10 UUT Sent Request
bool Tester::accept_request() {
    if (!send_ctrl_msg(PD_CTRL_MSGT::Accept)) { return false; }
    wait(tSrcTransReq.max);
    return send_ctrl_msg(PD_CTRL_MSGT::PS_RDY);
}

// VBUS timings as for Source Hard Reset response
auto Tester::hard_reset_cycle_vbus() -> uint32_t {
    send_hard_reset();
    wait(tPSHardReset.max);
    set_vbus(false);
    set_rp(Rp::Default);
    wait(tSrcRecover.min);
    set_vbus(true);
    return now();
}

//
// CTS checks
//

// COMMON.CHECK.* procedures, applied to every message received from UUT.
//
// VIF values are taken from DPM: it plays the role of the UUT declaration.

namespace {

auto pdo_variant(uint32_t pdo) -> PDO_VARIANT {
    if (PDO_FIXED{pdo}.pdo_type != PDO_TYPE::AUGMENTED) {
        return PDO_FIXED{pdo}.pdo_type == PDO_TYPE::FIXED ? PDO_VARIANT::FIXED : PDO_VARIANT::UNKNOWN;
    }
    switch (PDO_SPR_PPS{pdo}.apdo_subtype) {
        case PDO_AUGMENTED_SUBTYPE::SPR_PPS: return PDO_VARIANT::APDO_PPS;
        case PDO_AUGMENTED_SUBTYPE::SPR_AVS: return PDO_VARIANT::APDO_SPR_AVS;
        case PDO_AUGMENTED_SUBTYPE::EPR_AVS: return PDO_VARIANT::APDO_EPR_AVS;
        default: return PDO_VARIANT::UNKNOWN;
    }
}

// COMMON.CHECK.PD.2, COMMON.CHECK.PD.6
void check_header(const Frame& frame, PD_REVISION::Type revision) {
    const auto& hdr = frame.chunk.header;
    const auto data_size = frame.chunk.data_size();

    EXPECT_EQ(hdr.spec_revision, revision) << "Specification Revision"; // [PD.2#1]
    EXPECT_EQ(hdr.port_power_role, 0) << "Port Power Role must be Sink";  // [PD.2#1]
    EXPECT_EQ(hdr.port_data_role, 0) << "Port Data Role must be UFP";     // [PD.2#1]

    if (hdr.extended == 0) {
        EXPECT_EQ(data_size, hdr.data_obj_count * 4u) << "Data Objects vs payload"; // [PD.2#1]
    } else if (PD_EXT_HEADER{frame.chunk.read16(0)}.chunked) {
        EXPECT_EQ(data_size, hdr.data_obj_count * 4u) << "Data Objects vs chunk payload";
    } else {
        EXPECT_EQ(hdr.data_obj_count, 0) << "Unchunked extended message";
    }

    if (frame.is_data(PD_DATA_MSGT::Request) || frame.is_data(PD_DATA_MSGT::Alert)) {
        EXPECT_EQ(hdr.data_obj_count, 1);
    }
    if (frame.is_data(PD_DATA_MSGT::EPR_Request)) {
        EXPECT_EQ(hdr.data_obj_count, 2);
    }

    // [PD.6#1] Control message has no payload
    if (hdr.extended == 0 && hdr.data_obj_count == 0) { EXPECT_EQ(data_size, 0u); }
}

// COMMON.CHECK.PD.10
void check_ext_header(const Frame& frame) {
    if (frame.chunk.header.extended == 0) { return; }

    const PD_EXT_HEADER ext{frame.chunk.read16(0)};
    // Unchunked extended messages are not supported by the Tester
    EXPECT_EQ(ext.chunked, 1) << "Chunked bit";
    EXPECT_EQ(ext.raw_value & (1u << 9), 0u) << "Extended header reserved bit";

    // Chunk carries its own slice of the data block, a chunk request carries none
    const size_t sent = size_t(ext.chunk_number) * MaxExtendedMsgChunkLen;
    size_t chunk_len = 0;
    if (ext.request_chunk) {
        EXPECT_EQ(ext.data_size, 0u) << "Chunk request Data Size";
    } else {
        ASSERT_LE(sent, ext.data_size) << "Chunk Number vs Data Size";
        chunk_len = std::min<size_t>(MaxExtendedMsgChunkLen, ext.data_size - sent);
    }

    // Payload is padded to a 4-byte boundary with zeros
    const size_t payload = 2 + chunk_len;
    EXPECT_EQ(frame.chunk.data_size(), (payload + 3) / 4 * 4) << "Chunk size";

    for (size_t i = payload; i < frame.chunk.data_size(); i++) {
        EXPECT_EQ(frame.chunk.get_data()[i], 0) << "Padding at byte " << i;
    }
}

// COMMON.CHECK.PD.8, COMMON.CHECK.PD3.1
void check_request(const Frame& frame, const std::vector<uint32_t>& src_caps,
    const PDO_LIST& vif_pdos, PD_REVISION::Type revision)
{
    const bool is_epr = frame.is_data(PD_DATA_MSGT::EPR_Request);
    const auto rdo = frame.chunk.read32(0);
    const RDO_ANY rdo_any{rdo};

    ASSERT_FALSE(src_caps.empty()) << "No Source Capabilities sent before Request";
    ASSERT_GE(rdo_any.obj_position, 1u) << "Object Position";
    ASSERT_LE(rdo_any.obj_position, src_caps.size()) << "Object Position";

    const auto pdo = src_caps[rdo_any.obj_position - 1];

    // Reserved bits, [PD.8#1], [PD3.1#1]
    EXPECT_EQ((rdo >> 27) & 1u, 0u) << "RDO B27 reserved";
    EXPECT_EQ((rdo >> 20) & 1u, 0u) << "RDO B20 reserved";

    if (revision == PD_REVISION::REV20) {
        EXPECT_EQ((rdo >> 21) & 0x7u, 0u) << "RDO B23..21 reserved in PD2";
    } else {
        EXPECT_EQ((rdo >> 21) & 1u, 0u) << "RDO B21 reserved";
    }
    if (is_epr) { EXPECT_EQ(rdo_any.epr_capable, 1u) << "EPR Mode Capable"; }

    // VIF fields, [PD.8#1], [PD3.1#1]. USB Communications Capable has the same
    // source as the matching flag of the first Sink PDO. No USB Suspend has no
    // independent source in our VIF, so it is not checked.
    if (!vif_pdos.empty()) {
        EXPECT_EQ(rdo_any.usb_comm_capable, SNK_PDO_FIXED{vif_pdos[0]}.usb_comms_capable)
            << "USB Communications Capable";
    }
    // Unchunked extended messages are not supported by this stack
    EXPECT_EQ(rdo_any.unchunked_ext_msg_supported, 0u) << "Unchunked Extended Messages Supported";

    switch (pdo_variant(pdo)) {
        case PDO_VARIANT::FIXED: {
            const RDO_FIXED req{rdo};
            const PDO_FIXED offer{pdo};
            EXPECT_LE(req.operating_current, offer.max_current) << "Operating Current";
            EXPECT_LE(req.operating_current, req.max_current) << "Operating vs Maximum Current";
            if (req.max_current > offer.max_current) {
                EXPECT_EQ(req.capability_mismatch, 1u) << "Capability Mismatch";
            }
            break;
        }
        case PDO_VARIANT::APDO_PPS: {
            const RDO_PPS req{rdo};
            const PDO_SPR_PPS offer{pdo};
            EXPECT_LE(req.operating_current, offer.max_current) << "PPS Operating Current";
            // PPS output voltage is in 20mV steps, PDO limits are in 100mV steps
            EXPECT_GE(req.output_voltage * 20u, offer.min_voltage * 100u) << "PPS Output Voltage";
            EXPECT_LE(req.output_voltage * 20u, offer.max_voltage * 100u) << "PPS Output Voltage";
            EXPECT_EQ((rdo >> 7) & 0x3u, 0u) << "PPS RDO B8..7 reserved";
            break;
        }
        case PDO_VARIANT::APDO_SPR_AVS:
        case PDO_VARIANT::APDO_EPR_AVS: {
            EXPECT_EQ((rdo >> 7) & 0x3u, 0u) << "AVS RDO B8..7 reserved";
            break;
        }
        default:
            ADD_FAILURE() << "Requested PDO of unknown type";
            break;
    }

    // [PD3.1#1] The second object is a copy of the requested PDO
    if (is_epr) { EXPECT_EQ(frame.chunk.read32(4), pdo) << "EPR_Request PDO copy"; }
}

// COMMON.CHECK.PD.12, COMMON.CHECK.PD3.4. For EPR the message must be assembled.
void check_sink_caps(const I_PD_MSG& msg, const PDO_LIST& vif_pdos, PD_REVISION::Type revision) {
    const bool is_epr = msg.header.extended != 0;
    const size_t count = is_epr
        ? msg.data_size() / 4
        : msg.header.data_obj_count;

    // Expected list: everything for EPR, SPR part otherwise. PD2 has no APDOs,
    // [PD.12#1] excludes PPS from the count.
    std::vector<uint32_t> expected;
    for (size_t i = 0; i < vif_pdos.size(); i++) {
        if (!is_epr && i >= size_t(MaxPdoObjects_SPR)) { break; }
        if (revision == PD_REVISION::REV20 && pdo_variant(vif_pdos[i]) != PDO_VARIANT::FIXED) {
            continue;
        }
        expected.push_back(vif_pdos[i]);
    }

    ASSERT_GE(count, 1u) << "Number of Data Objects"; // [PD.12#1]
    EXPECT_EQ(count, expected.size()) << "Sink PDO count";

    // [PD3.4#3] Data Size covers whole PDOs only
    if (is_epr) {
        EXPECT_EQ(msg.data_size(), expected.size() * 4)
            << "EPR_Sink_Capabilities data size";
    }

    for (size_t i = 0; i < count && i < expected.size(); i++) {
        EXPECT_EQ(msg.read32(i * 4), expected[i]) << "Sink PDO " << (i + 1);
    }

    // First PDO, [PD.12#1], [PD3.4#4]
    const SNK_PDO_FIXED pdo1{msg.read32(0)};
    EXPECT_EQ(pdo1.pdo_type, PDO_TYPE::FIXED);
    EXPECT_EQ(pdo1.voltage, 5000u / 50u) << "First Sink PDO must be vSafe5V";
    EXPECT_EQ((msg.read32(0) >> 20) & 0x7u, 0u) << "First Sink PDO B22..20 reserved";

    // Order and duplicates, [PD.12#1] items e, f, i. Fixed PDOs go first, in
    // increasing voltage, then PPS in increasing maximum voltage. EPR positions
    // (8 and up) repeat the same rules.
    uint32_t prev_fixed_mv = 0;
    uint32_t prev_pps_max_mv = 0;
    bool apdo_seen = false;
    int epr_avs_count = 0;

    for (size_t i = 0; i < count; i++) {
        const auto pdo = msg.read32(i * 4);
        // Unused SPR positions are zero filled in EPR messages
        if (pdo == 0) { continue; }

        SCOPED_TRACE("Sink PDO " + std::to_string(i + 1));

        // The EPR block starts a new sequence
        if (i == size_t(MaxPdoObjects_SPR)) {
            prev_fixed_mv = 0;
            prev_pps_max_mv = 0;
            apdo_seen = false;
        }

        switch (pdo_variant(pdo)) {
            case PDO_VARIANT::FIXED: {
                const uint32_t mv = SNK_PDO_FIXED{pdo}.voltage * 50u;
                EXPECT_FALSE(apdo_seen) << "Fixed PDO after APDO";
                if (i > 0) { EXPECT_GT(mv, prev_fixed_mv) << "Fixed PDO voltage order"; }
                prev_fixed_mv = mv;
                break;
            }
            case PDO_VARIANT::APDO_PPS: {
                const uint32_t max_mv = SNK_PDO_SPR_PPS{pdo}.max_voltage * 100u;
                EXPECT_GT(max_mv, prev_pps_max_mv) << "PPS APDO voltage order";
                prev_pps_max_mv = max_mv;
                apdo_seen = true;
                break;
            }
            case PDO_VARIANT::APDO_EPR_AVS:
                // [rev3.2 v1.2] 6.5.18: one EPR AVS APDO, after the EPR Fixed PDOs
                EXPECT_GE(i, size_t(MaxPdoObjects_SPR)) << "EPR AVS APDO in the SPR block";
                EXPECT_EQ(++epr_avs_count, 1) << "More than one EPR AVS APDO";
                apdo_seen = true;
                break;

            case PDO_VARIANT::APDO_SPR_AVS:
                apdo_seen = true;
                break;
            default:
                ADD_FAILURE() << "Sink PDO of unknown type";
                break;
        }
    }
}

// COMMON.CHECK.PD3.2
void check_epr_mode(const Frame& frame, uint8_t operational_pdp) {
    EXPECT_EQ(frame.chunk.header.data_obj_count, 1) << "EPR_Mode data objects";

    const auto raw = frame.chunk.read32(0);
    const EPRMDO mdo{raw};

    EXPECT_GE(mdo.action, EPR_MODE_ACTION::ENTER) << "EPR Mode Action";
    EXPECT_LE(mdo.action, EPR_MODE_ACTION::EXIT) << "EPR Mode Action";
    EXPECT_EQ(raw & 0xFFFFu, 0u) << "EPRMDO B15..0 reserved";

    switch (mdo.action) {
        case EPR_MODE_ACTION::ENTER:
            EXPECT_EQ(mdo.data, operational_pdp) << "EPR Mode Data (operational PDP)";
            break;
        case EPR_MODE_ACTION::ENTER_ACKNOWLEDGED:
        case EPR_MODE_ACTION::ENTER_SUCCEEDED:
        case EPR_MODE_ACTION::EXIT:
            EXPECT_EQ(mdo.data, 0u) << "EPR Mode Data";
            break;
        case EPR_MODE_ACTION::ENTER_FAILED:
            EXPECT_LT(mdo.data, 0x06u) << "EPR Mode Data";
            break;
    }
}

} // namespace

void check_chunk_request(const Frame& frame, PD_EXT_MSGT::Type expected_type,
    uint8_t chunk_number)
{
    ASSERT_TRUE(frame.is_ext(expected_type));
    ASSERT_GE(frame.chunk.data_size(), 2u) << "Extended Header";

    const PD_EXT_HEADER header{frame.chunk.read16(0)};
    EXPECT_EQ(frame.chunk.header.data_obj_count, 1);
    EXPECT_EQ(header.chunked, 1) << "Chunked bit";
    EXPECT_EQ(header.request_chunk, 1) << "Request Chunk bit";
    EXPECT_EQ(header.chunk_number, chunk_number) << "Chunk Number";
    EXPECT_EQ(header.data_size, 0u) << "Data Size";
}

void check_ext_ctrl_msg(const Frame& frame, PD_EXT_CTRL_MSGT::type expected_type) {
    ASSERT_TRUE(frame.is_ext(PD_EXT_MSGT::Extended_Control));
    ASSERT_GE(frame.chunk.data_size(), 4u) << "Extended Header and ECDB";
    ASSERT_EQ(ECDB{frame.chunk.read16(2)}.type, expected_type) << "ECDB Type";

    // [rev3.2 v1.2] Table 6.62: ECDB is 2 bytes, Data is zero for all types
    const PD_EXT_HEADER header{frame.chunk.read16(0)};
    EXPECT_EQ(header.chunked, 1) << "Chunked bit";
    EXPECT_EQ(header.request_chunk, 0) << "Request Chunk bit";
    EXPECT_EQ(header.chunk_number, 0) << "Chunk Number";
    EXPECT_EQ(header.data_size, 2u) << "ECDB size";
    EXPECT_EQ(ECDB{frame.chunk.read16(2)}.data, 0) << "ECDB Data";
}

void Tester::run_common_checks(const Frame& frame) {
    if (frame.hard_reset) {
        response_due_us.reset();
        // Next message starts a new MessageID sequence
        last_uut_msg_id.reset();
        last_uut_msg.reset();
        return;
    }

    SCOPED_TRACE("COMMON.CHECK on message from UUT at " + std::to_string(frame.time) + "us");

    if (response_due_us) { // [PD.4#1]
        EXPECT_LE(frame.time, *response_due_us) << "Response to " << pending_request;
        response_due_us.reset();
    }

    check_header(frame, variant.revision);
    check_ext_header(frame);

    // [PD.2#1] MessageID: zero when the sequence restarts, the same value on
    // retry, incremented otherwise.
    const bool is_retry = last_uut_msg && !last_uut_msg_acked &&
        frame.same_msg(*last_uut_msg);

    if (frame.is_ctrl(PD_CTRL_MSGT::Soft_Reset)) { last_uut_msg_id.reset(); }

    if (!last_uut_msg_id) {
        EXPECT_EQ(frame.chunk.header.message_id, 0) << "MessageID after reset";
    } else if (!is_retry) {
        EXPECT_EQ(frame.chunk.header.message_id, (*last_uut_msg_id + 1) % 8) << "MessageID";
    }

    last_uut_msg_id = frame.chunk.header.message_id;
    last_uut_msg = frame.chunk;
    last_uut_msg_acked = goodcrc_reply;

    if (frame.is_data(PD_DATA_MSGT::Request) || frame.is_data(PD_DATA_MSGT::EPR_Request)) {
        check_request(frame, last_src_caps, dpm.get_sink_pdo_list(), variant.revision);
    }

    if (frame.is_data(PD_DATA_MSGT::Sink_Capabilities)) {
        check_sink_caps(frame.chunk, dpm.get_sink_pdo_list(), variant.revision);
    }

    if (frame.is_data(PD_DATA_MSGT::EPR_Mode)) {
        check_epr_mode(frame, dpm.get_sink_cap_extended().epr_sink_operational_pdp);
    }
}

// Content checks for chunked messages, run when all chunks are collected
void Tester::run_ext_msg_checks(const ExtendedMessage& message) {
    SCOPED_TRACE("COMMON.CHECK on extended message from UUT at " +
        std::to_string(message.time) + "us");

    if (message.msg.is_ext_msg(PD_EXT_MSGT::EPR_Sink_Capabilities)) {
        check_sink_caps(message.msg, dpm.get_sink_pdo_list(), variant.revision);
    }
}

//
// GoogleTest integration
//

int test_failure_count() {
    const auto* result = ::testing::UnitTest::GetInstance()->current_test_info()->result();
    int count = 0;
    for (int i = 0; i < result->total_part_count(); i++) {
        if (result->GetTestPartResult(i).failed()) { count++; }
    }
    return count;
}

namespace {

constexpr TCPC_FEATURES feature_sets[] = {
    {
        .cc_toggling = true,
        .rx_goodcrc_send = true,
        .tx_goodcrc_wait = true,
        .tx_retries = true,
    },
    {
        .cc_toggling = false,
        .rx_goodcrc_send = true,
        .tx_goodcrc_wait = true,
        .tx_retries = false,
    },
    //{
    //    .cc_toggling = false,
    //    .rx_goodcrc_send = false,
    //    .tx_goodcrc_wait = false,
    //    .tx_retries = false,
    //},
};

class CtsTest : public ::testing::Test {
public:
    CtsTest(BodyFunc body, const Variant& variant) : body{body}, variant{variant} {}

    void TestBody() override {
        Tester t{variant};
        body(t);
    }

private:
    BodyFunc body;
    Variant variant;
};

} // namespace

bool register_test(const char* suite_name, const char* test_name, Revisions revisions,
    BodyFunc body, const char* file, int line)
{
    const std::pair<PD_REVISION::Type, const char*> all_revisions[] = {
        {PD_REVISION::REV20, "Rev2"},
        {PD_REVISION::REV30, "Rev3"},
    };

    for (const auto& [revision, revision_name] : all_revisions) {
        if (revisions == REV3 && revision == PD_REVISION::REV20) { continue; }

        for (const auto& features : feature_sets) {
            const Variant variant{features, revision};
            std::string name = std::string(test_name) + "_{" + revision_name;
            if (features.cc_toggling) { name += ",toggling"; }
            if (features.rx_goodcrc_send) { name += ",rx_crc"; }
            if (features.tx_goodcrc_wait) { name += ",tx_crc"; }
            if (features.tx_retries) { name += ",retries"; }
            name += "}";

            ::testing::RegisterTest(suite_name, name.c_str(), nullptr, nullptr, file, line,
                [body, variant]() -> ::testing::Test* {
                    return new CtsTest(body, variant);
                });
        }
    }
    return true;
}

} // namespace cts
