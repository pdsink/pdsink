#include "harness/tester.h"

using namespace cts;
using namespace pd;

namespace {

// Data block 0..259 (mod 256), reserved Extended Message type.
// Size is fixed by CTS and must not depend on the stack limits.
std::vector<uint8_t> make_data_block() {
    std::vector<uint8_t> data(260);
    for (size_t i = 0; i < data.size(); i++) { data[i] = static_cast<uint8_t>(i); }
    return data;
}

constexpr auto reserved_ext_type = static_cast<PD_EXT_MSGT::Type>(0b11111);

} // namespace

CTS_TEST("TEST.PD.PROT.ALL3", "5_Chunked_Extended_Message_Response", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    const auto data = make_data_block();
    ASSERT_TRUE(t.send_ext_msg_chunk(reserved_ext_type, data, 0));

    // Step 3: Chunking_Implemented_SOP = YES, all 10 chunks requested
    for (uint8_t n = 1; n < 10; n++) {
        // [#2], [#3]
        ASSERT_TRUE(t.expect_chunk_request(reserved_ext_type, n, tChunkReceiverRequest));
        ASSERT_TRUE(t.send_ext_msg_chunk(reserved_ext_type, data, n));
    }

    // Step 4
    EXPECT_TRUE(t.expect_ctrl_msg(PD_CTRL_MSGT::Not_Supported, tReceiverResponse)); // [#4]
}

CTS_TEST("TEST.PD.PROT.ALL3", "6_ChunkSenderResponseTimer_Timeout", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    const auto data = make_data_block();
    ASSERT_TRUE(t.send_ext_msg_chunk(reserved_ext_type, data, 0));
    auto sent_at_us = t.now();

    // Step 3: send chunks 1-3, completing the first four chunks.
    for (uint8_t n = 1; n < 4; n++) {
        auto request = t.receive(tChunkingNotSupported.max);
        ASSERT_TRUE(request); // [#2]

        // Step 3c: the test stops here if chunking is not supported
        if (request->is_ctrl(PD_CTRL_MSGT::Not_Supported)) {
            EXPECT_GE(request->time - sent_at_us, tChunkingNotSupported.min); // [#2]
            return;
        }

        ASSERT_TRUE(request->is_ext(reserved_ext_type)); // [#2]
        EXPECT_LE(request->time - sent_at_us, tChunkReceiverRequest.max); // [#2]
        ASSERT_NO_FATAL_FAILURE(check_chunk_request(*request, reserved_ext_type, n)); // [#3]
        ASSERT_TRUE(t.send_ext_msg_chunk(reserved_ext_type, data, n));
        sent_at_us = t.now();
    }

    // Step 3c-d: check the request for chunk 4 and deliberately leave it unanswered.
    auto request = t.receive(tChunkingNotSupported.max);
    ASSERT_TRUE(request); // [#2]

    if (request->is_ctrl(PD_CTRL_MSGT::Not_Supported)) {
        EXPECT_GE(request->time - sent_at_us, tChunkingNotSupported.min); // [#2]
        return;
    }

    ASSERT_TRUE(request->is_ext(reserved_ext_type)); // [#2]
    EXPECT_LE(request->time - sent_at_us, tChunkReceiverRequest.max); // [#2]
    ASSERT_NO_FATAL_FAILURE(check_chunk_request(*request, reserved_ext_type, 4)); // [#3]

    // Step 4
    t.wait(tChunkSenderResponse.max + tCtsChunkMargin.max);
    ASSERT_TRUE(t.send_ext_msg_chunk(reserved_ext_type, data, 0));
    const auto resent_at_us = t.now();

    // Step 5
    auto response = t.receive(tChunkingNotSupported.max);
    ASSERT_TRUE(response); // [#5]

    if (response->is_ctrl(PD_CTRL_MSGT::Not_Supported)) {
        EXPECT_GE(response->time - resent_at_us, tChunkingNotSupported.min); // [#5]
        return;
    }

    ASSERT_TRUE(response->is_ext(reserved_ext_type));
    EXPECT_LE(response->time - resent_at_us, tChunkReceiverRequest.max); // [#5]
    ASSERT_NO_FATAL_FAILURE(check_chunk_request(*response, reserved_ext_type, 1)); // [#6]
    EXPECT_TRUE(t.send_ext_msg_chunk(reserved_ext_type, data, 1));
}

CTS_TEST("TEST.PD.PROT.ALL3", "8_Get_Revision_Response", REV3) {
    // Step 1
    ASSERT_TRUE(t.bring_up_sink()); // COMMON.PROC.BU.2

    // Step 2
    ASSERT_TRUE(t.send_ctrl_msg(PD_CTRL_MSGT::Get_Revision));

    // Step 3c
    auto revision = t.expect_data_msg(PD_DATA_MSGT::Revision, tReceiverResponse); // [#1]
    ASSERT_TRUE(revision);

    // Step 4: VIF PD_Spec_Revision 3.2, PD_Spec_Version 1.2
    EXPECT_EQ(revision->chunk.header.data_obj_count, 1u); // [#2]
    EXPECT_EQ(revision->chunk.read32(0), 0x32120000u);
}
