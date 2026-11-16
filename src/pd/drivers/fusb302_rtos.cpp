#include "../pd_conf.h"

#if defined(USE_FUSB302_RTOS)

#include <etl/vector.h>

#include "fusb302_rtos.h"
#include "../messages.h"
#include "../pd_log.h"
#include "../port.h"

namespace pd {

namespace fusb302 {

#define DRV_LOG_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
        } \
    } while (0)

#define DRV_RET_FALSE_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
            return false; \
        } \
    } while (0)

#define DRV_RET_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
            return; \
        } \
    } while (0)

// FIFO TX tokens
namespace TX_TKN {
    static constexpr uint8_t TXON = 0xA1;
    static constexpr uint8_t SOP1 = 0x12;
    static constexpr uint8_t SOP2 = 0x13;
    static constexpr uint8_t SOP3 = 0x1B;
    static constexpr uint8_t RESET1 = 0x15;
    static constexpr uint8_t RESET2 = 0x16;
    static constexpr uint8_t PACKSYM = 0x80;
    static constexpr uint8_t JAM_CRC = 0xFF;
    static constexpr uint8_t EOP = 0x14;
    static constexpr uint8_t TX_OFF = 0xFE;
}

bool Fusb302Rtos::fusb_setup() {
    if (flags.test(DRV_FLAG::FUSB_SETUP_FAILED)) { return false; }

    DRV_LOGI("FUSB302 setup starting...");
    flags.set(DRV_FLAG::FUSB_SETUP_FAILED);

    // Reset chip

    DRV_LOGI("SW (full) reset");
    Reset rst{0};
    rst.SW_RES = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Reset::reg, rst.raw_value));
    bist_mode.store(TCPC_BIST_MODE::Off);

    // Read ID to check connection
    DeviceID id;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, DeviceID::reg, id.raw_value));
    DRV_LOGI("FUSB302 ID: PROD={}, VER={}, REV={}",
        id.PRODUCT_ID, id.VERSION_ID, id.REVISION_ID);

    // Power up all blocks
    DRV_LOGI("Power up all blocks");
    Power pwr{0};
    pwr.PWR = 0xF;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Power::reg, pwr.raw_value));

    // By default, disable all interrupts except VBUSOK.
    DRV_LOGI("Disable all interrupts except VBUSOK");
    Mask1 mask{0xFF};
    mask.M_VBUSOK = 0;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Mask1::reg, mask.raw_value));
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Maska::reg, 0xFF));
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Maskb::reg, 0xFF));
    // ...and remove the global interrupt mask
    Control0 ctl0;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control0::reg, ctl0.raw_value));
    ctl0.INT_MASK = 0;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control0::reg, ctl0.raw_value));

    // Sync VBUSOK
    auto delay = pdMS_TO_TICKS(2);
    vTaskDelay(delay ? delay : 1); // instead of 250 us
    Status0 status0;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Status0::reg, status0.raw_value));
    vbus_ok.store(static_cast<bool>(status0.VBUSOK));
    DRV_LOGI("Read initial VBUSOK: {}", vbus_ok.load());

    DRV_RET_FALSE_ON_ERROR(fusb_set_polarity(TCPC_POLARITY::NONE));
    polarity.store(TCPC_POLARITY::NONE);
    flags.clear(DRV_FLAG::FUSB_SETUP_FAILED);
    flags.set(DRV_FLAG::FUSB_SETUP_DONE);

    DRV_RET_FALSE_ON_ERROR(fusb_set_rxtx_interrupts(true));

    // NOTE: We don't touch data/power role bits.
    // - defaults are OK for sink/UFP
    // - the driver API has no appropriate methods.
    DRV_LOGI("Setup done.");
    return true;
}

bool Fusb302Rtos::fusb_set_rxtx_interrupts(bool enable) {
    DRV_LOGI("Set RX/TX interrupts {}", enable ? "ON" : "OFF");
    //
    // NOTE: Use I_BC_LVL interrupts sparingly because there are many false
    // positives on BMC exchange. In most scenarios, better alternatives exist.
    //
    Mask1 mask;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Mask1::reg, mask.raw_value));
    mask.M_COLLISION = enable ? 0 : 1;
    mask.M_CRC_CHK = enable ? 0 : 1;
    mask.M_ACTIVITY = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Mask1::reg, mask.raw_value));

    Maska maska;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Maska::reg, maska.raw_value));
    maska.M_HARDRST = enable ? 0 : 1;
    maska.M_TXSENT = enable ? 0 : 1;
    maska.M_HARDSENT = enable ? 0 : 1;
    maska.M_RETRYFAIL = enable ? 0 : 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Maska::reg, maska.raw_value));

    return true;
}

bool Fusb302Rtos::fusb_set_activity_interrupt(bool enable) {
    Mask1 mask;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Mask1::reg, mask.raw_value));
    mask.M_ACTIVITY = enable ? 0 : 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Mask1::reg, mask.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_set_auto_goodcrc(bool enable) {
    DRV_LOGI("Set auto good crc {}", enable ? "ON" : "OFF");
    Switches1 sw1;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Switches1::reg, sw1.raw_value));
    sw1.AUTO_CRC = enable ? 1 : 0;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Switches1::reg, sw1.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_set_tx_auto_retries(uint8_t count) {
    DRV_LOGI("Set TX auto retries  to {}", count);
    Control3 ctl3;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control3::reg, ctl3.raw_value));
    ctl3.N_RETRIES = count & 3; // 0-3 retries
    // Required for TX completion interrupts, even with zero retries.
    ctl3.AUTO_RETRY = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control3::reg, ctl3.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_flush_rx_fifo() {
    Control1 ctrl1{0};
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control1::reg, ctrl1.raw_value));
    ctrl1.RX_FLUSH = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control1::reg, ctrl1.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_flush_tx_fifo() {
    Control0 ctrl0{0};
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control0::reg, ctrl0.raw_value));
    ctrl0.TX_FLUSH = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control0::reg, ctrl0.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_pd_reset() {
    DRV_LOGI("PD reset");
    Reset rst{0};
    rst.PD_RESET = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Reset::reg, rst.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_set_polarity(TCPC_POLARITY polarity) {
    DRV_LOGI("Set polarity to {}",
        polarity == TCPC_POLARITY::CC1 ? "CC1" :
        (polarity == TCPC_POLARITY::CC2 ? "CC2" : "NONE"));

    //
    // Any explicit selection stops autonomous detection.
    //
    Control2 ctl2;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control2::reg, ctl2.raw_value));
    ctl2.TOGGLE = 0;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control2::reg, ctl2.raw_value));

    Maska maska;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Maska::reg, maska.raw_value));
    maska.M_TOGDONE = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Maska::reg, maska.raw_value));

    //
    // Attach comparator
    //
    Switches0 sw0;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Switches0::reg, sw0.raw_value));
    sw0.MEAS_CC1 = 0;
    sw0.MEAS_CC2 = 0;

    if (polarity == TCPC_POLARITY::CC1) { sw0.MEAS_CC1 = 1; }
    if (polarity == TCPC_POLARITY::CC2) { sw0.MEAS_CC2 = 1; }
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Switches0::reg, sw0.raw_value));

    //
    // Attach BMC
    //
    Switches1 sw1;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Switches1::reg, sw1.raw_value));
    sw1.TXCC1 = 0;
    sw1.TXCC2 = 0;

    if (polarity == TCPC_POLARITY::CC1) { sw1.TXCC1 = 1; }
    if (polarity == TCPC_POLARITY::CC2) { sw1.TXCC2 = 1; }
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Switches1::reg, sw1.raw_value));

    if (polarity == TCPC_POLARITY::NONE) {
        DRV_RET_FALSE_ON_ERROR(fusb_set_rx_enable(false));
    }

    return true;
}

bool Fusb302Rtos::fusb_start_toggling() {
    DRV_LOGI("Start Sink-only CC toggling");

    // Reset CC selection before restarting toggling.
    DRV_RET_FALSE_ON_ERROR(fusb_set_polarity(TCPC_POLARITY::NONE));

    // Clear stale interrupts (read-to-clear) before restarting toggling.
    Interrupt interrupt;
    Interrupta interrupta;
    Interruptb interruptb;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Interrupt::reg, interrupt.raw_value));
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Interrupta::reg, interrupta.raw_value));
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Interruptb::reg, interruptb.raw_value));
    Status0 status0;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Status0::reg, status0.raw_value));
    const bool old_vbus_ok = vbus_ok.load();
    vbus_ok.store(status0.VBUSOK);
    if (old_vbus_ok != static_cast<bool>(status0.VBUSOK)) {
        has_deferred_wakeup = true;
    }

    Control2 ctl2;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control2::reg, ctl2.raw_value));
    ctl2.TOGGLE = 0;
    ctl2.MODE = 0b10; // Sink-only polling
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control2::reg, ctl2.raw_value));

    Maska maska;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Maska::reg, maska.raw_value));
    maska.M_TOGDONE = 0;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Maska::reg, maska.raw_value));

    ctl2.TOGGLE = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control2::reg, ctl2.raw_value));

    cc1_value.store(TCPC_CC_LEVEL::NONE);
    cc2_value.store(TCPC_CC_LEVEL::NONE);
    polarity.store(TCPC_POLARITY::TOGGLING);
    return true;
}

bool Fusb302Rtos::fusb_handle_togdone() {
    Status1a status1a;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Status1a::reg, status1a.raw_value));

    TCPC_POLARITY detected_polarity;
    if (status1a.TOGSS == 0b101) {
        detected_polarity = TCPC_POLARITY::CC1;
    } else if (status1a.TOGSS == 0b110) {
        detected_polarity = TCPC_POLARITY::CC2;
    } else {
        DRV_LOGE("Unsupported TOGDONE result: {}", status1a.TOGSS);
        return fusb_start_toggling();
    }

    DRV_RET_FALSE_ON_ERROR(fusb_set_polarity(detected_polarity));

    // Exact CC levels are fetched later, so use constants here to avoid
    // complicating the code with an extra measurement.
    cc1_value.store(detected_polarity == TCPC_POLARITY::CC1
        ? TCPC_CC_LEVEL::RP_3_0 : TCPC_CC_LEVEL::NONE);
    cc2_value.store(detected_polarity == TCPC_POLARITY::CC2
        ? TCPC_CC_LEVEL::RP_3_0 : TCPC_CC_LEVEL::NONE);
    polarity.store(detected_polarity);
    has_deferred_wakeup = true;
    return true;
}

bool Fusb302Rtos::fusb_set_rx_enable(bool enable) {
    DRV_LOGI("Set RX enable {}", enable ? "ON" : "OFF");

    if (!enable) {
        DRV_RET_FALSE_ON_ERROR(fusb_set_auto_goodcrc(false));
    }

    DRV_RET_FALSE_ON_ERROR(tx.reset());

    // Preserve pending RX on repeated enable.
    if (!enable || !rx_enabled) {
        reset_rx_state();
        DRV_RET_FALSE_ON_ERROR(fusb_flush_rx_fifo());
    }

    if (enable) {
        DRV_RET_FALSE_ON_ERROR(fusb_set_auto_goodcrc(true));
    }

    rx_enabled = enable;
    return true;
}

bool Fusb302Rtos::fusb_tx_pkt_load(PD_CHUNK& chunk) {
    DRV_RET_FALSE_ON_ERROR(fusb_flush_tx_fifo());

    DRV_LOGI("TX prepare");

    // NOTE: The spec says retries should NOT be used for unchunked extended
    // messages and cable plug messages. Since we do not support those, just
    // use the negotiated retry count, or zero when PRL handles retries.
    //
    // Software retries are for testing only. USB PD requires the retry
    // preamble to start within 195 us after CRCReceiveTimer expires.
    // Software retries over I2C cannot reliably meet this deadline.
    DRV_RET_FALSE_ON_ERROR(fusb_set_tx_auto_retries(
        tcpc_features.tx_retries ? port.max_retries() : 0));

    etl::vector<uint8_t, 40> fifo_buf{};

    // Prepared packet: SOP[4] + PACKSYM[1] + header[2] + data[28]
    //                  + JAM_CRC[1] + EOP[1] + TX_OFF[1].
    // TXON is written separately.
    static_assert(decltype(fifo_buf)::MAX_SIZE >=
        4 + 1 + 2 + PD_CHUNK::MAX_SIZE + 1 + 1 + 1,
        "TX buffer too small to fit all possible data");

    // Ensure only "legacy" packets are allowed. We do NOT support unchunked
    // extended packets or long vendor packets:
    // - those are not useful in sink mode.
    // - the chip's FIFO size limits the maximum packet size.
    static_assert(PD_CHUNK::MAX_SIZE <= 28,
        "Packet size should not exceed 28 bytes in this implementation");

    // Hardcode the message SOP, since the library supports only sink mode
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP2);

    uint8_t pack_sym = TX_TKN::PACKSYM;

    // Add data size (+ 2 for header). No need to mask: the value is restricted
    // by the static_assert above.

    pack_sym |= (chunk.data_size() + 2);
    fifo_buf.push_back(pack_sym);

    // Msg header
    fifo_buf.push_back(chunk.header.raw_value & 0xFF);
    fifo_buf.push_back((chunk.header.raw_value >> 8) & 0xFF);

    // Msg data
    fifo_buf.insert(fifo_buf.end(),
        chunk.get_data().begin(),
        chunk.get_data().end()
    );

    // Tail
    fifo_buf.push_back(TX_TKN::JAM_CRC);
    fifo_buf.push_back(TX_TKN::EOP);
    fifo_buf.push_back(TX_TKN::TX_OFF);

    // TXON will be written only after the final silence check
    // (interrupts should not happen when the line is silent).

    DRV_RET_FALSE_ON_ERROR(hal.write_block(i2c_addr, FIFOs::reg, fifo_buf.data(), fifo_buf.size()));
    return true;
}

// Read one packet into rx_buffer. The caller ensures the FIFO is not empty.
bool Fusb302Rtos::fusb_rx_fifo_fetch_pkt() {
    ETL_MAYBE_UNUSED uint8_t sop;
    uint8_t hdr[2];
    ETL_MAYBE_UNUSED uint8_t crc_junk[4];

    if (!hal.read_reg(i2c_addr, FIFOs::reg, sop) ||
        !hal.read_block(i2c_addr, FIFOs::reg, hdr, 2)) {
        DRV_LOGE("RX FIFO read failed (SOP/header)");
        return false;
    }
    rx_buffer.header.raw_value = (hdr[1] << 8) | hdr[0];

    // Chunked extended messages have non-zero data_obj_count.
    if (rx_buffer.header.extended && !rx_buffer.header.data_obj_count) {
        // Unchunked extended packets are unsupported; restore FIFO framing.
        DRV_LOGE("Unchunked extended packet received, ignoring");
        DRV_LOG_ON_ERROR(fusb_flush_rx_fifo());
        return false;
    }

    // Remaining packets have data_obj_count*4 bytes. Its 3-bit range limits
    // the block read to 28 bytes, which fits directly in the destination.
    rx_buffer.resize_by_data_obj_count();
    if (!hal.read_block(i2c_addr, FIFOs::reg, rx_buffer.get_data().data(), rx_buffer.data_size()) ||
        !hal.read_block(i2c_addr, FIFOs::reg, crc_junk, 4)) {
        DRV_LOGE("RX FIFO read failed (data/CRC)");
        return false;
    }
    return true;
}

bool Fusb302Rtos::fusb_hr_send() {
    DRV_LOGI("Send hard reset");

    Control3 ctrl3;
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control3::reg, ctrl3.raw_value));
    ctrl3.SEND_HARD_RESET = 1;
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control3::reg, ctrl3.raw_value));

    return true;
}

bool Fusb302Rtos::fusb_set_bist(TCPC_BIST_MODE mode) {
    DRV_LOGI("Set BIST mode to {}",
        mode == TCPC_BIST_MODE::Off ? "Off" :
        (mode == TCPC_BIST_MODE::Carrier ? "Carrier" :
        (mode == TCPC_BIST_MODE::TestData ? "TestData" : "Unknown")));

    Control1 ctrl1;
    Control3 ctrl3;

    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control1::reg, ctrl1.raw_value));
    DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control3::reg, ctrl3.raw_value));
    ctrl1.BIST_MODE2 = 0;
    ctrl3.BIST_TMODE = 0;

    switch (mode) {
        case TCPC_BIST_MODE::Carrier:
            ctrl1.BIST_MODE2 = 1;
            break;
        case TCPC_BIST_MODE::TestData:
            ctrl3.BIST_TMODE = 1;
            break;
        default:
            break;
    }

    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control1::reg, ctrl1.raw_value));
    DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control3::reg, ctrl3.raw_value));

    if (mode == TCPC_BIST_MODE::Carrier) {
        Control0 ctrl0;
        DRV_RET_FALSE_ON_ERROR(hal.read_reg(i2c_addr, Control0::reg, ctrl0.raw_value));
        ctrl0.TX_START = 1;
        DRV_RET_FALSE_ON_ERROR(hal.write_reg(i2c_addr, Control0::reg, ctrl0.raw_value));
    }

    bist_mode.store(mode);
    return true;
}

bool Fusb302Rtos::hr_cleanup() {
    // Clean up internal states after a hard reset is received or sent.
    DRV_LOG_ON_ERROR(tx.reset());
    reset_rx_state();
    DRV_RET_FALSE_ON_ERROR(fusb_pd_reset());
    return true;
}

void Fusb302Rtos::reset_rx_state() {
    rx_buffer_has_pkt.store(false);
    rx_fifo_may_have_pkt.store(false);
    rx_msg_id_stored = -1;
    rx_fetch_pending.store(false);
}

bool Fusb302Rtos::process_rx_fifo() {
    const bool had_pkt = rx_buffer_has_pkt.load();
    bool fifo_read_ok = true;

    for (;;) {
        if (!fifo_read_ok) { break; }
        if (rx_buffer_has_pkt.load()) { break; }
        if (!rx_fifo_may_have_pkt.exchange(false)) { break; }

        Status1 status1{};
        fifo_read_ok = hal.read_reg(i2c_addr, Status1::reg, status1.raw_value);
        DRV_LOG_ON_ERROR(fifo_read_ok);
        if (!fifo_read_ok) {
            rx_fifo_may_have_pkt.store(true);
            continue;
        }
        if (status1.RX_EMPTY) { continue; }

        fifo_read_ok = fusb_rx_fifo_fetch_pkt();
        if (!fifo_read_ok) {
            rx_fifo_may_have_pkt.store(true);
            continue;
        }

        fifo_read_ok = hal.read_reg(i2c_addr, Status1::reg, status1.raw_value);
        DRV_LOG_ON_ERROR(fifo_read_ok);
        if (!fifo_read_ok || !status1.RX_EMPTY) { rx_fifo_may_have_pkt.store(true); }

        // GoodCRC messages are ignored and not logged.
        if (rx_buffer.is_ctrl_msg(PD_CTRL_MSGT::GoodCRC)) { continue; }

        DRV_LOGI("Message received: type = {}, extended = {}, data size = {}",
            rx_buffer.header.message_type, rx_buffer.header.extended, rx_buffer.data_size());

        // With TX retries, the driver filters duplicates and Pings inline.
        // This is required for eventual consistency.
        bool ignored = false;
        if (tcpc_features.tx_retries) {
            ignored = (rx_msg_id_stored == rx_buffer.header.message_id &&
                !rx_buffer.is_ctrl_msg(PD_CTRL_MSGT::Soft_Reset)) ||
                rx_buffer.is_ctrl_msg(PD_CTRL_MSGT::Ping);
            rx_msg_id_stored = rx_buffer.header.message_id;
        }

        if (ignored) { continue; }
        rx_buffer_has_pkt.store(true);
        tx.on_rx_pkt_new();
    }

    // PRL must observe RX and any deferred FAILED together.
    // A failed read must also wake PRL so it can retry the pending RX.
    const bool fetch_done = rx_fetch_pending.exchange(false);
    if (fetch_done ||
        (!had_pkt && rx_buffer_has_pkt.load()) ||
        !fifo_read_ok)
    {
        has_deferred_wakeup = true;
    }
    return fifo_read_ok;
}

void Fusb302Rtos::handle_interrupt() {
    if (!hal.is_interrupt_active()) { return; }

    DRV_LOGD("Handle PD interrupt");

    for (;;) {
        Interrupt interrupt;
        Interrupta interrupta;
        Interruptb interruptb;

        // TODO: Consider a 5-byte block read (0x3E-0x42) with single call.
        DRV_LOG_ON_ERROR(hal.read_reg(i2c_addr, Interrupt::reg, interrupt.raw_value));
        DRV_LOG_ON_ERROR(hal.read_reg(i2c_addr, Interrupta::reg, interrupta.raw_value));
        DRV_LOG_ON_ERROR(hal.read_reg(i2c_addr, Interruptb::reg, interruptb.raw_value));

        if (interrupt.I_VBUSOK) {
            Status0 status0;
            DRV_LOG_ON_ERROR(hal.read_reg(i2c_addr, Status0::reg, status0.raw_value));
            vbus_ok.store(status0.VBUSOK);
            DRV_LOGI("IRQ: VBUS changed");
            has_deferred_wakeup = true;
        }

        if (interrupta.I_TOGDONE && polarity.load() == TCPC_POLARITY::TOGGLING) {
            DRV_LOGI("IRQ: CC toggling completed");
            DRV_LOG_ON_ERROR(fusb_handle_togdone());
        }

        if (interrupta.I_HARDRST) {
            DRV_LOGI("IRQ: hard reset received");
            DRV_LOG_ON_ERROR(fusb_set_bist(TCPC_BIST_MODE::Off));
            DRV_LOG_ON_ERROR(hr_cleanup());
            port.notify_prl(MsgToPrl_TcpcHardReset{});
            has_deferred_wakeup = true;
        }

        if (interrupta.I_HARDSENT) {
            DRV_LOGI("IRQ: hard reset sent");
            DRV_LOG_ON_ERROR(hr_cleanup());
            timers.stop(HR_SEND_TIMEOUT);
            sync_hr_send.job_finish();
            has_deferred_wakeup = true;
        }

        // A reset invalidates all ordinary TX/RX events in this IRQ snapshot.
        if (!interrupta.I_HARDRST && !interrupta.I_HARDSENT) {
            if (interrupta.I_TXSENT) {
                // Always process TX success before RX to avoid false discards.
                tx.on_tx_sent();
            }

            if (interrupt.I_CRC_CHK) {
                // Processing RX before TX errors speeds up TX FSM cleanup.
                if (rx_enabled) {
                    // Unread FIFO data does not retrigger IRQ; retain this until RX_EMPTY.
                    rx_fifo_may_have_pkt.store(true);
                    DRV_LOG_ON_ERROR(process_rx_fifo());
                } else {
                    DRV_LOG_ON_ERROR(fusb_flush_rx_fifo());
                }
            }

            if (interrupt.I_COLLISION) {
                DRV_LOGI("IRQ: tx collision");
                tx.on_tx_failed();
            }
            if (interrupta.I_RETRYFAIL) {
                DRV_LOGI("IRQ: tx retry failed");
                tx.on_tx_failed();
            }

            if (interrupt.I_ACTIVITY && rx_enabled && tx.is_awaiting_quiet()) {
                Status0 status0;
                if (hal.read_reg(i2c_addr, Status0::reg, status0.raw_value)) {
                    tx.on_activity(status0.ACTIVITY);
                } else {
                    DRV_LOGE("CC activity read failed");
                }
            }
        }

        if (!hal.is_interrupt_active()) { break; }

        DRV_LOGD("Interrupt handled, but still active. Repeat processing...");
    }
}

void Fusb302Rtos::rearm(uint32_t deadline_ticks) {
    // No kick_task() is needed with synchronous PD processing in this task.
    // Splitting PD processing across tasks or calling the PD loop from another
    // context is currently considered a design error and is not supported.
    // Revisit the notification logic if a concrete use case justifies such a design.
    timers.start_with_deadline(DriverTimer::PD_CORE_TIMERS, deadline_ticks);
}

void Fusb302Rtos::handle_tcpc_calls() {

    TCPC_POLARITY _polarity{};
    if (sync_set_polarity.get_job(_polarity)) {
        // Reset TX status and cancel pending CC operations.
        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
        sync_fetch_cc.reset();

        if (_polarity == TCPC_POLARITY::TOGGLING) {
            DRV_LOG_ON_ERROR(fusb_start_toggling());
        } else if (fusb_set_polarity(_polarity)) {
            polarity.store(_polarity);
        }
        sync_set_polarity.job_finish();
        has_deferred_wakeup = true;
    }

    // In this implementation, only the active CC is measured (TC attach
    // debounce, PRL SinkTxOK wait), so no settling delay is needed.
    TCPC_CC_REQ _cc_req{};
    if (sync_fetch_cc.get_job(_cc_req)) {
        Status0 status0{};
        DRV_LOG_ON_ERROR(hal.read_reg(i2c_addr, Status0::reg, status0.raw_value));
        const auto level = static_cast<TCPC_CC_LEVEL::Type>(status0.BC_LVL);
        if (polarity.load() == TCPC_POLARITY::CC1) {
            cc1_value.store(level);
        } else {
            cc2_value.store(level);
        }
        sync_fetch_cc.job_finish();
        has_deferred_wakeup = true;
    }

    bool _rx_enabled{};
    if (sync_rx_enable.get_job(_rx_enabled)) {
        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
        DRV_LOG_ON_ERROR(fusb_set_rx_enable(_rx_enabled));
        sync_rx_enable.job_finish();
        has_deferred_wakeup = true;
    }

    TCPC_BIST_MODE _bist_mode{};
    if (sync_set_bist.get_job(_bist_mode)) {
        DRV_LOG_ON_ERROR(fusb_set_bist(_bist_mode));
        sync_set_bist.job_finish();
        has_deferred_wakeup = true;
    }

    if (sync_hr_send.get_job()) {
        // Cancel pending TX starts and RX fetches before sending Hard Reset.
        port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
        DRV_LOG_ON_ERROR(tx.reset());
        reset_rx_state();
        DRV_LOG_ON_ERROR(fusb_flush_rx_fifo());
        DRV_LOG_ON_ERROR(fusb_flush_tx_fifo());

        if (fusb_hr_send()) {
            timers.start(HR_SEND_TIMEOUT);
        } else {
            DRV_LOGE("Hard Reset sending failed");
            timers.stop(HR_SEND_TIMEOUT);
            sync_hr_send.job_finish();
            has_deferred_wakeup = true;
        }
        return;
    }

    // If the RX buffer is empty, fetch pending FIFO data before TX.
    DRV_LOG_ON_ERROR(process_rx_fifo());

    auto expected = TCPC_TRANSMIT_STATUS::ENQUEUED;
    if (port.tcpc_tx_status.compare_exchange_strong(expected, TCPC_TRANSMIT_STATUS::SENDING)) {
        tx.on_tx_requested();
    }
}

void Fusb302Rtos::task() {
    uint32_t event_mask{0};

    // Allow setup to complete before continuing. Otherwise, an early VBUS_OK
    // interrupt can cause kick_task failures. That's not critical, but this
    // avoids unnecessary errors in the log.
    xTaskNotifyWait(0, UINT32_MAX, &event_mask, portMAX_DELAY);

    if (!flags.test(DRV_FLAG::FUSB_SETUP_DONE)) {
        hal.setup();
        if (fusb_setup()) {
            // Wake after setup and preserve events consumed by the initial wait.
            kick_task(event_mask | MSK_WAKEUP);
        }
    }

    while (true) {
        xTaskNotifyWait(0, UINT32_MAX, &event_mask, portMAX_DELAY);

        if (flags.test(DRV_FLAG::FUSB_SETUP_FAILED)) { continue; }

        for (;;) {
            if (event_mask & MSK_TIMER) {
                timers.cleanup();
                has_deferred_timer = true;
            }

            // Always check interrupt level to avoid deadlock.
            handle_interrupt();

            if (event_mask & MSK_API_CALL) {
                DRV_LOGI("Handle API call");
                handle_tcpc_calls();
            }

            if (tx.is_awaiting_quiet() && timers.is_expired(LINE_QUIET_TIME)) {
                tx.on_timer_expired();
            }

            if (timers.is_expired(HR_SEND_TIMEOUT)) {
                DRV_LOGE("Hard Reset sending timed out");
                timers.stop(HR_SEND_TIMEOUT);
                sync_hr_send.job_finish();
                has_deferred_wakeup = true;
            }

            if (event_mask & MSK_WAKEUP) {
                has_deferred_wakeup = true;
            }

            BaseType_t notified = xTaskNotifyWait(0, UINT32_MAX, &event_mask, 0);
            if (notified == pdFALSE) { break; }

            DRV_LOGI("Fusb302Rtos task: New event detected, repeat processing...");
        }

        if (has_deferred_wakeup) {
            has_deferred_wakeup = false;
            DRV_LOGD("Waking up port");
            port.wakeup();
        }
        if (has_deferred_timer) {
            has_deferred_timer = false;
            port.notify_task(MsgTask_Timer{});
        }

        // Rearm after synchronous engine callbacks have updated the deadlines.
        if (timers.timers_changed.exchange(false)) {
            if (auto next_deadline = timers.get_next_deadline()) {
                hal.rearm(*next_deadline);
            }
        }
    }
}

void Fusb302Rtos::kick_task(uint32_t event_mask, bool from_isr) {
    if (!started) {
        DRV_LOGE("Driver not started, can't notify [event mask: {}]", event_mask);
        return;
    }
    if (!xWaitingTaskHandle) {
        DRV_LOGE("Driver task handle is null, can't notify");
        return;
    }

    if (from_isr) {
        auto woken = pdFALSE;
        xTaskNotifyFromISR(xWaitingTaskHandle, event_mask, eSetBits, &woken);

#if defined(ESP_PLATFORM)  /* ESP-IDF */
        if (woken) { portYIELD_FROM_ISR(); }
#elif defined(portYIELD_FROM_ISR)
        portYIELD_FROM_ISR(woken);
#elif defined(portEND_SWITCHING_ISR)
        portEND_SWITCHING_ISR(woken);
#else
        (void)woken; /* no-op */
#endif

    } else {
        xTaskNotify(xWaitingTaskHandle, event_mask, eSetBits);
    }
}

void Fusb302Rtos::setup() {
    if (started) { return; }

    hal.set_event_handler(
        hal_event_handler_t::create<Fusb302Rtos, &Fusb302Rtos::on_hal_event>(*this)
    );

    auto result = xTaskCreate(
        [](void* params) {
            static_cast<Fusb302Rtos*>(params)->task();
        },
        "Fusb302Rtos",
        task_stack_size_bytes / sizeof(StackType_t),
        this,
        task_priority,
        &xWaitingTaskHandle
    );

    if (result != pdPASS) {
        DRV_LOGE("Failed to create Fusb302Rtos task");
        return;
    }

    started = true;

    // Activate task
    kick_task(0);

}

void Fusb302Rtos::on_hal_event(HAL_EVENT_TYPE event, bool from_isr) {
    switch (event) {
        case HAL_EVENT_TYPE::Timer:
            kick_task(MSK_TIMER, from_isr);
            break;
        case HAL_EVENT_TYPE::FUSB302_Interrupt:
            kick_task(MSK_PD_INTERRUPT, from_isr);
            break;
        default:
            DRV_LOGE("Unknown HAL event");
            break;
    }
}

//
// TCPC API methods.
//

void Fusb302Rtos::req_fetch_cc(TCPC_CC_REQ selector) {
    if (selector != TCPC_CC_REQ::ACTIVE_CC) {
        DRV_LOGE("Unsupported CC selector: {}", static_cast<int>(selector));
        // Complete unsupported requests without changing the CC cache.
        sync_fetch_cc.reset();
        kick_task(MSK_WAKEUP);
        return;
    }

    sync_fetch_cc.enqueue(selector);
    kick_task(MSK_API_CALL);
}

auto Fusb302Rtos::get_cc(TCPC_CC_GET selector) const -> TCPC_CC_LEVEL::Type {
    switch (selector) {
        case TCPC_CC_GET::CC1:
            return cc1_value.load();
        case TCPC_CC_GET::CC2:
            return cc2_value.load();
        case TCPC_CC_GET::ACTIVE_CC: {
            const auto active_cc = polarity.load();
            if (active_cc == TCPC_POLARITY::CC1) {
                return cc1_value.load();
            } else if (active_cc == TCPC_POLARITY::CC2) {
                return cc2_value.load();
            } else {
                DRV_LOGE("Can't read ACTIVE_CC without selected polarity");
            }
            break;
        }
        default:
            DRV_LOGE("Unsupported CC cache selector: {}", static_cast<int>(selector));
            break;
    }

    return TCPC_CC_LEVEL::NONE;
}

void Fusb302Rtos::req_transmit() {
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::UNSET);
    enqueued_tx_chunk = port.tx_chunk;
    port.tcpc_tx_status.store(TCPC_TRANSMIT_STATUS::ENQUEUED);
    kick_task(MSK_API_CALL);
}

bool Fusb302Rtos::check_vbus(TCPC_VBUS_LEVEL level) {
    switch (level) {
        case TCPC_VBUS_LEVEL::PRESENT:
            return vbus_ok.load();
        case TCPC_VBUS_LEVEL::SINK_DISCONNECTED:
        case TCPC_VBUS_LEVEL::SAFE0V:
            // Approximate both thresholds with VBUSOK to keep the code simple;
            // precise detection is not critical for the current sink implementation.
            return !vbus_ok.load();
    }

    DRV_LOGE("Unsupported VBUS level: {}", static_cast<int>(level));
    return false;
}

auto Fusb302Rtos::fetch_rx() -> FETCH_RX_STATUS {
    if (rx_buffer_has_pkt.load()) { return FETCH_RX_STATUS::READY; }
    if (rx_fetch_pending.load()) { return FETCH_RX_STATUS::IN_PROGRESS; }
    // RX IRQs publish new packets; fetch requests drain packets left in the FIFO.
    if (!rx_enabled || !rx_fifo_may_have_pkt.load()) {
        return FETCH_RX_STATUS::EMPTY;
    }
    rx_fetch_pending.store(true);
    kick_task(MSK_API_CALL);
    return FETCH_RX_STATUS::IN_PROGRESS;
}

bool Fusb302Rtos::get_rx() {
    if (!rx_buffer_has_pkt.load()) { return false; }
    port.rx_chunk = rx_buffer;
    rx_buffer_has_pkt.store(false);
    return true;
}

void Fusb302Rtos::TxController::on_tx_requested() {
    if (state != State::TX_IDLE) {
        // Safety guard. This should never happen.
        DRV_LOG_ON_ERROR(reset());
    }

    // TX FIFO is loaded after the quiet pause, when any GoodCRC is complete.
    DRV_LOG_ON_ERROR(go(State::TX_WAIT_LINE_QUIET));

    // RX may already be buffered before this TX request.
    if (driver.rx_enabled && driver.rx_buffer_has_pkt.load()) {
        on_rx_pkt_new();
    }
}

void Fusb302Rtos::TxController::on_tx_sent() {
    if (state != State::TX_ACTIVE) { return; }

    DRV_LOGI("IRQ: tx completed");
    DRV_LOG_ON_ERROR(go(State::TX_IDLE));
    publish_tx_result(TCPC_TRANSMIT_STATUS::SUCCEEDED);
}

void Fusb302Rtos::TxController::on_tx_failed() {
    if (state != State::TX_ACTIVE) { return; }

    DRV_LOG_ON_ERROR(go(State::TX_FAILURE_PENDING));
}

void Fusb302Rtos::TxController::on_rx_pkt_new() {
    // With software retries, PRL retains duplicate/Ping and discard decisions.
    const bool is_tx_pending = state == State::TX_WAIT_LINE_QUIET || state == State::TX_ACTIVE;

    if (state == State::TX_FAILURE_PENDING || (tcpc_features.tx_retries && is_tx_pending)) {
        DRV_LOG_ON_ERROR(go(State::TX_IDLE));
        publish_tx_result(TCPC_TRANSMIT_STATUS::FAILED);
    }
}

void Fusb302Rtos::TxController::on_activity(bool active) {
    if (!is_awaiting_quiet()) { return; }

    if (active) {
        driver.timers.stop(LINE_QUIET_TIME);
    } else {
        driver.timers.start(LINE_QUIET_TIME);
    }
}

void Fusb302Rtos::TxController::on_timer_expired() {
    if (!is_awaiting_quiet()) { return; }
    if (!driver.rx_enabled) { return; }

    if (driver.rx_buffer_has_pkt.load()) { on_rx_pkt_new(); }
    // Unread RX must be classified before TX starts or FAILED is published.
    if (driver.rx_buffer_has_pkt.load() || driver.rx_fifo_may_have_pkt.load()) { return; }

    // Reload on every attempt; the checks below cover activity during the load.
    if (state == State::TX_WAIT_LINE_QUIET &&
        !driver.fusb_tx_pkt_load(driver.enqueued_tx_chunk))
    {
        DRV_LOG_ON_ERROR(go(State::TX_FAILURE_PENDING));
        return;
    }

    if (!is_line_still_quiet()) { return; }

    if (state == State::TX_WAIT_LINE_QUIET) {
        DRV_LOG_ON_ERROR(go(State::TX_ACTIVE));
        if (!driver.hal.write_reg(driver.i2c_addr, FIFOs::reg, TX_TKN::TXON)) {
            DRV_LOG_ON_ERROR(go(State::TX_FAILURE_PENDING));
        }
    } else {
        // Normally, FAILED is reported on a new RX without waiting for the timeout.
        // The timeout is a fallback for cable-traffic collisions, unexpected in normal use.
        DRV_LOG_ON_ERROR(go(State::TX_IDLE));
        publish_tx_result(TCPC_TRANSMIT_STATUS::FAILED);
    }
}

bool Fusb302Rtos::TxController::reset() {
    return go(State::TX_IDLE);
}

bool Fusb302Rtos::TxController::go(State next) {
    // Self-transitions are not used; skip them to avoid restarting state actions.
    if (next == state) {
        return true;
    }

    bool exit_ok = true;
    switch (state) {
        case State::TX_IDLE: exit_ok = on_exit_TX_IDLE(); break;
        case State::TX_WAIT_LINE_QUIET: exit_ok = on_exit_TX_WAIT_LINE_QUIET(); break;
        case State::TX_ACTIVE: exit_ok = on_exit_TX_ACTIVE(); break;
        case State::TX_FAILURE_PENDING: exit_ok = on_exit_TX_FAILURE_PENDING(); break;
    }

    state = next;

    bool enter_ok = true;
    switch (state) {
        case State::TX_IDLE: enter_ok = on_enter_TX_IDLE(); break;
        case State::TX_WAIT_LINE_QUIET: enter_ok = on_enter_TX_WAIT_LINE_QUIET(); break;
        case State::TX_ACTIVE: enter_ok = on_enter_TX_ACTIVE(); break;
        case State::TX_FAILURE_PENDING: enter_ok = on_enter_TX_FAILURE_PENDING(); break;
    }

    return exit_ok && enter_ok;
}

bool Fusb302Rtos::TxController::on_enter_TX_IDLE() { return nop(); }
bool Fusb302Rtos::TxController::on_exit_TX_IDLE() { return nop(); }

bool Fusb302Rtos::TxController::on_enter_TX_WAIT_LINE_QUIET() {
    DRV_RET_FALSE_ON_ERROR(driver.fusb_set_activity_interrupt(true));
    Status0 status0;
    DRV_RET_FALSE_ON_ERROR(driver.hal.read_reg(driver.i2c_addr, Status0::reg, status0.raw_value));

    if (status0.ACTIVITY) {
        driver.timers.stop(LINE_QUIET_TIME);
    } else {
        driver.timers.start(LINE_QUIET_TIME);
    }
    return true;
}

bool Fusb302Rtos::TxController::on_exit_TX_WAIT_LINE_QUIET() {
    driver.timers.stop(LINE_QUIET_TIME);
    // Masked before TXON, so our own TX does not raise ACTIVITY.
    return driver.fusb_set_activity_interrupt(false);
}

// TXON is written by the caller, so a write failure can switch state normally.
bool Fusb302Rtos::TxController::on_enter_TX_ACTIVE() { return nop(); }
bool Fusb302Rtos::TxController::on_exit_TX_ACTIVE() { return driver.fusb_flush_tx_fifo(); }

bool Fusb302Rtos::TxController::on_enter_TX_FAILURE_PENDING() {
    return on_enter_TX_WAIT_LINE_QUIET();
}

bool Fusb302Rtos::TxController::on_exit_TX_FAILURE_PENDING() {
    driver.timers.stop(LINE_QUIET_TIME);
    return driver.fusb_set_activity_interrupt(false);
}

bool Fusb302Rtos::TxController::is_line_still_quiet() {
    Status0 status0;
    DRV_RET_FALSE_ON_ERROR(driver.hal.read_reg(driver.i2c_addr, Status0::reg, status0.raw_value));
    if (status0.ACTIVITY) {
        driver.timers.stop(LINE_QUIET_TIME);
        return false;
    }
    // Activity may have started and ended during the load or status read.
    // On a quiet line, no enabled interrupt is expected to fire.
    if (driver.hal.is_interrupt_active()) {
        driver.kick_task(MSK_PD_INTERRUPT);
        return false;
    }
    return true;
}

void Fusb302Rtos::TxController::publish_tx_result(TCPC_TRANSMIT_STATUS status) {
    // Ensure transmit was not invoked again; otherwise our info is outdated
    // and should be discarded.
    auto expected = TCPC_TRANSMIT_STATUS::SENDING;
    if (driver.port.tcpc_tx_status.compare_exchange_strong(expected, status)) {
        DRV_LOGI("TX end, status: {}", static_cast<int>(status));
        driver.has_deferred_wakeup = true;
    } else {
        DRV_LOGI("TX end failed: TCPC status changed from outside to {}", static_cast<int>(expected));
    }
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS
