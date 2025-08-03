#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS)

#include <etl/vector.h>
#include "fusb302_rtos.h"
#include "messages.h"
#include "pd_log.h"
#include "port.h"

namespace pd {

namespace fusb302 {

#define HAL_LOG_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 HAL IO error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
        } \
    } while (0)

#define HAL_FAIL_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 HAL IO error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
            return false; \
        } \
    } while (0)

#define DRV_LOG_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
        } \
    } while (0)

#define DRV_FAIL_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_LOGE("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
            return false; \
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

    flags.set(DRV_FLAG::FUSB_SETUP_FAILED);

    // Reset chip
    Reset rst{0};
    rst.SW_RES = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Reset::addr, rst.raw_value));

    // Read ID to check connection
    DeviceID id;
    HAL_FAIL_ON_ERROR(hal.read_reg(DeviceID::addr, id.raw_value));
    DRV_LOGI("FUSB302 ID: VER={}, PROD={}, REV={}",
        id.VERSION_ID, id.PRODUCT_ID, id.REVISION_ID);

    // Power up all blocks
    Power pwr{0};
    pwr.PWR = 0xF;
    HAL_FAIL_ON_ERROR(hal.write_reg(Power::addr, pwr.raw_value));

    // By default disable all interrupts except VBUSOK.
    Mask1 mask{0xFF};
    mask.M_VBUSOK = 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Mask1::addr, mask.raw_value));
    HAL_FAIL_ON_ERROR(hal.write_reg(Maska::addr, 0xFF));
    HAL_FAIL_ON_ERROR(hal.write_reg(Maskb::addr, 0xFF));

    // Sync VBUSOK
    vTaskDelay(pdMS_TO_TICKS(2) || 1); // instead of 250uS
    Status0 status0;
    HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
    vbus_ok.store(static_cast<bool>(status0.VBUSOK));

    // TODO: Retries are in PRL now, consider use here.
    DRV_FAIL_ON_ERROR(fusb_set_tx_auto_retries(0));

    DRV_FAIL_ON_ERROR(fusb_set_polarity(TCPC_POLARITY::NONE));
    flags.clear(DRV_FLAG::FUSB_SETUP_FAILED);
    flags.set(DRV_FLAG::FUSB_SETUP_DONE);

    // Note: we don't touch data/power role bits.
    // - defaults are ok for sink/ufp
    // - driver API has no appropriate methods.
    return true;
}

bool Fusb302Rtos::fusb_set_rxtx_interrupts(bool enable) {
    //
    // Note: I_BC_LVL interrupts usage should be restricted, due lot of false
    // positives on BMC exchange. Usually, in all scenarios better alternatives
    // exist.
    //
    Mask1 mask;
    HAL_FAIL_ON_ERROR(hal.read_reg(Mask1::addr, mask.raw_value));
    mask.M_COLLISION = enable ? 0 : 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Mask1::addr, mask.raw_value));

    Maska maska;
    HAL_FAIL_ON_ERROR(hal.read_reg(Maska::addr, maska.raw_value));
    maska.M_HARDRST = enable ? 0 : 1;
    maska.M_TXSENT = enable ? 0 : 1;
    maska.M_HARDSENT = enable ? 0 : 1;
    maska.M_RETRYFAIL = enable ? 0 : 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Maska::addr, maska.raw_value));

    Maskb maskb;
    HAL_FAIL_ON_ERROR(hal.read_reg(Maskb::addr, maskb.raw_value));
    maskb.M_GCRCSENT = enable ? 0 : 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Maskb::addr, maskb.raw_value));

    return true;
}

bool Fusb302Rtos::fusb_set_auto_goodcrc(bool enable) {
    Switches1 sw1;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches1::addr, sw1.raw_value));
    sw1.AUTO_CRC = enable ? 1 : 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches1::addr, sw1.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_set_tx_auto_retries(uint8_t count) {
    Control3 ctl3;
    HAL_FAIL_ON_ERROR(hal.read_reg(Control3::addr, ctl3.raw_value));
    ctl3.N_RETRIES = count & 3; // 0-3 retries
    ctl3.AUTO_RETRY = count > 0 ? 1 : 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control3::addr, ctl3.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_flush_rx_fifo() {
    Control1 ctrl1{0};
    HAL_FAIL_ON_ERROR(hal.read_reg(Control1::addr, ctrl1.raw_value));
    ctrl1.RX_FLUSH = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control1::addr, ctrl1.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_flush_tx_fifo() {
    Control0 ctrl0{0};
    HAL_FAIL_ON_ERROR(hal.read_reg(Control0::addr, ctrl0.raw_value));
    ctrl0.TX_FLUSH = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control0::addr, ctrl0.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_pd_reset() {
    Reset rst{0};
    rst.PD_RESET = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Reset::addr, rst.raw_value));
    return fusb_setup();
}

bool Fusb302Rtos::fusb_set_polarity(TCPC_POLARITY polarity) {
    Switches0 sw0;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches0::addr, sw0.raw_value));
    sw0.MEAS_CC1 = 0;
    sw0.MEAS_CC2 = 0;

    if (polarity == TCPC_POLARITY::CC1) {
        sw0.MEAS_CC1 = 1;
        DRV_LOGI("Selected polarity: CC1");
    } else if (polarity == TCPC_POLARITY::CC2) {
        sw0.MEAS_CC2 = 1;
        DRV_LOGI("Selected polarity: CC2");
    } else {
        DRV_LOGI("Selected polarity: NONE");
    }
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));

    this->polarity = polarity;
    return true;
}

bool Fusb302Rtos::fusb_set_rx_enable(bool enable) {
    DRV_FAIL_ON_ERROR(fusb_flush_rx_fifo());
    rx_queue.clear_from_producer();
    DRV_FAIL_ON_ERROR(fusb_flush_tx_fifo());
    DRV_FAIL_ON_ERROR(fusb_set_auto_goodcrc(enable));
    DRV_FAIL_ON_ERROR(fusb_set_rxtx_interrupts(enable));

    // Probably, this can be moved to fusb_set_polarity in favor of using
    // rx_enabled flag only (+ disabling auto goodcrc when off).
    Switches1 sw1;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches1::addr, sw1.raw_value));
    sw1.TXCC1 = 0;
    sw1.TXCC2 = 0;

    if (enable) {
        if (polarity == TCPC_POLARITY::CC1) { sw1.TXCC1 = 1; }
        else if (polarity == TCPC_POLARITY::CC2) { sw1.TXCC2 = 1; }
        else {
            DRV_LOGE("Can't route BMC without polarity set");
            return false;
        }
    }

    HAL_FAIL_ON_ERROR(hal.write_reg(Switches1::addr, sw1.raw_value));

    rx_enabled = enable;
    return true;
}

void Fusb302Rtos::fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::Type status) {
    sync_transmit.mark_finished();

    // Safety check. If TX was repeated without waiting previous one =>
    // status is outdated, don't update. Probably, this should not happen
    // because of FIFO flush before transmit.
    if (!sync_transmit.is_enquired()) {
        port.tcpc_tx_status.store(status);
        has_deferred_wakeup = true;
    }
}

bool Fusb302Rtos::fusb_tx_pkt_begin() {
    DRV_FAIL_ON_ERROR(fusb_flush_tx_fifo());

    etl::vector<uint8_t, 40> fifo_buf{};

    // TODO: ensure no races possible here.
    // Create a local copy, to reduce theoretical races
    auto chunk = port.tx_chunk;

    // Max raw data size is: SOP[4] + PACKSYM[1] + HEAD[2] + DATA[28] + TAIL[4]
    // = 39. One extra byte may be used if HAL API uses first byte as
    // i2c address (but current API takes it as separate parameter)
    static_assert(decltype(fifo_buf)::MAX_SIZE >=
        4 + 1 + 2 + decltype(chunk)::MAX_SIZE + 4,
        "TX buffer too small to fit all possible data");

    // Ensure only "legacy" packets allowed. We do NOT support unchunked extended
    // packets and long vendor packets (that's really useless for sink mode).
    static_assert(decltype(chunk)::MAX_SIZE <= 28,
        "Packet size should not exceed 28 bytes in this implementation");

    // Hardcode Msg SOP, since library supports only sink mode
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP2);

    uint8_t pack_sym = TX_TKN::PACKSYM;

    // Add data size (+ 2 for header). No need to mask - value restricted by
    // static_assert above.

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
    fifo_buf.push_back(TX_TKN::TXON);

    HAL_FAIL_ON_ERROR(hal.write_block(FIFOs::addr, fifo_buf.data(), fifo_buf.size()));
    return true;
}

bool Fusb302Rtos::fusb_rx_pkt() {
    PD_CHUNK pkt{};
    uint8_t sop;
    uint8_t hdr[2];
    uint8_t crc_junk[4];

    Status1 status1{};
    HAL_FAIL_ON_ERROR(hal.read_reg(Status1::addr, status1.raw_value));

    if (status1.RX_EMPTY) {
        DRV_LOGI("Can't read from empty FIFO");
        return true;
    }

    // Pick all pending packets from RX FIFO.
    // Note, this should not happen, but in theory we can get several packets
    // per interrupt handler call. Use loop to be sure all those are extracted.
    while (!status1.RX_EMPTY) {
        HAL_FAIL_ON_ERROR(hal.read_reg(FIFOs::addr, sop));

        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, hdr, 2));
        pkt.header.raw_value = (hdr[1] << 8) | hdr[0];

        if (pkt.header.extended == 1 && pkt.header.data_obj_count == 0) {
            // Unchunked extended packets are not supported.
            DRV_LOGE("Unchunked extended packet received, ignoring");
            fusb_flush_rx_fifo();
            return false;
        }

        // Chunks have only 7 bits for data objects count. Max 28 bytes in total.
        // So, it's impossible to overflow chunk buffer.
        pkt.resize_by_data_obj_count();
        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, pkt.get_data().data(), pkt.data_size()));

        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, crc_junk, 4));

        // Just silently ignore GoodCRC, coming after TX
        if (!pkt.is_ctrl_msg(PD_CTRL_MSGT::GoodCRC)) {
            DRV_LOGI("Message received: type = {}, extended = {}, data size = {}",
                pkt.header.message_type, pkt.header.extended, pkt.data_size());
            rx_queue.push(pkt);
            has_deferred_wakeup = true;
        }

        HAL_FAIL_ON_ERROR(hal.read_reg(Status1::addr, status1.raw_value));
    }
    return true;
}

bool Fusb302Rtos::fusb_hr_send_begin() {
    sync_hr_send.mark_started();

    // Cleanup before send
    sync_transmit.reset();
    rx_queue.clear_from_producer();

    Control3 ctrl3;
    HAL_FAIL_ON_ERROR(hal.read_reg(Control3::addr, ctrl3.raw_value));
    ctrl3.SEND_HARD_RESET = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control3::addr, ctrl3.raw_value));

    return true;
}

bool Fusb302Rtos::fusb_hr_send_end() {
    sync_hr_send.mark_finished();
    has_deferred_wakeup = true;
    return true;
}

bool Fusb302Rtos::hr_cleanup() {
    // Cleanup internal states after hard reset received or been sent.
    DRV_FAIL_ON_ERROR(fusb_pd_reset());
    rx_queue.clear_from_producer();
    sync_transmit.reset();
    return true;
}

bool Fusb302Rtos::fusb_set_bist(TCPC_BIST_MODE mode) {
    Control1 ctrl1;
    Control3 ctrl3;

    HAL_FAIL_ON_ERROR(hal.read_reg(Control1::addr, ctrl1.raw_value));
    HAL_FAIL_ON_ERROR(hal.read_reg(Control3::addr, ctrl3.raw_value));
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

    HAL_FAIL_ON_ERROR(hal.write_reg(Control1::addr, ctrl1.raw_value));
    HAL_FAIL_ON_ERROR(hal.write_reg(Control3::addr, ctrl3.raw_value));

    if (mode == TCPC_BIST_MODE::Carrier) {
        Control0 ctrl0;
        HAL_FAIL_ON_ERROR(hal.read_reg(Control0::addr, ctrl0.raw_value));
        ctrl0.TX_START = 1;
        HAL_FAIL_ON_ERROR(hal.write_reg(Control0::addr, ctrl0.raw_value));
    }

    return true;
}

bool Fusb302Rtos::handle_interrupt() {
    while (hal.is_interrupt_active()) {
        Interrupt interrupt;
        Interrupta interrupta;
        Interruptb interruptb;

        // TODO: Consider 5-bytes block read (0x3E-0x42) with single call.
        HAL_FAIL_ON_ERROR(hal.read_reg(Interrupt::addr, interrupt.raw_value));
        HAL_FAIL_ON_ERROR(hal.read_reg(Interrupta::addr, interrupta.raw_value));
        HAL_FAIL_ON_ERROR(hal.read_reg(Interruptb::addr, interruptb.raw_value));

        if (interrupt.I_VBUSOK) {
            Status0 status0;
            HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
            vbus_ok.store(status0.VBUSOK);
            DRV_LOGI("IRQ: VBUS changed");
            has_deferred_wakeup = true;
        }

        if (interrupta.I_HARDRST) {
            DRV_LOGI("IRQ: hard reset received");
            DRV_FAIL_ON_ERROR(fusb_set_bist(TCPC_BIST_MODE::Off));
            DRV_FAIL_ON_ERROR(hr_cleanup());
            port.notify_prl(MsgToPrl_TcpcHardReset{});
        }

        if (interrupta.I_HARDSENT) {
            DRV_LOGI("IRQ: hard reset sent");
            DRV_FAIL_ON_ERROR(hr_cleanup());
            fusb_hr_send_end();
        }

        if (interrupt.I_COLLISION) {
            DRV_LOGI("IRQ: tx collision");
            // Discarding logic is part of PRL, here we just report tx failure
            fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::FAILED);
        }
        if (interrupta.I_RETRYFAIL) {
            DRV_LOGI("IRQ: tx retry failed");
            fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::FAILED);
        }
        if (interrupta.I_TXSENT) {
            fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::SUCCEEDED);
            DRV_LOGI("IRQ: tx completed");
            // That's not necessary, but force GoodCRC peek to free FIFO faster.
            fusb_rx_pkt();
        }
        if (interruptb.I_GCRCSENT) {
            if (rx_enabled) {
                DRV_LOGI("IRQ: GoodCRC sent");
                fusb_rx_pkt();
            } else {
                fusb_flush_rx_fifo();
            }
        }
    }

    return true;
}

// Since FUSB302 does not allow make different measurements in parallel,
// do all in single place to avoid collisions. Also, use simple FSM to implement
// non-blocking delays.
bool Fusb302Rtos::meter_tick(bool &repeat) {
    repeat = false;
    Status0 status0;
    Switches0 sw0;

    // Should be 250uS, but FreeRTOS not allows that precise timing.
    // Use 2 timer ticks (2ms) to guarantee at least 1ms after jitter.
    static constexpr uint32_t MEASURE_DELAY_MS = 2;

    switch (meter_state) {
        case MeterState::IDLE:
            if (sync_active_cc.is_enquired()) {
                sync_active_cc.mark_started();
                meter_state = MeterState::CC_ACTIVE_BEGIN;
                repeat = true;
                return true;
            }
            if (sync_scan_cc.is_enquired()) {
                sync_scan_cc.mark_started();
                meter_state = MeterState::SCAN_CC_BEGIN;
                repeat = true;
                return true;
            }
            break;

        case MeterState::CC_ACTIVE_BEGIN:
            if (polarity == TCPC_POLARITY::NONE) {
                DRV_LOGE("Can't measure active CC without polarity set");
                meter_state = MeterState::CC_ACTIVE_END;
                repeat = true;
            }

            static constexpr uint32_t DEBOUNCE_PERIOD_MS = 5;

            HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
            if (status0.ACTIVITY) {
                meter_wait_until_ts = get_timestamp() + DEBOUNCE_PERIOD_MS;
                meter_state = MeterState::CC_ACTIVE_MEASURE_WAIT;
                repeat = true;
                break;
            }

            {
                const auto cc_new = static_cast<TCPC_CC_LEVEL::Type>(status0.BC_LVL);
                if (polarity == TCPC_POLARITY::CC1) { cc1_cache = cc_new; }
                else { cc2_cache = cc_new; }
            }

            meter_state = MeterState::CC_ACTIVE_END;
            repeat = true;
            break;

        case MeterState::CC_ACTIVE_MEASURE_WAIT:
            if (get_timestamp() < meter_wait_until_ts) { break; }
            meter_state = MeterState::CC_ACTIVE_BEGIN;
            repeat = true;
            break;

        case MeterState::CC_ACTIVE_END:
            sync_active_cc.mark_finished();
            meter_state = MeterState::IDLE;
            has_deferred_wakeup = true;
            break;

        case MeterState::SCAN_CC_BEGIN:
            HAL_FAIL_ON_ERROR(hal.read_reg(Switches0::addr, sw0.raw_value));
            meter_backup_cc1 = sw0.MEAS_CC1;
            meter_backup_cc2 = sw0.MEAS_CC2;

            // Measure CC1
            sw0.MEAS_CC1 = 1;
            sw0.MEAS_CC2 = 0;
            HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));

            // Technically, 250uS is ok, but precise match would be platform-dependant
            // and, probably, blocking. We rely on FreeRTOS ticks instead.
            // Minimal value is 1, and we add one more to guard against jitter.
            meter_wait_until_ts = get_timestamp() + MEASURE_DELAY_MS;
            meter_state = MeterState::SCAN_CC1_MEASURE_WAIT;
            repeat = true;
            break;

        case MeterState::SCAN_CC1_MEASURE_WAIT:
            if (get_timestamp() < meter_wait_until_ts) { break; }

            HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
            cc1_cache = static_cast<TCPC_CC_LEVEL::Type>(status0.BC_LVL);

            // Measure CC2
            HAL_FAIL_ON_ERROR(hal.read_reg(Switches0::addr, sw0.raw_value));
            sw0.MEAS_CC1 = 0;
            sw0.MEAS_CC2 = 1;
            HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));
            meter_wait_until_ts = get_timestamp() + MEASURE_DELAY_MS;
            meter_state = MeterState::SCAN_CC2_MEASURE_WAIT;
            repeat = true;

            break;

        case MeterState::SCAN_CC2_MEASURE_WAIT:
            if (get_timestamp() < meter_wait_until_ts) { break; }

            HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
            cc2_cache = static_cast<TCPC_CC_LEVEL::Type>(status0.BC_LVL);

            // Restore previous state
            HAL_FAIL_ON_ERROR(hal.read_reg(Switches0::addr, sw0.raw_value));
            sw0.MEAS_CC1 = meter_backup_cc1;
            sw0.MEAS_CC2 = meter_backup_cc2;
            HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));

            sync_scan_cc.mark_finished();
            meter_state = MeterState::IDLE;
            has_deferred_wakeup = true;

            break;
    }

    return true;
}

bool Fusb302Rtos::handle_meter() {
    bool repeat = false;

    while (1) {
        DRV_FAIL_ON_ERROR(meter_tick(repeat));
        if (!repeat) { break; }
    }

    return true;
}

bool Fusb302Rtos::handle_timer() {
    if (!flags.test_and_clear(DRV_FLAG::TIMER_EVENT)) { return true; }

    port.notify_task(MsgTask_Timer{});
    return true;
}

bool Fusb302Rtos::handle_tcpc_calls() {
    if (sync_set_polarity.is_enquired()) {
        sync_set_polarity.mark_started();
        DRV_LOG_ON_ERROR(fusb_set_polarity(sync_set_polarity.get_param()));
        sync_set_polarity.mark_finished();
        has_deferred_wakeup = true;
    }

    if (sync_rx_enable.is_enquired()) {
        // TODO: ensure discard pending TX and in-progress operations.
        if (sync_transmit.is_enquired()) { sync_transmit.reset(); }

        sync_rx_enable.mark_started();
        DRV_LOG_ON_ERROR(fusb_set_rx_enable(sync_rx_enable.get_param()));
        sync_rx_enable.mark_finished();
        has_deferred_wakeup = true;
    }

    if (sync_hr_send.is_enquired()) {
        DRV_LOG_ON_ERROR(fusb_hr_send_begin());
    }

    if (sync_set_bist.is_enquired()) {
        sync_set_bist.mark_started();
        DRV_LOG_ON_ERROR(fusb_set_bist(sync_set_bist.get_param()));
        sync_set_bist.mark_finished();
        has_deferred_wakeup = true;
    }

    if (sync_transmit.is_enquired()) {
        sync_transmit.mark_started();
        DRV_LOG_ON_ERROR(fusb_tx_pkt_begin());
    }

    return true;
}

void Fusb302Rtos::task() {
    if (!flags.test(DRV_FLAG::FUSB_SETUP_DONE)) { fusb_setup(); }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (flags.test(DRV_FLAG::FUSB_SETUP_FAILED)) { continue; }
        handle_interrupt();
        handle_timer();
        handle_tcpc_calls();
        handle_meter();

        if (has_deferred_wakeup) {
            has_deferred_wakeup = false;
            port.wakeup();
        }
    }
}

void Fusb302Rtos::setup() {
    if (started) { return; }

    auto result = xTaskCreate(
        [](void* params) {
            static_cast<Fusb302Rtos*>(params)->task();
        }, "Fusb302Rtos", 1024*4, this, 10, &xWaitingTaskHandle
    );

    if (result != pdPASS) {
        DRV_LOGE("Failed to create Fusb302Rtos task");
        return;
    }

    started = true;

    hal.set_event_handler(
        hal_event_handler_t::create<Fusb302Rtos, &Fusb302Rtos::on_hal_event>(*this)
    );

    hal.setup();
}

void Fusb302Rtos::kick_task(bool from_isr) {
    if (!started) { return; }
    if (!xWaitingTaskHandle) { return; }

    if (from_isr) {
        auto woken = pdFALSE;
        vTaskNotifyGiveFromISR(xWaitingTaskHandle, &woken);
        if (woken != pdFALSE) { portYIELD_FROM_ISR(); }
    } else {
        xTaskNotifyGive(xWaitingTaskHandle);
    }
}

void Fusb302Rtos::on_hal_event(HAL_EVENT_TYPE event, bool from_isr) {
    if (event == HAL_EVENT_TYPE::Timer) { flags.set(DRV_FLAG::TIMER_EVENT); }

    kick_task(from_isr);
}

//
// TCPC API methods.
//

auto Fusb302Rtos::get_cc(TCPC_CC cc) -> TCPC_CC_LEVEL::Type {
    switch (cc) {
        case TCPC_CC::CC1:
            return cc1_cache;
        case TCPC_CC::CC2:
            return cc2_cache;
        case TCPC_CC::ACTIVE:
            if (polarity == TCPC_POLARITY::CC1) { return cc1_cache; }
            if (polarity == TCPC_POLARITY::CC2) { return cc2_cache; }
            // fallthrough to NONE if polarity not selected
            DRV_LOGE("get_cc: Polarity not selected, returning NONE");
            break;
        default:
            // Invalid CC type, return NONE (should not happen)
            break;
    }
    return TCPC_CC_LEVEL::NONE;
}

bool Fusb302Rtos::is_vbus_ok() {
    return vbus_ok.load();
}

bool Fusb302Rtos::fetch_rx_data() {
    return rx_queue.pop(port.rx_chunk);
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS