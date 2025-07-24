#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS)

#include <etl/vector.h>
#include "sink.h"
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

#define HAL_LOG_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_ERR("FUSB302 HAL IO error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
        } \
    } while (0)

#define HAL_FAIL_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_ERR("FUSB302 HAL IO error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
            return false; \
        } \
    } while (0)

#define DRV_LOG_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_ERR("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
        } \
    } while (0)

#define DRV_FAIL_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_ERR("FUSB302 driver error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
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

Fusb302Rtos::Fusb302Rtos(Sink& sink, IFusb302RtosHal& hal, bool emulate_vbus_ok)
    : sink{sink}, hal{hal}, emulate_vbus_ok{emulate_vbus_ok}
{
    sink.driver = this;
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
    DRV_LOG("FUSB302 ID: VER={}, PROD={}, REV={}",
        id.VERSION_ID, id.PRODUCT_ID, id.REVISION_ID);

    // Power up all blocks
    Power pwr{0};
    pwr.PWR = 0xF;
    HAL_FAIL_ON_ERROR(hal.write_reg(Power::addr, pwr.raw_value));

    // Setup interrupts

    Mask1 mask{0xFF};
    mask.M_COLLISION = 0;
    mask.M_ACTIVITY = 0; // used to debounce active CC levels reading
    mask.M_ALERT = 0;
    if (!emulate_vbus_ok) { mask.M_VBUSOK = 0; }
    HAL_FAIL_ON_ERROR(hal.write_reg(Mask1::addr, mask.raw_value));

    Maska maska{0xFF};
    maska.M_HARDRST = 0;
    maska.M_TXSENT = 0;
    maska.M_HARDSENT = 0;
    maska.M_RETRYFAIL = 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Maska::addr, maska.raw_value));

    Maskb maskb{0xFF};
    maskb.M_GCRCSENT = 0; // New RX packet (received + confirmed)
    HAL_FAIL_ON_ERROR(hal.write_reg(Maskb::addr, maskb.raw_value));

    // Setup controls/switches. Rely on defaults, modify only the necessary bits.

    Switches1 sw1;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches1::addr, sw1.raw_value));
    sw1.AUTO_CRC = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches1::addr, sw1.raw_value));

    DRV_FAIL_ON_ERROR(fusb_set_polarity(TCPC_POLARITY::NONE));

    if (!emulate_vbus_ok) {
        Status0 status0;
        HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
        vbus_ok.store(static_cast<bool>(status0.VBUSOK));
    }

    flags.clear(DRV_FLAG::FUSB_SETUP_FAILED);
    flags.set(DRV_FLAG::FUSB_SETUP_DONE);
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

bool Fusb302Rtos::fusb_set_polarity(TCPC_POLARITY::Type polarity) {
    Switches0 sw0;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches0::addr, sw0.raw_value));
    sw0.MEAS_CC1 = 0;
    sw0.MEAS_CC2 = 0;

    if (polarity == TCPC_POLARITY::CC1) {
        sw0.MEAS_CC1 = 1;
        DRV_LOG("Selected polarity: CC1");
    } else if (polarity == TCPC_POLARITY::CC2) {
        sw0.MEAS_CC2 = 1;
        DRV_LOG("Selected polarity: CC2");
    } else {
        DRV_LOG("Selected polarity: NONE");
    }
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));

    this->polarity = polarity;
    last_activity_ts = hal.get_timestamp();
    return true;
}

bool Fusb302Rtos::fusb_set_rx_enable(bool enable) {
    DRV_FAIL_ON_ERROR(fusb_flush_rx_fifo());
    rx_queue.clear_from_producer();
    DRV_FAIL_ON_ERROR(fusb_flush_tx_fifo());

    Switches1 sw1;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches1::addr, sw1.raw_value));
    sw1.TXCC1 = 0;
    sw1.TXCC2 = 0;

    if (enable) {
        if (polarity == TCPC_POLARITY::CC1) { sw1.TXCC1 = 1; }
        else if (polarity == TCPC_POLARITY::CC2) { sw1.TXCC2 = 1; }
        else {
            DRV_ERR("Can't route BMC without polarity set");
            return false;
        }
    }

    HAL_FAIL_ON_ERROR(hal.write_reg(Switches1::addr, sw1.raw_value));

    if (enable) { DRV_LOG("BMC attached, polarity selected"); }
    else { DRV_LOG("BMC detached"); }

    return true;
}

void Fusb302Rtos::fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::Type status) {
    sync_transmit.mark_finished();
    pass_up(MsgTcpcTransmitStatus(status));
}

bool Fusb302Rtos::fusb_tx_pkt_begin() {
    DRV_FAIL_ON_ERROR(fusb_flush_tx_fifo());

    etl::vector<uint8_t, 40> fifo_buf{};

    // Max raw data size is: SOP[4] + PACKSYM[1] + HEAD[2] + DATA[28] + TAIL[4] = 39
    static_assert(decltype(fifo_buf)::MAX_SIZE >=
        4 + 1 + 2 + decltype(call_arg_transmit)::MAX_SIZE + 4,
        "TX buffer too small to fit all possible data");

    // Ensure only "legacy" packets allowed. We do NOT support unchunked extended
    // packets and long vendor packets (that's really useless for sink mode).
    static_assert(decltype(call_arg_transmit)::MAX_SIZE <= 28,
        "Packet size should not exceed 28 bytes in this implementation");

    // Hardcode Msg SOP, since library supports only sink mode
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP2);

    uint8_t pack_sym = TX_TKN::PACKSYM;
    // Add data size (+ 2 for header). No need to mask - value restricted by
    // static_assert above.
    pack_sym |= (call_arg_transmit.data_size() + 2);
    fifo_buf.push_back(pack_sym);

    // Msg header
    fifo_buf.push_back(call_arg_transmit.header.raw_value & 0xFF);
    fifo_buf.push_back((call_arg_transmit.header.raw_value >> 8) & 0xFF);

    // Msg data
    fifo_buf.insert(fifo_buf.end(),
        call_arg_transmit.get_data().begin(),
        call_arg_transmit.get_data().end()
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
        DRV_ERR("Can't read from empty FIFO");
        return false;
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
            DRV_ERR("Unchunked extended packet received, ignoring");
            fusb_flush_rx_fifo();
            return false;
        }

        // Chunks have only 7 bits for data objects count. Max 28 bytes in total.
        // So, it's impossible to overflow chunk buffer.
        pkt.resize_by_data_obj_count();
        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, pkt.get_data().data(), pkt.data_size()));

        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, crc_junk, 4));

        if (pkt.is_ctrl_msg(PD_CTRL_MSGT::GoodCRC)) {
            DRV_LOG("GoodCRC received");
            fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::SUCCEEDED);
        } else {
            DRV_LOG("Message received: type = {}, extended = {}, data size = {}",
                pkt.header.message_type, pkt.header.extended, pkt.data_size());
            rx_queue.push(pkt);
            pass_up(MsgTcpcWakeup());
        }

        HAL_FAIL_ON_ERROR(hal.read_reg(Status1::addr, status1.raw_value));
    }
    return true;
}

bool Fusb302Rtos::fusb_hr_send_begin() {
    sync_hr_send.mark_started();

    Control3 ctrl3;
    HAL_FAIL_ON_ERROR(hal.read_reg(Control3::addr, ctrl3.raw_value));
    ctrl3.SEND_HARD_RESET = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control3::addr, ctrl3.raw_value));
    // Cleanup buffers for sure
    rx_queue.clear_from_producer();

    sync_transmit.reset();
    return true;
}

bool Fusb302Rtos::fusb_hr_send_end() {
    // TODO: (?) add chip reset
    sync_hr_send.mark_finished();
    pass_up(MsgTcpcWakeup());
    return true;
}

bool Fusb302Rtos::fusb_bist(bool enable) {
    Control1 ctrl1;
    HAL_FAIL_ON_ERROR(hal.read_reg(Control1::addr, ctrl1.raw_value));
    ctrl1.BIST_MODE2 = enable ? 1 : 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control1::addr, ctrl1.raw_value));
    return true;
}

bool Fusb302Rtos::handle_interrupt() {
    while (hal.is_interrupt_active()) {
        Interrupt interrupt;
        Interrupta interrupta;
        Interruptb interruptb;
        Status0 status0;

        HAL_FAIL_ON_ERROR(hal.read_reg(Interrupt::addr, interrupt.raw_value));
        HAL_FAIL_ON_ERROR(hal.read_reg(Interrupta::addr, interrupta.raw_value));
        HAL_FAIL_ON_ERROR(hal.read_reg(Interruptb::addr, interruptb.raw_value));
        HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));

        if (interrupt.I_ACTIVITY) {
            DRV_LOG("IRQ: activity detected");
            // ANY toggle => pop last activity mark
            last_activity_ts = get_timestamp();
        }
        activity_on = status0.ACTIVITY;

        if (interrupt.I_VBUSOK) {
            vbus_ok.store(status0.VBUSOK);
            DRV_LOG("IRQ: VBUS changed");
            pass_up(MsgTcpcWakeup());
        }

        if (interrupta.I_HARDRST) {
            DRV_LOG("IRQ: hard reset received");
            Reset rst{0};
            rst.PD_RESET = 1;
            HAL_FAIL_ON_ERROR(hal.write_reg(Reset::addr, rst.raw_value));
            // Cleanup buffers for sure
            rx_queue.clear_from_producer();
            sync_transmit.reset();
            pass_up(MsgTcpcHardReset());
        }

        if (interrupta.I_HARDSENT) {
            DRV_LOG("IRQ: hard reset sent");
            fusb_hr_send_end();
        }

        if (interrupt.I_COLLISION) {
            DRV_LOG("IRQ: tx collision");
            // Discarding logic is part of PRL, here we just report tx failure
            fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::FAILED);
        }
        if (interrupta.I_RETRYFAIL) {
            DRV_LOG("IRQ: tx retry failed");
            fusb_tx_pkt_end(TCPC_TRANSMIT_STATUS::FAILED);
        }
        if (interrupta.I_TXSENT) {
            DRV_LOG("IRQ: tx completed");
            // We should peek GoodCRC from FIFO first.
            fusb_rx_pkt();
        }
        if (interruptb.I_GCRCSENT) {
            DRV_LOG("IRQ: GoodCRC sent");
            fusb_rx_pkt();
        }

        if (interrupt.I_ALERT) {
            DRV_LOG("IRQ: ALERT received (fifo overflow)");
            // Fuckup. Reset FIFO-s for sure.
            fusb_flush_rx_fifo();
            fusb_flush_tx_fifo();
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
                DRV_ERR("Can't measure active CC without polarity set");
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

                if (emulate_vbus_ok) {
                    vbus_ok_emulator_last_measure_ts = get_timestamp();

                    auto prev_vbus_ok = vbus_ok.load();
                    auto new_vbus_ok = (cc_new != TCPC_CC_LEVEL::NONE);

                    vbus_ok.store(new_vbus_ok);
                    if (new_vbus_ok != prev_vbus_ok) {
                        DRV_LOG("Emulator: VBUS changed by Active CC");
                        pass_up(MsgTcpcWakeup());
                    }
                }
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
            pass_up(MsgTcpcWakeup());
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

            if (emulate_vbus_ok) {
                vbus_ok_emulator_last_measure_ts = get_timestamp();

                auto prev_vbus_ok = vbus_ok.load();
                auto new_vbus_ok = (cc1_cache != TCPC_CC_LEVEL::NONE) ||
                    (cc2_cache != TCPC_CC_LEVEL::NONE);

                vbus_ok.store(new_vbus_ok);
                if (new_vbus_ok != prev_vbus_ok) {
                    DRV_LOG("Emulator: VBUS changed by CC1/CC2 scan");
                    pass_up(MsgTcpcWakeup());
                }
            }

            sync_scan_cc.mark_finished();
            meter_state = MeterState::IDLE;
            pass_up(MsgTcpcWakeup());

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

    // Update last activity timestamp if activity is on. This is used to
    // calculate safe time for active CC measure
    if (activity_on) { last_activity_ts = get_timestamp(); }

    if (emulate_vbus_ok) {
        // In VBUS_OK emulation mode enforce probe requests
        static constexpr uint32_t ACTIVE_CC_DEBOUNCE_MS = 100;
        static constexpr uint32_t FULL_CC_SCAN_DEBOUNCE_MS = 40;

        // Rearm CC requests if debounce interval passed
        if (polarity == TCPC_POLARITY::NONE) {
            if ((get_timestamp() > vbus_ok_emulator_last_measure_ts + FULL_CC_SCAN_DEBOUNCE_MS) &&
                !sync_scan_cc.is_enquired() && !sync_scan_cc.is_started()) {
                sync_scan_cc.enquire();
            }
        } else {
            if ((get_timestamp() > vbus_ok_emulator_last_measure_ts + ACTIVE_CC_DEBOUNCE_MS) &&
                !sync_active_cc.is_enquired() && !sync_active_cc.is_started()) {
                sync_active_cc.enquire();
            }
        }
    }

    pass_up(MsgTimerEvent());
    return true;
}

bool Fusb302Rtos::handle_tcpc_calls() {
    if (sync_hr_send.is_enquired()) {
        DRV_LOG_ON_ERROR(fusb_hr_send_begin());
    }

    if (sync_set_polarity.is_enquired()) {
        sync_set_polarity.mark_started();
        DRV_LOG_ON_ERROR(fusb_set_polarity(sync_set_polarity.get_param()));
        sync_set_polarity.mark_finished();
        pass_up(MsgTcpcWakeup());
    }

    if (sync_rx_enable.is_enquired()) {
        sync_rx_enable.mark_started();
        DRV_LOG_ON_ERROR(fusb_set_rx_enable(sync_rx_enable.get_param()));
        sync_rx_enable.mark_finished();
        pass_up(MsgTcpcWakeup());
    }

    if (sync_bist_carrier_enable.is_enquired()) {
        sync_bist_carrier_enable.mark_started();
        DRV_LOG_ON_ERROR(fusb_bist(sync_bist_carrier_enable.get_param()));
        sync_bist_carrier_enable.mark_finished();
        pass_up(MsgTcpcWakeup());
    }

    if (sync_transmit.is_enquired()) {
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
    }
}

void Fusb302Rtos::start() {
    if (started) { return; }

    auto result = xTaskCreate(
        [](void* params) {
            static_cast<Fusb302Rtos*>(params)->task();
        }, "Fusb302Rtos", 1024*4, this, 10, &xWaitingTaskHandle
    );

    if (result != pdPASS) {
        DRV_ERR("Failed to create Fusb302Rtos task");
        return;
    }

    started = true;

    hal.set_event_handler(
        hal_event_handler_t::create<Fusb302Rtos, &Fusb302Rtos::on_hal_event>(*this)
    );

    hal.start();
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

auto Fusb302Rtos::get_cc(TCPC_CC::Type cc) -> TCPC_CC_LEVEL::Type {
    switch (cc) {
        case TCPC_CC::CC1:
            return cc1_cache;
        case TCPC_CC::CC2:
            return cc2_cache;
        case TCPC_CC::ACTIVE:
            if (polarity == TCPC_POLARITY::CC1) { return cc1_cache; }
            if (polarity == TCPC_POLARITY::CC2) { return cc2_cache; }
            // fallthrough to NONE if polarity not selected
            DRV_ERR("get_cc: Polarity not selected, returning NONE");
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

bool Fusb302Rtos::fetch_rx_data(PD_CHUNK& data) {
    return rx_queue.pop(data);
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS