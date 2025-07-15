#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS)

#include <etl/vector.h>
#include "sink.h"
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

#define HAL_FAIL_ON_ERROR(expr) \
    do { \
        if (!(expr)) { \
            DRV_ERR("FUSB302 HAL IO error at {}:{} [{}] in {}", __FILE__, __LINE__, #expr, __func__); \
            return false; \
        } \
    } while (0)

#define DRV_FAIL_ON_ERROR(x) HAL_FAIL_ON_ERROR(x)

// FIFO TX tokens
namespace TX_TKN {
    static constexpr uint8_t TXON = 0xA1;
    static constexpr uint8_t SOP = 0x12;
    static constexpr uint8_t SOP1 = 0x13;
    static constexpr uint8_t SOP2 = 0x1B;
    static constexpr uint8_t SOP3 = 0x15;
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
    hal.set_msg_router(hal_msg_handler);
}

bool Fusb302Rtos::fusb_setup() {
    if (flags.test(DRV_FLAG::FUSB_SETUP_FAILED)) { return false; }

    flags.set(DRV_FLAG::FUSB_SETUP_FAILED);

    // Reset chip
    Reset rst{.raw_value = 0};
    rst.SW_RES = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Reset::addr, rst.raw_value));

    // Read ID to check connection
    DeviceID id;
    HAL_FAIL_ON_ERROR(hal.read_reg(DeviceID::addr, id.raw_value));
    DRV_LOG("FUSB302 ID: VER=%d, PROD=%d, REV=%d",
        id.VERSION_ID, id.PRODUCT_ID, id.REVISION_ID);

    // Power up all blocks
    Power pwr{.raw_value = 0};
    pwr.PWR = 0xF;
    HAL_FAIL_ON_ERROR(hal.write_reg(Power::addr, pwr.raw_value));

    // Setup interrupts

    Mask1 mask{.raw_value = 0xFF};
    mask.M_COLLISION = 0;
    mask.M_ACTIVITY = 0; // used to debounce active CC levels reading
    if (!emulate_vbus_ok) { mask.M_VBUSOK = 0; }
    HAL_FAIL_ON_ERROR(hal.write_reg(Mask1::addr, mask.raw_value));

    Maska maska{.raw_value = 0xFF};
    maska.M_HARDRST = 0;
    maska.M_TXSENT = 0; // TX packet sent (and confirmed)
    maska.M_HARDSENT = 0;
    maska.M_RETRYFAIL = 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Maska::addr, maska.raw_value));

    Maskb maskb{.raw_value = 0xFF};
    maskb.M_GCRCSENT = 0; // New RX packet (received + confirmed)
    HAL_FAIL_ON_ERROR(hal.write_reg(Maskb::addr, maskb.raw_value));

    // Setup controls/switches. Rely on defaults, modify only the necessary bits.

    Control2 ctrl2;
    HAL_FAIL_ON_ERROR(hal.read_reg(Control3::addr, ctrl2.raw_value));
    ctrl2.MODE = 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control3::addr, ctrl2.raw_value));

    Switches1 sw1;
    HAL_FAIL_ON_ERROR(hal.read_reg(Switches1::addr, sw1.raw_value));
    sw1.AUTO_CRC = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches1::addr, sw1.raw_value));

    DRV_FAIL_ON_ERROR(fusb_set_polarity(TCPC_POLARITY::NONE));

    flags.clear(DRV_FLAG::FUSB_SETUP_FAILED);
    flags.set(DRV_FLAG::FUSB_SETUP_DONE);
    return true;
}

bool Fusb302Rtos::fusb_flush_rx_fifo() {
    Control1 ctrl1{.raw_value = 0};
    HAL_FAIL_ON_ERROR(hal.read_reg(Control1::addr, ctrl1.raw_value));
    ctrl1.RX_FLUSH = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control1::addr, ctrl1.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_flush_tx_fifo() {
    Control0 ctrl0{.raw_value = 0};
    HAL_FAIL_ON_ERROR(hal.read_reg(Control0::addr, ctrl0.raw_value));
    ctrl0.TX_FLUSH = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control0::addr, ctrl0.raw_value));
    return true;
}

bool Fusb302Rtos::fusb_scan_cc() {
    // This function is used only for initial CC detection.
    Switches0 sw0;
    Status0 status0;

    HAL_FAIL_ON_ERROR(hal.read_reg(Switches0::addr, sw0.raw_value));
    auto backup_cc1 = sw0.MEAS_CC1;
    auto backup_cc2 = sw0.MEAS_CC2;

    // Measure CC1
    sw0.MEAS_CC1 = 1;
    sw0.MEAS_CC2 = 0;
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));
    // Technically, 250uS is ok, but precise match would be platform-dependant
    // and, probably, blocking.
    vTaskDelay(pdMS_TO_TICKS(2));
    HAL_FAIL_ON_ERROR(hal.read_reg(Measure::addr, status0.raw_value));
    cc1_cache = static_cast<TCPC_CC_LEVEL::Type>(status0.BC_LVL);

    // Measure CC2
    sw0.MEAS_CC1 = 0;
    sw0.MEAS_CC2 = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));
    vTaskDelay(pdMS_TO_TICKS(2));
    HAL_FAIL_ON_ERROR(hal.read_reg(Measure::addr, status0.raw_value));
    cc2_cache = static_cast<TCPC_CC_LEVEL::Type>(status0.BC_LVL);

    // Restore previous state
    sw0.MEAS_CC1 = backup_cc1;
    sw0.MEAS_CC2 = backup_cc2;
    HAL_FAIL_ON_ERROR(hal.write_reg(Switches0::addr, sw0.raw_value));

    if (emulate_vbus_ok) {
        vbus_ok_emulator_last_measure_ts = hal.get_timestamp();

        auto prev_vbus_ok = vbus_ok.load();
        vbus_ok.store((cc1_cache != TCPC_CC_LEVEL::NONE) ||
                      (cc2_cache != TCPC_CC_LEVEL::NONE));

        if (vbus_ok.load() != prev_vbus_ok) {
            DRV_LOG("Emulator: VBUS changed by CC1/CC2 scan");
            msg_router->receive(MsgTcpcWakeup());
        }
    }

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

    DRV_LOG("Polarity selected");
    return true;
}

void Fusb302Rtos::fusb_complete_tx(TCPC_TRANSMIT_STATUS::Type status) {
    state.clear(TCPC_CALL_FLAG::TRANSMIT);
    msg_router->receive(MsgTcpcTransmitStatus(status));
}

bool Fusb302Rtos::fusb_tx_pkt() {
    DRV_FAIL_ON_ERROR(fusb_flush_tx_fifo());

    // Max buffer size is SOP[4] + PACKSYM[1] + HEAD[2] + DATA[28] + TAIL[4] = 39
    etl::vector<uint8_t, 40> fifo_buf{};

    // Msg SOP (SOP'/SOP" not needed, use constant one)
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP1);
    fifo_buf.push_back(TX_TKN::SOP2);

    // PACKSYM with data size
    auto pack_sym = TX_TKN::PACKSYM;
    pack_sym |= (call_arg_transmit.data_size() + 2); // data + header
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

    while (!status1.RX_EMPTY) {
        HAL_FAIL_ON_ERROR(hal.read_reg(FIFOs::addr, sop));

        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, hdr, 2));
        pkt.header.raw_value = (hdr[1] << 8) | hdr[0];

        pkt.resize_by_data_obj_count();
        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, pkt.get_data().data(), pkt.data_size()));

        HAL_FAIL_ON_ERROR(hal.read_block(FIFOs::addr, crc_junk, 4));

        if (pkt.is_ctrl_msg(PD_CTRL_MSGT::GoodCRC)) {
            DRV_LOG("GoodCRC received");
            fusb_complete_tx(TCPC_TRANSMIT_STATUS::SUCCEEDED);
        } else {
            DRV_LOG("Message received: type = {}, extended = {}, data size = {}",
                pkt.header.message_type, pkt.header.extended, pkt.data_size());
            rx_queue.push(pkt);
            msg_router->receive(MsgTcpcWakeup());
        }

        HAL_FAIL_ON_ERROR(hal.read_reg(Status1::addr, status1.raw_value));
    }
    return true;
}

bool Fusb302Rtos::fusb_hr_send_begin() {
    Control3 ctrl3;
    HAL_FAIL_ON_ERROR(hal.read_reg(Control3::addr, ctrl3.raw_value));
    ctrl3.SEND_HARD_RESET = 1;
    HAL_FAIL_ON_ERROR(hal.write_reg(Control3::addr, ctrl3.raw_value));
    // Cleanup buffers for sure
    rx_queue.clear_from_producer();
    state.clear(TCPC_CALL_FLAG::TRANSMIT);
    flags.clear(DRV_FLAG::ENQUIRED_TRANSMIT);
    return true;
}

bool Fusb302Rtos::fusb_hr_send_end() {
    // TODO: (?) add chip reset
    state.clear(TCPC_CALL_FLAG::HARD_RESET);
    msg_router->receive(MsgTcpcWakeup());
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
            msg_router->receive(MsgTcpcWakeup());
        }

        if (interrupta.I_HARDRST) {
            DRV_LOG("IRQ: hard reset received");
            Reset rst{.raw_value = 0};
            rst.PD_RESET = 1;
            HAL_FAIL_ON_ERROR(hal.write_reg(Reset::addr, rst.raw_value));
            // Cleanup buffers for sure
            rx_queue.clear_from_producer();
            state.clear(TCPC_CALL_FLAG::TRANSMIT);
            flags.clear(DRV_FLAG::ENQUIRED_TRANSMIT);
            msg_router->receive(MsgTcpcHardReset());
        }

        if (interrupta.I_HARDSENT) {
            DRV_LOG("IRQ: hard reset sent");
            fusb_hr_send_end();
        }

        if (interrupt.I_COLLISION) {
            DRV_LOG("IRQ: tx collision");
            // Discarding logic is part of PRL, here we just report tx failure
            fusb_complete_tx(TCPC_TRANSMIT_STATUS::FAILED);
        }
        if (interrupta.I_RETRYFAIL) {
            DRV_LOG("IRQ: tx retry failed");
            fusb_complete_tx(TCPC_TRANSMIT_STATUS::FAILED);
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
    }
    return true;
}

bool Fusb302Rtos::handle_get_active_cc() {
    if (polarity == TCPC_POLARITY::NONE) {
        DRV_ERR("Can't get active CC without polarity set, ignore request");
        return true;
    }

    static constexpr uint32_t DEBOUNCE_PERIOD_MS = 5;

    if (get_timestamp() < last_activity_ts + DEBOUNCE_PERIOD_MS) { return false; }

    // Wrap BC_LVL fetch with direct ACTIVITY check to avoid false values
    Status0 status0;
    HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
    if (status0.ACTIVITY) { activity_on = true; return false; }

    auto bc_lvl = status0.BC_LVL;

    HAL_FAIL_ON_ERROR(hal.read_reg(Status0::addr, status0.raw_value));
    if (status0.ACTIVITY) { activity_on = true; return false; }

    auto cc_new = static_cast<TCPC_CC_LEVEL::Type>(bc_lvl);

    if (polarity == TCPC_POLARITY::CC1) { cc1_cache = cc_new; }
    else { cc2_cache = cc_new; }

    if (emulate_vbus_ok) {
        vbus_ok_emulator_last_measure_ts = hal.get_timestamp();

        auto prev_vbus_ok = vbus_ok.load();
        vbus_ok.store(cc_new != TCPC_CC_LEVEL::NONE);

        if (vbus_ok.load() != prev_vbus_ok) {
            DRV_LOG("Emulator: VBUS changed by Active CC");
            msg_router->receive(MsgTcpcWakeup());
        }
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
                (sem_req_cc_scan.load() == 0)) {
                sem_req_cc_scan.fetch_add(1);
            }
        } else {
            if ((get_timestamp() > vbus_ok_emulator_last_measure_ts + ACTIVE_CC_DEBOUNCE_MS) &&
                (sem_req_cc_active.load() == 0)) {
                sem_req_cc_active.fetch_add(1);
            }
        }
    }

    // "Tasks"

    if (sem_req_cc_scan.load()) {
        auto sem_ver = sem_req_cc_scan.load();
        while (1) {
            if (!fusb_scan_cc()) { break; }
            vbus_ok_emulator_last_measure_ts = hal.get_timestamp();
            if (sem_req_cc_scan.compare_exchange_strong(sem_ver, 0)) {
                state.clear(TCPC_CALL_FLAG::REQ_CC_BOTH);
                msg_router->receive(MsgTcpcWakeup());
                break;
            }
        }
    }

    if (sem_req_cc_active.load()) {
        auto sem_ver = sem_req_cc_active.load();
        while (1) {
            // on failure - postpone to next tick
            if (!handle_get_active_cc()) { break; }
            vbus_ok_emulator_last_measure_ts = hal.get_timestamp();

            if (sem_req_cc_active.compare_exchange_strong(sem_ver, 0)) {
                state.clear(TCPC_CALL_FLAG::REQ_CC_ACTIVE);
                msg_router->receive(MsgTcpcWakeup());
                break;
            }
        }
    }

    msg_router->receive(MsgTimerEvent());
    return true;
}

bool Fusb302Rtos::handle_tcpc_calls() {
    // Simple requests
    while (flags.test_and_clear(DRV_FLAG::API_CALL)) {
        if (flags.test_and_clear(DRV_FLAG::ENQUIRED_HR_SEND)) {
            fusb_hr_send_begin();
        }

        if (state.test(TCPC_CALL_FLAG::SET_POLARITY)) {
            DRV_FAIL_ON_ERROR(fusb_set_polarity(call_arg_set_polarity));
            state.clear(TCPC_CALL_FLAG::SET_POLARITY);
            msg_router->receive(MsgTcpcWakeup());
        }

        if (state.test(TCPC_CALL_FLAG::SET_RX_ENABLE)) {
            DRV_FAIL_ON_ERROR(fusb_set_rx_enable(call_arg_set_rx_enable));
            state.clear(TCPC_CALL_FLAG::SET_RX_ENABLE);
            msg_router->receive(MsgTcpcWakeup());
        }

        if (state.test(TCPC_CALL_FLAG::BIST_CARRIER_ENABLE)) {
            Control1 ctl1;
            HAL_FAIL_ON_ERROR(hal.read_reg(Control1::addr, ctl1.raw_value));
            ctl1.BIST_MODE2 = call_arg_bist_carrier_enable;
            HAL_FAIL_ON_ERROR(hal.write_reg(Control1::addr, ctl1.raw_value));
            state.clear(TCPC_CALL_FLAG::BIST_CARRIER_ENABLE);
            msg_router->receive(MsgTcpcWakeup());
        }

        if (flags.test_and_clear(DRV_FLAG::ENQUIRED_TRANSMIT)) {
            DRV_FAIL_ON_ERROR(fusb_tx_pkt());
        }
    }

    return true;
}

void Fusb302Rtos::task() {
    if (!flags.test(DRV_FLAG::FUSB_SETUP_DONE)) { fusb_setup(); }
    if (flags.test(DRV_FLAG::FUSB_SETUP_FAILED)) { return; }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        handle_interrupt();
        handle_timer();
        handle_tcpc_calls();
    }
}

void Fusb302Rtos::start() {
    if (started) { return; }
    started = true;

    xTaskCreate(
        [](void* params) {
            auto* self = static_cast<Fusb302Rtos*>(params);
            while (true) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                self->task();
            }
        }, "Fusb302Rtos", 1024*4, this, 10, &xWaitingTaskHandle
    );

    hal.start();
}

void Fusb302Rtos::notify_task() {
    if (!started) { return; }
    if (!xWaitingTaskHandle) { return; }

    if (xPortInIsrContext()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xWaitingTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken != pdFALSE) {
            portYIELD_FROM_ISR();
        }
    } else {
        xTaskNotifyGive(xWaitingTaskHandle);
    }
}

//
// TCPC methods. All async ones just forward data to task().
//

void Fusb302Rtos::req_cc_both() {
    state.set(TCPC_CALL_FLAG::REQ_CC_BOTH);
    sem_req_cc_scan.fetch_add(1);
    notify_task();
}

void Fusb302Rtos::req_cc_active() {
    state.set(TCPC_CALL_FLAG::REQ_CC_ACTIVE);
    sem_req_cc_active.fetch_add(1);
    notify_task();
}

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

void Fusb302Rtos::set_polarity(TCPC_POLARITY::Type active_cc) {
    call_arg_set_polarity = active_cc;
    state.set(TCPC_CALL_FLAG::SET_POLARITY);
    flags.set(DRV_FLAG::API_CALL);
    notify_task();
}

void Fusb302Rtos::set_rx_enable(bool enable) {
    call_arg_set_rx_enable = enable;
    state.set(TCPC_CALL_FLAG::SET_RX_ENABLE);
    flags.set(DRV_FLAG::API_CALL);
    notify_task();
}

bool Fusb302Rtos::fetch_rx_data(PD_CHUNK& data) {
    return rx_queue.pop(data);
}

void Fusb302Rtos::transmit(const PD_CHUNK& tx_info) {
    call_arg_transmit = tx_info;
    state.set(TCPC_CALL_FLAG::TRANSMIT);
    flags.set(DRV_FLAG::ENQUIRED_TRANSMIT);
    flags.set(DRV_FLAG::API_CALL);
    notify_task();
}

void Fusb302Rtos::bist_carrier_enable(bool enable) {
    call_arg_bist_carrier_enable = enable;
    state.set(TCPC_CALL_FLAG::BIST_CARRIER_ENABLE);
    flags.set(DRV_FLAG::ENQUIRED_HR_SEND);
    flags.set(DRV_FLAG::API_CALL);
    notify_task();
}

void Fusb302Rtos::hr_send() {
    state.set(TCPC_CALL_FLAG::HARD_RESET);
    flags.set(DRV_FLAG::API_CALL);
    notify_task();
}

HalMsgHandler::HalMsgHandler(Fusb302Rtos& drv) : drv{drv} {}

void HalMsgHandler::on_receive(const MsgHal_Timer& msg) {
    drv.flags.set(DRV_FLAG::TIMER_EVENT);
    drv.notify_task();
}

void HalMsgHandler::on_receive(const MsgHal_Interrupt& msg) {
    // fusb302 interrupt is level-based. No need to set extra flag.
    drv.notify_task();
}

void HalMsgHandler::on_receive_unknown(const etl::imessage& msg) {
    DRV_ERR("Unknown event from HAL, id: {}", msg.get_message_id());
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS