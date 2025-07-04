#include "pd_conf.h"

#if defined(USE_FUSB302_RTOS)

#include "sink.h"
#include "fusb302_rtos.h"

namespace pd {

namespace fusb302 {

Fusb302Rtos::Fusb302Rtos(Sink& sink, IFusb302RtosHal& hal)
    : sink{sink}, hal{hal}
{
    sink.driver = this;
    hal.set_msg_router(hal_msg_handler);
}

void Fusb302Rtos::task() {
    // TODO
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
    notify_task();
}

void Fusb302Rtos::req_cc_active() {
    state.set(TCPC_CALL_FLAG::REQ_CC_ACTIVE);
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
            break;
        default:
            // Invalid CC type, return NONE (should not happen)
            break;
    }
    return TCPC_CC_LEVEL::NONE;
}

void Fusb302Rtos::set_polarity(TCPC_POLARITY::Type active_cc) {
    call_arg_set_polarity = active_cc;
    state.set(TCPC_CALL_FLAG::SET_POLARITY);
    notify_task();
}

void Fusb302Rtos::set_rx_enable(bool enable) {
    call_arg_set_rx_enable = enable;
    state.set(TCPC_CALL_FLAG::SET_RX_ENABLE);
    notify_task();
}

bool Fusb302Rtos::has_rx_data() {
    return !rx_queue.empty();
}

bool Fusb302Rtos::fetch_rx_data(PKT_INFO& data) {
    return rx_queue.pop(data);
}

void Fusb302Rtos::transmit(const PKT_INFO& tx_info) {
    call_arg_transmit = tx_info;
    state.set(TCPC_CALL_FLAG::TRANSMIT);
    notify_task();
}

void Fusb302Rtos::bist_carrier_enable(bool enable) {
    call_arg_bist_carrier_enable = enable;
    state.set(TCPC_CALL_FLAG::BIST_CARRIER_ENABLE);
    notify_task();
}

void Fusb302Rtos::hard_reset() {
    state.set(TCPC_CALL_FLAG::HARD_RESET);
    notify_task();
}

HalMsgHandler::HalMsgHandler(Fusb302Rtos& drv) : drv{drv} {}

void HalMsgHandler::on_receive(const MsgHal_Timer& msg) {
    drv.hal_events.set(HAL_EVENT_FLAG::TIMER);
    drv.notify_task();
}

void HalMsgHandler::on_receive(const MsgHal_Interrupt& msg) {
    drv.hal_events.set(HAL_EVENT_FLAG::FUSB302_INTERRUPT);
    drv.notify_task();
}

void HalMsgHandler::on_receive_unknown(const etl::imessage& msg) {}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS