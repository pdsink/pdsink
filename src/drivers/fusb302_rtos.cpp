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

void Fusb302Rtos::start() {
}

void Fusb302Rtos::notify_task() {
}

void Fusb302Rtos::req_cc_both() {
}

void Fusb302Rtos::req_cc_active() {
}

auto Fusb302Rtos::get_cc(TCPC_CC::Type cc) -> TCPC_CC_LEVEL::Type {
    return TCPC_CC_LEVEL::Type::RP_3_0; // Temporary stub
}

void Fusb302Rtos::set_polarity(TCPC_CC::Type active_cc) {
}

void Fusb302Rtos::set_rx_enable(bool enable) {
}

bool Fusb302Rtos::has_rx_data() {
    return false; // Temporary stub
}

void Fusb302Rtos::fetch_rx_data(PKT_INFO& data) {
}

void Fusb302Rtos::transmit(const PKT_INFO& tx_info) {
}

void Fusb302Rtos::bist_carrier_enable(bool enable) {
}

void Fusb302Rtos::hard_reset() {
}

HalMsgHandler::HalMsgHandler(Fusb302Rtos& drv) : drv{drv} {}

void HalMsgHandler::on_receive(const MsgHal_Timer& msg) {
}

void HalMsgHandler::on_receive(const MsgHal_Interrupt& msg) {
}

void HalMsgHandler::on_receive_unknown(const etl::imessage& msg) {
    // Handle unknown messages
}

} // namespace fusb302

} // namespace pd

#endif // USE_FUSB302_RTOS