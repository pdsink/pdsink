#include "messages.h"
#include "port.h"

namespace pd {

void Port::notify_task(const etl::imessage& msg) {
    msgbus.receive(ROUTER_ID::TASK, msg);
}

void Port::notify_tc(const etl::imessage& msg) {
    msgbus.receive(ROUTER_ID::TC, msg);
}

void Port::notify_pe(const etl::imessage& msg) {
    msgbus.receive(ROUTER_ID::PE, msg);
}

void Port::notify_prl(const etl::imessage& msg) {
    msgbus.receive(ROUTER_ID::PRL, msg);
}

void Port::notify_dpm(const etl::imessage& msg) {
    msgbus.receive(ROUTER_ID::DPM, msg);
}

void Port::wakeup() {
    msgbus.receive(ROUTER_ID::TASK, MsgTask_Wakeup{});
}

bool Port::is_ams_active() {
    return pe_flags.test(PE_FLAG::AMS_ACTIVE);
}

void Port::wait_dpm_transit_to_default(bool enable) {
    if (enable) {
        pe_flags.set(PE_FLAG::WAIT_DPM_TRANSIT_TO_DEFAULT);
    } else {
        pe_flags.clear(PE_FLAG::WAIT_DPM_TRANSIT_TO_DEFAULT);
        wakeup();
    }
}

bool Port::is_prl_running() {
    // Those defaults will be returned if PRL instance not yet subscribed
    // to the message bus. That's an acceptable behavior.
    bool is_running = false, is_busy = false;

    msgbus.receive(ROUTER_ID::PRL, MsgToPrl_GetPrlStatus{is_running, is_busy});
    return is_running;
}

bool Port::is_prl_busy() {
    bool is_running = false, is_busy = false;

    msgbus.receive(ROUTER_ID::PRL, MsgToPrl_GetPrlStatus{is_running, is_busy});
    return is_busy;
}

} // namespace pd
