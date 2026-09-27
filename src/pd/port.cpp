#include "messages.h"
#include "port.h"

namespace pd {

void Port::notify_task(const etl::imessage& msg) {
    if (task_rtr) { task_rtr->receive(msg); }
}

void Port::notify_tc(const etl::imessage& msg) {
    if (tc_rtr) { tc_rtr->receive(msg); }
}

void Port::notify_pe(const etl::imessage& msg) {
    if (pe_rtr) { pe_rtr->receive(msg); }
}

void Port::notify_prl(const etl::imessage& msg) {
    if (prl_rtr) { prl_rtr->receive(msg); }
}

void Port::notify_dpm(const etl::imessage& msg) {
    if (dpm_rtr) { dpm_rtr->receive(msg); }
}

void Port::wakeup() {
    notify_task(MsgTask_Wakeup{});
}

bool Port::is_prl_running() {
    // Those defaults will be returned if PRL instance not yet subscribed
    // to the message bus. That's an acceptable behavior.
    bool is_running = false, is_busy = false;

    // NOTE: The message bus is strictly synchronous. MsgBus::receive()
    // returns only after the receiver processes the message, so passing
    // references to stack variables here is safe.
    notify_prl(MsgToPrl_GetPrlStatus{is_running, is_busy});
    return is_running;
}

bool Port::is_prl_busy() {
    bool is_running = false, is_busy = false;

    notify_prl(MsgToPrl_GetPrlStatus{is_running, is_busy});
    return is_busy;
}

} // namespace pd
