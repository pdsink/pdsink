#include <app_dpm.h>

void DPM_EventListener::on_receive(const pd::MsgToDpm_Startup& msg) {
    // Happens after cable insert or Hard Reset
    DPM_LOGI("Policy Engine started");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_TransitToDefault& msg) {
    // Happens on Hard Reset (usually nothing to do)
    DPM_LOGI("Transitioning to default state");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_SrcCapsReceived& msg) {
    DPM_LOGI("Source capabilities received");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_SelectCapDone& msg) {
    DPM_LOGI("Select capability done");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_SrcDisabled& msg) {
    // PD not available, failed to handshake
    DPM_LOGI("Source disabled");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_Alert& msg) {
    // May be useful to control overheating, if you can reduce the load
    // If overheating ignored, charger will reset the power.
    DPM_LOGI("Alert received, value 0x{:08X}", msg.value);
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_EPREntryFailed& msg) {
    // When charger has EPR mode, but entry declined
    // Abnormal situation, this should not happen
    DPM_LOGI("EPR entry failed, reason: 0x{:08X}, stay in SPR", msg.reason);
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_SnkReady& msg) {
    DPM_LOGI("Sink ready");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_HandshakeDone& msg) {
    // Main event to detect successful handshake. That means:
    // - PD communication is established
    // - Upgraded to EPR mode if available
    // - From this moment we can change available PD profiles anyhow
    DPM_LOGI("Handshake done");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_NewPowerLevelRejected& msg) {
    DPM_LOGI("New power level request rejected");
}

void DPM_EventListener::on_receive(const pd::MsgToDpm_NewPowerLevelAccepted& msg) {
    DPM_LOGI("New power level request accepted");
}

void DPM_EventListener::on_receive_unknown(const etl::imessage& msg) {
    // When you listen events partially, this one handles the rest.
    // Usually do nothing.
    DPM_LOGI("Unknown message received, ID: {}", msg.get_message_id());
}