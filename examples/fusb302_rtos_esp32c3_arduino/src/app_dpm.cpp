#include "app_dpm.h"

using namespace pd;

extern PE pe;
extern Port port;

void DPM_EventListener::on_receive(const MsgToDpm_Startup& msg) {
    // Happens after a cable is inserted or after a Hard Reset
    DPM_LOGI("Policy Engine started");
}

void DPM_EventListener::on_receive(const MsgToDpm_TransitToDefault& msg) {
    // Happens on Hard Reset (usually nothing to do)
    DPM_LOGI("Transitioning to default state");
}

void DPM_EventListener::on_receive(const MsgToDpm_SrcCapsReceived& msg) {
    using namespace dobj_utils;

    int caps_count = port.source_caps.size();

    DPM_LOGI("Source capabilities received [{}]", caps_count);

    for (int i = 0; i < caps_count; i++) {
        auto pdo = port.source_caps[i];

        if (pdo == 0) {
            DPM_LOGV("  PDO[{}]: <PLACEHOLDER> (zero)", i+1);
            continue;
        }

        auto id = get_src_pdo_variant(pdo);

        if (id == PDO_VARIANT::UNKNOWN) {
            DPM_LOGV("  PDO[{}]: 0x{:08X} <UNKNOWN>", i+1, pdo);
        }
        else if (id == PDO_VARIANT::FIXED) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            DPM_LOGV("  PDO[{}]: 0x{:08X} <FIXED> {}mV {}mA",
                i+1, pdo, limits.mv_min, limits.ma);
        }
        else if (id == PDO_VARIANT::APDO_PPS) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            DPM_LOGV("  PDO[{}]: 0x{:08X} <APDO_PPS> {}-{}mV {}mA",
                i+1, pdo, limits.mv_min, limits.mv_max, limits.ma);
        }
        else if (id == PDO_VARIANT::APDO_SPR_AVS) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            DPM_LOGV("  PDO[{}]: 0x{:08X} <APDO_SPR_AVS> {}-{}mV {}mA",
                i+1, pdo, limits.mv_min, limits.mv_max, limits.ma);
        }
        else if (id == PDO_VARIANT::APDO_EPR_AVS) {
            ETL_MAYBE_UNUSED auto limits = get_src_pdo_limits(pdo);
            DPM_LOGV("  PDO[{}]: 0x{:08X} <APDO_EPR_AVS> {}-{}mV {}W",
                i+1, pdo, limits.mv_min, limits.mv_max, limits.pdp);
        }
        else {
            DPM_LOGV("  PDO[{}]: 0x{:08X} <!!!UNHANDLED!!!>", i+1, pdo);
        }
    }
}

void DPM_EventListener::on_receive(const MsgToDpm_SelectCapDone& msg) {
    DPM_LOGI("Select capability done");
}

void DPM_EventListener::on_receive(const MsgToDpm_SrcDisabled& msg) {
    // PD not available; the handshake failed
    DPM_LOGI("Source disabled [USB PD not supported]");
}

void DPM_EventListener::on_receive(const MsgToDpm_Alert& msg) {
    // Can be useful to control overheating if you can reduce the load.
    // If overheating is ignored, the charger will reset the power.
    DPM_LOGI("Alert received, value 0x{:08X}", msg.value);

    PD_ALERT alert{msg.value};
    DPM_LOGI("  OCP={}, OTP={}", alert.ocp, alert.otp);
}

void DPM_EventListener::on_receive(const MsgToDpm_EPREntryFailed& msg) {
    // When the charger supports EPR mode but entry is declined
    // Abnormal situation; this should not happen
    DPM_LOGI("EPR entry failed, reason: 0x{:08X}, stay in SPR", msg.reason);
}

void DPM_EventListener::on_receive(const MsgToDpm_SnkReady& msg) {
    DPM_LOGI("Sink ready");
}

void DPM_EventListener::on_receive(const MsgToDpm_CableAttached& msg) {
    DPM_LOGI("Cable attached");
}

void DPM_EventListener::on_receive(const MsgToDpm_CableDetached& msg) {
    DPM_LOGI("Cable detached");
}

void DPM_EventListener::on_receive(const MsgToDpm_HandshakeDone& msg) {
    // Main event to detect a successful handshake. That means:
    // - PD communication is established
    // - Upgraded to EPR mode if available
    // - From this moment we can change the available PD profiles as needed
    DPM_LOGI("Handshake done");
}

void DPM_EventListener::on_receive(const MsgToDpm_NewPowerLevelRejected& msg) {
    DPM_LOGI("New power level request rejected");
}

void DPM_EventListener::on_receive(const MsgToDpm_NewPowerLevelAccepted& msg) {
    DPM_LOGI("New power level request accepted");
}

void DPM_EventListener::on_receive_unknown(const etl::imessage& msg) {
    // If you subscribe to only some events, this one handles the rest.
    // Usually do nothing here.
    DPM_LOGI("Unknown message received, ID: {}", msg.get_message_id());
}
