#pragma once

#include "data_objects.h"

#include <etl/fsm.h>

namespace pd {

//
// Messages for DPM notifications
//

enum msg_dpm_id {
    // Generic events
    MSG_DPM_STARTUP = 200,
    MSG_DPM_TRANSIT_TO_DEFAULT,
    MSG_DPM_SRC_CAPS_RECEIVED,
    MSG_DPM_SELECT_CAP_DONE,
    MSG_DPM_SRC_DISABLED,
    MSG_DPM_ALERT,
    MSG_DPM_EPR_ENTRY_FAILED,
    // Results of DPM requests
    MSG_DPM_NEW_POWER_LEVEL,
    MSG_DPM_PPS_STATUS,
    MSG_DPM_PARTNER_REVISION,
    MSG_DPM_SOURCE_INFO,
    // Sugar
    MSG_DPM_HANDSHAKE_DONE, // Contracted and upgraded to EPR if possible.
    MSG_DPM_IDLE, // PE_SNK_Ready with no activity.
};

class MsgDpm_Startup : public etl::message<MSG_DPM_STARTUP> {};
class MsgDpm_TransitToDefault : public etl::message<MSG_DPM_TRANSIT_TO_DEFAULT> {};
class MsgDpm_SrcCapsReceived : public etl::message<MSG_DPM_SRC_CAPS_RECEIVED> {};
class MsgDpm_SelectCapDone : public etl::message<MSG_DPM_SELECT_CAP_DONE> {};
class MsgDpm_SrcDisabled : public etl::message<MSG_DPM_SRC_DISABLED> {};

class MsgDpm_Alert : public etl::message<MSG_DPM_ALERT> {
public:
    explicit MsgDpm_Alert(uint32_t value) : value{value} {}
    uint32_t value; // Alert data object raw data.
};

class MsgDpm_EPREntryFailed : public etl::message<MSG_DPM_EPR_ENTRY_FAILED> {
public:
    explicit MsgDpm_EPREntryFailed(uint32_t value) : value{value} {}
    uint32_t value; // EPR mode data object raw data (with failure reason).
};

class MsgDpm_NewPowerLevel : public etl::message<MSG_DPM_NEW_POWER_LEVEL> {
public:
    explicit MsgDpm_NewPowerLevel(uint32_t ok) : ok{ok} {}
    uint32_t ok; // succeeded/rejected.
};

class MsgDpm_PPSStatus : public etl::message<MSG_DPM_PPS_STATUS> {
public:
    explicit MsgDpm_PPSStatus(uint32_t value) : value{value} {}
    uint32_t value; // PPS status data object raw data.
};

class MsgDpm_PartnerRevision : public etl::message<MSG_DPM_PARTNER_REVISION> {
public:
    explicit MsgDpm_PartnerRevision(uint32_t value) : value{value} {}
    uint32_t value; // Revision data object raw data.
};

class MsgDpm_SourceInfo : public etl::message<MSG_DPM_SOURCE_INFO> {
public:
    explicit MsgDpm_SourceInfo(uint32_t value) : value{value} {}
    uint32_t value; // Source info data object raw data.
};

class MsgDpm_HandshakeDone : public etl::message<MSG_DPM_HANDSHAKE_DONE> {};
class MsgDpm_Idle : public etl::message<MSG_DPM_IDLE> {};


class Sink;
class PE;

class IDPM {
public:
    virtual uint32_t get_request_data_object() = 0;
    virtual PDO_LIST get_sink_pdo_list() = 0;
    virtual uint32_t get_epr_watts() = 0;
    virtual void notify(const etl::imessage& msg) = 0;
};

class DPM : public IDPM {
public:
    DPM(Sink& sink, PE& pe);

    // Disable unexpected use
    DPM() = delete;
    DPM(const DPM&) = delete;
    DPM& operator=(const DPM&) = delete;

    // Override to listen events from PE
    virtual void notify(const etl::imessage& msg) {}

    //
    // Methods below provide default implementation of DPM logic
    // and can be customized
    //

    virtual bool has_usb_comm() { return false; }

    // This one is called from `SRC Capabilities` and `EPR SRC Capabilities`
    // Override with your custom logic to evaluate capabilities and return
    virtual uint32_t get_request_data_object();

    // This one is called via `Sink Capabilities` and `EPR Sink Capabilities`
    // requests from SRC. By default, provides max possible list of Sink PDOs.
    // Override with your custom logic to provide Sink capabilities.
    // See details in default implementation
    virtual PDO_LIST get_sink_pdo_list();

    // Required by spec to enter EPR mode (no ideas why)
    virtual uint32_t get_epr_watts() { return 140; }

    // Helper to fill request RDO flags
    virtual void fill_rdo_flags(uint32_t &rdo);

    //
    // Sugar methods for simple mode select. You are not forced to use those.
    //
    void trigger_fixed(uint32_t mv) { trigger_mv = mv; trigger_mode = FIXED; };
    void trigger_spr_pps(uint32_t mv, uint32_t ma) {
        trigger_mv = mv; trigger_ma = ma; trigger_mode = SPR_PPS;
    };

    enum TRIGGER_MODE {
        INACTIVE,
        FIXED,
        SPR_PPS,
    } trigger_mode{INACTIVE};
    uint32_t trigger_mv{0};
    uint32_t trigger_ma{0};

    // SNK RDOs cache, to build only once
    PDO_LIST sink_rdo_list;

private:
    Sink& sink;
    PE& pe;
};

} // namespace pd
