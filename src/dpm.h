#pragma once

#include "data_objects.h"

#include <etl/fsm.h>

namespace pd {

// Messages for DPM notifications
enum msg_dpm_id {
    MSG_DPM_ALERT,
};

class MsgDpmAlert : public etl::message<MSG_DPM_ALERT> {
public:
    explicit MsgDpmAlert(uint32_t value) : value{value} {}
    // Alert data object. Use simple type for now. May be will change later.
    uint32_t value;
};


class Sink;
class PE;

class DPM: public etl::fsm {
public:
    DPM(Sink& sink, PE& pe);
    void dispatch(const MsgPdEvents& events, const bool pd_enabled);

    // Disable unexpected use
    DPM() = delete;
    DPM(const DPM&) = delete;
    DPM& operator=(const DPM&) = delete;

    virtual void notify(const etl::imessage& msg) {}

    //
    // Methods below provide default implementation of DPM logic
    // and can be customized
    //

    virtual bool has_usb_comm() { return false; }

    // This one is called from `SRC Capabilities` and `ERP SRC Capabilities`
    // Override with your custom logic to evaluate capabilities and return
    virtual uint32_t get_request_data_object();

    // This one is called via `Sink Capabilities` and `ERP Sink Capabilities`
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
