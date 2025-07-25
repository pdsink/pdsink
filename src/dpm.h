#pragma once

#include <etl/fsm.h>

#include "data_objects.h"
#include "port.h"

namespace pd {

class Sink;
class PE;

class IDPM {
public:
    virtual uint32_t get_request_data_object(const etl::ivector<uint32_t>& src_caps) = 0;
    virtual PDO_LIST get_sink_pdo_list() = 0;
    virtual uint32_t get_epr_watts() = 0;
    virtual void notify(const etl::imessage& msg) = 0;
};

class DPM : public IDPM {
public:
    DPM(Port& port, Sink& sink, PE& pe);

    // Disable unexpected use
    DPM() = delete;
    DPM(const DPM&) = delete;
    DPM& operator=(const DPM&) = delete;

    // Override to listen events from PE
    virtual void notify(const etl::imessage& msg) override {}

    //
    // Methods below provide default implementation of DPM logic
    // and can be customized
    //

    virtual bool has_usb_comm() { return false; }

    // This one is called from `SRC Capabilities` and `EPR SRC Capabilities`
    // Override with your custom logic to evaluate capabilities and return
    virtual uint32_t get_request_data_object(const etl::ivector<uint32_t>& src_caps) override;

    // This one is called via `Sink Capabilities` and `EPR Sink Capabilities`
    // requests from SRC. By default, provides max possible list of Sink PDOs.
    // Override with your custom logic to provide Sink capabilities.
    // See details in default implementation
    virtual PDO_LIST get_sink_pdo_list() override;

    // Required by spec to enter EPR mode (no ideas why)
    virtual uint32_t get_epr_watts() override { return 140; }

    // Helper to fill request RDO flags
    virtual void fill_rdo_flags(uint32_t &rdo);

    //
    // Sugar methods for simple mode select. You are not forced to use those.
    //
    void trigger_fixed(uint32_t mv);
    void trigger_spr_pps(uint32_t mv, uint32_t ma);

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
    Port& port;
    Sink& sink;
    PE& pe;
};

} // namespace pd
