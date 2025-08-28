#pragma once

#include <etl/utility.h>

#include "data_objects.h"
#include "utils/dobj_utils.h"

namespace pd {

class Port; class PE;

class IDPM {
public:
    virtual void setup() = 0;
    virtual etl::pair<uint32_t, uint32_t> get_request_data_object(const etl::ivector<uint32_t>& src_caps) = 0;
    virtual PDO_LIST get_sink_pdo_list() = 0;
    virtual uint32_t get_epr_watts() = 0;
};

class DPM : public IDPM {
public:
    DPM(Port& port) : port{port} {};

    // Disable unexpected use
    DPM() = delete;
    DPM(const DPM&) = delete;
    DPM& operator=(const DPM&) = delete;

    // You can override this method in inherited class, to listen events
    virtual void setup() override {
        //port.msgbus.subscribe(dpm_event_listener);
    }

    //
    // Methods below provide default implementation of DPM logic
    // and can be customized
    //

    // This one is called from `SRC Capabilities` and `EPR SRC Capabilities`
    // Override with your custom logic to evaluate capabilities and return
    virtual etl::pair<uint32_t, uint32_t> get_request_data_object(const etl::ivector<uint32_t>& src_caps) override;

    // This one is called via `Sink Capabilities` and `EPR Sink Capabilities`
    // requests from SRC. By default, provides max possible list of Sink PDOs.
    // Override with your custom logic to provide Sink capabilities.
    // See details in default implementation
    virtual PDO_LIST get_sink_pdo_list() override;

    // Required by spec to enter EPR mode (no ideas why)
    virtual uint32_t get_epr_watts() override { return epr_watts; }

    //
    // Sugar methods for simple mode select. You are not forced to use those.
    // If current not set - use max available from profile. For EPR AVS, where
    // current not available in profile directly, use watts instead, to
    // calculate (and clamp to 5A if above).
    //
    void trigger_fixed(uint32_t mv, uint32_t ma = 0);
    void trigger_spr_pps(uint32_t mv, uint32_t ma);
    // Scans all available profiles, and use the first suitable.
    void trigger_any(uint32_t mv, uint32_t ma = 0);

protected:
    Port& port;

    uint32_t trigger_mv{0};
    uint32_t trigger_ma{0};

    dobj_utils::SRCSNK_PDO_ID trigger_pdo_id{dobj_utils::SRCSNK_PDO_ID::UNKNOWN};
    bool trigger_any_pdo{false}; // try to use any suitable PDO type

    // Basic value for source with EPR. Update for your needs.
    uint32_t epr_watts{140};

    // SNK PDOs cache, to build only once
    PDO_LIST sink_pdo_list;

    // Helper to fill request RDO flags
    virtual void fill_rdo_flags(uint32_t &rdo);
    virtual bool has_usb_comm() { return false; }

    virtual void request_new_power_level();
};

} // namespace pd
