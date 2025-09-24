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

    // The most simple trigger, without diving into details.
    void trigger_any(uint32_t mv, uint32_t ma = 0) {
        trigger_variant(PDO_VARIANT::UNKNOWN, mv, ma);
    }

    // Trigger the selected PDO variant. May be useful, if you wish to work with
    // APDO and use fast mv/ma update.
    void trigger_variant(PDO_VARIANT pdo_variant, uint32_t mv, uint32_t ma = 0);

    // Trigger specific PDO by position in SRC Capabilities list. For
    // complicated scenarios, you may need to implement additional logic.
    // `mv` is ignored for FIXED PDO. If non-zero, both `mv` and `ma` clamped to
    // profile limits (position has priority over the rest).
    void trigger_by_position(uint8_t position, uint32_t mv=0, uint32_t ma = 0);

protected:
    Port& port;

    uint32_t trigger_mv{0};
    uint32_t trigger_ma{0};
    PDO_VARIANT trigger_pdo_variant{PDO_VARIANT::UNKNOWN};
    uint32_t trigger_position{0};

    enum class TRIGGER_MATCH_TYPE {
        USE_ANY,
        BY_PDO_VARIANT,
        BY_POSITION
    } trigger_match_type{TRIGGER_MATCH_TYPE::USE_ANY};

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
