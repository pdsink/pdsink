#pragma once

#include <pd/pd.h>

using DPM_EventListener_Base = etl::message_router<class DPM_EventListener,
    pd::MsgToDpm_Startup,
    pd::MsgToDpm_TransitToDefault,
    pd::MsgToDpm_SrcCapsReceived,
    pd::MsgToDpm_SelectCapDone,
    pd::MsgToDpm_SrcDisabled,
    pd::MsgToDpm_Alert,
    pd::MsgToDpm_EPREntryFailed,
    pd::MsgToDpm_SnkReady,
    pd::MsgToDpm_HandshakeDone,
    pd::MsgToDpm_NewPowerLevelRejected,
    pd::MsgToDpm_NewPowerLevelAccepted
>;

class DPM_EventListener : public DPM_EventListener_Base {
public:
    DPM_EventListener(pd::DPM& dpm) : dpm(dpm) {}
    void on_receive(const pd::MsgToDpm_Startup& msg);
    void on_receive(const pd::MsgToDpm_TransitToDefault& msg);
    void on_receive(const pd::MsgToDpm_SrcCapsReceived& msg);
    void on_receive(const pd::MsgToDpm_SelectCapDone& msg);
    void on_receive(const pd::MsgToDpm_SrcDisabled& msg);
    void on_receive(const pd::MsgToDpm_Alert& msg);
    void on_receive(const pd::MsgToDpm_EPREntryFailed& msg);
    void on_receive(const pd::MsgToDpm_SnkReady& msg);
    void on_receive(const pd::MsgToDpm_HandshakeDone& msg);
    void on_receive(const pd::MsgToDpm_NewPowerLevelRejected& msg);
    void on_receive(const pd::MsgToDpm_NewPowerLevelAccepted& msg);
    void on_receive_unknown(const etl::imessage& msg);
private:
    pd::DPM& dpm;
};

class AppDPM : public pd::DPM {
public:
    AppDPM(pd::Port& port) : pd::DPM(port), dpm_event_listener(*this) {}

    void setup() override { port.dpm_rtr = &dpm_event_listener; }

    DPM_EventListener dpm_event_listener;
};
