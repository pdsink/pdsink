#include <gtest/gtest.h>
#include <array>
#include "utils/afsm.h"

using afsm::fsm;
using afsm::state;
using afsm::state_pack;
using afsm::interceptor;
using afsm::interceptor_pack;

enum StateID : etl::fsm_state_id_t { SID0 = 0, SID1, SID2, SID3, SID4, SID_Count };
static constexpr etl::fsm_state_id_t InvalidID = SID_Count;

class TestFSM : public fsm<TestFSM> {
public:
    std::array<int, SID_Count> enter_cnt{};
    std::array<int, SID_Count> exit_cnt{};
    std::array<int, SID_Count> run_cnt{};
    bool self_used = false;
    bool force_invalid = false;
    std::array<int, 3> interceptor_enter_cnt{};  // track up to 3 interceptors
    std::array<int, 3> interceptor_run_cnt{};
    std::array<int, 3> interceptor_exit_cnt{};
};

// Test interceptors
class LogInterceptor : public interceptor<TestFSM, LogInterceptor> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.interceptor_enter_cnt[0]++; return No_State_Change; }
    static etl::fsm_state_id_t on_run_state(FSMType& f) { f.interceptor_run_cnt[0]++; return No_State_Change; }
    static void on_exit_state(FSMType& f) { f.interceptor_exit_cnt[0]++; }
};

class ControlInterceptor : public interceptor<TestFSM, ControlInterceptor> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) {
        f.interceptor_enter_cnt[1]++;
        return f.force_invalid ? static_cast<etl::fsm_state_id_t>(SID1) : No_State_Change;
    }
    static etl::fsm_state_id_t on_run_state(FSMType& f) {
        f.interceptor_run_cnt[1]++;
        return f.self_used ? Self_Transition : No_State_Change;
    }
    static void on_exit_state(FSMType& f) { f.interceptor_exit_cnt[1]++; }
};

// S0: after 2 ticks -> S1
class S0 : public state<TestFSM, S0, SID0> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
    static etl::fsm_state_id_t on_run_state  (FSMType& f) {
        f.run_cnt[STATE_ID]++;
        if (f.run_cnt[STATE_ID] == 2) return SID1;
        return No_State_Change;
    }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S1: first tick -> Self_Transition (re-enter), next tick -> S2
class S1 : public state<TestFSM, S1, SID1> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
    static etl::fsm_state_id_t on_run_state  (FSMType& f) {
        f.run_cnt[STATE_ID]++;
        if (!f.self_used) { f.self_used = true; return Self_Transition; }
        return SID2;
    }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S2: enter() chains to S3
class S2 : public state<TestFSM, S2, SID2> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return SID3; }
    static etl::fsm_state_id_t on_run_state  (FSMType& f) { f.run_cnt[STATE_ID]++; return No_State_Change; }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S3: run() may return invalid id
class S3 : public state<TestFSM, S3, SID3> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
    static etl::fsm_state_id_t on_run_state  (FSMType& f) {
        f.run_cnt[STATE_ID]++;
        return f.force_invalid ? InvalidID : No_State_Change;
    }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S4: enter() returns Self_Transition (must be ignored)
class S4 : public state<TestFSM, S4, SID4> {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return Self_Transition; }
    static etl::fsm_state_id_t on_run_state  (FSMType& f) { f.run_cnt[STATE_ID]++; return No_State_Change; }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

using Pack = state_pack<S0, S1, S2, S3, S4>;

// State with interceptors using mixin approach
class S0_Mixin :
    public state<TestFSM, S0_Mixin, SID0>,
    public interceptor_pack<LogInterceptor, ControlInterceptor>
{
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
    static etl::fsm_state_id_t on_run_state(FSMType& f) { f.run_cnt[STATE_ID]++; return No_State_Change; }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// Interceptor pack for reuse
class TestInterceptorPack : public interceptor_pack<LogInterceptor, ControlInterceptor> {};

// MIXIN approach: state inherits from interceptor pack directly
class StateMixin : public state<TestFSM, StateMixin, 0>,
                   public TestInterceptorPack {
public:
    static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
    static etl::fsm_state_id_t on_run_state(FSMType& f) { f.run_cnt[STATE_ID]++; return No_State_Change; }
    static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};


// State pack inheriting from afsm::state_pack
class TestStatePack : public state_pack<StateMixin> {};

TEST(TickFsm, InitAndBasicTransition) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 1);

    fsm.run(); // S0 tick 1
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
    fsm.run(); // S0 tick 2 -> S1
    EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
    EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 1);
    EXPECT_EQ(fsm.enter_cnt[S1::STATE_ID], 1);
}

TEST(TickFsm, SelfTransitionReenterFromRun) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    fsm.run(); // S0 #1
    fsm.run(); // S0 #2 -> S1

    fsm.run(); // S1 first run => Self_Transition (exit1, enter1)
    EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
    EXPECT_EQ(fsm.exit_cnt[S1::STATE_ID], 1);
    EXPECT_EQ(fsm.enter_cnt[S1::STATE_ID], 2);
}

TEST(TickFsm, EnterChainingWorks) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    fsm.run(); // S0 #1
    fsm.run(); // S0 #2 -> S1
    fsm.run(); // S1 self
    fsm.run(); // S1 -> S2 (enter chains to S3)

    EXPECT_EQ(fsm.get_state_id(), S3::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S2::STATE_ID], 1);
    EXPECT_EQ(fsm.exit_cnt[S2::STATE_ID], 1);
    EXPECT_EQ(fsm.enter_cnt[S3::STATE_ID], 1);
}

TEST(TickFsm, InvalidIdAndSelfFromEnterAreIgnored) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    fsm.change_state(S3::STATE_ID);
    EXPECT_EQ(fsm.get_state_id(), S3::STATE_ID);

    fsm.force_invalid = true;
    fsm.run(); // returns InvalidID -> ignored
    EXPECT_EQ(fsm.get_state_id(), S3::STATE_ID);

    fsm.change_state(S4::STATE_ID); // enter returns Self -> ignored
    EXPECT_EQ(fsm.get_state_id(), S4::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S4::STATE_ID], 1);
    EXPECT_EQ(fsm.exit_cnt[S4::STATE_ID], 0);
}

TEST(TickFsm, EnumOverloadChangeState) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    fsm.change_state(S4::STATE_ID);
    EXPECT_EQ(fsm.get_state_id(), S4::STATE_ID);
    fsm.change_state(S0::STATE_ID);
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
}

TEST(TickFsm, StartUninitialized_NoEnter_NoRun) {
    TestFSM fsm;
    fsm.set_states<Pack>(afsm::Uninitialized);

    EXPECT_EQ(fsm.get_state_id(), afsm::Uninitialized);

    int sum = 0;
    for (auto v : fsm.enter_cnt) sum += v;
    EXPECT_EQ(sum, 0);

    fsm.run(); // no-op

    sum = 0;
    for (auto v : fsm.run_cnt) sum += v;
    EXPECT_EQ(sum, 0);
}

TEST(TickFsm, ResetToUninitialized_ExitOnly_ThenEnterOnNextState) {
    TestFSM fsm;
    fsm.set_states<Pack>(0); // enters S0 once
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 1);
    EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 0);

    // reset sugar
    fsm.change_state(afsm::Uninitialized);
    EXPECT_EQ(fsm.get_state_id(), afsm::Uninitialized);
    EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 1); // exit S0 happened once

    int sum = 0;
    for (auto v : fsm.enter_cnt) sum += v;
    EXPECT_EQ(sum, 1); // only S0 entered once total

    fsm.run(); // no-op
    sum = 0;
    for (auto v : fsm.run_cnt) sum += v;
    EXPECT_EQ(sum, 0);

    // next valid state: should call only its enter (no extra exit)
    fsm.change_state(S1::STATE_ID);
    EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S1::STATE_ID], 1);
    EXPECT_EQ(fsm.exit_cnt[S1::STATE_ID], 0);
}

TEST(TickFsm, ResetWhenAlreadyUninitialized_IsIdempotent) {
    TestFSM fsm;
    fsm.set_states<Pack>(afsm::Uninitialized);

    fsm.change_state(afsm::Uninitialized); // no-op

    EXPECT_EQ(fsm.get_state_id(), afsm::Uninitialized);

    int esum = 0, rsum = 0, xsum = 0;
    for (auto v : fsm.enter_cnt) esum += v;
    for (auto v : fsm.run_cnt)   rsum += v;
    for (auto v : fsm.exit_cnt)  xsum += v;

    EXPECT_EQ(esum, 0);
    EXPECT_EQ(rsum, 0);
    EXPECT_EQ(xsum, 0);

    fsm.run(); // still no-op

    esum = rsum = xsum = 0;
    for (auto v : fsm.enter_cnt) esum += v;
    for (auto v : fsm.run_cnt)   rsum += v;
    for (auto v : fsm.exit_cnt)  xsum += v;

    EXPECT_EQ(esum, 0);
    EXPECT_EQ(rsum, 0);
    EXPECT_EQ(xsum, 0);
}

TEST(TickFsm, ChangeStateSame_NoReenter_NoOps) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 1);
    EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 0);

    // same state, default (reenter=false) => no-op
    fsm.change_state(S0::STATE_ID);
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
    EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 1);
    EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 0);
}

TEST(TickFsm, ChangeStateSame_WithReenter_DoesExitEnter) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);

    fsm.change_state(S0::STATE_ID, true); // force reenter
    EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
    EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 1);
    EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 2);
}

TEST(TickFsm, ReenterFromUninitialized_HasNoExtraExit) {
    TestFSM fsm;
    fsm.set_states<Pack>(afsm::Uninitialized);
    EXPECT_EQ(fsm.get_state_id(), afsm::Uninitialized);

    fsm.change_state(S1::STATE_ID, true); // reenter=true ok, but no prior exit
    EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
    EXPECT_EQ(fsm.exit_cnt[S1::STATE_ID], 0);
    EXPECT_EQ(fsm.enter_cnt[S1::STATE_ID], 1);
}

TEST(TickFsm, IsUninitializedTest) {
    TestFSM fsm;

    EXPECT_TRUE(fsm.is_uninitialized());

    fsm.set_states<Pack>(0);
    EXPECT_FALSE(fsm.is_uninitialized());

    fsm.change_state(afsm::Uninitialized);
    EXPECT_TRUE(fsm.is_uninitialized());
}

TEST(TickFsm, PreviousStateTracking) {
    TestFSM fsm;
    fsm.set_states<Pack>(0);

    // Initially previous is Uninitialized
    EXPECT_EQ(fsm.get_previous_state_id(), afsm::Uninitialized);

    // S0 -> S1 via run()
    fsm.run(); fsm.run(); // S0 tick 2 -> S1
    EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
    EXPECT_EQ(fsm.get_previous_state_id(), S0::STATE_ID);

    // Self transition updates previous to current state
    fsm.run(); // S1 Self_Transition
    EXPECT_EQ(fsm.get_previous_state_id(), S1::STATE_ID);

    // Manual change_state
    fsm.change_state(S3::STATE_ID);
    EXPECT_EQ(fsm.get_previous_state_id(), S1::STATE_ID);
}

TEST(TickFsm, InterceptorMixinApproach) {
    TestFSM fsm;
    fsm.set_states<TestStatePack>(0);

    EXPECT_EQ(fsm.interceptor_enter_cnt[0], 1);  // LogInterceptor
    EXPECT_EQ(fsm.interceptor_enter_cnt[1], 1);  // ControlInterceptor
    EXPECT_EQ(fsm.enter_cnt[0], 1);              // StateMixin itself
}

TEST(TickFsm, InterceptorCanChangeState) {
    TestFSM fsm;
    fsm.set_states<state_pack<S0_Mixin, S1>>(0);

    fsm.force_invalid = true;  // ControlInterceptor will return SID1
    fsm.change_state(SID0, true); // Re-enter S0, but interceptor redirects to S1

    EXPECT_EQ(fsm.get_state_id(), SID1);
    EXPECT_EQ(fsm.interceptor_enter_cnt[1], 2);  // ControlInterceptor ran twice
}

TEST(TickFsm, InterceptorRollbackOnEarlyExit) {
    TestFSM fsm;
    class EarlyExitInterceptor : public interceptor<TestFSM, EarlyExitInterceptor> {
    public:
        static etl::fsm_state_id_t on_enter_state(FSMType& f) {
            f.interceptor_enter_cnt[2]++;
            return SID1;
        }
        static etl::fsm_state_id_t on_run_state(FSMType&) { return No_State_Change; }
        static void on_exit_state(FSMType& f) { f.interceptor_exit_cnt[2]++; }
    };

    class TestState : public state<TestFSM, TestState, SID0>,
                      public interceptor_pack<LogInterceptor, EarlyExitInterceptor> {
    public:
        static etl::fsm_state_id_t on_enter_state(TestFSM&) { return No_State_Change; }
        static etl::fsm_state_id_t on_run_state(TestFSM&) { return No_State_Change; }
        static void on_exit_state(TestFSM&) {}
    };

    fsm.set_states<state_pack<TestState, S1>>(0);

    EXPECT_EQ(fsm.get_state_id(), SID1);
    EXPECT_EQ(fsm.interceptor_enter_cnt[0], 1);  // LogInterceptor entered
    EXPECT_EQ(fsm.interceptor_exit_cnt[0], 1);   // LogInterceptor exited (rollback)
    EXPECT_EQ(fsm.interceptor_enter_cnt[2], 1);  // EarlyExitInterceptor ran
}

TEST(TickFsm, InterceptorSelfTransition) {
    TestFSM fsm;
    fsm.set_states<state_pack<S0_Mixin>>(0);

    fsm.self_used = true;  // ControlInterceptor will return Self_Transition
    fsm.run();

    EXPECT_EQ(fsm.get_state_id(), SID0);
    EXPECT_EQ(fsm.interceptor_run_cnt[1], 1);   // ControlInterceptor ran
    EXPECT_EQ(fsm.interceptor_exit_cnt[0], 1);  // LogInterceptor exit called
    EXPECT_EQ(fsm.interceptor_enter_cnt[0], 2); // LogInterceptor enter called again
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
