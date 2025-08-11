#include <gtest/gtest.h>
#include <array>
#include "utils/tick_fsm.h"

using etl_ext::tick_fsm;
using etl_ext::tick_fsm_state;
using etl_ext::tick_fsm_state_pack;

enum StateID : etl::fsm_state_id_t { SID0 = 0, SID1, SID2, SID3, SID4, SID_Count };
static constexpr etl::fsm_state_id_t InvalidID = SID_Count;

struct TestFSM : tick_fsm<TestFSM> {
  std::array<int, SID_Count> enter_cnt{};
  std::array<int, SID_Count> exit_cnt{};
  std::array<int, SID_Count> run_cnt{};
  bool self_used = false;
  bool force_invalid = false;
};

// S0: after 2 ticks -> S1
struct S0 : tick_fsm_state<TestFSM, S0, SID0> {
  static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
  static etl::fsm_state_id_t on_run_state  (FSMType& f) {
    f.run_cnt[STATE_ID]++;
    return (f.run_cnt[STATE_ID] == 2) ? ID(SID1) : No_State_Change;
  }
  static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S1: first tick -> Self_Transition (re-enter), next tick -> S2
struct S1 : tick_fsm_state<TestFSM, S1, SID1> {
  static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
  static etl::fsm_state_id_t on_run_state  (FSMType& f) {
    f.run_cnt[STATE_ID]++;
    if (!f.self_used) { f.self_used = true; return Self_Transition; }
    return SID2;
  }
  static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S2: enter() chains to S3
struct S2 : tick_fsm_state<TestFSM, S2, SID2> {
  static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return SID3; }
  static etl::fsm_state_id_t on_run_state  (FSMType& f) { f.run_cnt[STATE_ID]++; return No_State_Change; }
  static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S3: run() may return invalid id
struct S3 : tick_fsm_state<TestFSM, S3, SID3> {
  static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return No_State_Change; }
  static etl::fsm_state_id_t on_run_state  (FSMType& f) {
    f.run_cnt[STATE_ID]++;
    return f.force_invalid ? InvalidID : No_State_Change;
  }
  static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

// S4: enter() returns Self_Transition (must be ignored)
struct S4 : tick_fsm_state<TestFSM, S4, SID4> {
  static etl::fsm_state_id_t on_enter_state(FSMType& f) { f.enter_cnt[STATE_ID]++; return Self_Transition; }
  static etl::fsm_state_id_t on_run_state  (FSMType& f) { f.run_cnt[STATE_ID]++; return No_State_Change; }
  static void on_exit_state(FSMType& f) { f.exit_cnt[STATE_ID]++; }
};

using Pack = tick_fsm_state_pack<S0, S1, S2, S3, S4>;

TEST(TickFsm, InitAndBasicTransition) {
  TestFSM fsm;
  fsm.set_states<Pack>();
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
  fsm.set_states<Pack>();
  fsm.run(); // S0 #1
  fsm.run(); // S0 #2 -> S1

  fsm.run(); // S1 first run => Self_Transition (exit1, enter1)
  EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
  EXPECT_EQ(fsm.exit_cnt[S1::STATE_ID], 1);
  EXPECT_EQ(fsm.enter_cnt[S1::STATE_ID], 2);
}

TEST(TickFsm, EnterChainingWorks) {
  TestFSM fsm;
  fsm.set_states<Pack>();
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
  fsm.set_states<Pack>();
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
  fsm.set_states<Pack>();
  fsm.change_state(S4::STATE_ID);
  EXPECT_EQ(fsm.get_state_id(), S4::STATE_ID);
  fsm.change_state(S0::STATE_ID);
  EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
}

TEST(TickFsm, StartUninitialized_NoEnter_NoRun) {
  TestFSM fsm;
  fsm.set_states<Pack>(tick_fsm<TestFSM>::Uninitialized);

  EXPECT_EQ(fsm.get_state_id(), tick_fsm<TestFSM>::Uninitialized);

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
  fsm.set_states<Pack>(); // enters S0 once
  EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
  EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 1);
  EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 0);

  // reset sugar
  fsm.change_state(tick_fsm<TestFSM>::Uninitialized);
  EXPECT_EQ(fsm.get_state_id(), tick_fsm<TestFSM>::Uninitialized);
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
  fsm.set_states<Pack>(tick_fsm<TestFSM>::Uninitialized);

  fsm.change_state(tick_fsm<TestFSM>::Uninitialized); // no-op

  EXPECT_EQ(fsm.get_state_id(), tick_fsm<TestFSM>::Uninitialized);

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
  fsm.set_states<Pack>();
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
  fsm.set_states<Pack>();
  EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);

  fsm.change_state(S0::STATE_ID, true); // force reenter
  EXPECT_EQ(fsm.get_state_id(), S0::STATE_ID);
  EXPECT_EQ(fsm.exit_cnt[S0::STATE_ID], 1);
  EXPECT_EQ(fsm.enter_cnt[S0::STATE_ID], 2);
}

TEST(TickFsm, ReenterFromUninitialized_HasNoExtraExit) {
  TestFSM fsm;
  fsm.set_states<Pack>(tick_fsm<TestFSM>::Uninitialized);
  EXPECT_EQ(fsm.get_state_id(), tick_fsm<TestFSM>::Uninitialized);

  fsm.change_state(S1::STATE_ID, true); // reenter=true ok, but no prior exit
  EXPECT_EQ(fsm.get_state_id(), S1::STATE_ID);
  EXPECT_EQ(fsm.exit_cnt[S1::STATE_ID], 0);
  EXPECT_EQ(fsm.enter_cnt[S1::STATE_ID], 1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
