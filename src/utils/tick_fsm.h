#pragma once

#include <etl/fsm.h>
#include <etl/type_traits.h>
#include <etl/integral_limits.h>
#include <stddef.h>

namespace etl_ext {

namespace tfsm_details {
    template<typename...>
    struct first_type;

    template<typename First, typename... Rest>
    struct first_type<First, Rest...> {
        using type = First;
    };

    // Check all states have same FSMType
    template<typename ExpectedFSM, typename... States>
    struct check_fsm_types;

    template<typename ExpectedFSM>
    struct check_fsm_types<ExpectedFSM> {
        static constexpr bool value = true;
    };

    template<typename ExpectedFSM, typename First, typename... Rest>
    struct check_fsm_types<ExpectedFSM, First, Rest...> {
        static constexpr bool value =
            etl::is_same<ExpectedFSM, typename First::FSMType>::value &&
            check_fsm_types<ExpectedFSM, Rest...>::value;
    };

    // Check state IDs are sequential from 0
    template<size_t Expected, typename... States>
    struct check_state_ids;

    template<size_t Expected>
    struct check_state_ids<Expected> {
        static constexpr bool value = true;
    };

    template<size_t Expected, typename First, typename... Rest>
    struct check_state_ids<Expected, First, Rest...> {
        static constexpr bool value =
            (First::STATE_ID == Expected) &&
            check_state_ids<Expected + 1, Rest...>::value;
    };
}

template<typename FSM, typename Derived, etl::fsm_state_id_t StateID>
class tick_fsm_state {
public:
    using FSMType = FSM;
    static constexpr etl::fsm_state_id_t STATE_ID = StateID;

    static constexpr auto No_State_Change = etl::ifsm_state::No_State_Change;
    static constexpr auto Self_Transition = etl::ifsm_state::Self_Transition;

    static etl::fsm_state_id_t on_enter_state(FSMType& fsm) {
        return Derived::on_enter_state(fsm);
    }

    static etl::fsm_state_id_t on_run_state(FSMType& fsm) {
        return Derived::on_run_state(fsm);
    }

    static void on_exit_state(FSMType& fsm) {
        Derived::on_exit_state(fsm);
    }
};

template<typename... States>
struct tick_fsm_state_pack {
    using FirstState = typename tfsm_details::first_type<States...>::type;
    using FSMType = typename FirstState::FSMType;

    // Compile-time checks
    static_assert(sizeof...(States) > 0, "State pack cannot be empty");
    static_assert(tfsm_details::check_fsm_types<FSMType, States...>::value,
                  "All states must have the same FSMType");
    static_assert(tfsm_details::check_state_ids<0, States...>::value,
                  "State IDs must be sequential starting from 0");
    static_assert((sizeof...(States) - 1) < etl::integral_limits<etl::fsm_state_id_t>::max,
                  "last state id overflow");
    static_assert(!etl::integral_limits<etl::fsm_state_id_t>::is_signed,
                  "fsm_state_id_t must be unsigned");

    using on_enter_fn = etl::fsm_state_id_t(*)(FSMType&);
    using on_run_fn = etl::fsm_state_id_t(*)(FSMType&);
    using on_exit_fn = void(*)(FSMType&);

    static constexpr size_t state_count = sizeof...(States);

    static const on_enter_fn* get_enter_table() {
        static const on_enter_fn table[state_count] = { &States::on_enter_state... };
        return table;
    }

    static const on_run_fn* get_run_table() {
        static const on_run_fn table[state_count] = { &States::on_run_state... };
        return table;
    }

    static const on_exit_fn* get_exit_table() {
        static const on_exit_fn table[state_count] = { &States::on_exit_state... };
        return table;
    }

    static constexpr size_t get_state_count() {
        return state_count;
    }
};

template<typename FSMImpl>
class tick_fsm {
public:
    using on_enter_fn = etl::fsm_state_id_t(*)(FSMImpl&);
    using on_run_fn = etl::fsm_state_id_t(*)(FSMImpl&);
    using on_exit_fn = void(*)(FSMImpl&);

    static constexpr etl::fsm_state_id_t Uninitialized =
        etl::integral_limits<etl::fsm_state_id_t>::max;

private:
    const on_enter_fn* enter_table = nullptr;
    const on_run_fn* run_table = nullptr;
    const on_exit_fn* exit_table = nullptr;
    size_t state_count = 0;
    etl::fsm_state_id_t current_state_id = Uninitialized;

    FSMImpl& impl() { return static_cast<FSMImpl&>(*this); }

public:
    template<typename StatePack>
    void set_states(etl::fsm_state_id_t initial = Uninitialized) {
        static_assert(etl::is_same<FSMImpl, typename StatePack::FSMType>::value,
                      "StatePack FSMType must match tick_fsm FSMImpl type");

        enter_table = StatePack::get_enter_table();
        run_table = StatePack::get_run_table();
        exit_table = StatePack::get_exit_table();
        state_count = StatePack::get_state_count();

        current_state_id = initial;
        if (current_state_id < state_count) {
            etl::fsm_state_id_t result = enter_table[current_state_id](impl());
            if (result < state_count && result != current_state_id) {
                change_state(result);
            }
        }
    }

    bool is_uninitialized() const { return current_state_id == Uninitialized; }

    etl::fsm_state_id_t get_state_id() const {
        return current_state_id;
    }

    void run() {
        if (current_state_id >= state_count) return;

        etl::fsm_state_id_t result = run_table[current_state_id](impl());

        if (result == etl::ifsm_state::Self_Transition) {
            exit_table[current_state_id](impl());
            etl::fsm_state_id_t enter_result = enter_table[current_state_id](impl());
            if (enter_result < state_count && enter_result != current_state_id) {
                change_state(enter_result);
            }
        } else if (result < state_count && result != current_state_id) {
            change_state(result);
        }
    }

    void change_state(etl::fsm_state_id_t new_state_id, bool reenter = false) {
        // Allow "reset to uninitialized" sugar.
        if (new_state_id == Uninitialized) {
            if (current_state_id < state_count) {
                exit_table[current_state_id](impl());
            }
            current_state_id = Uninitialized;
            return;
        }

        if (new_state_id >= state_count) return;

        const bool same = (current_state_id < state_count) && (new_state_id == current_state_id);
        if (same && !reenter) return;

        etl::fsm_state_id_t next_state_id = new_state_id;
        bool have_current = (current_state_id < state_count);

        do {
            if (have_current) {
                exit_table[current_state_id](impl());
            }
            current_state_id = next_state_id;

            etl::fsm_state_id_t result = enter_table[current_state_id](impl());
            next_state_id = (result < state_count) ? result : current_state_id;
            have_current = true;
        } while (next_state_id != current_state_id);
    }

    template<typename E, typename = typename etl::enable_if<!etl::is_same<E, etl::fsm_state_id_t>::value>::type>
    void change_state(E e, bool reenter = false) { change_state(static_cast<etl::fsm_state_id_t>(e), reenter); }
};

} // namespace etl_ext
