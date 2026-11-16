#pragma once

#include <etl/error_handler.h>
#include <etl/fsm.h>
#include <etl/type_lookup.h>
#include <etl/type_traits.h>
#include <etl/integral_limits.h>
#include <stddef.h>

namespace afsm {

using state_id_t = etl::fsm_state_id_t;
static constexpr state_id_t No_State_Change = etl::ifsm_state::No_State_Change;
static constexpr state_id_t Self_Transition = etl::ifsm_state::Self_Transition;
static constexpr state_id_t Uninitialized = etl::integral_limits<etl::fsm_state_id_t>::max;

namespace details {
    template<typename...>
    struct first_type;

    template<typename First, typename... Rest>
    struct first_type<First, Rest...> {
        using type = First;
    };

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

    template<typename FSM, typename... Hooks>
    struct hook_callbacks;

    template<typename FSM>
    struct hook_callbacks<FSM> {
        static void enter(FSM&) {}
        static void run(FSM&) {}
        static void exit(FSM&) {}
    };

    template<typename FSM, typename First, typename... Rest>
    struct hook_callbacks<FSM, First, Rest...> {
        static_assert(etl::is_same<FSM, typename First::FSMType>::value,
                      "Hook FSMType must match state FSMType");
        static_assert(etl::is_same<void, decltype(First::on_enter_state(*static_cast<FSM*>(nullptr)))>::value,
                      "Hook on_enter_state must return void");
        static_assert(etl::is_same<void, decltype(First::on_run_state(*static_cast<FSM*>(nullptr)))>::value,
                      "Hook on_run_state must return void");
        static_assert(etl::is_same<void, decltype(First::on_exit_state(*static_cast<FSM*>(nullptr)))>::value,
                      "Hook on_exit_state must return void");

        static void enter(FSM& fsm) {
            First::on_enter_state(fsm);
            hook_callbacks<FSM, Rest...>::enter(fsm);
        }

        static void run(FSM& fsm) {
            First::on_run_state(fsm);
            hook_callbacks<FSM, Rest...>::run(fsm);
        }

        static void exit(FSM& fsm) {
            hook_callbacks<FSM, Rest...>::exit(fsm);
            First::on_exit_state(fsm);
        }
    };

    template<typename...>
    using void_t = void;

    template<typename State, typename = void>
    struct state_hooks {
        using type = hook_callbacks<typename State::FSMType>;
    };

    template<typename State>
    struct state_hooks<State, void_t<typename State::hook_pack_type>> {
        using type = typename State::hook_pack_type::template callbacks<typename State::FSMType>;
    };

} // namespace details

template<typename FSM, typename Derived>
class state_base {
public:
    using FSMType = FSM;

    static constexpr auto No_State_Change = afsm::No_State_Change;
    static constexpr auto Self_Transition = afsm::Self_Transition;
    static constexpr auto Uninitialized = afsm::Uninitialized;

    static state_id_t _afsm_on_enter_state(FSMType& fsm) {
        using Hooks = typename details::state_hooks<Derived>::type;
        Hooks::enter(fsm);
        return Derived::on_enter_state(fsm);
    }

    static state_id_t _afsm_on_run_state(FSMType& fsm) {
        using Hooks = typename details::state_hooks<Derived>::type;
        Hooks::run(fsm);
        return Derived::on_run_state(fsm);
    }

    static void _afsm_on_exit_state(FSMType& fsm) {
        Derived::on_exit_state(fsm);
        using Hooks = typename details::state_hooks<Derived>::type;
        Hooks::exit(fsm);
    }
};

template<typename FSM, typename Derived>
class hook {
public:
    using FSMType = FSM;
};

template<typename FSM, typename Derived, state_id_t StateID>
class state : public state_base<FSM, Derived> {
public:
    static constexpr state_id_t STATE_ID = StateID;
};

template<typename... Elements>
class pack_base {
public:
    using FirstElement = typename details::first_type<Elements...>::type;

    using on_enter_fn = state_id_t(*)(typename FirstElement::FSMType&);
    using on_run_fn = state_id_t(*)(typename FirstElement::FSMType&);
    using on_exit_fn = void(*)(typename FirstElement::FSMType&);

    static constexpr size_t element_count = sizeof...(Elements);

    static_assert(sizeof...(Elements) > 0, "Pack cannot be empty");
    static_assert(details::check_fsm_types<typename FirstElement::FSMType, Elements...>::value,
                  "All elements must have the same FSMType");

    static const on_enter_fn* get_enter_table() {
        static const on_enter_fn table[element_count] = { &Elements::_afsm_on_enter_state... };
        return table;
    }

    static const on_run_fn* get_run_table() {
        static const on_run_fn table[element_count] = { &Elements::_afsm_on_run_state... };
        return table;
    }

    static const on_exit_fn* get_exit_table() {
        static const on_exit_fn table[element_count] = { &Elements::_afsm_on_exit_state... };
        return table;
    }

    static constexpr size_t get_element_count() {
        return element_count;
    }
};

template<typename... Hooks>
class hook_pack {
public:
    using hook_pack_type = hook_pack<Hooks...>;
    using FirstElement = typename details::first_type<Hooks...>::type;

    using on_enter_fn = void(*)(typename FirstElement::FSMType&);
    using on_run_fn = void(*)(typename FirstElement::FSMType&);
    using on_exit_fn = void(*)(typename FirstElement::FSMType&);

    static constexpr size_t element_count = sizeof...(Hooks);

    static_assert(sizeof...(Hooks) > 0, "Pack cannot be empty");
    static_assert(details::check_fsm_types<typename FirstElement::FSMType, Hooks...>::value,
                  "All elements must have the same FSMType");

    template<typename FSM>
    using callbacks = details::hook_callbacks<FSM, Hooks...>;

    static constexpr size_t get_element_count() {
        return element_count;
    }
};

template<typename... States>
class state_pack : public pack_base<States...> {
public:
    static_assert(details::check_state_ids<0, States...>::value,
                  "State IDs must be sequential starting from 0");
    static_assert((sizeof...(States) - 1) < etl::integral_limits<etl::fsm_state_id_t>::max,
                  "last state id overflow");
    static_assert(!etl::integral_limits<etl::fsm_state_id_t>::is_signed,
                  "fsm_state_id_t must be unsigned");

    static constexpr size_t state_count = pack_base<States...>::element_count;

    static constexpr size_t get_state_count() {
        return state_count;
    }
};

template<typename FSMImpl>
class fsm {
public:
    using on_enter_fn = state_id_t(*)(FSMImpl&);
    using on_run_fn = state_id_t(*)(FSMImpl&);
    using on_exit_fn = void(*)(FSMImpl&);

private:
    const on_enter_fn* enter_table = nullptr;
    const on_run_fn* run_table = nullptr;
    const on_exit_fn* exit_table = nullptr;

    size_t state_count{0};
    state_id_t current_state_id{Uninitialized};
    state_id_t previous_state_id{Uninitialized};
    bool is_busy{false};

    FSMImpl& impl() { return static_cast<FSMImpl&>(*this); }

public:
    template<typename StatePack>
    void set_states(state_id_t initial = Uninitialized) {
        static_assert(etl::is_same<FSMImpl, typename StatePack::FirstElement::FSMType>::value,
                    "StatePack FSMType must match fsm FSMImpl type");

        enter_table = StatePack::get_enter_table();
        run_table = StatePack::get_run_table();
        exit_table = StatePack::get_exit_table();
        state_count = StatePack::get_state_count();

        current_state_id = Uninitialized;
        previous_state_id = Uninitialized;

        if (initial < state_count) {
            change_state(initial);
        }
    }

    bool is_uninitialized() const { return current_state_id == Uninitialized; }

    state_id_t get_state_id() const {
        return current_state_id;
    }

    state_id_t get_previous_state_id() const {
        return previous_state_id;
    }

    void run() {
        if (is_busy) {
            struct run_recursion_err : etl::exception {
                run_recursion_err(string_type file, numeric_type line)
                    : exception(ETL_ERROR_TEXT(
                        ".run() recursive call in afsm", "Eafsm"),
                        file, line) {}
            };

            ETL_ASSERT_FAIL(ETL_ERROR(run_recursion_err));
            return;
        }

        if (current_state_id >= state_count) { return; }

        is_busy = true;
        auto result = run_table[current_state_id](impl());
        is_busy = false;

        if (result == Self_Transition) {
            change_state(current_state_id, true);
        } else if (result < state_count && result != current_state_id) {
            change_state(result);
        }
    }

    void change_state(state_id_t new_state_id, bool reenter = false) {
        if (is_busy) {
            struct change_state_recursion_err : etl::exception {
                change_state_recursion_err(string_type file, numeric_type line)
                    : exception(ETL_ERROR_TEXT(
                        ".change_state() recursive call in afsm", "Eafsm"),
                        file, line) {}
            };

            ETL_ASSERT_FAIL(ETL_ERROR(change_state_recursion_err));
            return;
        }

        if (new_state_id == Uninitialized) {
            if (current_state_id < state_count) {
                previous_state_id = current_state_id;
                is_busy = true;
                exit_table[current_state_id](impl());
                is_busy = false;
            }
            current_state_id = Uninitialized;
            return;
        }

        if (new_state_id >= state_count) { return; }

        const bool same = (current_state_id < state_count) && (new_state_id == current_state_id);
        if (same && !reenter) { return; }

        auto next_state_id = new_state_id;
        bool have_current = (current_state_id < state_count);

        if (have_current || reenter) { previous_state_id = current_state_id; }
        if (have_current) {
            is_busy = true;
            exit_table[current_state_id](impl());
            is_busy = false;
        }

        do {
            current_state_id = next_state_id;

            is_busy = true;
            auto result = enter_table[current_state_id](impl());
            is_busy = false;

            if (result == No_State_Change) {
                next_state_id = current_state_id;
            } else if (result == Uninitialized) {
                is_busy = true;
                exit_table[current_state_id](impl());
                is_busy = false;
                previous_state_id = current_state_id;
                current_state_id = Uninitialized;
                return;
            } else if (result < state_count) {
                is_busy = true;
                exit_table[current_state_id](impl());
                is_busy = false;
                previous_state_id = current_state_id;
                next_state_id = result;
            } else {
                next_state_id = current_state_id;
            }
        } while (next_state_id != current_state_id);
    }

    template<typename E, typename = typename etl::enable_if<!etl::is_same<E, state_id_t>::value>::type>
    void change_state(E e, bool reenter = false) { change_state(static_cast<state_id_t>(e), reenter); }
};

} // namespace afsm
