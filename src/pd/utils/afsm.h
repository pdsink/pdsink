#pragma once

#include <etl/error_handler.h>
#include <etl/fsm.h>
#include <etl/type_lookup.h>
#include <etl/type_traits.h>
#include <etl/utility.h>
#include <etl/integral_limits.h>
#include <stddef.h>

namespace afsm {

using state_id_t = etl::fsm_state_id_t;
static constexpr state_id_t No_State_Change = etl::ifsm_state::No_State_Change;
static constexpr state_id_t Self_Transition = etl::ifsm_state::Self_Transition;
static constexpr state_id_t Uninitialized = etl::integral_limits<etl::fsm_state_id_t>::max;

class fsm_core;

namespace details {
    using on_enter_fn = state_id_t(*)(fsm_core&, state_id_t);
    using on_run_fn = state_id_t(*)(fsm_core&, state_id_t);
    using on_exit_fn = void(*)(fsm_core&, state_id_t);

    template<size_t Index, typename First, typename... Rest>
    struct type_at : type_at<Index - 1, Rest...> {};

    template<typename First, typename... Rest>
    struct type_at<0, First, Rest...> {
        using type = First;
    };

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

    static state_id_t _afsm_on_enter_state(fsm_core& core) {
        auto& fsm = static_cast<FSMType&>(core);
        using Hooks = typename details::state_hooks<Derived>::type;
        Hooks::enter(fsm);
        return Derived::on_enter_state(fsm);
    }

    static state_id_t _afsm_on_run_state(fsm_core& core) {
        auto& fsm = static_cast<FSMType&>(core);
        using Hooks = typename details::state_hooks<Derived>::type;
        Hooks::run(fsm);
        return Derived::on_run_state(fsm);
    }

    static void _afsm_on_exit_state(fsm_core& core) {
        auto& fsm = static_cast<FSMType&>(core);
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

    using on_enter_fn = details::on_enter_fn;
    using on_run_fn = details::on_run_fn;
    using on_exit_fn = details::on_exit_fn;

    static constexpr size_t element_count = sizeof...(Elements);

    static_assert(sizeof...(Elements) > 0, "Pack cannot be empty");
    static_assert(details::check_fsm_types<typename FirstElement::FSMType, Elements...>::value,
                  "All elements must have the same FSMType");

    struct enter_op {
        template<typename S> static state_id_t call(fsm_core& c) { return S::_afsm_on_enter_state(c); }
        static state_id_t none() { return No_State_Change; }
    };
    struct run_op {
        template<typename S> static state_id_t call(fsm_core& c) { return S::_afsm_on_run_state(c); }
        static state_id_t none() { return No_State_Change; }
    };
    struct exit_op {
        template<typename S> static void call(fsm_core& c) { S::_afsm_on_exit_state(c); }
        static void none() {}
    };

    template<typename Op, size_t I>
    static auto at(fsm_core& core) {
        if constexpr (I < element_count) {
            return Op::template call<typename details::type_at<I, Elements...>::type>(core);
        } else {
            return Op::none();
        }
    }

    // 64-way switch, chained for larger packs (same idea as boost::mp11::mp_with_index)
    template<typename Op, size_t Base>
    static auto dispatch(fsm_core& core, state_id_t id) {
        switch (id - Base) {
            case 0: return at<Op, Base + 0>(core);
            case 1: return at<Op, Base + 1>(core);
            case 2: return at<Op, Base + 2>(core);
            case 3: return at<Op, Base + 3>(core);
            case 4: return at<Op, Base + 4>(core);
            case 5: return at<Op, Base + 5>(core);
            case 6: return at<Op, Base + 6>(core);
            case 7: return at<Op, Base + 7>(core);
            case 8: return at<Op, Base + 8>(core);
            case 9: return at<Op, Base + 9>(core);
            case 10: return at<Op, Base + 10>(core);
            case 11: return at<Op, Base + 11>(core);
            case 12: return at<Op, Base + 12>(core);
            case 13: return at<Op, Base + 13>(core);
            case 14: return at<Op, Base + 14>(core);
            case 15: return at<Op, Base + 15>(core);
            case 16: return at<Op, Base + 16>(core);
            case 17: return at<Op, Base + 17>(core);
            case 18: return at<Op, Base + 18>(core);
            case 19: return at<Op, Base + 19>(core);
            case 20: return at<Op, Base + 20>(core);
            case 21: return at<Op, Base + 21>(core);
            case 22: return at<Op, Base + 22>(core);
            case 23: return at<Op, Base + 23>(core);
            case 24: return at<Op, Base + 24>(core);
            case 25: return at<Op, Base + 25>(core);
            case 26: return at<Op, Base + 26>(core);
            case 27: return at<Op, Base + 27>(core);
            case 28: return at<Op, Base + 28>(core);
            case 29: return at<Op, Base + 29>(core);
            case 30: return at<Op, Base + 30>(core);
            case 31: return at<Op, Base + 31>(core);
            case 32: return at<Op, Base + 32>(core);
            case 33: return at<Op, Base + 33>(core);
            case 34: return at<Op, Base + 34>(core);
            case 35: return at<Op, Base + 35>(core);
            case 36: return at<Op, Base + 36>(core);
            case 37: return at<Op, Base + 37>(core);
            case 38: return at<Op, Base + 38>(core);
            case 39: return at<Op, Base + 39>(core);
            case 40: return at<Op, Base + 40>(core);
            case 41: return at<Op, Base + 41>(core);
            case 42: return at<Op, Base + 42>(core);
            case 43: return at<Op, Base + 43>(core);
            case 44: return at<Op, Base + 44>(core);
            case 45: return at<Op, Base + 45>(core);
            case 46: return at<Op, Base + 46>(core);
            case 47: return at<Op, Base + 47>(core);
            case 48: return at<Op, Base + 48>(core);
            case 49: return at<Op, Base + 49>(core);
            case 50: return at<Op, Base + 50>(core);
            case 51: return at<Op, Base + 51>(core);
            case 52: return at<Op, Base + 52>(core);
            case 53: return at<Op, Base + 53>(core);
            case 54: return at<Op, Base + 54>(core);
            case 55: return at<Op, Base + 55>(core);
            case 56: return at<Op, Base + 56>(core);
            case 57: return at<Op, Base + 57>(core);
            case 58: return at<Op, Base + 58>(core);
            case 59: return at<Op, Base + 59>(core);
            case 60: return at<Op, Base + 60>(core);
            case 61: return at<Op, Base + 61>(core);
            case 62: return at<Op, Base + 62>(core);
            case 63: return at<Op, Base + 63>(core);
            default: break;
        }
        if constexpr (Base + 64 < element_count) {
            return dispatch<Op, Base + 64>(core, id);
        } else {
            return Op::none();
        }
    }

    static state_id_t dispatch_enter(fsm_core& core, state_id_t id) { return dispatch<enter_op, 0>(core, id); }
    static state_id_t dispatch_run(fsm_core& core, state_id_t id) { return dispatch<run_op, 0>(core, id); }
    static void dispatch_exit(fsm_core& core, state_id_t id) { dispatch<exit_op, 0>(core, id); }

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

// Common engine shared by all FSM types. State wrappers restore the concrete type.
class fsm_core {
private:
    details::on_enter_fn enter_dispatch = nullptr;
    details::on_run_fn run_dispatch = nullptr;
    details::on_exit_fn exit_dispatch = nullptr;

    size_t state_count{0};
    state_id_t current_state_id{Uninitialized};
    state_id_t previous_state_id{Uninitialized};
    bool is_busy{false};

protected:
    void set_dispatchers(details::on_enter_fn enter,
                         details::on_run_fn run,
                         details::on_exit_fn exit,
                         size_t count,
                         state_id_t initial)
    {
        enter_dispatch = enter;
        run_dispatch = run;
        exit_dispatch = exit;
        state_count = count;

        current_state_id = Uninitialized;
        previous_state_id = Uninitialized;

        if (initial < state_count) {
            change_state(initial);
        }
    }

public:
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
        auto result = run_dispatch(*this, current_state_id);
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
                exit_dispatch(*this, current_state_id);
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
            exit_dispatch(*this, current_state_id);
            is_busy = false;
        }

        do {
            current_state_id = next_state_id;

            is_busy = true;
            auto result = enter_dispatch(*this, current_state_id);
            is_busy = false;

            if (result == No_State_Change) {
                next_state_id = current_state_id;
            } else if (result == Uninitialized) {
                is_busy = true;
                exit_dispatch(*this, current_state_id);
                is_busy = false;
                previous_state_id = current_state_id;
                current_state_id = Uninitialized;
                return;
            } else if (result < state_count) {
                is_busy = true;
                exit_dispatch(*this, current_state_id);
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

template<typename FSMImpl>
class fsm : public fsm_core {
public:
    using on_enter_fn = state_id_t(*)(FSMImpl&);
    using on_run_fn = state_id_t(*)(FSMImpl&);
    using on_exit_fn = void(*)(FSMImpl&);

    template<typename StatePack>
    void set_states(state_id_t initial = Uninitialized) {
        static_assert(etl::is_same<FSMImpl, typename StatePack::FirstElement::FSMType>::value,
                    "StatePack FSMType must match fsm FSMImpl type");

        set_dispatchers(&StatePack::dispatch_enter,
                        &StatePack::dispatch_run,
                        &StatePack::dispatch_exit,
                        StatePack::get_state_count(),
                        initial);
    }
};

} // namespace afsm
