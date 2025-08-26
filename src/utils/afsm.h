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

#define AFSM_STR_(x) #x
#define AFSM_STR(x)  AFSM_STR_(x)

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

    struct interceptor_pack_interface {
        const void* enter_table;
        const void* run_table;
        const void* exit_table;
        size_t element_count;
    };

    struct enter_result {
        state_id_t next_state;
        size_t interceptors_executed;
        bool main_state_executed;
    };

} // namespace details

template<typename FSM, typename Derived>
class state_base {
public:
    using FSMType = FSM;

    static constexpr auto No_State_Change = afsm::No_State_Change;
    static constexpr auto Self_Transition = afsm::Self_Transition;
    static constexpr auto Uninitialized = afsm::Uninitialized;

    static state_id_t on_enter_state(FSMType& fsm) {
        return Derived::on_enter_state(fsm);
    }

    static state_id_t on_run_state(FSMType& fsm) {
        return Derived::on_run_state(fsm);
    }

    static void on_exit_state(FSMType& fsm) {
        Derived::on_exit_state(fsm);
    }
};

template<typename FSM, typename Derived>
using interceptor = state_base<FSM, Derived>;

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
        static const on_enter_fn table[element_count] = { &Elements::on_enter_state... };
        return table;
    }

    static const on_run_fn* get_run_table() {
        static const on_run_fn table[element_count] = { &Elements::on_run_state... };
        return table;
    }

    static const on_exit_fn* get_exit_table() {
        static const on_exit_fn table[element_count] = { &Elements::on_exit_state... };
        return table;
    }

    static constexpr size_t get_element_count() {
        return element_count;
    }
};

template<typename... Interceptors>
class interceptor_pack : public pack_base<Interceptors...> {
public:
    using interceptor_pack_type = interceptor_pack<Interceptors...>;

    static const details::interceptor_pack_interface* get_interface() {
        static const details::interceptor_pack_interface interface = {
            pack_base<Interceptors...>::get_enter_table(),
            pack_base<Interceptors...>::get_run_table(),
            pack_base<Interceptors...>::get_exit_table(),
            pack_base<Interceptors...>::element_count
        };
        return &interface;
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

    using interceptor_pack_ptr = const details::interceptor_pack_interface*;

    static constexpr size_t state_count = pack_base<States...>::element_count;

    template<typename State>
    struct has_interceptors {
        template<typename T>
        static auto test(int) -> decltype(typename T::interceptor_pack_type{}, etl::true_type{});
        template<typename>
        static etl::false_type test(...);

        static constexpr bool value = decltype(test<State>(0))::value;
    };

    template<typename State, bool HasInterceptors>
    struct interceptor_pack_extractor {
        static constexpr interceptor_pack_ptr extract() {
            return nullptr;
        }
    };

    template<typename State>
    struct interceptor_pack_extractor<State, true> {
        static constexpr interceptor_pack_ptr extract() {
            return State::interceptor_pack_type::get_interface();
        }
    };

    template<typename State>
    static constexpr interceptor_pack_ptr extract_interceptor_pack() {
        return interceptor_pack_extractor<State, has_interceptors<State>::value>::extract();
    }

    static const interceptor_pack_ptr* get_interceptor_table() {
        static const interceptor_pack_ptr table[state_count] = {
            extract_interceptor_pack<States>()...
        };
        return table;
    }

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
    const details::interceptor_pack_interface* const* interceptor_table = nullptr;

    size_t state_count{0};
    state_id_t current_state_id{Uninitialized};
    state_id_t previous_state_id{Uninitialized};
    bool is_busy{false};

    FSMImpl& impl() { return static_cast<FSMImpl&>(*this); }

    details::enter_result execute_enter(state_id_t state_id) {
        details::enter_result result = {state_id, 0, false};

        if (interceptor_table[state_id]) {
            const auto& pack = *interceptor_table[state_id];
            auto enter_table_interceptors = static_cast<const on_enter_fn*>(pack.enter_table);

            for (size_t i = 0; i < pack.element_count; ++i) {
                auto transition_result = enter_table_interceptors[i](impl());
                if (transition_result != No_State_Change) {
                    result.next_state = transition_result;
                    result.interceptors_executed = i + 1;
                    return result;
                }
            }
            result.interceptors_executed = pack.element_count;
        }

        result.next_state = enter_table[state_id](impl());
        result.main_state_executed = true;

        return result;
    }

    state_id_t execute_run(state_id_t state_id) {
        if (interceptor_table[state_id]) {
            const auto& pack = *interceptor_table[state_id];
            auto run_table_interceptors = static_cast<const on_run_fn*>(pack.run_table);

            for (size_t i = 0; i < pack.element_count; ++i) {
                auto result = run_table_interceptors[i](impl());
                if (result != No_State_Change) {
                    return result;
                }
            }
        }
        return run_table[state_id](impl());
    }

    void execute_exit(state_id_t state_id, const details::enter_result* rollback_info = nullptr) {
        if (rollback_info) {
            // If enter "transaction" was incomplete, do symmetric rollback
            if (rollback_info->main_state_executed) {
                exit_table[state_id](impl());
            }

            if (interceptor_table[state_id] && rollback_info->interceptors_executed > 0) {
                const auto& pack = *interceptor_table[state_id];
                auto exit_table_interceptors = static_cast<const on_exit_fn*>(pack.exit_table);

                for (size_t i = rollback_info->interceptors_executed; i > 0; --i) {
                    exit_table_interceptors[i-1](impl());
                }
            }
        } else {
            exit_table[state_id](impl());
            if (interceptor_table[state_id]) {
                const auto& pack = *interceptor_table[state_id];
                auto exit_table_interceptors = static_cast<const on_exit_fn*>(pack.exit_table);

                for (size_t i = pack.element_count; i > 0; --i) {
                    exit_table_interceptors[i-1](impl());
                }
            }
        }
    }

public:
    template<typename StatePack>
    void set_states(state_id_t initial = Uninitialized) {
        static_assert(etl::is_same<FSMImpl, typename StatePack::FirstElement::FSMType>::value,
                    "StatePack FSMType must match fsm FSMImpl type");

        enter_table = StatePack::get_enter_table();
        run_table = StatePack::get_run_table();
        exit_table = StatePack::get_exit_table();
        interceptor_table = StatePack::get_interceptor_table();
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
                        "run() recursive call in " AFSM_STR(FSMImpl), "EFSM"),
                        file, line) {}
            };

            ETL_ASSERT_FAIL(ETL_ERROR(run_recursion_err));
            return;
        }

        if (current_state_id >= state_count) { return; }

        is_busy = true;
        auto result = execute_run(current_state_id);
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
                        "change_state() recursive call in " AFSM_STR(FSMImpl), "EFSM"),
                        file, line) {}
            };

            ETL_ASSERT_FAIL(ETL_ERROR(change_state_recursion_err));
            return;
        }

        if (new_state_id == Uninitialized) {
            if (current_state_id < state_count) {
                previous_state_id = current_state_id;
                is_busy = true;
                execute_exit(current_state_id);
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
            execute_exit(current_state_id);
            is_busy = false;
        }

        do {
            current_state_id = next_state_id;

            is_busy = true;
            auto enter_result = execute_enter(current_state_id);
            is_busy = false;

            if (enter_result.next_state == No_State_Change) {
                next_state_id = current_state_id;
            } else if (enter_result.next_state == Uninitialized) {
                is_busy = true;
                execute_exit(current_state_id, &enter_result);
                is_busy = false;
                previous_state_id = current_state_id;
                current_state_id = Uninitialized;
                return;
            } else if (enter_result.next_state < state_count) {
                is_busy = true;
                execute_exit(current_state_id, &enter_result);
                is_busy = false;
                previous_state_id = current_state_id;
                next_state_id = enter_result.next_state;
            } else {
                next_state_id = current_state_id;
            }
        } while (next_state_id != current_state_id);
    }

    template<typename E, typename = typename etl::enable_if<!etl::is_same<E, state_id_t>::value>::type>
    void change_state(E e, bool reenter = false) { change_state(static_cast<state_id_t>(e), reenter); }
};

} // namespace afsm
