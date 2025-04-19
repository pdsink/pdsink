#pragma once

#include <etl/fsm.h>
#include <etl/tuple.h>
#include <etl/type_traits.h>

namespace etl_ext {

namespace detail {

// Compile-time: S::ID must equal its index in the type list (0..N-1)
template <size_t N, typename...> struct check_ids : etl::true_type {};

template <size_t N, typename S0, typename... Rest>
struct check_ids<N, S0, Rest...>
  : etl::integral_constant<bool,
        (S0::STATE_ID == N) && detail::check_ids<N + 1, Rest...>::value> {};

} // namespace detail

template <typename... FsmStates>
struct fsm_state_pack {
    static_assert(detail::check_ids<0, FsmStates...>::value,
        "State IDs must be 0..N-1 and in order");
    static_assert(sizeof...(FsmStates) > 0, "At least one state is required");

    etl::tuple<FsmStates...> storage{};
    etl::ifsm_state* states[sizeof...(FsmStates)]{ &etl::get<FsmStates>(storage)... };

    static constexpr size_t size = sizeof...(FsmStates);

    template <typename S>
    S& get() { return etl::get<S>(storage); }

    template <typename S>
    const S& get() const { return etl::get<S>(storage); }
};

} // namespace etl_ext
