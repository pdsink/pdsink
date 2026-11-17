#pragma once

#include "../pd_log.h"

#include <etl/type_traits.h>

#include <stddef.h>

namespace pd {

// const char* by default, or the logger's static string type when
// PD_LOG_ASSUME_STATIC_STR is configured.
using state_name_t = etl::decay_t<decltype(PD_LOG_ASSUME_STATIC_STR(""))>;

struct StateNameEntry {
    int id;
    state_name_t name;
};

template <size_t N>
struct StateNames {
    state_name_t names[N];
    state_name_t unknown;

    constexpr auto operator()(int id) const -> state_name_t {
        return (id >= 0 && static_cast<size_t>(id) < N) ? names[id] : unknown;
    }
};

template <size_t N>
constexpr auto state_names_ordered(const StateNameEntry (&entries)[N]) -> bool {
    for (size_t i = 0; i < N; i++) {
        if (entries[i].id != static_cast<int>(i)) { return false; }
    }
    return true;
}

template <size_t N>
constexpr auto make_state_names(const StateNameEntry (&entries)[N], state_name_t unknown) -> StateNames<N> {
    StateNames<N> result{};
    for (size_t i = 0; i < N; i++) { result.names[i] = entries[i].name; }
    result.unknown = unknown;
    return result;
}

} // namespace pd

#define PD_STATE_NAME(id) ::pd::StateNameEntry{ id, PD_LOG_ASSUME_STATIC_STR(#id) }

// Defines `name(state_id)` returning the state name. Entries must follow the
// enum order; ids are checked at compile time and do not go to flash.
#define PD_DEFINE_STATE_NAMES(name, unknown_name, ...) \
    constexpr ::pd::StateNameEntry name##_entries[] = { __VA_ARGS__ }; \
    static_assert(::pd::state_names_ordered(name##_entries), \
        #name ": entries must follow the enum order"); \
    constexpr auto name = ::pd::make_state_names(name##_entries, \
        PD_LOG_ASSUME_STATIC_STR(unknown_name))
