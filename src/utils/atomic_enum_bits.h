#pragma once

#include "atomic_bits.h"

// Strictly typed variant of AtomicBits for enum classes.
// When you have a lot of enums, this helps to avoid mistypes.
template<typename E>
class AtomicEnumBits
{
    static_assert(etl::is_enum<E>::value, "Template param must be an enum class");
    static constexpr size_t NumBits = static_cast<size_t>(E::_Count);

    AtomicBits<NumBits> bits_;

public:
    void set(E f) noexcept { bits_.set(static_cast<size_t>(f)); }
    void clear(E f) noexcept { bits_.clear(static_cast<size_t>(f)); }
    bool test(E f) const noexcept { return bits_.test(static_cast<size_t>(f)); }
    bool test_and_set(E f) noexcept { return bits_.test_and_set(static_cast<size_t>(f)); }
    bool test_and_clear(E f) noexcept { return bits_.test_and_clear(static_cast<size_t>(f)); }

    void set_all() noexcept { bits_.set_all(); }
    void clear_all() noexcept { bits_.clear_all(); }
};
