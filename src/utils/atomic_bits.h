#pragma once

#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/binary.h>

#include <stddef.h>
#include <stdint.h>

// For old compilers ETL can fallback to spinlocks, that's not acceptable.
// Can be workarounded by using separate bool flag for each bit at cost of
// extra memory. But seems to not worth efforts - just disable ancient garbage.
#if __cplusplus < 201402L
    #error "At least C++14 compiler required (use -std=c++14 or higher)"
#endif


template<size_t NumBits, typename StorageType = uint32_t>
class AtomicBits {
    static_assert(etl::is_unsigned<StorageType>::value, "StorageType must be unsigned integer");
    static_assert(NumBits > 0, "Number of bits must be positive");

    static constexpr size_t BITS_PER_STORAGE = sizeof(StorageType) * 8;
    static constexpr size_t STORAGE_SIZE = (NumBits + BITS_PER_STORAGE - 1) / BITS_PER_STORAGE;
    static constexpr size_t STORAGE_MASK = BITS_PER_STORAGE - 1;
    static constexpr size_t STORAGE_SHIFT = etl::count_trailing_zeros(BITS_PER_STORAGE);

public:
    AtomicBits() {
        clear_all();
    }

    void set(size_t bitIndex) {
        if (bitIndex >= NumBits) return;
        const size_t storageIndex = bitIndex >> STORAGE_SHIFT;
        const size_t bitOffset = bitIndex & STORAGE_MASK;
        storage[storageIndex].fetch_or(StorageType(1) << bitOffset);
    }

    void clear(size_t bitIndex) {
        if (bitIndex >= NumBits) return;
        const size_t storageIndex = bitIndex >> STORAGE_SHIFT;
        const size_t bitOffset = bitIndex & STORAGE_MASK;
        storage[storageIndex].fetch_and(~(StorageType(1) << bitOffset));
    }

    bool test(size_t bitIndex) const {
        if (bitIndex >= NumBits) return false;
        const size_t storageIndex = bitIndex >> STORAGE_SHIFT;
        const size_t bitOffset = bitIndex & STORAGE_MASK;
        return (storage[storageIndex].load() & (StorageType(1) << bitOffset)) != 0;
    }

    bool test_and_set(size_t bitIndex) {
        if (bitIndex >= NumBits) return false;
        const size_t storageIndex = bitIndex >> STORAGE_SHIFT;
        const size_t bitOffset = bitIndex & STORAGE_MASK;
        const StorageType mask = StorageType(1) << bitOffset;
        StorageType old = storage[storageIndex].fetch_or(mask);
        return (old & mask) != 0;
    }

    bool test_and_clear(size_t bitIndex) {
        if (bitIndex >= NumBits) return false;
        const size_t storageIndex = bitIndex >> STORAGE_SHIFT;
        const size_t bitOffset = bitIndex & STORAGE_MASK;
        const StorageType mask = StorageType(1) << bitOffset;
        StorageType old = storage[storageIndex].fetch_and(~mask);
        return (old & mask) != 0;
    }

    void set_all() {
        constexpr StorageType value = ~StorageType(0);
        for (size_t i = 0; i < STORAGE_SIZE; ++i) {
            storage[i].store(value);
        }
    }

    void clear_all() {
        for (size_t i = 0; i < STORAGE_SIZE; ++i) {
            storage[i].store(0);
        }
    }

private:
    etl::array<etl::atomic<StorageType>, STORAGE_SIZE> storage;
};