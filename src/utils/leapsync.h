#pragma once

#include <etl/atomic.h>
#include <etl/type_traits.h>

template<typename ParamType = void>
class LeapSync {
private:
    struct empty_storage {};
    struct param_storage { etl::atomic<ParamType> value; };

    etl::atomic<uint16_t> target_version;
    etl::atomic<uint16_t> completed_version;
    etl::atomic<uint16_t> processing_version;

    static bool is_greater(uint16_t a, uint16_t b) {
        return (int16_t)(a - b) > 0;
    }

    typename etl::conditional<
        etl::is_void<ParamType>::value,
        empty_storage,
        param_storage
    >::type storage;

public:
    LeapSync() : target_version(0), completed_version(0), processing_version(0) {}

    //
    // Producer methods
    //

    // enquire for non-void types
    template<typename T = ParamType>
    typename etl::enable_if<!etl::is_void<T>::value>::type
    enquire(const T& params) {
        storage.value.store(params);
        target_version.fetch_add(1);
    }

    // enquire for void
    template<typename T = ParamType>
    typename etl::enable_if<etl::is_void<T>::value>::type
    enquire() {
        target_version.fetch_add(1);
    }

    bool is_ready() const {
        return target_version.load() == completed_version.load();
    }

    //
    // Consumer methods
    //

    bool is_enquired() const {
        return is_greater(target_version.load(), processing_version.load());
    }

    bool is_started() const {
        return is_greater(processing_version.load(), completed_version.load());
    }

    void mark_started() {
        processing_version.store(target_version.load());
    }

    void mark_finished() {
        completed_version.store(processing_version.load());
    }

    void reset() {
        uint16_t target = target_version.load();
        completed_version.store(target);
        processing_version.store(target);
    }

    // access to params (only for non-void)
    template<typename T = ParamType>
    typename etl::enable_if<!etl::is_void<T>::value, etl::atomic<T>&>::type
    get_param() {
        return storage.value;
    }
};
