#pragma once

#include <etl/atomic.h>
#include <etl/type_traits.h>

template<typename ParamType = void>
class LeapSync {
private:
    struct empty_storage {};
    struct param_storage { ParamType value; };

    enum class STATE{
        IDLE,
        ENQUIRED,
        WORKING
    };

    etl::atomic<STATE> state{STATE::IDLE};

    typename etl::conditional<
        etl::is_void<ParamType>::value,
        empty_storage,
        param_storage
    >::type storage;

public:
    //
    // Producer methods
    //

    // enquire for non-void types
    template<typename T = ParamType>
    typename etl::enable_if<!etl::is_void<T>::value>::type
    enquire(const T& params) {
        // 2-phase commit
        state.store(STATE::IDLE);
        storage.value = params;
        state.store(STATE::ENQUIRED);
    }

    // enquire for void
    template<typename T = ParamType>
    typename etl::enable_if<etl::is_void<T>::value>::type
    enquire() {
        state.store(STATE::ENQUIRED);
    }

    bool is_idle() const { return state.load() == STATE::IDLE; }

    // Both producer and consumer
    void reset() { state.store(STATE::IDLE); }

    //
    // Consumer methods
    //

    // job fetch for non-void types
    template<typename T = ParamType>
    typename etl::enable_if<!etl::is_void<T>::value, bool>::type
    get_job(T& params) {
        // Empty or working => no new job to fetch
        if (state.load() != STATE::ENQUIRED) { return false; }

        T tmp = storage.value;
        auto expected = STATE::ENQUIRED;

        if (state.compare_exchange_strong(expected, STATE::WORKING)) {
            params = tmp;
            return true;
        }
        return false;
    }

    // job fetch for void
    template<typename T = ParamType>
    typename etl::enable_if<etl::is_void<T>::value, bool>::type
    get_job() {
        auto expected = STATE::ENQUIRED;
        if (state.compare_exchange_strong(expected, STATE::WORKING)) {
            return true;
        }
        return false;
    }

    // mark job finished
    void job_finish() {
        auto expected = STATE::WORKING;
        // Update status only if NOT overwritten by producer (with new job)
        state.compare_exchange_strong(expected, STATE::IDLE);
    }
};
