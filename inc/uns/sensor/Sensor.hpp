#pragma once

#include <uns/base/Runnable.hpp>

namespace uns {

template <typename M>
class Sensor : public Runnable {

public:
    explicit Sensor(millisecond_t _period) : Runnable(_period) {}

    /* @brief Gets measured value.
     * @returns The measured value.
     **/
    const M& getMeasured() const {
        return this->meas;
    }

protected:
    M meas; // The measured value.
};

} // namespace uns
