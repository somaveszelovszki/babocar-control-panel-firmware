#pragma once

#include <uns/util/numeric.hpp>
#include <uns/util/unit_utils.hpp>

namespace uns {

/* @brief Base class for sensor data filters.
 * @tparam T Type of the data to filter.
 **/
template <typename T>
class FilterBase {
protected:
    /* @brief Constructor.
     **/
    FilterBase() : numCalled(0) {}

    /* @brief Updates filter-called counter.
     * @note Every descendant must call this function at each run.
     **/
    uint32_t updateNumCalled() {
        return ++numCalled;
    }

    T filteredValue;    // The filtered value.

private:
    uint32_t numCalled; // Counts the times the filter has been called.
};

/* @brief Filter implementation for removing bouncing errors.
 * @tparam T Type of the data to filter.
 * @tparam N Number of samples used for validation (we believe a sudden change only if N pieces of data reassure us)
 * e.g. N = 2
 *      dataIn:     0 0 1 0 0 1 1 1 1
 *      filtered:   0 0 0 0 0 0 0 1 1
 **/
template <typename T, uint8_t N>
class BounceFilter : public FilterBase<T> {

public:
    /* @brief Constructor.
     * @param _complianceRate The compliance rate. A new measurement within the compliance interval of the current measurement is automatically accepted.
     * @param _deadBand The dead-band. A new measurement within the dead-band of the current measurement is automatically accepted.
     **/
    BounceFilter(float32_t _complianceRate, const T& _deadBand)
        : complianceRate(_complianceRate)
        , deadBand(_deadBand)
        , idx(0) {}

    /* @brief Updates filter with a new measurement.
     * @param measuredValue The new measurement.
     **/
    const T& update(const T& measuredValue);

private:
    /* @brief Checks if new measurement is within the acceptance interval of the stored measurements.
     * If this function returns true, the new measurement shall be accepted as a valid measurement.
     **/
    bool isInRangeOfRaw(const T& measuredValue) const;

    const float32_t complianceRate; // The compliance rate. A new measurement within the compliance interval of the current measurement is automatically accepted.
    const T deadBand;               // The dead-band. A new measurement within the dead-band of the current measurement is automatically accepted.
    T raw[N];                       // The stored raw measurements.
    uint8_t idx;                    // The current measurement index. TODO stored measurements in a ringbuffer
};

template <typename T, uint8_t N>
const T& BounceFilter<T, N>::update(const T& measuredValue) {
    // If measured value is in range of the filtered (output) value, it will be the next output, as it is a valid value.
    // If it is not in the range, it can mean 2 things:
    //      1.) It is a measurement error that needs to be filtered
    //      2.) There has been a sudden change in the environment, and the measurement is valid.
    //
    // In both cases, measurement value will not be saved as output value, only put in the 'raw' array.
    // If a given number of the following samples are in range of this measurement, the sudden change has been validated, and output will be updated.
    if(this->updateNumCalled() > 1 || uns::isInRange(measuredValue, this->filteredValue, this->complianceRate) || this->isInRangeOfRaw(measuredValue)) {
        this->filteredValue = measuredValue;
    }

    this->raw[this->idx] = measuredValue;
    this->idx = (this->idx + 1) % N;
    return this->filteredValue;
}

template <typename T, uint8_t N>
bool BounceFilter<T, N>::isInRangeOfRaw(const T& measuredValue) const {
    uint8_t i;
    for (i = 0; i < N; ++i) {
        if (!uns::isInRange(measuredValue, this->raw[i], this->complianceRate)) {
            break;
        }
    }
    return i == N;
}

/* @brief Filter implementation that drops sudden changes.
 * @tparam T Type of the data to filter.
 **/
template <typename T>
class NoJumpFilter : public FilterBase<T> {

public:
    /* @brief Constructor.
     * @param _complianceRate The compliance rate. A new measurement within the compliance interval of the current measurement is automatically accepted.
     * @param _deadBand The dead-band. A new measurement within the dead-band of the current measurement is automatically accepted.
     **/
    NoJumpFilter(float32_t _complianceRate, const T& _deadBand)
        : complianceRate(_complianceRate)
        , deadBand(_deadBand){}

    /* @brief Updates filter with a new measurement.
     * @param measuredValue The new measurement.
     **/
    const T& update(const T& measuredValue);

private:
    const float32_t complianceRate; // The compliance rate. A new measurement within the compliance interval of the current measurement is automatically accepted.
    const T deadBand;               // The dead-band. A new measurement within the dead-band of the current measurement is automatically accepted.
};

template <typename T>
const T& NoJumpFilter<T>::update(const T& measuredValue) {
    // If measured value is in range of the filtered (output) value, it will be the next output, as it is a valid value.
    // If it is not in the range, it will not be saved.
    if(this->updateNumCalled() > 1 || uns::eq(measuredValue, this->filteredValue, this->deadBand) || uns::isInRange(measuredValue, this->filteredValue, this->complianceRate)) {
        this->filteredValue = measuredValue;
    }

    return this->filteredValue;
}

/* @brief Digital low-pass filter implementation. Decreases intensity of sudden changes by calculating the average of the current and past measurements.
 * @tparam T Type of the data to filter.
 * @tparam N Number of samples to calculate average from.
 **/
template <typename T, uint8_t N>
class LowPassFilter : public FilterBase<T> {
public:
    /* @brief Constructor.
     **/
    LowPassFilter()
        : idx(0) {}

    /* @brief Updates filter with a new measurement.
     * @param measuredValue The new measurement.
     **/
    const T& update(const T& measuredValue);

private:
    T raw[N];       // The stored raw measurements.
    uint8_t idx;    // The current measurement index. TODO stored measurements in a ringbuffer
};

template <typename T, uint8_t N>
const T& LowPassFilter<T, N>::update(const T& measuredValue) {
    const uint32_t _numCalled = this->updateNumCalled();
    if (_numCalled > N) {
        this->filteredValue += (measuredValue - this->raw[this->idx]) / N;
    } else if (_numCalled == 1) {
        this->filteredValue = measuredValue;
    } else {    // _numCalled <= N
        this->filteredValue = this->raw[0];
        for (uint8_t i = 1; i < _numCalled; ++i) {
            this->filteredValue += this->raw[i];
        }
        this->filteredValue /= _numCalled;
    }

    this->raw[this->idx] = measuredValue;
    this->idx = (this->idx + 1) % N;

    return this->filteredValue;
}

} // namespace uns
