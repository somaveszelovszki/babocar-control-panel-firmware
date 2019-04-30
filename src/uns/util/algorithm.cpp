#include <uns/util/algorithm.hpp>
#include <uns/bsp/it.hpp>
#include <algorithm>

using namespace uns;

bool uns::testAndSet(bool *pValue, bool valueToSet) {
    uns::enterCritical();
    bool result = *pValue;
    *pValue = valueToSet;
    uns::exitCritical();

    return result;
}

bool uns::testAndSet_ISR(bool *pValue, bool valueToSet) {
    uint32_t uxSavedInterruptStatus = uns::enterCritical_ISR();
    bool result = *pValue;
    *pValue = valueToSet;
    uns::exitCritical_ISR(uxSavedInterruptStatus);

    return result;
}

void SwapExchange::swap() {
    uns::enterCritical();
    std::swap(this->value_GET, this->value_SET);
    uns::exitCritical();
}
