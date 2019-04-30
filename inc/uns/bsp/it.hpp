#pragma once

#include <uns/util/units.hpp>

namespace uns {

/* @brief Enters critical section.
 **/
void enterCritical();

/* @brief Enters critical section from ISR.
 * @returns The interrupt mask state as it was before the macro was called.
 **/
uint32_t enterCritical_ISR();

/* @brief Exits critical section.
 **/
void exitCritical();

/* @brief Exits critical section from ISR.
 * @param uxSavedInterruptStatus The interrupt mask state as it was before entering the critical section (this must be the result of the previously called enterCritical()).
 */
void exitCritical_ISR(uint32_t uxSavedInterruptStatus);

} // namespace uns
