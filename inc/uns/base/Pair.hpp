#pragma once

#include <uns/util/numeric.hpp>

namespace uns {
namespace base {

template <typename A, typename B>
class Pair {
public:
    /* @brief The left element.
     **/
    A left;

    /* @brief The right element.
     **/
    B right;
};

} // namespace base
} // namespace uns
