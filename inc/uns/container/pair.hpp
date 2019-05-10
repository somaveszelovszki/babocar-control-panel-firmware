#ifndef UNS_UTIL_PAIR_HPP_
#define UNS_UTIL_PAIR_HPP_

template <typename T1, typename T2>
struct pair {

    pair() {}

    pair(const T1& first, const T2& second)
        : first(first)
        , second(second) {}

    T1 first;
    T2 second;
};

#endif /* UNS_UTIL_PAIR_HPP_ */
