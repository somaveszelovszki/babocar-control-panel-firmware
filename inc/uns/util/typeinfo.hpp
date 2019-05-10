#pragma once

namespace uns {

template <typename T> struct typeinfo {};

#define DEFINE_TYPEINFO(type)               \
template <> struct typeinfo<type> {         \
    static const char* name() {             \
        static const char *name_ = #type;   \
        return name_;                       \
    }                                       \
};

} // namespace uns
