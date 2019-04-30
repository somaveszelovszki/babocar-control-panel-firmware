#pragma once

#include <uns/container/Vec.hpp>

namespace uns {

/* @brief Maps implementation for storing matched key-value pairs.
 * @tparam K The key type.
 * @tparam V The value type.
 * @tparam capacity The map capacity.
  **/
template <typename K, typename V, uint32_t capacity>
class Map {
private:
    Vec<K, capacity> _keys;    // Vector of the keys.
    Vec<V, capacity> _values;  // Vector of the values.

public:
    /* @brief Default constructor - does not set keys and values.
     **/
    Map() {}

    /* @brief Puts a key-value pair into the map.
     * @param key The key.
     * @param Value The value.
     **/
    void put(const K& key, const V& value) {
        _keys[_keys.size++] = key;
        _values[_values.size++] = value;
    }

    /* @brief Gets value by key.
     * @param key The key.
     * @returns Pointer to the value or nullptr if key not found.
     **/
    V* get(const K& key) {
        V *res = nullptr;
        for (uint32_t i = 0; !res && i < _keys.size; ++i) {
            if (_keys[i] == key) {
                res = &_values[i];
            }
        }
        return res;
    }

    /* @brief Gets value by key.
     * @param key The key.
     * @returns Pointer to the value or nullptr if key not found.
     **/
    const V* get(const K& key) const {
        V *res = nullptr;
        for (uint32_t i = 0; !res && i < _keys.size; ++i) {
            if (_keys[i] == key) {
                res = _values + i;
            }
        }
        return static_cast<const V*>(res);
    }
};
} // namespace uns