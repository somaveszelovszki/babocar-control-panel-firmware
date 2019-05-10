#pragma once

#include <uns/container/vec.hpp>
#include <uns/container/pair.hpp>

namespace uns {

/* @brief Stores sorted key-value pairs.
 * @tparam K The key type.
 * @tparam V The value type.
 * @tparam capacity The map capacity.
  **/
template <typename K, typename V, uint32_t capacity_>
class map {
public:
    typedef pair<K, V> entry_type;

private:
    typedef vec<entry_type, capacity_> vec_type;

public:
    typedef typename vec_type::iterator iterator;
    typedef typename vec_type::const_iterator const_iterator;

    uint32_t size() const {
        return this->values_.size();
    }

    uint32_t capacity() const {
        return this->values_.capacity();
    }

    /* @brief Puts a key-value pair into the map.
     * @param key The key.
     * @param Value The value.
     **/
    void put(const K& key, const V& value) {
        if (this->values_.size() < this->values_.capacity()) {
            if (!this->size()) {
                this->values_.emplace_back(key, value);
            } else {
                for (iterator it = this->begin(); it != this->end(); ++it) {
                    if (key < it->first) {
                        this->values_.emplace(it, key, value);
                        break;
                    }
                }
            }
        }
    }

    /* @brief Gets value by key.
     * @param key The key.
     * @returns Pointer to the value or nullptr if key not found.
     **/
    const V* get(const K& key) const {
        const V *res = nullptr;

        if (this->size() > 0) {
            uint32_t first = 0;
            uint32_t last = this->size() - 1;

            do {

                uint32_t mid = uns::avg(first, last);
                const entry_type& entry = this->values_[mid];

                if (key < entry.first) {
                    last = mid - 1;
                } else if (key > entry.first) {
                    first = mid + 1;
                } else {
                    res = &entry.second;
                }

            } while (!res && first <= last);
        }

        return res;
    }

    /* @brief Gets value by key.
     * @param key The key.
     * @returns Pointer to the value or nullptr if key not found.
     **/
    V* get(const K& key) {
        return const_cast<V*>(const_cast<const map*>(this)->get(key));
    }

    /* @brief Clears vector.
     **/
    void clear() {
        this->values_.clear();
    }

    iterator begin() {
        return this->values_.begin();
    }

    const_iterator begin() const {
        return this->values_.begin();
    }

    iterator end() {
        return this->values_.end();
    }

    const_iterator end() const {
        return this->values_.end();
    }

private:
    vec<entry_type, capacity_> values_;  // Vector of the values.
};
} // namespace uns
